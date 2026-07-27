import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import  TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import tf_transformations as tft
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import math
import scipy.sparse.linalg as spla
from scipy.sparse import lil_matrix
import scipy.sparse

class PoseNode:
    def __init__(self, id, pose):
        self.node_id = id
        self.pose = pose

    def set_pose(self,new_pose):
        self.pose = new_pose

    def get_pose(self):
        return self.pose
    
    def get_id(self):
        return self.node_id

class Edge:
    def __init__(self):
        self.i = 0
        self.j = 0
        self.relative_pose = np.zeros((3,1))
        self.information_matrix = np.eye(3)
        self.eij = np.zeros((3,1))
        self.is_sequential = True

    def update_edge(self, i, j, pose_i, pose_j, information_matrix):
        self.i = i
        self.j = j
        self.relative_pose = self.compute_relative_pose(pose_i, pose_j)
        self.information_matrix = information_matrix

    def compute_relative_pose(self, pose_i, pose_j):
        delx  =  np.cos(pose_i[2,0])*(pose_j[0,0] - pose_i[0,0]) + np.sin(pose_i[2,0])*(pose_j[1,0] - pose_i[1,0])
        dely  = -np.sin(pose_i[2,0])*(pose_j[0,0] - pose_i[0,0]) + np.cos(pose_i[2,0])*(pose_j[1,0] - pose_i[1,0])
        deltheta  =  pose_j[2,0] - pose_i[2,0]
        deltheta = np.arctan2(np.sin(deltheta), np.cos(deltheta))
        return np.array([[delx], [dely], [deltheta]])

    def get_from_node_id(self):
        return self.i

    def get_to_node_id(self):
        return self.j

    def get_relative_pose(self):
        return self.relative_pose
    
    def get_inf_matrix(self):
        return self.information_matrix

class PoseGraph:
    def __init__(self, logger):
        self._logger = logger
        self.pose_nodes = []
        self.edges = []
        self.scans = []
        self.node_count = 0
        self.edge_count = 0

    def add_pose(self, pose_node, scan):
        self.pose_nodes.append(pose_node)
        self.scans.append(scan)
        self.node_count += 1
        #self._logger.info(f"Added node {pose_node.get_id()} to pose graph with pose {pose_node.get_pose().flatten()}")
        #self._logger.info(f"{self.node_count} ")

    def add_edge(self, edge, type):
        self.edges.append(edge)
        self.edge_count += 1
        #self._logger.info(f"Added {type} edge to pose graph from node {edge.get_from_node_id()} to node {edge.get_to_node_id()} with relative pose {edge.get_relative_pose().flatten()}")

    def optimise_graph(self, max_iterations=25, tol=1e-7):
        self._logger.info("Attempting optimisation")
        prev_chi2 = None
        for it in range(max_iterations):
            H_red, b_red, chi2 = self.compute_H_and_b_matrix()
            dx_reduced = self.solve_sparse_cholesky(H_red, b_red)
            dx_reduced = -dx_reduced

            # re-insert the fixed anchor node's zero update
            dx = np.zeros((3 * self.node_count, 1))
            dx[3:] = dx_reduced

            self.apply_update(dx)   # see below

            self._logger.info(f"iter {it}: chi2={chi2.item():.6f}")

            # convergence checks
            if np.linalg.norm(dx) < tol:
                self._logger.info("Converged: step size below tolerance")
                break
            if prev_chi2 is not None and abs(prev_chi2 - chi2) < tol:
                self._logger.info("Converged: chi2 change below tolerance")
                break
            prev_chi2 = chi2
    
    def apply_update(self, dx):
        self._logger.info("Updating poses")
        for idx, node in enumerate(self.pose_nodes):
            d = dx[3*idx:3*idx+3]
            pose = node.get_pose()
            new_pose = pose + d
            new_pose[2, 0] = np.arctan2(np.sin(new_pose[2, 0]), np.cos(new_pose[2, 0]))
            node.set_pose(new_pose)
            
    def compute_H_and_b_matrix(self):
        H = lil_matrix((3*self.node_count, 3*self.node_count))
        b = np.zeros((3*self.node_count, 1))
        chi2 = 0.0
        for edge in self.edges:
            i = edge.get_from_node_id()
            j = edge.get_to_node_id()
            pose_i = self.pose_nodes[i].get_pose()
            pose_j = self.pose_nodes[j].get_pose()
            relative_pose = edge.get_relative_pose()
            edge.eij = relative_pose - edge.compute_relative_pose(pose_i, pose_j)
            edge.eij[2, 0] = np.arctan2(np.sin(edge.eij[2, 0]), np.cos(edge.eij[2, 0]))
            Aij = self.compute_Aij(edge)
            Bij = self.compute_Bij(edge)
            om_ij = edge.get_inf_matrix()
            chi2 += edge.eij.T @ om_ij @ edge.eij
            H[3*i:3*i+3, 3*i:3*i+3] += Aij.T @ om_ij @ Aij
            H[3*i:3*i+3, 3*j:3*j+3] += Aij.T @ om_ij @ Bij
            H[3*j:3*j+3, 3*i:3*i+3] += Bij.T @ om_ij @ Aij
            H[3*j:3*j+3, 3*j:3*j+3] += Bij.T @ om_ij @ Bij
            b[3*i:3*i+3] += Aij.T @ om_ij @ edge.eij
            b[3*j:3*j+3] += Bij.T @ om_ij @ edge.eij

        H_red = H[3:, 3:]   # gauge-fix node 0
        b_red = b[3:]
        return H_red, b_red, chi2
    """
    def solve_sparse_cholesky(self, H_reduced_sparse, b_reduced):
    
        H_csc = H_reduced_sparse.tocsc()
        lu = spla.splu(H_csc)
        dx_reduced = lu.solve(b_reduced)
        return dx_reduced
    """
    def solve_sparse_cholesky(self, H_reduced_sparse, b_reduced, lam=1e-6):
        H_csc = H_reduced_sparse.tocsc()
        n = H_csc.shape[0]
        H_damped = H_csc + lam * scipy.sparse.eye(n, format='csc')
        lu = spla.splu(H_damped)
        dx_reduced = lu.solve(b_reduced)
        return dx_reduced
    
    def compute_Aij(self , edge):
        i = edge.get_from_node_id()
        j = edge.get_to_node_id()
        pose_i = self.pose_nodes[i].get_pose()
        pose_j = self.pose_nodes[j].get_pose()
        Aij = np.zeros((3,3))
        Aij[0,0] = np.cos(pose_i[2,0])
        Aij[0,1] = np.sin(pose_i[2,0])
        Aij[0,2] = np.sin(pose_i[2,0])*(pose_j[0,0] - pose_i[0,0]) - np.cos(pose_i[2,0])*(pose_j[1,0] - pose_i[1,0])
        Aij[1,0] = -np.sin(pose_i[2,0])
        Aij[1,1] = np.cos(pose_i[2,0])
        Aij[1,2] = np.cos(pose_i[2,0])*(pose_j[0,0] - pose_i[0,0]) + np.sin(pose_i[2,0])*(pose_j[1,0] - pose_i[1,0])
        Aij[2,2] = 1
        return Aij

    def compute_Bij(self, edge):
        i = edge.get_from_node_id()
        pose_i = self.pose_nodes[i].get_pose()
        Bij = np.zeros((3,3))
        Bij[0,0] = -np.cos(pose_i[2,0])
        Bij[0,1] = -np.sin(pose_i[2,0])
        Bij[1,0] = np.sin(pose_i[2,0])
        Bij[1,1] = -np.cos(pose_i[2,0])
        Bij[2,2] = -1
        return Bij
    
    def get_recent_pose(self):
        return self.pose_nodes[self.node_count - 1]
    
    def get_recent_scan(self):
        return self.scans[self.node_count - 1]

    def get_edges(self):
        return self.edges

class SelfMapper(Node):
    def __init__(self):
        super().__init__('self_mapper')
        self.odom_tf_subscriber = self.create_subscription(TFMessage, "/tf" , self.odom_tf_callback, 100)
        self.robot_truth_subscriber = self.create_subscription(TFMessage, "/model/slam_bot/pose_static" , self.robot_truth_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 100)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.tf_brodcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.map_timer = self.create_timer(0.3, self.mapper)
        self.grid = OccupancyGrid()
        self.width = 400*2
        self.height = 200*2
        self.res = 0.025
        self.L_occ  =  0.85   # log odds added when cell is hit
        self.L_free = -0.4    # log odds added when cell is passed through
        self.L_min  = -5.0
        self.L_max  =  5.0
        self.lidar_offx = 0.05
        self.lidar_offy = 0.0
        self.grid.info.height = self.height
        self.grid.info.width = self.width
        self.grid.info.resolution = self.res
        self.grid.info.origin.position.x = -self.width/2*self.res 
        self.grid.info.origin.position.y = -self.height/2*self.res 
        self.grid.header.frame_id = 'map'
        self.log_odds = [0.0] * (self.width*self.height)
        self.grid.data = [-1] * (self.width*self.height)
        for i in range(self.width*self.height):
            self.grid.data[i] = self.logodds_to_prob(self.log_odds[i])
        self.scan = LaserScan()
        
        self.is_first_scan = True
        self.is_first_tf = True
        self.robot_odom_msg = Odometry()
        self.graph = PoseGraph(self.get_logger())
        self.keyframe_min_dist = 0.2
        self.keyframe_min_angle = 0.1
        self.loop_closure_min_nodes = 25
        self.loop_closure_min_dist = 0.5
        self.max_icp_iteration = 20
        self.robot_truth_msg = TFMessage()
        self.icp_cooloff = 5
        self.icp_cooloff_max = 5
        self.tot_d = 0.0
        self.tot_r = 0.0
        self.count = 0

    def robot_truth_callback(self, msg):
        self.robot_truth_msg = msg
        return

    def odom_tf_callback(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
                if self.is_first_tf:
                    self.map_to_odom = TransformStamped()
                    self.map_to_odom.header.stamp = t.header.stamp
                    self.map_to_odom.header.frame_id = 'map'
                    self.map_to_odom.child_frame_id = 'odom'
                    self.map_to_odom.transform.translation.x = 0.0
                    self.map_to_odom.transform.translation.y = 0.0
                    self.map_to_odom.transform.translation.z = 0.0
                    quat = tft.quaternion_from_euler(0.0, 0.0, 0.0)
                    self.map_to_odom.transform.rotation.x = quat[0]
                    self.map_to_odom.transform.rotation.y = quat[1]
                    self.map_to_odom.transform.rotation.z = quat[2]
                    self.map_to_odom.transform.rotation.w = quat[3]
                    self.tf_brodcaster.sendTransform(self.map_to_odom)
                    self.get_logger().info(f"First transform published at {self.get_clock().now().to_msg()}")
                    self.is_first_tf = False
                    return

                self.map_to_odom.header.stamp = t.header.stamp
                self.tf_brodcaster.sendTransform(self.map_to_odom)
                return
        
    def logodds_to_prob(self, l):
        if l == 0.0:
            return -1
        p = 1 - (1/(1 + math.exp(l)))
        return int(p*100)
    
    def robot_odom_callback(self, msg):
        self.robot_odom_msg = msg     
        return
    
    def lidar_callback(self, msg):
        self.scan = msg
        if not self.is_first_tf:
            self.mapper()   
        return

    def xy_to_cell_xy(self, x, y):
        cell_x = int(math.floor(x/self.res)) + self.width/2 
        cell_y = int(math.floor(y/self.res)) + self.height/2 
        return cell_x, cell_y
    
    
    def icp(self,qf, p, prv_pose, cur_pose):
        inf_mat = np.eye(3)

        qf = qf + np.array([[self.lidar_offx], 
                            [self.lidar_offy]])
        
        p = p + np.array([[self.lidar_offx],
                          [self.lidar_offy]])

        R1_inv = np.array([[math.cos(-prv_pose[2,0]), -math.sin(-prv_pose[2,0])],
                           [math.sin(-prv_pose[2,0]), math.cos(-prv_pose[2,0])]])
        R1 = np.array([[math.cos(prv_pose[2,0]), -math.sin(prv_pose[2,0])],
                       [math.sin(prv_pose[2,0]), math.cos(prv_pose[2,0])]])
        T = R1_inv @ np.array([[cur_pose[0,0] - prv_pose[0,0]],
                               [cur_pose[1,0] - prv_pose[1,0]]])
        R = np.array([[math.cos(cur_pose[2,0] - prv_pose[2,0]), -math.sin(cur_pose[2,0] - prv_pose[2,0])],
                      [math.sin(cur_pose[2,0] - prv_pose[2,0]), math.cos(cur_pose[2,0] - prv_pose[2,0])]])
        q = R @ p + T
        threshold = 0.02
        it = 0
        max_corr_dist = 0.3      # meters, tune to your lidar noise/motion scale
        max_corr_dist_sq = max_corr_dist ** 2
        prev_error = float('inf')
        min_improvement = 0.001  # stop if error isn't dropping fast enough
        stagnation_patience = 3 # how many consecutive slow-improvement iters to tolerate
        stagnant_count = 0        

        while it < self.max_icp_iteration:

            dist_sq = np.zeros(len(q[0]))
            C = np.zeros((2, len(q[0])))
            valid = np.zeros(len(q[0]), dtype=bool)   # NEW: track inliers

            for i in range(len(q[0])):
                min_dist = 1e9
                best_j = -1
                for j in range(len(qf[0])):
                    d_sq = ((q[0][i] - qf[0][j])**2 + (q[1][i] - qf[1][j])**2)
                    if d_sq < min_dist:
                        min_dist = d_sq
                        best_j = j
                C[:, i] = qf[:, best_j]
                dist_sq[i] = min_dist
                valid[i] = min_dist < max_corr_dist_sq   # NEW: gate outliers

            inliers = np.count_nonzero(valid)
            ratio = inliers / len(q[0])

            if inliers < 3:
                #self.get_logger().info("not enough inliers")
                break

            C_valid = C[:, valid]
            p_valid = p[:, valid]
            dist_valid = dist_sq[valid]

            C_mean = np.array([[np.mean(C_valid[0])], [np.mean(C_valid[1])]])
            p_mean = np.array([[np.mean(p_valid[0])], [np.mean(p_valid[1])]])
            dist_mean = np.mean(np.sqrt(dist_valid))     # also fixed sq-dist bug from before

            C_centered = C_valid - C_mean
            p_centered = p_valid - p_mean
            H = p_centered @ C_centered.T
            u, S, vt = np.linalg.svd(H)
            Ut = u.T
            V = vt.T
            R_est = V @ Ut
            if np.linalg.det(R_est) < 0:
                V[:, -1] *= -1.0
                R_est = V @ Ut
            T_est = C_mean - R_est @ p_mean

            q = R_est @ p + T_est     # transform ALL points (not just inliers) for next iter
            it += 1

            improvement = prev_error - dist_mean      
            if dist_mean < threshold:
                robot_xy = (R1 @ T_est) + np.array([[prv_pose[0,0]], [prv_pose[1,0]]])
                yaw = prv_pose[2,0] + math.atan2(R_est[1,0], R_est[0,0])
                inf_mat = self.compute_inf_mat(p_valid, C_valid, R_est, T_est)
                return robot_xy[0,0],robot_xy[1,0], yaw , inf_mat              

            if improvement < min_improvement:
                stagnant_count += 1
                if stagnant_count >= stagnation_patience:
                    robot_xy = (R1 @ T_est) + np.array([[prv_pose[0,0]], [prv_pose[1,0]]])
                    yaw = prv_pose[2,0] + math.atan2(R_est[1,0], R_est[0,0])
                    inf_mat = self.compute_inf_mat(p_valid, C_valid, R_est, T_est)
                    return robot_xy[0,0],robot_xy[1,0], yaw , inf_mat  
            else:
                stagnant_count = 0  
            prev_error = dist_mean
        self.get_logger().info(f"ICP Failed. dist mean: {dist_mean} iterations: {it} ratio: {ratio}")
        return None, None, None, None

    def compute_inf_mat(self, p, C, R_est, T_est):
        residual = R_est @ p + T_est - C
        theta = math.atan2(R_est[1,0], R_est[0,0])
        H = np.zeros((3,3))
        N = len(p[0])
        sum_sq_residuals = 0.0
        for i in range(N):
            dtheta_row0 = -p[0,i] * math.sin(theta) - p[1,i] * math.cos(theta)
            dtheta_row1 =  p[0,i] * math.cos(theta) - p[1,i] * math.sin(theta)
            J = np.array([[1.0 , 0.0 , dtheta_row0], [0.0, 1.0, dtheta_row1]])
            H += J.T @ J
            sum_sq_residuals += residual[0,i]**2 + residual[1,i]**2
        dof = 2 * N - 3   # 2 residual components per point, 3 parameters estimated
        if dof <= 0:
            self.get_logger().info("Not enough correspondence")
            return np.eye(3) * 1e-3
        #sigma2 = sum_sq_residuals / dof
        sigma2 = max(sum_sq_residuals / dof, 1e-6)
        w = 1/sigma2
        #H = H * w + np.eye(3) * 1e-9
        H = H * w 
        eigvals, eigvecs = np.linalg.eigh(H)
        min_eig = 1e-3
        eigvals = np.clip(eigvals, min_eig, None)
        H = eigvecs @ np.diag(eigvals) @ eigvecs.T
        return H

    def get_map_pose(self, time):
        if not self.tf_buffer.can_transform('map', 'base_link', time, timeout=Duration(seconds=0.05)):
            #self.get_logger().warn('map->base_link not available for this stamp yet')
            return None, None, None
        try:
            map_to_base = self.tf_buffer.lookup_transform(
                'map', 'base_link', time, timeout=Duration(seconds=0.05)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get map->base_link: {e}')
            return None, None, None

        quat = map_to_base.transform.rotation
        q_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, theta) = euler_from_quaternion(q_list)
        return map_to_base.transform.translation.x, map_to_base.transform.translation.y, theta
        
    def update_grid_once(self, robot_x, robot_y, yaw, scan_cart, time):
        x1 = robot_x + self.lidar_offx * math.cos(yaw) - self.lidar_offy * math.sin(yaw)
        y1 = robot_y + self.lidar_offx * math.sin(yaw) + self.lidar_offy * math.cos(yaw)
        for i in range(len(scan_cart[0])):
            lx = scan_cart[0,i]
            ly = scan_cart[1,i]
            x2 = x1 + lx * math.cos(yaw) - ly * math.sin(yaw)
            y2 = y1 + lx * math.sin(yaw) + ly * math.cos(yaw)
            self.bres_line_update(x1,y1,x2,y2)

        for i in range(self.width*self.height):
            self.grid.data[i] = self.logodds_to_prob(self.log_odds[i])
        self.grid.header.stamp = time
        #self.map_publisher.publish(self.grid)
        #self.get_logger().info(f"Map updated with robot pose ({robot_x:.2f}, {robot_y:.2f}, {yaw:.2f}) at time {time.sec}.{time.nanosec} ")

    def remap(self, time):
        t0 = self.get_clock().now()
        self.get_logger().info(f"Remapping. Start Time: {self.get_clock().now().to_msg()}")
        self.log_odds = [0.0] * (self.width * self.height)
        for node in self.graph.pose_nodes:
            pose = node.get_pose()
            id = node.get_id()
            scan = self.graph.scans[id]
            self.update_grid_once(pose[0,0], pose[1,0], pose[2,0], scan, time)
          
        self.map_publisher.publish(self.grid)
        dt = (self.get_clock().now() - t0).nanoseconds
        self.get_logger().info(f"Remap took {self.get_clock().now().to_msg()} over {self.graph.node_count} nodes")

    def mapper(self):
        cur_scan = self.scan
        time = cur_scan.header.stamp
        cur_scan_polar = np.zeros((2,360), dtype=np.float32)
        cur_scan_polar[0] = cur_scan.ranges
        cur_scan_cart = np.zeros((2,360))
        cur_pose = np.zeros((3,1))

        pose_result = self.get_map_pose(time)
        if pose_result[0] is None:
            return
        cur_pose[0,0], cur_pose[1,0], cur_pose[2,0] = pose_result
        #self.get_logger().info(f"cur pose from map to baselink: {cur_pose.flatten()}")
        """
        for i in range(len(cur_scan_polar[0])):
            if cur_scan_polar[0,i] > 10.0 or math.isnan(cur_scan_polar[0,i]) or math.isinf(cur_scan_polar[0,i]):
                cur_scan_polar[0,i] = cur_scan_polar[0,i-1] 
            if cur_scan_polar[0,i] < 0.02:
                cur_scan_polar[0,i] = cur_scan_polar[0,i-1]
            cur_scan_polar[1,i] = cur_scan.angle_min + i*cur_scan.angle_increment
            
            cur_scan_cart[0,i] = cur_scan_cart[0,i-1] if (cur_scan_polar[0,i] > 10.0 or math.isnan(cur_scan_polar[0,i]) or math.isinf(cur_scan_polar[0,i]) or cur_scan_polar[0,i] < 0.02) else cur_scan_polar[0,i]*math.cos(cur_scan_polar[1,i])
            cur_scan_cart[1,i] = cur_scan_cart[1,i-1] if (cur_scan_polar[0,i] > 10.0 or math.isnan(cur_scan_polar[0,i]) or math.isinf(cur_scan_polar[0,i]) or cur_scan_polar[0,i] < 0.02) else cur_scan_polar[0,i]*math.sin(cur_scan_polar[1,i]) 
        """
        valid_ranges = []
        valid_angles = []
        valid_x = []
        valid_y = []

        for i in range(len(cur_scan_polar[0])):
            r = cur_scan_polar[0, i]
            angle = cur_scan.angle_min + i * cur_scan.angle_increment

            # skip bad readings entirely
            if r > 10.0 or math.isnan(r) or math.isinf(r) or r < 0.02:
                continue

            valid_ranges.append(r)
            valid_angles.append(angle)
            valid_x.append(r * math.cos(angle))
            valid_y.append(r * math.sin(angle))

        cur_scan_polar = np.array([valid_ranges, valid_angles])
        cur_scan_cart = np.array([valid_x, valid_y])        
        if self.is_first_scan:
            self.graph.add_pose(PoseNode(0, cur_pose), cur_scan_cart)
            self.update_grid_once(cur_pose[0,0], cur_pose[1,0], cur_pose[2,0], cur_scan_cart, time)
            self.map_publisher.publish(self.grid)
            self.is_first_scan = False
            self.get_logger().info(f"First map published.")
            return
        total_nodes = self.graph.node_count
        last_keyframe_pose = self.graph.get_recent_pose().pose
        last_keyframe_scan = self.graph.get_recent_scan()
        #self.get_logger().info(f"total nodes: {total_nodes}")
        #self.get_logger().info(f"last_keyframe_pose: {last_keyframe_pose.flatten()}")
        #self.get_logger().info(f"cur pose before icp: {cur_pose.flatten()}")

        
        #self.get_logger().info(f"cur pose after icp: {cur_pose.flatten()}")
        
        pose_error = abs((last_keyframe_pose[0,0] - cur_pose[0,0])**2 + (last_keyframe_pose[1,0] - cur_pose[1,0])**2)
        theta_error = abs(last_keyframe_pose[2,0] - cur_pose[2,0])
        #self.get_logger().info(f"Pose error: {pose_error} Theta error: {theta_error}")
        if (pose_error < self.keyframe_min_dist**2 and theta_error < self.keyframe_min_angle):
            #self.get_logger().info(f"Not adding keyframe")
            return
        #self.get_logger().info(f"last_keyframe_pose: {last_keyframe_pose.flatten()}")
        #self.get_logger().info(f"cur pose before icp: {cur_pose.flatten()}")
        pose_before_icp = cur_pose.copy()
        m = self.robot_truth_msg
        #self.get_logger().info(f"scan time: {time} truth time: {m.transforms[1].header.stamp}")
        truth_x = m.transforms[1].transform.translation.x
        truth_y = m.transforms[1].transform.translation.y

        q = m.transforms[1].transform.rotation
        q_list = [q.x, q.y, q.z, q.w]
        (_, _, y) = euler_from_quaternion(q_list)
        #self.get_logger().info(f"truth pose: {truth_x},{truth_y}, {y}")       
        a , b, c, inf_mat = self.icp(last_keyframe_scan.copy(), cur_scan_cart.copy(), last_keyframe_pose.copy(), cur_pose.copy())
        if a == None:
            return

        cur_pose[0,0] = a
        cur_pose[1,0] = b
        cur_pose[2,0] = c

    
        #self.get_logger().info(f"cur pose after icp: {cur_pose.flatten()}")
        pose_after_icp = cur_pose.copy()
        #self.get_logger().info(f"mod = {self.graph.node_count % 8}")
        """
        if self.graph.node_count % 4 == 0:
            self.get_logger().info(f"map to odom correction")
            self.publish_map_to_odom_tf(pose_before_icp, pose_after_icp, time)  
        """

        #self.get_logger().info(f"Adding new keyframe at pose ({cur_pose[0,0]}, {cur_pose[1,0]}, {cur_pose[2,0]})")
        self.graph.add_pose(PoseNode(self.graph.node_count, cur_pose), cur_scan_cart)
        edge = Edge()
        edge.update_edge(self.graph.node_count - 2, self.graph.node_count - 1, last_keyframe_pose, cur_pose, inf_mat)
        self.graph.add_edge(edge, "sequential")
        self.update_grid_once(cur_pose[0,0], cur_pose[1,0], cur_pose[2,0], cur_scan_cart, time)
        self.map_publisher.publish(self.grid)
        
        if self.graph.node_count > self.loop_closure_min_nodes:
            if self.icp_cooloff != self.icp_cooloff_max:
                self.icp_cooloff += 1
                return
            #self.get_logger().info(f"Checking for loop closure with {self.graph.node_count - self.loop_closure_min_nodes} previous nodes.")
            dist_min = 1000.0
            candidate_node_id = -1
            candidate_pose = np.zeros((3,1))
            for i in range(self.graph.node_count - self.loop_closure_min_nodes):
                ref_pose = self.graph.pose_nodes[i].pose
                dist = math.sqrt((ref_pose[0,0] - cur_pose[0,0])**2 + (ref_pose[1,0] - cur_pose[1,0])**2 )
                if dist < dist_min:
                    dist_min = dist
                    candidate_node_id = i 
                    candidate_pose = ref_pose
            if dist_min < self.loop_closure_min_dist:
                new_edge = Edge()
                a, b, c, cand_inf_mat = self.icp(self.graph.scans[candidate_node_id].copy(), cur_scan_cart.copy(), candidate_pose.copy(), cur_pose.copy())
                if a == None:
                    return
                self.get_logger().info(f"\n\n\nLoop closure detected between node {candidate_node_id} and node {self.graph.node_count - 1}\n\n\n")
                self.icp_cooloff = 0
                cur_pose[0,0] = a
                cur_pose[1,0] = b
                cur_pose[2,0] = c

                new_edge.update_edge(candidate_node_id, self.graph.node_count - 1, candidate_pose, cur_pose, cand_inf_mat)
                self.graph.add_edge(new_edge, "loop closure")
                final_pose_odom = self.graph.pose_nodes[self.graph.node_count - 1].get_pose().copy()
                self.graph.optimise_graph()
                final_pose_map = self.graph.pose_nodes[self.graph.node_count - 1].get_pose().copy()
                self.get_logger().info(f"Initial pose: {final_pose_odom.flatten()} Final pose: {final_pose_map.flatten()}")
                remap_time = self.get_clock().now().to_msg()
                self.remap(remap_time)
                self.publish_map_to_odom_tf(final_pose_odom, final_pose_map, remap_time)  
                
    def transform_to_matrix(self, t):
        trans = t.transform.translation
        rot = t.transform.rotation
        mat = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        mat[0, 3] = trans.x
        mat[1, 3] = trans.y
        mat[2, 3] = trans.z
        return mat
    
    def matrix_to_transform_stamped(self, mat, parent, child, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child

        t.transform.translation.x = float(mat[0, 3])
        t.transform.translation.y = float(mat[1, 3])
        t.transform.translation.z = float(mat[2, 3])

        quat = tft.quaternion_from_matrix(mat)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def publish_map_to_odom_tf(self, final_pose_odom, final_pose_map, time):
        """
        T_odom_base_true = tft.inverse_matrix(mat_map_to_odom_old) @ mat_P_before
        T_map_odom_new = mat_P_after @ tft.inverse_matrix(T_odom_base_true)
        """
        q = self.map_to_odom.transform.rotation
        t = self.map_to_odom.transform.translation
        mat_map_to_odom_old = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        mat_map_to_odom_old[0, 3] = t.x
        mat_map_to_odom_old[1, 3] = t.y
        mat_map_to_odom_old[2, 3] = 0.0
        
        q = tft.quaternion_from_euler(0.0, 0.0, final_pose_map[2,0])
        mat_P_after = tft.quaternion_matrix([q[0], q[1], q[2], q[3]])
        mat_P_after[0, 3] = final_pose_map[0,0]
        mat_P_after[1, 3] = final_pose_map[1,0]
        mat_P_after[2, 3] = 0.0

        q = tft.quaternion_from_euler(0.0, 0.0, final_pose_odom[2,0])
        mat_P_before = tft.quaternion_matrix([q[0], q[1], q[2], q[3]])
        mat_P_before[0, 3] = final_pose_odom[0,0]
        mat_P_before[1, 3] = final_pose_odom[1,0]
        mat_P_before[2, 3] = 0.0

        T_odom_base_true = tft.inverse_matrix(mat_map_to_odom_old) @ mat_P_before
        T_map_odom_new = mat_P_after @ tft.inverse_matrix(T_odom_base_true)

        self.map_to_odom = self.matrix_to_transform_stamped(
            T_map_odom_new, 'map', 'odom', time
        )

        self.tf_brodcaster.sendTransform(self.map_to_odom)
        self.get_logger().info(f"published transform")
        return


    def bres_line_update(self,x1,y1,x2,y2):
        cell_x1 , cell_y1 = self.xy_to_cell_xy(x1,y1)
        cell_x2 , cell_y2 = self.xy_to_cell_xy(x2,y2)
        del_x = abs(cell_x2 - cell_x1)
        del_y = abs(cell_y2 - cell_y1)
        sx = 1 if cell_x1 < cell_x2 else -1
        sy = 1 if cell_y1 < cell_y2 else -1
        e1 = del_x - del_y
        
        while(True):
            #self.get_logger().info(f"cellx1 = {cell_x1}\ncelly1 = {cell_y1}\ncellx2 = {cell_x2}\ncelly2 = {cell_y2}\n")
            if not (0 <= cell_x1 < self.width and 0 <= cell_y1 < self.height):
                #self.get_logger().warn("condition failed: cell_x1 or cell_y1 < 0")
                break
            if cell_x1 == cell_x2 and cell_y1 == cell_y2:
                i = int(cell_x2 + cell_y2*self.width)
                self.log_odds[i] += self.L_occ
                if self.log_odds[i] > self.L_max:
                    self.log_odds[i] = self.L_max
                #self.get_logger().info(f"line completed breaking")
                break

            else:
                i = int(cell_x1 + cell_y1*self.width)
                self.log_odds[i] += self.L_free
                if self.log_odds[i] < self.L_min:
                    self.log_odds[i] = self.L_min

            e2 = 2 * e1
            if e2 > -del_y:
                e1 -= del_y
                cell_x1 += sx
            if e2 < del_x:
                e1 += del_x
                cell_y1 += sy

def main(args = None):
    rclpy.init(args=args)
    self_mapper = SelfMapper()
    rclpy.spin(self_mapper)
    self_mapper.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

