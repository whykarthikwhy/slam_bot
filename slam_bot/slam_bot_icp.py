import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import  TransformBroadcaster, Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from tf_transformations import euler_from_quaternion
import tf_transformations as tft
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
import math
import time

class SelfMapper(Node):
    def __init__(self):
        super().__init__('self_mapper')
        self.robot_tf_subscriber = self.create_subscription(TFMessage, "/model/slam_bot/pose_static" , self.robot_tf_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.tf_brodcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map_timer = self.create_timer(0.3, self.mapper)
        self.grid = OccupancyGrid()
        self.width = 320*2
        self.height = 220*2
        self.res = 0.025
        self.L_occ  =  0.85   # log odds added when cell is hit
        self.L_free = -0.2    # log odds added when cell is passed through
        self.L_min  = -2.0
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
        self.cur_scan_cart = np.zeros((2,360))
        self.prv_scan_cart = np.zeros((2,360))
        self.prv_pose = np.zeros((3,1))
        self.cur_pose = np.zeros((3,1))
        self.is_first_scan = True
        self.robot_tf_msg = TFMessage()
        
    def logodds_to_prob(self, l):
        if l == 0.0:
            return -1
        p = 1 - (1/(1 + math.exp(l)))
        return int(p*100)
    
    def robot_tf_callback(self, msg):
        self.robot_tf_msg = msg
        
        return
    
    def lidar_callback(self, msg):
        self.scan = msg      
        return

    def xy_to_cell_xy(self, x, y):
        cell_x = int(math.floor(x/self.res)) + self.width/2 
        cell_y = int(math.floor(y/self.res)) + self.height/2 
        return cell_x, cell_y
    
    def icp(self,qf, p, prv_pose, cur_pose):

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
        threshold = 0.0001
        while True:
            dist_sq = np.zeros(len(q[0]))
            C = np.zeros((2,len(q[0])))
            for i in range(len(q[0])):
                min_dist = 1e9
                
                for j in range(len(qf[0])):
                    d_sq = ((q[0][i] - qf[0][j])**2 + (q[1][i] - qf[1][j])**2)
                    if d_sq < min_dist:
                        min_dist = d_sq
                        C[:,i] = qf[:,j]
                    dist_sq[i] = min_dist
            C_mean = np.array([[np.mean(C[0])], [np.mean(C[1])]])
            p_mean = np.array([[np.mean(p[0])], [np.mean(p[1])]])
            dist_mean = np.mean(dist_sq)
            C_centered = C - C_mean
            p_centered = p - p_mean
            H = p_centered @ C_centered.T
            u, S, vt = np.linalg.svd(H)
            Ut = u.T
            V = vt.T
            R_est = V @ Ut
            if (  np.linalg.det(R_est) < -0.9999 and np.linalg.det(R_est) > -1.0001):
                V[:, -1] *= -1.0
                R_est = V @ Ut
            T_est = C_mean - R_est @ p_mean
            q = R_est @ p + T_est

            if dist_mean < threshold:
                robot_xy = (R1 @ T_est) + np.array([[prv_pose[0,0]], [prv_pose[1,0]]])
                yaw = prv_pose[2,0] + math.atan2(R_est[1,0], R_est[0,0])
                return robot_xy[0,0],robot_xy[1,0], yaw
                
            

    def get_odom_pose(self, time):
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                time,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get odom->base_link: {e}')
            return 
        
        quat = odom_to_base.transform.rotation
        q_list = [quat.x, quat.y, quat.z, quat.w]
        (_, _, theta) = euler_from_quaternion(q_list)
        return odom_to_base.transform.translation.x, odom_to_base.transform.translation.y, theta
        

    def mapper(self):
        cur_scan = self.scan
        cur_robot_tf = self.robot_tf_msg
        time = cur_robot_tf.transforms[1].header.stamp
        cur_scan_polar = np.zeros((2,360), dtype=np.float32)
        cur_scan_polar[0] = cur_scan.ranges

        for i in range(len(cur_scan_polar[0])):
            if cur_scan_polar[0,i] > 10.0 or math.isnan(cur_scan_polar[0,i]) or math.isinf(cur_scan_polar[0,i]):
                cur_scan_polar[0,i] = 100.0
            if cur_scan_polar[0,i] < 0.02:
                cur_scan_polar[0,i] = 100.0
            cur_scan_polar[1,i] = cur_scan.angle_min + i*cur_scan.angle_increment
            self.cur_scan_cart[0,i] = cur_scan_polar[0,i]*math.cos(cur_scan_polar[1,i]) if cur_scan_polar[0,i] != 100.0 else 100.0
            self.cur_scan_cart[1,i] =  cur_scan_polar[0,i]*math.sin(cur_scan_polar[1,i]) if cur_scan_polar[0,i] != 100.0 else 100.0
        
        self.cur_pose[0,0], self.cur_pose[1,0], self.cur_pose[2,0] = self.get_odom_pose(time)

        if self.is_first_scan:
            #self.cur_pose[0,0], self.cur_pose[1,0], self.cur_pose[2,0] = self.get_odom_pose(time)
            self.prv_scan_cart = self.cur_scan_cart
            self.prv_pose = self.cur_pose
            self.is_first_scan = False
            return
        
        #self.cur_pose = self.prv_pose * 1.2

        self.cur_pose[0,0], self.cur_pose[1,0], self.cur_pose[2,0] = self.icp(self.prv_scan_cart, self.cur_scan_cart, self.prv_pose, self.cur_pose)
        
        robot_x = self.cur_pose[0,0]
        robot_y = self.cur_pose[1,0]    
        yaw = self.cur_pose[2,0]

        x1 = robot_x + self.lidar_offx * math.cos(yaw) - self.lidar_offy * math.sin(yaw)
        y1 = robot_y + self.lidar_offx * math.sin(yaw) + self.lidar_offy * math.cos(yaw)
        for i in range(len(self.cur_scan_cart[0])):
            lx = self.cur_scan_cart[0,i]
            ly = self.cur_scan_cart[1,i]
            if (lx == 100.0 or ly == 100.0):
                continue
            x2 = x1 + lx * math.cos(yaw) - ly * math.sin(yaw)
            y2 = y1 + lx * math.sin(yaw) + ly * math.cos(yaw)
            #self.get_logger().info(f"\npoint no = {i}\nrobotx = {robot_x}\nroboty = {robot_y}\nyaw = {yaw}\nx1 = {x1}\ny1 = {y1}\nx2 = {x2}  y2 = {y2}")
            self.bres_line_update(x1,y1,x2,y2)

        for i in range(self.width*self.height):
            self.grid.data[i] = self.logodds_to_prob(self.log_odds[i])
        self.grid.header.stamp = time
        self.map_publisher.publish(self.grid)
        self.publish_map_to_odom_tf(cur_robot_tf)
        self.prv_scan_cart = self.cur_scan_cart
        self.prv_pose = self.cur_pose

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

        

        
    def publish_map_to_odom_tf(self, robot_tf):
        t = TransformStamped()
        t.header.stamp = robot_tf.transforms[1].header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform = robot_tf.transforms[1].transform
        try:
            odom_to_base = self.tf_buffer.lookup_transform(
                'odom',
                'base_link',
                t.header.stamp,
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'Could not get odom->base_link: {e}')
            return
        
        T_map_base = self.transform_to_matrix(t)
        T_odom_base = self.transform_to_matrix(odom_to_base)
        T_odom_base_inv = tft.inverse_matrix(T_odom_base)
        T_map_odom = np.dot(T_map_base, T_odom_base_inv)

        map_to_odom = self.matrix_to_transform_stamped(
            T_map_odom, 'map', 'odom', t.header.stamp
        )

        self.tf_brodcaster.sendTransform(map_to_odom)
        #self.get_logger().info(f"published transform")
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

