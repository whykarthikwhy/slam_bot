import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import  TransformBroadcaster
from tf_transformations import quaternion_from_euler
import numpy as np
import math
from tf2_msgs.msg import TFMessage


class Mapper_Using_Raw_Odom(Node):
    def __init__(self):
        super().__init__('mapper_using_raw_odom')
        self.odom_subscriber = self.create_subscription(Odometry, "/odom" , self.odom_callback, 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, "/map", 10)
        self.tf_publisher = self.create_publisher(TFMessage, "/raw_odom/tf", 10)
        self.map_timer = self.create_timer(0.3, self.mapper)
        self.is_first_time = True
        self.grid = OccupancyGrid()
        self.width = 400
        self.height = 500
        self.res = 0.025
        self.L_occ  =  0.09   
        self.L_free = -0.03    
        self.L_min  = -5.0
        self.L_max  =  5.0
        self.lidar_offx = 0.05
        self.lidar_offy = 0.0
        self.mapper_count = 0
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
        self.odom_msg = Odometry()
        self.scan = LaserScan()
        
    def logodds_to_prob(self, l):
        if l == 0.0:
            return -1
        p = 1 - (1/(1 + math.exp(l)))
        return int(p*100)
    
    def odom_callback(self, msg):
        self.odom_msg = msg
        if self.is_first_time:
            self.is_first_time = False
            self.publish_map_to_odom_tf()
        return
    
    def lidar_callback(self, msg):
        self.scan = msg      
        return

    def xy_to_cell_xy(self, x, y):
        cell_x = int(x/self.res) + self.width/2 
        cell_y = int(y/self.res) + self.height/2 
        return cell_x, cell_y

    def mapper(self):
        cur_odom = self.odom_msg
        cur_scan = self.scan
        t = cur_odom.header.stamp
        
        cur_scan_polar = np.zeros((2,360), dtype=np.float32)
        cur_scan_cart = np.zeros((2,360))
        cur_scan_polar[0] = cur_scan.ranges
        for i in range(len(cur_scan_polar[0])):
            if cur_scan_polar[0,i] > 10.0 or math.isnan(cur_scan_polar[0,i]) or math.isinf(cur_scan_polar[0,i]):
                cur_scan_polar[0,i] =10.0
            if cur_scan_polar[0,i] < 0.02:
                cur_scan_polar[0,i] =0.02
            cur_scan_polar[1,i] = cur_scan.angle_min + i*cur_scan.angle_increment
            cur_scan_cart[0,i] = cur_scan_polar[0,i]*math.cos(cur_scan_polar[1,i])
            cur_scan_cart[1,i] =  cur_scan_polar[0,i]*math.sin(cur_scan_polar[1,i])

        robot_x = cur_odom.pose.pose.position.x
        robot_y = cur_odom.pose.pose.position.y

        q = cur_odom.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        #self.get_logger().info(f"yaw = {yaw}")

        x1 = robot_x + self.lidar_offx * math.cos(yaw) - self.lidar_offy * math.sin(yaw)
        y1 = robot_y + self.lidar_offx * math.sin(yaw) + self.lidar_offy * math.cos(yaw)

        for i in range(len(cur_scan_cart[0])):
            lx = cur_scan_cart[0,i]
            ly = cur_scan_cart[1,i]
            if not (-15.0 < lx < 15.0 and -15.0 < ly < 15.0):
                continue
            x2 = x1 + lx * math.cos(yaw) - ly * math.sin(yaw)
            y2 = y1 + lx * math.sin(yaw) + ly * math.cos(yaw)
            #self.get_logger().info(f"\npoint no = {i}\nrobotx = {robot_x}\nroboty = {robot_y}\nyaw = {yaw}\nx1 = {x1}\ny1 = {y1}\nx2 = {x2}  y2 = {y2}")
            self.bres_line_update(x1,y1,x2,y2)

        for i in range(self.width*self.height):
            self.grid.data[i] = self.logodds_to_prob(self.log_odds[i])
        self.grid.header.stamp = t
        self.map_publisher.publish(self.grid)
        self.publish_map_to_odom_tf()

    def publish_map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_publisher.publish(TFMessage(transforms=[t]))
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
    mapper_using_raw_odom = Mapper_Using_Raw_Odom()
    rclpy.spin(mapper_using_raw_odom)
    mapper_using_raw_odom.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

