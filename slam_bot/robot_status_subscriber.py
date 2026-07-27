import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotStatusSubscriber(Node):
    def __init__(self):
        super().__init__('robot_status_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/robot_status',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i=0

    def listener_callback(self, msg):
        self.i+=1
        self.get_logger().info(f"Received: {msg.data}, no of messages received: {self.i}")

def main(args=None):
    rclpy.init(args=args)
    robot_status_subscriber = RobotStatusSubscriber()
    rclpy.spin(robot_status_subscriber)
    robot_status_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()