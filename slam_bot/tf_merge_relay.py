import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFMergeRelay(Node):
    def __init__(self):
        super().__init__('tf_merge_relay')
        self.declare_parameter('input_topics', ['/tf'])
        self.declare_parameter('output_topic', '/tf_merged')
        inputs = self.get_parameter('input_topics').value
        output = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(TFMessage, output, 50)
        for topic in inputs:
            self.create_subscription(TFMessage, topic, self.cb, 50)

    def cb(self, msg):
        self.pub.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    tf_merge = TFMergeRelay()
    rclpy.spin(tf_merge)
    tf_merge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()