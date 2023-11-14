import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from movt_mapping import MAPPING

class MovtSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            String, '/movement/gesture', self.listener_callback, 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {MAPPING[msg.data]['linear']}")

def main(args=None):
    rclpy.init(args=args)
    movt_sub = MovtSubscriber()

    rclpy.spin(movt_sub)


    movt_sub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()