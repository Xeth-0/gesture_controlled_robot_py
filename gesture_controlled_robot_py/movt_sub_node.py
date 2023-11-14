import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg._twist import Twist

from gesture_controlled_robot_py.movt_mapping import MAPPING


class MovtSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            String, '/movement/gesture', self.listener_callback, 10)
        self.publiser_ = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"I heard: {MAPPING[msg.data]['linear']}")
        return_msg = Twist()
        return_msg.linear.x, return_msg.linear.y, return_msg.linear.z = (
            MAPPING[msg.data]['linear'])
        return_msg.angular.z = MAPPING[msg.data]['angular']
        self.publiser_.publish(return_msg)


def main(args=None):
    rclpy.init(args=args)
    movt_sub = MovtSubscriber()

    rclpy.spin(movt_sub)

    movt_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
