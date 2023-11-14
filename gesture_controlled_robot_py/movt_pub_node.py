import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MovtPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0      # counter for the callback

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello world {self.i}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    movt_pub = MovtPublisher()
    rclpy.spin(movt_pub)


    movt_pub.destroy_node()
    rclpy.shutdown


if __name__ == '__main__':
    main()

