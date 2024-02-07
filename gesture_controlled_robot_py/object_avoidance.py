import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

"""
This node sends a signal(publishes to a topic) when the robot is close enough
    to an obstacle using a sensor.(could be useful to prevent damage or sth)
"""


# range its allowed to get close to the obstacle.
SAFE_SPACE = 0.5


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        self.__publisher = self.create_publisher(
            Bool, '/movement/obstacle', 10)
        self.bridge = CvBridge()

        self.create_subscription(
            Image, '/camera_sensor/depth/image_raw', self.__depth_sensor_callback, 1)

    def __depth_sensor_callback(self, message):
        # automatically brake (DRAMATICALLY) before hitting an object.
        depth_image = self.bridge.imgmsg_to_cv2(
            message, desired_encoding='passthrough')
        closest_object_distance = depth_image.min()

        msg = Bool()
        if closest_object_distance < SAFE_SPACE:
            msg.data = True
        else:
            msg.data = False
        self.__publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    avoider = ObstacleAvoider()
    rclpy.spin(avoider)
    # Destroy the node explicitly
    avoider.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
