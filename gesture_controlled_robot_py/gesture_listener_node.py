import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg._twist import Twist

from gesture_controlled_robot_py.movt_mapping import MAPPING

"""
This node runs the device(laptop) camera for gesture inputs for control of the wheeled
    robot. It then recieves, translates and then publishes the final command for the 
    movt_pub_node for the final command to the robot.
"""

class GestureSubscriber(Node):
    def __init__(self):
        super().__init__('movement_subscriber')
        self.subscription = self.create_subscription(
            String, '/movement/gesture', self.gesture_callback, 10)
        self.obstacle_avoidance_subscription = self.create_subscription(Bool, '/movement/obstacle', self.obstacle_callback, 10)
        self.publiser_ = self.create_publisher(
            Twist, '/cmd_vel', 10)
        self.subscription
        self.obstacle_detected = False

    def gesture_callback(self, msg):
        self.get_logger().info(f"I heard: {msg.data}")
        self.get_logger().info(f"I heard: {MAPPING[msg.data]['linear']}")
        return_msg = Twist()

        # if an obstacle is detected, do not allow forward motion to keep the robot safe
        if self.obstacle_detected:
            return_msg.linear.x, return_msg.linear.y, return_msg.linear.z = (
            MAPPING['Stop']['linear'])
            
        else:
            return_msg.linear.x, return_msg.linear.y, return_msg.linear.z = (
                MAPPING[msg.data]['linear'])
            
        return_msg.angular.z = MAPPING[msg.data]['angular']
        self.publiser_.publish(return_msg)
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data

        if msg.data:
            self.get_logger().info(f"Obstacle Detected: {msg.data}")
            self.get_logger().info(f"Keep holding left/right until cleared to continue moving.")
            return_msg = Twist()
            return_msg.linear.x, return_msg.linear.y, return_msg.linear.z = (
                MAPPING['Stop']['linear'])
            self.publiser_.publish(return_msg)

        


def main(args=None):
    rclpy.init(args=args)
    movt_sub = GestureSubscriber()

    rclpy.spin(movt_sub)

    movt_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
