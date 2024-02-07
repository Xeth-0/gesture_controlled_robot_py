import rclpy
from rclpy.node import Node

from gesture_controlled_robot_py.hand_gesture_recognition_mediapipe.app import main as gesture_detector
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist

# import threading

"""
This node controls the robot movement.
    Was made separate to add other forms of mov't control to the robot.
"""

class MovtPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(String, '/movement/gesture', 10)
        gesture_detection = gesture_detector(self.gesture_reciever)

        # bg_thread = threading.Thread(target=gesture_detector, args=[self.gesture_reciever])
        # bg_thread.run()

    def gesture_reciever(self, gesture):
        # can publish direct to the robot, but separating this to add other forms of controls as well
        msg = String()
        msg.data = gesture

        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    movt_pub = MovtPublisher()
    rclpy.spin(movt_pub)
    movt_pub.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()

