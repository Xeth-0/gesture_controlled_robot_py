import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32
import teleop_twist_keyboard
from geometry_msgs.msg import Twist
# from gesture import gesture_reciever
from gesture_controlled_robot_py.hand_gesture_recognition_mediapipe.app import main as gesture_detector

import threading

class MovtPublisher(Node):
    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(Int32, '/movement/gesture', 10)
        bg_thread = threading.Thread(target=gesture_detector, args=[self.gesture_reciever])
        bg_thread.run()
        # timer_period = 1
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0      # counter for the callback

    def gesture_reciever(self, gesture):
        print(gesture)
        msg = Int32()
        
        # msg.data = f'{gesture}'

        if gesture == "Go":
            msg.data = 8
            # msg.data = 'linear: {x: -0.5}, angular: {z: 0.0}'
            # msg.linear.x = -0.5
            # msg.angular.z = 0.0
        
        self.publisher_.publish(msg)

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

