import os
from gesture_controlled_robot_py.hand_gesture_recognition_mediapipe.app import main as gesture_detector
import threading
from gesture import gesture_reciever


bg_thread = threading.Thread(target=gesture_detector, args=[gesture_reciever])
# runs the camera and the detection package in the background

bg_thread.start()

