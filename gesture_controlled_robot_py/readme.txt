Gesture Recognition from https://github.com/kinivi/hand-gesture-recognition-mediapipe.


DONT FORGET TO TURN ON THE BRIDGE, ign wont hear the ros topics otherwise



ros2 run gesture_controlled_robot_py talker
ros2 run gesture_controlled_robot_py talker
ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist