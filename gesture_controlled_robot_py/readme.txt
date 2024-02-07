Gesture Recognition from https://github.com/kinivi/hand-gesture-recognition-mediapipe.


DONT FORGET TO TURN ON THE BRIDGE, ign wont hear the ros topics otherwise



ros2 run gesture_controlled_robot_py talker
ros2 run gesture_controlled_robot_py talker
ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist



# NEW COMMANDS
Run the launcher file. this will run the gesture recognition and the trajectory listeners.
>>>ros2 launch gesture_controlled_robot_py gazebo.launch.py 

The robot can be controlled by keyboard via teleop or with gestures. gestures launches automatically,
keyboard needs a terminal
>>>ros2 run teleop_twist_keyboard teleop_twist_keyboard


Arms can be moved by giving the desired 3 angles of the robot
>>>ros2 run gesture_controlled_robot_py trajectory_publisher 


Camera Doesnt have much use rn.