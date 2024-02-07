# Robot Design
The robot design is split among many different .xacro files that represent the components of the robot.

<b>arm2.xacro</b>: The entire robot arm on top of the wheeled base of the robot<br>
<b>mobile_base.xacro</b>: The wheeled portion of the robot along with the main link<br>
<b>depth_camera.xacro</b>: The depth camera at the front of the mobile_base used for obstacle avoidance<br>
<b>common_properties.xacro</b>: Holds some common properties like colors used across files to avoid duplication<br>
<b>*_gazebo.xacro</b>: Specific properties used with the parts notified by the prefix of the file name. (ARM_gazebo.xacro would be for the arm)<br>
<b>my_robot.urdf.xacro</b>: The final robot, where we combine the different components into one.<br>