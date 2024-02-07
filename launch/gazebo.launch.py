from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import os

def generate_launch_description():
    ld = []

    package_name = 'gesture_controlled_robot_py'

    urdf_path = os.path.join(get_package_share_path(package_name),
                             'models', 'my_robot.urdf.urdf')
    rviz_config_path = os.path.join(get_package_share_path(package_name),
                                    'rviz', 'urdf_config.rviz')
    
    # robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    

    # for rviz2, to visualize the depth camera
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # kills any currently running gazebo servers
    kill_gazebo = ExecuteProcess(
        cmd=['sudo pkill gzserver && sleep(100)']
    )

    # launches gazebo. the original way was crashing on our pc's
    gazebo_launch = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', PathJoinSubstitution([FindPackageShare(package_name), 'worlds', 'my_world.world'])],
        output='screen'
    )
    
    # spawns the robot
    simple_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-x', '0.0', '-y', '0.0', '-z', '0.2',
                   '-file', urdf_path])
  

    # node that runs the camera for the gesture input
    gesture_node = ExecuteProcess(
        cmd=['ros2', 'run', 'gesture_controlled_robot_py', 'talker'],
        output='screen'
    )

    # listens to the command from the gesture node above and outputs mov't commands
    gesture_listener = Node(
        package=package_name,
        executable="listener",
        name='gesture_subscriber_node',
        output='screen',
        arguments=[]
    )

    # detects whether there's an obstacle immediately infront of the robot, publishes T/F.
    object_avoidance = Node(
        package=package_name,
        executable="object_avoidance",
        name='object_avoider_node',
        output='screen',
        arguments=[]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[PathJoinSubstitution([FindPackageShare(package_name), urdf_path])])

    # add robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[PathJoinSubstitution([FindPackageShare(package_name), urdf_path])])

    # Subscriber Node for the robot arm joint angles
    robot_joint_state_subscriber = Node(
        package="gesture_controlled_robot_py",
        name='subscriber',
        executable="trajectory_subscriber",
    )

    # robot_joint_state_publisher = Node(
    #     package="gesture_controlled_robot_py",
    #     name='publisher',
    #     executable="trajectory_publisher",
    # )

    # controllers for the robot arms
    cmjsb = Node(
        package="controller_manager",
        executable="spawner",
        name="cm_jsb",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],

    )
    cmjtc = Node(
        package="controller_manager",
        executable="spawner",
        name="cm_jtc",
        arguments=["joint_trajectory_controller",
                   "-c", "/controller_manager"]
    )


    ld.append(robot_joint_state_subscriber)
    ld.append(kill_gazebo)
    # ld.append(rsp)
    ld.append(gazebo_launch)

    ld.append(gesture_node)

    ld.append(simple_robot)
    ld.append(gesture_listener)

    ld.append(object_avoidance)

    ld.append(joint_state_publisher)
    ld.append(robot_state_publisher)

    ld.append(cmjsb)
    ld.append(cmjtc)


    return LaunchDescription(ld)



