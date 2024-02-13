import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from three_joint_angles.msg import ThreeJointAngles

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from builtin_interfaces.msg import Duration


"""
This node controls the robot arm in tangent with the trajectory_publisher node.
    It works by inputting the 3 required angles for the robot joints.
"""

JOINTS = ['joint_1', 'joint_2', 'joint_3']


class TrajectorySubscriber(Node):

    def __init__(self):
        super().__init__('trajectory_subscriber')
        self.get_logger().info(
            "Listening to String messages of (j1, j2, j3) on topic:\n/movement/cmd_joint_angles")
        self.subscription = self.create_subscription(
            ThreeJointAngles,
            '/movement/cmd_joint_angles',
            self.publisher_callback,
            10)
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory", 10)

        self.subscription  # prevent unused variable warning
        self.publisher

    def publisher_callback(self, msg):
        # should be 3 angles
        self.get_logger().info(f'I heard: {msg.j1, msg.j2, msg.j3}')
        j1 = np.radians(msg.j1)
        j2 = np.radians(msg.j2)
        j3 = np.radians(msg.j3)

        goal_positions = [j1, j2, j3]

        point_msg = JointTrajectoryPoint()
        point_msg.positions = goal_positions
        point_msg.time_from_start = Duration(sec=2)

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = JOINTS
        trajectory_msg.points.append(point_msg)

        self.publisher.publish(trajectory_msg)


def main(args=None):
    rclpy.init(args=args)

    trajectory_subscriber = TrajectorySubscriber()

    rclpy.spin(trajectory_subscriber)

    # Destroy the node explicitly
    trajectory_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
