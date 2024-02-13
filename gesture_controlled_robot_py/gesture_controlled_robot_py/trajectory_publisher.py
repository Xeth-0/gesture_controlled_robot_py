import rclpy



from three_joint_angles.msg import ThreeJointAngles

from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

"""
This node controls the robot arm in tangent with the trajectory_subscriber node.
    It works by inputting the 3 required angles for the robot joints.
"""


# Define a class named TrajectoryPublisher that inherits from the Node class
class TrajectoryPublisher(Node):

    # Constructor method
    def __init__(self):
        # Call the constructor of the parent class (Node) using super()
        super().__init__('robot_joint_trajectory_publisher')
        
        # Set the timer period to 1 second
        timer_period = 1

        # Create a timer and associate it with the timer_callback method
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create a publisher for the JointTrajectory message on the specified topic
        self.publisher = self.create_publisher(ThreeJointAngles,"/movement/cmd_joint_angles", 10)


    # Callback method for the timer
    def timer_callback(self):
        while True:
            input_msg = input("Enter the three joint angles space separated: \n")

            [j1, j2, j3] = input_msg.split()
            three_joint = ThreeJointAngles()
            three_joint.j1 = float(j1)
            three_joint.j2 = float(j2)
            three_joint.j3 = float(j3)

            self.publisher.publish(three_joint)



# Main function
def main(args=None):

    # Initialize the ROS Client Library (rclpy)
    rclpy.init(args=args)

    # Create an instance of the TrajectoryPublisher class
    joint_trajectory_object = TrajectoryPublisher()

    # Spin (keep the node running) until it is shutdown
    rclpy.spin(joint_trajectory_object)
    
    # Destroy the node
    joint_trajectory_object.destroy_node()

    # Shutdown the ROS Client Library
    rclpy.shutdown()



if __name__ == '__main__':
    main()
