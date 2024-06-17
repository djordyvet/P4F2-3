#!/usr/bin/env python

"""
Author: Jurre Fikkers
Student number: 2196902
Description: Control unit for the manipulator with build in test functions to preform movements 
             based on xyz input or a positionvariable
""" 

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import json
from pyniryo import NiryoRobot
from std_msgs.msg import Float32MultiArray

class NiryoNedController:
    def __init__(self, robot_ip, positions_file):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('niryo_ned_controller', anonymous=True)
        
        # Initialize robot commander and planning scene interface
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize move group commander for the arm
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_group.set_planning_time(10)
        
        # Publisher for displaying the planned path
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', 
            moveit_msgs.msg.DisplayTrajectory, 
            queue_size=1
        )

        # Publisher for the end-effector's XYZ coordinates
        self.xyz_publisher = rospy.Publisher('/end_effector_position', Float32MultiArray, queue_size=10)
        
        # Connect to the physical robot
        self.pyrobot = NiryoRobot(robot_ip)

        # Load predefined positions from the JSON file
        self.positions = self.load_positions(positions_file)

    def load_positions(self, positions_file):
        """
        Load predefined positions from a JSON file.
        """
        with open(positions_file, 'r') as f:
            return json.load(f)

    def go_to_named_position(self, position_name):
        """
        Move the robot to a named position defined in the JSON file.
        """
        if position_name in self.positions:
            joint_goal = self.positions[position_name]
            self.arm_group.go(joint_goal, wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            self.output_xyz_position()
        else:
            rospy.logwarn("Position '{}' not found in the positions file.".format(position_name))

    def go_to_xyz(self, x, y, z):
        """
        Move the robot to specific XYZ coordinates.
        """
        rospy.loginfo(f"Moving to XYZ coordinates: {x}, {y}, {z}")
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        rotation = self.pyrobot.get_pose_quat()
        pose_goal.orientation.x = rotation[0]
        pose_goal.orientation.y = rotation[1]
        pose_goal.orientation.z = rotation[2]
        pose_goal.orientation.w = rotation[3]

        self.arm_group.set_pose_target(pose_goal)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        self.output_xyz_position()

    def ouptus_xyz_position(self):
        """
        Publish the current XYZ coordinates of the end-effector.
        """
        current_pose = self.arm_group.get_current_pose().pose
        position = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
        rospy.loginfo(f"Current position: {position}")
        self.xyz_publisher.publish(Float32MultiArray(data=position))
    
    def output_joint_positions(self, output_file):
    """
    Output the current joint positions in the terminal and log file
    """
    joint_positions = self.arm_group.get_current_joint_values()
    with open(output_file, 'a') as f:	
        f.write("current joint position: {}\n".format(joint_positions))

    def shutdown(self):
        """
        Shutdown moveit_commander properly.
        """
        moveit_commander.roscpp_shutdown()

def get_user_choice():
    """
    Get the user's choice for the operation type.
    """
    try:
        choice = int(input("Choose operation (1: Named Position, 2: XYZ Coordinates): "))
        if choice in [1, 2]:
            return choice
        else:
            print("Invalid choice. Please enter 1 or 2.")
            return get_user_choice()
    except ValueError:
        print("Invalid input. Please enter a number.")
        return get_user_choice()

def main():
    robot_ip = "10.10.10.10"  
    positions_file = "standard_positions.json"
    controller = NiryoNedController(robot_ip, positions_file)
    
    try:
        while not rospy.is_shutdown():
            user_choice = get_user_choice()
            
            if user_choice == 1:
                position_name = input("Enter named position: ")
                controller.go_to_named_position(position_name)
            elif user_choice == 2:
                x = float(input("Enter X coordinate: "))
                y = float(input("Enter Y coordinate: "))
                z = float(input("Enter Z coordinate: "))
                controller.go_to_xyz(x, y, z)
    except rospy.ROSInterruptException:
        pass
    finally:
        controller.shutdown()

if __name__ == '__main__':
    main()