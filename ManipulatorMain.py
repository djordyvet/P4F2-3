#! /usr/bin/env python

"""
Author: Jurre Fikkers
Student number: 2196902
Description: Main program for controlling manipulator/end effector and retrieving/sorting the objects 
             found by the vision camera
""" 

import rospy
import json
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Int8, String

# Global Variables
start_robot = False
sorting_bin = 0
number_of_bins = 4
error_message = ""
object_list = ["", "obj1", "obj2", "obj3", "obj4"]

def robot_start_callback(msg):
    global start_robot
    start_robot = msg.data

def inspection_result_callback(msg):
    global sorting_bin
    global error_message
    print("Analysing vision data")
    if msg.data == 0:
        error_message = "ERROR: No location found"
        sorting_bin = -1
    elif 0 < msg.data <= number_of_bins:
        sorting_bin = msg.data
    else:
        error_message = "ERROR: No sorting location found!"
        sorting_bin = -2

class MoveitRobotController:
    def __init__(self, positions_file):
        # Initialize the moveit_commander and rospy nodes
        roscpp_initialize(sys.argv)
        rospy.init_node('moveit_robot_controller', anonymous=True)
        
        # Initialize robot commander and planning scene interface
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        
        # Initialize move group commander for the arm
        self.arm_group = MoveGroupCommander("arm")
        self.arm_group.set_planning_time(10)
        
        # Load predefined positions from the JSON file
        self.positions = self.load_positions(positions_file)

    def load_positions(self, positions_file):
        with open(positions_file, 'r') as f:
            return json.load(f)

    def go_to_named_position(self, position_name):
        if position_name in self.positions:
            joint_goal = self.positions[position_name]
            self.arm_group.set_joint_value_target(joint_goal)
            self.arm_group.go(wait=True)
            self.arm_group.stop()
            self.arm_group.clear_pose_targets()
            self.output_joint_positions("joint_positions.log")
        else:
            rospy.logwarn("Position '{}' not found in file.".format(position_name))

    def go_to_xyz(self, x, y, z):
        rospy.loginfo("Moving to XYZ coordinates: {}, {}, {}".format(x, y, z))
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        self.arm_group.set_pose_target(pose_goal)
        self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
    
    def output_joint_positions(self, output_file):
        joint_positions = self.arm_group.get_current_joint_values()
        with open(output_file, 'a') as f:
            f.write("current joint position: {}\n".format(joint_positions))

    def shutdown(self):
        roscpp_shutdown()

class GripperController:
    def __init__(self):
        pass

    def open(self):
        print("Gripper open")

    def close(self):
        print("Gripper closed")

    def shutdown(self):
        print("Gripper shutdown")

class Main:
    def __init__(self):
        rospy.on_shutdown(self.ros_shutdown)
        self.robot_controller = MoveitRobotController("positions.json")
        self.gripper_controller = GripperController()
        self.program_start = Bool()
        self.robot_state = String()
        self.read_program_start = rospy.Subscriber('/robot_start', Bool, robot_start_callback)
        self.read_inspection_results = rospy.Subscriber('/robot_sort_pose', Int8, inspection_result_callback)
        self.write_program_start = rospy.Publisher('/robot_start', Bool, queue_size=1)
        self.write_robot_state = rospy.Publisher('/robot_status', String, queue_size=1)
        rospy.sleep(0.5)
        self.robot_state.data = "STANDBY"
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(0.5)
        self.main()

    def main(self):
        global start_robot
        while not rospy.is_shutdown():
            if start_robot:
                print("Starting program...")
                rospy.sleep(0.5)
                self.robot_state.data = "ACTIVE"
                self.write_robot_state.publish(self.robot_state)
                self.program_start.data = False
                self.write_program_start.publish(self.program_start)
                rospy.sleep(0.5)
                
                self.robot_controller.go_to_named_position('home')
                sort_location = self.inspect()
                if sort_location is not None:
                    self.pickup()
                    self.robot_controller.go_to_named_position('home')
                    self.sort(sort_location)
                    self.robot_controller.go_to_named_position('home')
                else:
                    print("No detection")
                    self.robot_controller.go_to_named_position('home')
                
                self.robot_state.data = "STANDBY"
                self.write_robot_state.publish(self.robot_state)
                print("Standby")
            rospy.sleep(1)
    
    def inspect(self):
        self.robot_controller.go_to_named_position('inspection')
        self.robot_state.data = "VISION"
        self.write_robot_state.publish(self.robot_state)
        rospy.sleep(1)
        if sorting_bin in [-2, -1]:
            print(error_message)
            self.robot_state.data = "WARN"
            self.write_robot_state.publish(self.robot_state)
        elif 0 < sorting_bin <= number_of_bins:
            print("Object detected as: " + object_list[sorting_bin])
            return sorting_bin
        else:
            print("ERROR: Unknown failure")
            self.robot_state.data = "ERROR"
            self.write_robot_state.publish(self.robot_state)
            while not rospy.is_shutdown():
                print("ERROR: Unknown failure")
                rospy.sleep(30)

    def pickup(self):
        print("Picking up object...")
        self.gripper_controller.open()
        self.robot_controller.go_to_named_position('pickup')
        self.gripper_controller.close()

    def sort(self, location):
        pose = "bin" + str(location)
        print("Moving object to: " + pose)
        self.robot_controller.go_to_named_position(pose)
        self.gripper_controller.open()
        
    def ros_shutdown(self):
        print("Shutting down")
        self.robot_controller.shutdown()
        self.gripper_controller.shutdown()
	
if __name__ == '__main__':
    rospy.init_node('robot_control')
    test_mode = True
    while test_mode:
        test = input("Test system? (Y/n): ")
        if test == 'Y':
            start_robot = True
            test_mode = False
        elif test == 'n':
            start_robot = False
            test_mode = False
        else:
            print("False argument")
    Main()
    rospy.spin()