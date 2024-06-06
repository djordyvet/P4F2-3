#!/usr/bin/env python

import rospy
import sys
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander
from geometry_msgs.msg import Pose
from sensor_msgs.msg  import JointState

def compute_cp(cartesian_point):
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('compute_cp_node', anonymous=True)

	robot = RobotCommander()
	scene = PlanningSceneInterface()
	group_name = "my_niryo_robot"
	move_group = MoveGroupCommander(my_niryo_robot)

	pose_target = Pose()
	pose_target.orientation.w = 1.0
	pose_target.orientation.x = cartesian_point[0]
	pose_target.orientation.y = cartesian_point[1]
	pose_target.orientation.z = cartesian_point[2]

	move_group.set_pose_target(pose_target)
	plan = move_group.go(wait=True)
	move_group.stop()
	move_group.clear_pose_targets()

	current_joint_values = move_group.get_current_joint_values()
	return current_joint_values

def publish_joint_states(joint_angles):
	pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
	joint_state_msg = JointState()
	joint_state_msg.header.stamp = rospy.Time.now()
	joint_state_msg.name = ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")
	joint_state_msg.position = joint_angles
	pub.publish(joint_state_msg)

if __name__ == '__main__':
	try:
		cartesian_point = [0.4, 0.1, 0.4]
		joint_angles = compute_cp(cartesian_point)
		print("computed joint angles: ", joint_angles)
		publish_joint_states(joint_angles)
	except rospy.ROSInterruptException
		pass 

