#! /usr/bin/env python

"""
Auteur: Jurre Fikkers
Studentnumber: 2196902
Description: Control unit for the end effector mounted on the manipulator
""" 

from pyniryo import *
import rospy

class GripperController():

	def __init__(self):
		self.robot = NiryoRobot("10.10.10.10")
		self.robot.update_tool()
	
	def open(self):
		self.robot.release_with_tool()

	def close(self):
		self.robot.grasp_with_tool()
		
	def controller_shutdown(self):
		self.robot.close_connection()
		
if __name__ == '__main__':
	rospy.init_node('Gripper_Control', anonymous=True)
	gripper = Gripper_One_Controller()
	while not rospy.is_shutdown():
        	state = input("State: ")
		if state == 'open':
			gripper.open()
		elif state == 'close':
			gripper.close()
		else:
			print('ERROR: Command not found')
	gripper.controller_shutdown()