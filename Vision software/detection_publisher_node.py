#!/usr/bin/env python

import rospy
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import PoseStamped

# Define the offsets to be added to the coordinates
OFFSET_X = 1.0  # Replace with your desired offset for x
OFFSET_Y = 1.0  # Replace with your desired offset for y
OFFSET_Z = 1.0  # Replace with your desired offset for z

def detection_callback(msg):
    for detection in msg.detections:
        pose_msg = PoseStamped()
        pose_msg.header = msg.header

        # Add offsets to the coordinates
        pose_msg.pose.position.x = detection.position.x + OFFSET_X
        pose_msg.pose.position.y = detection.position.y + OFFSET_Y
        pose_msg.pose.position.z = detection.position.z + OFFSET_Z

        # Assuming the detection object has an orientation field
        pose_msg.pose.orientation = detection.orientation

        # Printing the details for debug purposes
        rospy.loginfo("Detection ID: %d", detection.label)
        rospy.loginfo("Original Position: x=%.2f, y=%.2f, z=%.2f", 
                      detection.position.x, detection.position.y, detection.position.z)
        rospy.loginfo("Updated Position: x=%.2f, y=%.2f, z=%.2f", 
                      pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z)
        rospy.loginfo("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, 
                      pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)

        pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('detection_processor_node')
    pub = rospy.Publisher('detection_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('color/detections', SpatialDetectionArray, detection_callback)
    rospy.spin()
