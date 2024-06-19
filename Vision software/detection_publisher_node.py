#!/usr/bin/env python

import rospy
from depthai_ros_msgs.msg import SpatialDetectionArray
from geometry_msgs.msg import PoseStamped

def detection_callback(msg):
    for detection in msg.detections:
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose.position.x = detection.bbox.center.x
        pose_msg.pose.position.y = detection.bbox.center.y
        pose_msg.pose.position.z = detection.bbox.center.z

        # Assuming the detection object has an orientation field
        pose_msg.pose.orientation = detection.orientation

        # Printing the details for debug purposes
        rospy.loginfo("Detection ID: %d", detection.label)
        rospy.loginfo("Position: x=%.2f, y=%.2f, z=%.2f", detection.bbox.center.x, detection.bbox.center.y, detection.bbox.center.z)
        rospy.loginfo("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f",
                      pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w)

        pub.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('detection_publisher_node')
    pub = rospy.Publisher('detection_pose', PoseStamped, queue_size=10)
    rospy.Subscriber('detections', SpatialDetectionArray, detection_callback)
    rospy.spin()
