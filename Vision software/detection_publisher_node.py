import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize the node
rospy.init_node('angle_detection_node', anonymous=True)

# Create a CvBridge object
bridge = CvBridge()

def image_callback(msg):
    # Convert the ROS Image message to a OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Process the image and calculate the angle
    angle = detect_angle(cv_image)
    
    # Log the angle
    rospy.loginfo("Detected angle: {:.2f} degrees".format(angle))

def detect_angle(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Use edge detection (Canny, for example)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    
    # Detect lines in the image using Hough Transform
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
    
    if lines is not None:
        # Get the first line (rho, theta)
        rho, theta = lines[0][0]
        
        # Calculate the angle in degrees
        angle = theta * 180 / np.pi
        
        return angle
    else:
        return 0  # No lines found

# Subscribe to the image topic
image_sub = rospy.Subscriber('/stereo_inertial_nn_publisher/color/image', Image, image_callback)

# Keep the node running
rospy.spin()
