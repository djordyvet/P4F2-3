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
    # Convert the ROS Image message to an OpenCV image
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Process the image and calculate the angle
    angle, annotated_image = detect_angle(cv_image)
    
    # Log the angle
    rospy.loginfo("Detected angle: {:.2f} degrees".format(angle))
    
    # Display the image with annotations
    cv2.imshow('Annotated Image', annotated_image)
    cv2.waitKey(1)

def detect_angle(image):
    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Use edge detection (Canny, for example)
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    
    # Detect lines in the image using Hough Transform
    lines = cv2.HoughLines(edges, 1, np.pi / 180, 100)
    
    annotated_image = image.copy()
    
    if lines is not None:
        # Get the first line (rho, theta)
        rho, theta = lines[0][0]
        
        # Calculate the angle in degrees
        angle = theta * 180 / np.pi
        
        # Draw the detected line on the image
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        
        cv2.line(annotated_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # Annotate the image with the angle
        cv2.putText(annotated_image, "Angle: {:.2f} degrees".format(angle), 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        return angle, annotated_image
    else:
        # No lines found
        cv2.putText(annotated_image, "No lines detected", 
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
        return 0, annotated_image

# Subscribe to the image topic
image_sub = rospy.Subscriber('/stereo_inertial_nn_publisher/color/image', Image, image_callback)

# Keep the node running
rospy.spin()

# Close OpenCV windows on shutdown
cv2.destroyAllWindows()
