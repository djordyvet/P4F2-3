import rospy
from sensor_msgs.msg import Image
from depthai_ros_msgs.msg import SpatialDetectionArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from BoundingBoxAngle.msg import BoundingBoxAngle  

class ObjectAngleDetector:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('object_angle_detector', anonymous=True)

        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscribe to the image and bounding box topics
        rospy.Subscriber('/stereo_inertial_nn_publisher/color/image', Image, self.image_callback)
        rospy.Subscriber('/stereo_inertial_nn_publisher/color/detections', SpatialDetectionArray, self.detection_callback)

        # Publisher for the bounding box angle
        self.angle_pub = rospy.Publisher('/object_angle_detector/angle_centroid', BoundingBoxAngle, queue_size=10)

        self.latest_image = None
        self.detections = []

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_callback(self, msg):
        # Store the latest detections
        self.detections = msg.detections

    def process_detections(self):
        if self.latest_image is None or not self.detections:
            return

        for detection in self.detections:
            bbox = detection.bbox
            x_min = int(bbox.center.x - bbox.size_x / 2)
            y_min = int(bbox.center.y - bbox.size_y / 2)
            x_max = int(bbox.center.x + bbox.size_x / 2)
            y_max = int(bbox.center.y + bbox.size_y / 2)

            # Ensure the bounding box is within the image dimensions
            x_min = max(0, x_min)
            y_min = max(0, y_min)
            x_max = min(self.latest_image.shape[1], x_max)
            y_max = min(self.latest_image.shape[0], y_max)

            # Crop the region from the image
            cropped_image = self.latest_image[y_min:y_max, x_min:x_max]

            # Calculate the angle and centroid of the object in the cropped region
            angle, centroid, annotated_image = self.calculate_angle_and_centroid(cropped_image)

            # Log the angle and centroid
            rospy.loginfo("Detected angle: {:.2f} degrees, Centroid: ({}, {})".format(angle, centroid[0], centroid[1]))

            # Publish the angle and centroid
            angle_msg = BoundingBoxAngle()
            angle_msg.angle = angle
            angle_msg.centroid_x = centroid[0]
            angle_msg.centroid_y = centroid[1]
            self.angle_pub.publish(angle_msg)

            # Display the annotated cropped image
            cv2.imshow('Annotated Cropped Image', annotated_image)
            cv2.waitKey(1)

    def calculate_angle_and_centroid(self, image):
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

            # Calculate the centroid of the bounding box
            M = cv2.moments(edges)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = image.shape[1] // 2, image.shape[0] // 2

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

            # Annotate the image with the angle and centroid
            cv2.putText(annotated_image, "Angle: {:.2f} degrees".format(angle), (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(annotated_image, (cX, cY), 5, (0, 255, 0), -1)
            cv2.putText(annotated_image, "Centroid: ({}, {})".format(cX, cY), (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            return angle, (cX, cY), annotated_image
        else:
            # No lines found
            cv2.putText(annotated_image, "No lines detected", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
            return 0, (image.shape[1] // 2, image.shape[0] // 2), annotated_image

    def run(self):
        # Keep the node running
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.process_detections()
            rate.sleep()

        # Close OpenCV windows on shutdown
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detector = ObjectAngleDetector()
    detector.run()
