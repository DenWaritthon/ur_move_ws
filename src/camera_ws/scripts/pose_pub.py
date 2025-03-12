#!/usr/bin/python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from target_interfaces.srv import DetectionTarget  
class RedBoxDetectorService(Node):
    def __init__(self):
        super().__init__('red_box_detector_service')

        self.bridge = CvBridge()
        self.latest_image = None  # Store the latest image

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Create a service server
        self.srv = self.create_service(DetectionTarget, '/detect_red_box', self.detect_red_box_callback)

        self.get_logger().info('Red Box Detector Service has started')

    def image_callback(self, msg):
        """ Store the latest image message """
        self.latest_image = msg

    def get_latest_image(self):
        """Waits for an image message (up to a timeout) and returns it."""
        self.get_logger().info("Waiting for an image...")

        timeout_sec = 10.0
        start_time = self.get_clock().now()
        
        while self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.get_logger().warning("Timeout reached! No image received.")
                return None

        return self.latest_image


    def detect_red_box_callback(self, request, response):
        """
        Service callback function that processes an image and returns 
        the detected position of the red box.
        """
        if not request.detect:
            self.get_logger().info("Detection request received but flag is False.")
            response.target = Pose()  # Return default Pose (0,0,0)
            return response

        msg = self.get_latest_image()

        if msg is None:
            self.get_logger().warning("No image received within timeout.")
            response.target = Pose()  # Return default Pose
            return response

        # Convert ROS2 image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Create a mask for the red color
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        # Find the red box position
        X, Y = self.find_box_position(mask_red)

        if X is not None and Y is not None:
            response.target.position.x = X
            response.target.position.y = Y
            response.target.position.z = 0.0
            response.target.orientation.w = 1.0  # No rotation
            self.get_logger().info(f"Detected Red Box at X={X:.2f}, Y={Y:.2f}")
        else:
            response.target = Pose()  # Return default Pose
            self.get_logger().warning("No red box detected.")

        return response

    def find_box_position(self, mask):
        """Find the red box in the given mask and return its position (X, Y)."""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 600:  # Filter out small noise
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])

                    # Convert pixel coordinates to world coordinates
                    X = round((u / 640) * 1.2 - 0.6, 2)
                    Y = round((v / 640) * 1.2 - 0.6, 2)
                    return X, Y

        return None, None  # No red box found

def main(args=None):
    rclpy.init(args=args)
    node = RedBoxDetectorService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
