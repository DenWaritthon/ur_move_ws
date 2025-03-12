#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from target_interfaces.srv import DetectionTarget

class DetectTargetServer(Node):
    def __init__(self):
        super().__init__('detect_target_server')
        # Create the DetectionTarget service
        self.srv = self.create_service(DetectionTarget, 'detect_target', self.detect_callback)
        self.get_logger().info("DetectTarget service is ready.")
        
        # Subscribe to the red_box_target topic published by red_box_detector
        self.latest_pose = None
        self.pose_sub = self.create_subscription(
            Pose,
            'red_box_target',  # Topic where red_box_detector publishes the detected pose
            self.pose_callback,
            10
        )

    def pose_callback(self, msg):
        # Update the latest pose from the red_box_detector
        self.latest_pose = msg
        # No logging here; we log only when a service call is made

    def detect_callback(self, request, response):
        # When detection is requested, use the latest pose from red_box_detector
        if request.detect:
            if self.latest_pose is not None:
                response.target = self.latest_pose
                self.get_logger().info("Detection successful, sending latest red box pose.")
            else:
                # No pose received yet; send a default pose
                response.target.position.x = 0.0
                response.target.position.y = 0.0
                response.target.position.z = 0.0
                response.target.orientation.x = 0.0
                response.target.orientation.y = 0.0
                response.target.orientation.z = 0.0
                response.target.orientation.w = 1.0
                self.get_logger().warn("No red box pose available, sending default pose.")
        else:
            # If detection is not requested, send a default pose
            response.target.position.x = 0.0
            response.target.position.y = 0.0
            response.target.position.z = 0.0
            response.target.orientation.x = 0.0
            response.target.orientation.y = 0.0
            response.target.orientation.z = 0.0
            response.target.orientation.w = 1.0
            self.get_logger().info("Detection not requested, sending default pose.")
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = DetectTargetServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
