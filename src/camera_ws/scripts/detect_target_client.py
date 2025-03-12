#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from target_interfaces.srv import DetectionTarget, SetTarget

class TestTargetClient(Node):
    def __init__(self):
        super().__init__('test_target_client')
        self.detection_client = self.create_client(DetectionTarget, 'detect_target')
        self.set_target_client = self.create_client(SetTarget, 'set_target')

    def call_detection_service(self):
        # Wait until the detection service is available
        while not self.detection_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for detection service...')
        request = DetectionTarget.Request()
        request.detect = True  # Set request field (input) as needed
        future = self.detection_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Detection Service: target = {response.target}")
            return response
        else:
            self.get_logger().error('Detection service call failed.')
            return None

    def call_set_target_service(self, pick_target: Pose, place_target: Pose):
        # Wait until the set target service is available
        while not self.set_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set target service...')
        request = SetTarget.Request()
        request.pick_target = pick_target
        request.place_target = place_target
        future = self.set_target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Set Target Service: success = {response.success}")
            return response.success
        else:
            self.get_logger().error('Set target service call failed.')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = TestTargetClient()

    # Call the detection service to get the red box target
    detection_response = client.call_detection_service()

    if detection_response is not None:
        # Use the detected target as the pick target
        pick_target = detection_response.target

        # Define the place target (for example, offset the pick target by +1.0 in X and Y)
        place_target = Pose()
        place_target.position.x = pick_target.position.x + 1.0
        place_target.position.y = pick_target.position.y + 1.0
        place_target.position.z = pick_target.position.z  # same height
        place_target.orientation.x = 0.0
        place_target.orientation.y = 0.0
        place_target.orientation.z = 0.0
        place_target.orientation.w = 1.0

        # Now call the set target service with both pick and place targets
        client.call_set_target_service(pick_target, place_target)
    else:
        client.get_logger().info("No target detected or detection failed.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
