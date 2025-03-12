#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from target_interfaces.srv import DetectionTarget


class CameraDetection(Node):
    def __init__(self):
        super().__init__('camera_detection')

        # Create a service server
        self.create_service(DetectionTarget, '/set_target', self.set_target_callback)

        self.get_logger().info(f'Camera Detection Node has been started')

    def set_target_callback(self, request:DetectionTarget.Request, response:DetectionTarget.Response):
        self.get_logger().info(f'service request received')
        if request.detect:
            response.target.position.x = 0.1
            response.target.position.y = 0.2
            response.target.position.z = 0.3
            response.target.orientation.x = 0.4
            response.target.orientation.y = 0.5
            response.target.orientation.z = 0.6
            response.target.orientation.w = 1.0
            self.get_logger().info(f'Setting target success')
        else:
            self.get_logger().error(f'Setting target failed')
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = CameraDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()