#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Replace with your custom service type


class UR5ControlNode(Node):
    def __init__(self):
        super().__init__('ur5_control_node')
        self.cli1 = self.create_client(AddTwoInts, 'custom_service_1')
        self.cli2 = self.create_client(AddTwoInts, 'custom_service_2')
        
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service 1...')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service 2...')
        
        self.req1 = AddTwoInts.Request()
        self.req2 = AddTwoInts.Request()
    
    def call_detection(self):
        pass

    def control_ur5(self):
        pass

    def send_request(self):
        self.req1.a = 2
        self.req1.b = 3
        self.future1 = self.cli1.call_async(self.req1)
        
        self.req2.a = 5
        self.req2.b = 7
        self.future2 = self.cli2.call_async(self.req2)


def main(args=None):
    rclpy.init(args=args)
    node = UR5ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()