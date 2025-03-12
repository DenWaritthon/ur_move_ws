#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from target_interfaces.srv import *


class TargetManagement(Node):
    def __init__(self):
        super().__init__('target_management')

        # Create a service server
        self.create_service(Start, '/start', self.start_callback)

        # Create a service client
        self.set_target_group = MutuallyExclusiveCallbackGroup()
        self.set_target_client = self.create_client(SetTarget, '/set_target',callback_group = self.set_target_group)

        self.get_target_group = MutuallyExclusiveCallbackGroup()
        self.get_target_client = self.create_client(GetTarget, '/get_target',callback_group = self.get_target_group)

        while not self.set_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /set_target not available, waiting again...')
        self.get_logger().info('service /set_target is available')

        while not self.get_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service /get_target not available, waiting again...')
        self.get_logger().info('service /get_target is available')

        self.get_logger().info(f'Target Management Node has been started')

        # Variables
        self.pick_target = [0.4, -0.1, 0.0, -0.013209, 0.999893, 0.002859 , 0.005563]
        self.place_target = [-0.1, -0.5, 0.0, -0.013209, 0.999893, 0.002859 , 0.005563]
        
        # pick_target position
        # pose:
        #     X:0.434576
        #     Y:-0.132511
        #     Z:0.061977
        # orientation:
        #     x:-0.701155
        #     y:0.712983
        #     z:0.005717
        #     w:0.002191

    def start_callback(self, request:Start.Request, response:Start.Response):
        self.get_logger().info(f'Start call')
        # Call get target
        self.call_get_target()
        return response

    def call_get_target(self):
        request = GetTarget.Request()
        request.call = True
        result = self.get_target_client.call(request)

        self.pick_target[0] = result.target.position.x
        self.pick_target[1] = result.target.position.y
        self.pick_target[2] = result.target.position.z
        
        self.get_logger().info(f'Pick Target: {self.pick_target}')
        self.call_set_target()

    def call_set_target(self):
        request = SetTarget.Request()

        # set pick target
        request.pick_target.position.x = self.pick_target[0]
        request.pick_target.position.y = self.pick_target[1]
        request.pick_target.position.z = self.pick_target[2]
        request.pick_target.orientation.x = self.pick_target[3]
        request.pick_target.orientation.y = self.pick_target[4]
        request.pick_target.orientation.z = self.pick_target[5]
        request.pick_target.orientation.w = self.pick_target[6]

        # set place target
        request.place_target.position.x = self.place_target[0]
        request.place_target.position.y = self.place_target[1]
        request.place_target.position.z = self.place_target[2]
        request.place_target.orientation.x = self.place_target[3]
        request.place_target.orientation.y = self.place_target[4]
        request.place_target.orientation.z = self.place_target[5]
        request.place_target.orientation.w = self.place_target[6]

        result = self.set_target_client.call(request)

        if result.success:
            self.get_logger().info('Target set successfully')
        else:
            self.get_logger().info('Failed to set target')

def main(args=None):
    rclpy.init(args=args)
    node = TargetManagement()
    # rclpy.spin(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
