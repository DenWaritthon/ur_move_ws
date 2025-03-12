#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from target_interfaces.srv import SetTarget


class DummyControllerServiceServer(Node):
    def __init__(self):
        super().__init__('dummy_controller_service_server')

        # Create a service server
        self.create_service(SetTarget, '/set_target', self.get_target_callback)

        self.get_logger().info(f'Dummy Controller Service Server Node has been started')

    def get_target_callback(self, request:SetTarget.Request, response:SetTarget.Response):
        self.get_logger().info(f'service request received')

        self.get_logger().info(f'Setting Pick Target')
        self.get_logger().info(f'x :{request.pick_target.position.x}\n\
                               y :{request.pick_target.position.y}\n\
                               z :{request.pick_target.position.z}\n\
                               ox :{request.pick_target.orientation.x}\n\
                               oy :{request.pick_target.orientation.y}\n\
                               oz :{request.pick_target.orientation.z}\n\
                               ow :{request.pick_target.orientation.w}')
        
        self.get_logger().info(f'Setting Place Target')
        self.get_logger().info(f'x :{request.place_target.position.x}\n\
                               y :{request.place_target.position.y}\n\
                               z :{request.place_target.position.z}\n\
                               ox :{request.place_target.orientation.x}\n\
                               oy :{request.place_target.orientation.y}\n\
                               oz :{request.place_target.orientation.z}\n\
                               ow :{request.place_target.orientation.w}')
        
        self.get_logger().info(f'Setting target success')
        response.success = True

        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = DummyControllerServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
