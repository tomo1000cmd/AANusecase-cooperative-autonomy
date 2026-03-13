#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agent_msgs.msg import DriveOrder, OperationControl
from agent_msgs.srv import RouteAvailable, GetRoute, ControlAvailable, GetControl,StandardAction


class CommandHandler(Node):
    """
    This class is a ROS2 node.
    It handles commands received from the CA.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('voa_command_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value

        # Initialize variables
        self.control_available = False
        self.operation_status = False
        self.route_available = False
       
        self.route_id = -1

        # Create subscriber(s)
        self.operation_control_sub = self.create_subscription(
            OperationControl, 'operation_control', self.on_operation_control_msg, 10)
        self.drive_order_sub = self.create_subscription(
            DriveOrder, 'drive_order', self.on_drive_order_msg, 10)

        # Create publisher(s)

        # Create service(s)
        self.command_available_service = self.create_service(
            ControlAvailable, "control_available", self.on_control_available_srv)
        self.get_command_service = self.create_service(
            GetControl, "get_control", self.on_get_control_srv)
        self.route_available_service = self.create_service(
            RouteAvailable, "route_available", self.on_route_available_srv)
        self.get_route_service = self.create_service(
            GetRoute, "get_route", self.on_get_route_srv)
        

        # Create timer(s)

        # Finally
        self.get_logger().info("initialized")

    def on_operation_control_msg(self, msg):
        """
        Callback function that gets called on receiving a OperationControl message.
        """
        if msg.vehicle_id == self.my_id:
            self.operation_status = msg.status
            self.control_available = True
           

    def on_control_available_srv(self, request, response):
        """
        Callback function that gets called on receiving a CommandAvailable request.
        """
        response.success = self.control_available
        return response

    def on_get_control_srv(self, request, response):
        """
        Callback function that gets called on receiving a GetCommand request.
        """
        if self.control_available:
            response.success = True
            response.status = self.operation_status
            self.control_available = False
        
        else:
            response.success = False
        return response

    def on_drive_order_msg(self, msg):
        """
        Callback function that gets called on receiving a DriveOrder message.
        """
        if msg.vehicle_id == self.my_id:
            self.route_id = msg.route_id
            self.route_available = True
          

    def on_route_available_srv(self, request, response):
        """
        Callback function that gets called on receiving a RouteAvailable request.
        """
        response.success = self.route_available
        return response

    def on_get_route_srv(self, request, response):
        """
        Callback function that gets called on receiving a GetRoute request.
        """
        if self.route_available:
            response.success = True
            response.route_id = self.route_id
            self.route_available = False
            
        else:
            response.success = False
        return response

 



def main(args=None):

    rclpy.init(args=args)

    # Create the node
    command_handler = CommandHandler()

    # Spin the node so the callback functions are called.
    rclpy.spin(command_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_handler.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
