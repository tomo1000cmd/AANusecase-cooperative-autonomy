#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agent_msgs.msg import VehicleStatus,FleetStatus
from agent_msgs.srv import StandardAction
from bt_msgs.srv import LocationServer


class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('mla_bt_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value
        self.declare_parameter('number_of_robots', 3)
        self.number_of_robots = self.get_parameter('number_of_robots').value

        # Initialize variables
        self.robotIDs = [""] * self.number_of_robots
        for i in range(self.number_of_robots):
            self.robotIDs[i] = "Robot" + (i).__str__()
        
        self.feetstatus = FleetStatus()

        for i in range(self.number_of_robots):
            self.feetstatus.vehicles_status.append(VehicleStatus())
        
        
        self.robotliveness = []
        self.isnewmessage = False
        
        # Create subscriber(s)

        for i in range(self.number_of_robots):
            self.create_subscription(
                VehicleStatus,
                self.robotIDs[i]+'/vehicle_status',
                self.on_update_status,
                10)   
 
        
        # Create publisher(s)
        self.publish_fleetstatus = self.create_publisher(
            FleetStatus, 
            'Fleet_status_message', 
            10)
        
        # Create service(s) for bt coditions
        
        self.is_message_get_service = self.create_service(
            StandardAction, "is_message_get", self.on_is_message_get__srv)

        # Create service(s) for bt actions
      
        self.send_status_service = self.create_service(
            StandardAction, "send_status", self.on_send_Status_srv)


        #self.send_emgency_service = self.create_service(
         #   StandardAction, "send_emgency_status", self.on_send_emgency_Status_srv)

        

        # Finally
        self.get_logger().info("initialized")

        
    """
        Subscribers callback ------------------------------------------------------------------------

    """
    
    def on_update_status(self, msg):
        """
        Callback function that gets called on receiving a GetCommand request.

        """
        for i in range(len(self.robotIDs)):
            if(msg.vehicle_id == self.robotIDs[i]): # see which one has a message
                self.feetstatus.vehicles_status[i] = msg
                self.isnewmessage = True
                
                self.get_logger().info("[Node]: Message from "+msg.vehicle_id + " has received prepare sending ... ... ")



                break
    """
        BT conditions service callback------------------------------------------------------------------------

    """
    def on_is_message_get__srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

        """
      
        response.success = False
        if self.isnewmessage:
            response.success = True

        return response



    """
        BT actions service callback------------------------------------------------------------------------

    """
    def on_send_Status_srv(self, request, response):
        """
        Callback function that gets called on receiving a send_Status request.

        """
        self.publish_fleetstatus.publish(self.feetstatus)
        self.isnewmessage = False

        response.success = True

        return response


    """
        Standard membership functions------------------------------------------------------------------------

    """
        
       
       



  


def main(args=None):

    rclpy.init(args=args)

    # Create the node
    Bt_handler = BtHandler()

    # Spin the node so the callback functions are called.
    rclpy.spin(Bt_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Bt_handler.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
