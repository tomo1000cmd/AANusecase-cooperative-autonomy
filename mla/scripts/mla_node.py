#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from mas_msgs.msg import VehicleStatus,FleetStatus,TaskList


class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('mla_node')

        # Parameters
        self.declare_parameter('my_id','server')
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
            self.feetstatus.tasklists.append(TaskList())
        
        
        self.robotliveness = []
        self.isnewmessage = False
        
        # Create subscriber(s)

        for i in range(self.number_of_robots):
            self.create_subscription(
                VehicleStatus,
                self.robotIDs[i]+'/vehicle_status',
                self.on_update_status,
                10)   
 
        for i in range(self.number_of_robots):
            self.create_subscription(
                TaskList,
                self.robotIDs[i]+'/task_list',
                self.on_update_tasklist,
                10)   
        # Create publisher(s)
        self.publish_fleetstatus = self.create_publisher(
            FleetStatus, 
            self.my_id + '/Fleet_status_message', 
            10)
        

        
        ## Create timer(s)
        self.timer_publish = self.create_timer(0.05, self.on_timer_publish)
        # Finally
        self.get_logger().info("initialized")

        
    # """
    #     Subscribers callback ------------------------------------------------------------------------
    # """
    
    def on_update_status(self, msg:VehicleStatus):
        # """
        # Callback function that gets called on receiving a GetCommand request.

        # """
        for i in range(len(self.robotIDs)):
            if(msg.status.vehicle_id == self.robotIDs[i]): # see which one has a message
                self.feetstatus.vehicles_status[i] = msg
                self.isnewmessage = True
                self.get_logger().info("[Node]: Message from "+msg.status.vehicle_id + " has received prepare sending ... ... ")
                break

    def on_update_tasklist(self, msg:TaskList):
        # """
        # Callback function that gets called on receiving a GetCommand request.

        # """
        for i in range(len(self.robotIDs)):
            if(msg.header.frame_id == self.robotIDs[i]): # see which one has a message
                self.feetstatus.tasklists[i] = msg
                self.isnewmessage = True
                self.get_logger().info("[Node]: Message from "+msg.header.frame_id + " has received prepare sending ... ... ")
                break               
    # """
    #     Standard membership functions------------------------------------------------------------------------
    # """
        
    def on_timer_publish(self):
        self.publish_fleetstatus.publish(self.feetstatus)
        self.isnewmessage = False
       



  


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
