#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agent_msgs.msg import VehicleStatus, FleetStatus, Forecast, CAcommand, Task
from agent_msgs.srv import StandardAction
from bt_msgs.srv import LocationServer
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from helper import *
from H2robot import Harvester, Chaserbin


class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('fa_bt_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value
        self.declare_parameter('number_of_robots', 3)
        self.number_of_collectors = self.get_parameter(
            'number_of_robots').value

        # Initialize variables
        self.robotIDs = [""] * self.number_of_collectors
        for i in range(self.number_of_collectors):
            self.robotIDs[i] = "Robot" + (i).__str__()

        self.isFleetnewmessage = False

        self.CAcommand = CAcommand()
        self.Harvester = Harvester(self.robotIDs[1])
        self.Chaserbins = [
            Chaserbin(self.robotIDs[0]), Chaserbin(self.robotIDs[2])]

        self.feetstatus = FleetStatus()
        self.isFleetnewmessage = False

        self.FAcommand = Forecast()
        self.isnewcommandCA = False

        # ----------------------------------A-star map
        self.ox, self.oy = [], []
        resolution = 2
        # form the border
        for i in range(-10*resolution, 10*resolution):
            self.ox.append(i/resolution)
            self.oy.append(-10.0)
        for i in range(-10*resolution, 10*resolution):
            self.ox.append(10.0)
            self.oy.append(i/resolution)
        for i in range(-10*resolution, 10*resolution):
            self.ox.append(i/resolution)
            self.oy.append(10.0)
        for i in range(-10*resolution, 10*resolution):
            self.ox.append(-10.0)
            self.oy.append(i/resolution)

        # add obs
        for j in range(0, 27):
            for i in range(-1*resolution, 9*resolution):
                self.ox.append(i/resolution)
                self.oy.append(-3.76+j*0.475)

        # Create subscriber(s)
        self.get_fleetstatus = self.create_subscription(
            FleetStatus,
            'Fleet_status_message',
            self.on_update_fleet_status,
            10)

        self.get_CAcommnad = self.create_subscription(
            CAcommand,
            'CA_command',
            self.on_update_cacommand,
            10)

        # Create publisher(s)
        self.publish_forcast = self.create_publisher(
            Forecast,
            'forecast_message',
            10)

        # Create service(s) for bt coditions

        self.is_request_get_service = self.create_service(
            StandardAction, "is_request_get", self.on_is_request_get_srv)

        self.is_fleet_message_get_fa_service = self.create_service(
            StandardAction, "is_fleet_message_get_fa", self.on_is_fleet_message_get_fa_srv)

        # Create service(s) for bt actions

        self.compute_send_service = self.create_service(
            StandardAction, "compute_send", self.on_compute_send_srv)

        # Finally
        self.get_logger().info("initialized")

    """
        Subscribers callback ------------------------------------------------------------------------

    """

    def on_update_fleet_status(self, msg=FleetStatus()):
        """
        Callback function that gets called on receiving a on_update_fleet_status message.

        """
        #self.get_logger().info("[Node: ] " + format(self.feetstatus))
        self.Harvester.update_status(msg.vehicles_status[1])
        self.Harvester.liveness_check()

        self.Chaserbins[0].update_status(msg.vehicles_status[0])
        self.Chaserbins[1].update_status(msg.vehicles_status[2])

        for chaserbin in self.Chaserbins:
            chaserbin.liveness_check()
        self.isFleetnewmessage = True
        # self.get_logger().info("[Node: ] " + format(self.Harvester.get_pos())+ "liveness: " + format(self.Harvester.is_alive))
        # self.get_logger().info("[Node: ] " + format(self.Chaserbins[0].get_pos())+ "liveness: " + format(self.Chaserbins[0].is_alive))
        # self.get_logger().info("[Node: ] " + format(self.Chaserbins[1].get_pos())+ "liveness: " + format(self.Chaserbins[1].is_alive))

    def on_update_cacommand(self, msg=CAcommand()):
        """
        Callback function that gets called on receiving a GetCommand request.

        """
        self.isnewcommandCA = False
        if msg.operation == CAcommand.OPERATION_ASK_FOR_PATH:
            self.isnewcommandCA = True
            self.get_logger().info(
                "[Node: ] MESSAGE: OPERATION_ASK_FOR_PATH GET")
            self.CAcommand = msg

    """
        BT conditions service callback------------------------------------------------------------------------

    """

    def on_is_request_get_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

        """
        response.success = False
        if self.isnewcommandCA:
            response.success = True

        return response

    def on_is_fleet_message_get_fa_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_fleet_message_get_fa_srv request.

        """

        response.success = False

        if (self.Harvester.is_alive and self.Chaserbins[0].is_alive and self.Chaserbins[1].is_alive):
            response.success = True
        else:
            self.get_logger().info(
                "one of the robot is not alive please check the internet connection.")
        return response

    """
        BT actions service callback------------------------------------------------------------------------

    """

    def on_compute_send_srv(self, request, response):
        """
        Callback function that gets called on receiving a send_Status request.

        """
        response.success = False

        if self.CAcommand:
            try:
                facommand = command_set_path(self.CAcommand.start_pos.cord_x, self.CAcommand.start_pos.cord_y,
                                            self.CAcommand.goal_pos.cord_x, self.CAcommand.goal_pos.cord_y, self.ox, self.oy)
                self.publish_forcast.publish(facommand)
                self.get_logger().info(
                    "message sent : " + facommand.__str__())
            except BaseException as e:
                self.get_logger().error(e)
            self.isnewcommandCA = False
        return response

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
