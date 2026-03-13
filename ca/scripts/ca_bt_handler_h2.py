#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from agent_msgs.msg import VehicleStatus, FleetStatus, CAcommand, OperationControl, Task, Forecast, Status
from agent_msgs.srv import StandardAction
from helper import *
from H2robot import Harvester, Chaserbin
from map_msgs.msg import OccupancyGridUpdate
#change it to xml 
import uuid
FIELD_DIAMENTION_X = [-1.0, 8.0]
FIELD_DIAMENTION_Y = [9.3, -4.0]

AVERAGE_HARVESTER_SPEED = 0.14  # average harvest speed in m/s
AVERAGE_CHASERBIN_SPEED = 0.6   # average chaserbin speed in m/s
AVERAGE_HARVESTER_RATE = 0.66    # in %/s
MAX_CHASERBIN_STORAGE = 100     # in %
CHASERBIN_SWITCH_THRESHOLD = 90  # in #

LEFT_ENTER_POINT = FIELD_DIAMENTION_X[0] - 1.0
RIGHT_ENTER_POINT = FIELD_DIAMENTION_X[1] + 1.0
inside_out_index_first_field = [2,3,1,4,0,5,  14,15,13,16,12,17, 6,11,7,10,8,9,  18,23,19,22,20,21 ]

H_CONTROL_MODE = -1
S_CONTROL_MODE = 1

class WaitState:
    IDLE = 99
    PAUSE_CHASERBIN =  0
    RESUME_CHASERBIN_PAUSE_HARVESTER = 1 
    RESUME_HARVESTER=  2

class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('ca_bt_handler')

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

        self.is_switching = False
        self.running_chaserin = None
        self.idle_chaserin = None

        # set obstacle positions
        self.ox, self.oy = [], []
        self.goal = []
        self.is_exiting = False
        self.is_command_path = False
        self.is_command_exit = False
        self.is_sort_robot = True
        self.pause_state_flag = WaitState.IDLE
        self.finished_row = []
        self.is_id_update = True
       
        self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 
        
        # Create subscriber(s)
        self.get_fleetstatus = self.create_subscription(
            FleetStatus,
            'Fleet_status_message',
            self.on_update_fleet_status,
            10)
        # self.get_forcast = self.create_subscription(
        #     Forecast,
        #     'forecast_message',
        #     self.on_update_forcast,
        #     10)

        # Create publisher(s)
        self.publish_CAcommand = self.create_publisher(
            CAcommand,
            'CA_command',
            10)

        self.publish_OperationControl = self.create_publisher(
            OperationControl,
            'operation_control',
            10)
        self.publish_updatesMap = self.create_publisher(
            OccupancyGridUpdate, 
            'map_updates', 
            10)
        # Create service(s) for bt coditions

        self.create_service(
            StandardAction, "is_fleet_message_get", self.on_is_fleet_message_get_srv)

        self.create_service(
            StandardAction, "is_current_chaserbin_exit", self.on_is_current_chaserbin_exit_srv)

        self.create_service(
            StandardAction, "is_current_chaserbin_full", self.on_is_current_chaserbin_full_srv)

        self.create_service(
            StandardAction, "is_standby_chaser_start", self.on_is_standby_chaser_start_srv)

        self.create_service(
            StandardAction, "is_standby_chaserbin_there", self.on_is_standby_chaserbin_there_srv)

        # Create service(s) for bt actions

        self.create_service(
            StandardAction, "send_command", self.on_send_command_srv)

        # self.send_emgency_service = self.create_service(
        #   StandardAction, "send_emgency_status", self.on_send_emgency_Status_srv)
        from datetime import datetime
        timestring = str(datetime.fromtimestamp(
                self.get_clock().now().to_msg().sec))
        self.logfile_ca = open("ca_log_"+".log", "w")


        
        # Finally
        self.get_logger().info("initialized")


    """
        Subscribers callback ------------------------------------------------------------------------

    """

    def on_update_fleet_status(self, msg=FleetStatus()):
        """
        Callback function that gets a fleet status

        """

        #self.get_logger().info("[Node: ] " + format(self.feetstatus))
        self.Harvester.update_status(msg.vehicles_status[1])
        self.Harvester.liveness_check()

        self.Chaserbins[0].update_status(msg.vehicles_status[0])
        self.Chaserbins[1].update_status(msg.vehicles_status[2])

        for chaserbin in self.Chaserbins:
            chaserbin.liveness_check()
        self.isFleetnewmessage = True
                    ## update map

        #     self.get_logger().info("[Node: ] " + format(self.Harvester.get_pos())+ "liveness: " + format(self.Harvester.is_alive))
        # self.get_logger().info("[Node: ] " + format(self.Chaserbins[0].get_pos())+ "liveness: " + format(self.Chaserbins[0].is_alive))
        # self.get_logger().info("[Node: ] " + format(self.Chaserbins[1].get_pos())+ "liveness: " + format(self.Chaserbins[1].is_alive))

    """
        BT conditions service callback------------------------------------------------------------------------

    """

    def on_is_fleet_message_get_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a on_is_message_get request.
        1. First, we check if all robots are alive. If all robots are alive, we can continue to the next step.
        2. If one of the robots is not alive, we will send a warning message and return a response.
        3. If all robots are alive, we check if the robots are in the harvesting state, if one of the robots is in the harvesting state, we will start the harvesting state for the harvester robot, if not, we will continue to the next step.
        4. If none of the robots is in the harvesting state, we will check the distance between the harvester robot and the chaser robots, and then we will start the chasing state for the chaser robot that is nearest to the harvester robot. 
        """

        response.success = False

        if (self.Harvester.is_alive and self.Chaserbins[0].is_alive and self.Chaserbins[1].is_alive):
            if (self.Harvester.get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING and self.Harvester.my_status.status.route_id == H_CONTROL_MODE):
                offset_distance = -0.5
                yaw_rad = self.Harvester.get_angle() * (np.pi/180)
                
                x = self.Harvester.get_pos()[0]
                y = self.Harvester.get_pos()[1]
                xOffset = offset_distance * np.cos(yaw_rad)
                yOffset = offset_distance * np.sin(yaw_rad)
                offset_x =  x + xOffset
                offset_y =  y + yOffset
                resolution =  0.05
                origin = [-10.5, -10.3, 0]
                clear_layer = 0
                mapupdate_msg = OccupancyGridUpdate()
                mapupdate_msg.header.stamp = self.get_clock().now().to_msg()
                mapupdate_msg.header.frame_id = "map"
                mapupdate_msg.height = int(0.5 * 1/resolution)
                mapupdate_msg.width = int(0.5 * 1/resolution)
                mapupdate_msg.x = int((-origin[0] - mapupdate_msg.width * resolution/2 +  offset_x )*1/resolution)
                mapupdate_msg.y = int((-origin[1] - mapupdate_msg.height* resolution/2 +  offset_y )*1/resolution) 
                mapupdate_msg.data = [clear_layer]*int(mapupdate_msg.height *mapupdate_msg.width)
                self.publish_updatesMap .publish(mapupdate_msg)
            # auto start function
            if (self.Chaserbins[0].get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING or self.Chaserbins[1].get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING):
                if self.Harvester.get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING:
                    response.success = True
                else:
                    self.CAcommand = command_start_harvesting(
                        self.Harvester.my_id)
                    self.CAcommand.task.timestamp = self.get_clock().now().to_msg().sec
                    self.CAcommand.task.route_id = self.mission_id
                    self.publish_CAcommand.publish(self.CAcommand)
                    
                    self.get_logger().info("MESSAGE : Command: " + format(self.CAcommand)+"sent")
                    self.get_logger().info("start harvesting")
            else:
                dis = float("inf")
                id = 0
                for chaserbin in self.Chaserbins:
                    if dis > calc_distance(chaserbin.get_pos(), self.Harvester.get_pos()):
                        dis = calc_distance(
                            chaserbin.get_pos(), self.Harvester.get_pos())
                        id = self.Chaserbins.index(chaserbin)
                closer_chaserbin = self.Chaserbins[id]
                if calc_distance(closer_chaserbin.get_pos(), self.Harvester.get_pos()) < 1:
                    # start chaseing
                    self.CAcommand = command_start_harvesting(
                        closer_chaserbin.my_id)
                    self.CAcommand.task.timestamp = self.get_clock().now().to_msg().sec
                    self.CAcommand.task.route_id = self.mission_id
                    self.publish_CAcommand.publish(self.CAcommand)

                    self.get_logger().info("MESSAGE : Command: " + format(self.CAcommand)+"sent")
                    self.get_logger().info("start chasing")
                else:
                    # start_apporaching
                    pass
        else:
            self.get_logger().info(
                "one of the robot is not alive please check the internet connection.")

        return response

    def on_is_current_chaserbin_exit_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a on_is_message_get request.
        1. When the chaserbin is in the exit state, it will check the distance between the running chaserbin and the idle chaserbin.
        2. If the distance is less than the running chaserbin's speed * 3.0, then it will start to exit.
        3. When it exits, it will check if the chaserbin is actually started path following.
        4. If so, it will switch the chaserbin and print the remaining in the logfile.
        """

        response.success = False
        if self.is_exiting:
            
            #if abs(self.running_chaserin.get_pos()[0] - self.idle_chaserin.get_pos()[0]) < 1.2:
            thres =np.clip(abs(self.idle_chaserin.my_status.status.current_speed * 3.0),1.4,3)
            self.get_logger().info("exiting" + " , threshold : = " + str(thres))
            if abs(calc_distance(self.running_chaserin.get_pos(),self.idle_chaserin.get_pos())) < thres:    
                    
                    self.is_command_exit = True
                    response.success = True
                    if self.is_id_update:
                        self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 
                        self.is_id_update = False
                    # check if the robot is actually started path following
            if self.running_chaserin.get_operation_status() == Status.CONTROLLER_STATUS_OPERATIONAL:
                self.is_id_update = True
                self.is_exiting = False
                self.is_switching = False
                self.is_sort_robot = True
                self.is_command_exit = False
                self.get_logger().info("switch complete")
                self.get_logger().info("The exiting chaserbin status:"+str(self.running_chaserin.check_cabin_remaing()))
                self.logfile_ca.write(str(self.running_chaserin.check_cabin_remaing()) + " \n")
                response.success = False

        return response

    def on_is_current_chaserbin_full_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a on_is_message_get request.
        1. The current chaserbin is the chaserbin that is harvesting and the standby chaserbin is the chaserbin that is in the idle status.
        2. The threshold value is the value that when the current chaserbin has more than the threshold value, the chaserbin will switch to the standby chaserbin.
        3. The threshold value is calculated by the following formula:
            th = MAX_CHASERBIN_STORAGE - (FIELD_DIAMENTION_X[1]-FIELD_DIAMENTION_X[0]) / AVERAGE_HARVESTER_SPEED * AVERAGE_HARVESTER_RATE
            MAX_CHASERBIN_STORAGE is the maximum storage of the chaserbin
            FIELD_DIAMENTION_X[1]-FIELD_DIAMENTION_X[0] is the total length of the field
            AVERAGE_HARVESTER_SPEED is the average speed of the harvester
            AVERAGE_HARVESTER_RATE is the average rate of the harvester
        4. When the current chaserbin has more than the threshold value, the current chaserbin will switch to the standby chaserbin. The switch is done by the sorting robot, which will be explained in the following part. 
        """

        response.success = False
        # make sure the system is finish the last switching
        # when the system start switching the standby chaserbin and the current chaserbin has to be the same and only be changed after the switch is over
        if self.is_sort_robot:
            self.idle_chaserin = None
            #self.get_logger().info("sorting the robot")
            # find the current chaserbin and standby/idle chaserbin
            for chaserbin in self.Chaserbins:
                if chaserbin.get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING:
                    self.running_chaserin = chaserbin
                if chaserbin.get_operation_status() == Status.CONTROLLER_STATUS_MANUAL:
                    self.idle_chaserin = chaserbin
            self.is_sort_robot = False
        # the threshold value for start switch
        th = MAX_CHASERBIN_STORAGE - \
            (FIELD_DIAMENTION_X[1]-FIELD_DIAMENTION_X[0]) / \
            AVERAGE_HARVESTER_SPEED * AVERAGE_HARVESTER_RATE
        if self.running_chaserin.check_cabin_remaing() >= (th+3):
         
            response.success = True

        else:
            self.is_sort_robot = True
        harveter_start_point_y = 9.0625
        width_row = 0.475
        index = (harveter_start_point_y - self.Harvester.get_pos()[1])/width_row
        cur_row = int(round(index,0))
        
        if (cur_row not in self.finished_row) and  (self.Harvester.my_status.status.route_id == H_CONTROL_MODE) :
                self.finished_row.append(cur_row)

        ##waiting routing 

        ##pause the current chaserbin 
        ##wait until the haverster is at the startpos of the next row
        ##resume the current chaserbin
        ##pause the current haversert
        ##wait until they are both at the start pos of the next row
        ##resume all

        ## h control means the robot is in horiziontal control s means its in switch mode
        # if self.running_chaserin.my_status.status.route_id == S_CONTROL_MODE and (self.Harvester.my_status.status.current_action == self.running_chaserin.my_status.status.current_action):     
        #     self.pause_state_flag = WaitState.PAUSE_CHASERBIN
            
        # if self.pause_state_flag == WaitState.PAUSE_CHASERBIN:
        #     ca_command = command_pause_robot(self.running_chaserin.my_id)
        #     self.publish_CAcommand.publish(ca_command)

        #     if self.Harvester.my_status.status.route_id == H_CONTROL_MODE and self.Harvester.my_status.status.route_status == str(False)  and (self.Harvester.my_status.status.current_action == self.running_chaserin.my_status.status.current_action) :
        #         self.pause_state_flag = WaitState.RESUME_CHASERBIN_PAUSE_HARVESTER
        #         ca_command = command_pause_robot(self.Harvester.my_id)
        #         self.publish_CAcommand.publish(ca_command)
        # if self.pause_state_flag == WaitState.RESUME_CHASERBIN_PAUSE_HARVESTER:

        #     ca_command = command_resume_robot(self.running_chaserin.my_id)
        #     self.publish_CAcommand.publish(ca_command)

        #     if self.running_chaserin.my_status.status.route_id == H_CONTROL_MODE :
        #         self.pause_state_flag = WaitState.RESUME_HARVESTER

        # if self.pause_state_flag == WaitState.RESUME_HARVESTER:
        #     ca_command = command_resume_robot(self.Harvester.my_id)
        #     self.publish_CAcommand.publish(ca_command)
        #     ca_command = command_resume_robot(self.running_chaserin.my_id)
        #     self.publish_CAcommand.publish(ca_command)
        #     self.pause_state_flag = WaitState.IDLE

        # self.get_logger().info(str(self.pause_state_flag))
 
    
        return response

    def on_is_standby_chaser_start_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a on_is_message_get request.
        1. This function is called when the service is triggered.
        2. The function checks if the robots are ready for switching.
        3. The function checks if the robots are in the right position to switch.
        4. The function calculates the time it will take for the robot to fill up the bin and checks if it is greater than the threshold.
        5. The function starts harvesting if the threshold is met.
        6. The function returns True if the switch is successful and False if it is not. 
        """

        response.success = False
        if self.is_switching and self.is_exiting == False:
            self.get_logger().info("start switching")
            # switch mode
            currnet_heading = 0
            self.get_logger().info(self.idle_chaserin.get_operation_status().__str__())
            self.get_logger().info(self.goal.__str__())
            cur_angle = self.running_chaserin.get_angle()
            # heading left to right
            if cur_angle < 10 and cur_angle > -10:
                currnet_heading = LEFT_ENTER_POINT
            # heading right to left
            if cur_angle > 170 or cur_angle < -170:
                currnet_heading = RIGHT_ENTER_POINT
            if (currnet_heading == self.goal[0]) and (self.idle_chaserin.get_operation_status() == Status.CONTROLLER_STATUS_MANUAL):
                if self.running_chaserin.get_pos()[0] < (FIELD_DIAMENTION_X[1] ) and self.running_chaserin.get_pos()[0] > (FIELD_DIAMENTION_X[0] ):
                    threshold = abs(calc_distance(self.running_chaserin.get_pos(), self.idle_chaserin.get_pos(
                    ))) / (AVERAGE_CHASERBIN_SPEED - AVERAGE_HARVESTER_SPEED) * AVERAGE_HARVESTER_RATE + self.running_chaserin.check_cabin_remaing()
                    self.get_logger().info("prediction for bin status after switch : " + threshold.__str__())
                    if threshold >= CHASERBIN_SWITCH_THRESHOLD:
                        self.CAcommand = command_start_harvesting(
                            self.idle_chaserin.my_id)
                        self.CAcommand.task.task_info = [(self.Harvester.my_status.status.current_action)]
                        self.CAcommand.mission_operation = self.Harvester.my_status.status.current_action
                        response.success = True
                        if self.is_id_update:
                            self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 
                            self.is_id_update = False
                        # check if the robot is actually started harvesting
            if self.idle_chaserin.get_operation_status() == Status.CONTROLLER_STATUS_HARVESTING:
                self.get_logger().info("is exiting true")
                self.is_id_update = True
                self.is_exiting = True
                response.success = False

        return response

    def on_is_standby_chaserbin_there_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a on_is_standby_chaserbin_there_srv request.
        1. The code is called when the controller receives a request from the referee node to check if the standby chaserbin is at the entrance
        2. The code will check if the idle chaserbin is at the entrance.
        3. If the idle chaserbin is at the entrance, the code will send a command to the idle chaserbin to start path following
        4. The code will return a response with success = True if the idle chaserbin is at the entrance and a response with success = False if the idle chaserbin is not at the entrance
        """

        response.success = False
        if self.is_switching == False and self.is_command_exit == False:
            self.get_logger().info("find the entrance")
            # compute the goal for the enter point for the standby chaserbin
            next_cor_x = -88  # defualt value
            next_cor_y = self.running_chaserin.get_pos()[1] - 3.0
            cur_angle = self.running_chaserin.get_angle()
            if cur_angle < 10 and cur_angle > -10:
                next_cor_x = RIGHT_ENTER_POINT
            # heading right to left
            if cur_angle > 170 or cur_angle < -170:
                next_cor_x = LEFT_ENTER_POINT

            # if self.running_chaserin.get_pos()[0] <= (FIELD_DIAMENTION_X[0] + 1):
            #     next_cor_x = LEFT_ENTER_POINT
            # if self.running_chaserin.get_pos()[0] >= (FIELD_DIAMENTION_X[1] - 1):
            #     next_cor_x = RIGHT_ENTER_POINT
            self.goal = [next_cor_x, next_cor_y]

            self.get_logger().info(self.goal.__str__())

            if self.idle_chaserin != None:
                self.get_logger().info(self.idle_chaserin.get_pos().__str__())
                if next_cor_x != -88:

                    # send the command to the standby chaserbin to start following
                    # proceed to the next sequnce

                    self.is_command_path = True
                    response.success = True
                    if self.is_id_update:
                            self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 
                            self.is_id_update = False
                    # check if the robot is actually started path following
                    if (self.idle_chaserin.get_operation_status() == Status.CONTROLLER_STATUS_OPERATIONAL):
                        self.is_id_update = True
                        self.is_switching = True
                        self.get_logger().info("is_switching true")


        return response

    """
        BT actions service callback------------------------------------------------------------------------

    """


    def on_send_command_srv(self, request, response=StandardAction.Response):
        """
        Callback function that gets called on receiving a send_Status request.

        """

        resolution = 2
        self.ox = []
        self.oy = []
        border_size = 10.5

            # form the border
        for i in range(-int(border_size*resolution), int(border_size*resolution)):
                self.ox.append(i/resolution)
                self.oy.append(-border_size)
        for i in range(-int(border_size*resolution), int(border_size*resolution)):
                self.ox.append(border_size)
                self.oy.append(i/resolution)
        for i in range(-int(border_size*resolution), int(border_size*resolution)):
                self.ox.append(i/resolution)
                self.oy.append(border_size)
        for i in range(-int(border_size*resolution), int(border_size*resolution)):
                self.ox.append(-border_size)
                self.oy.append(i/resolution)

            # add obs
        for j in range(0, 28):
                for i in range(-1*resolution, 9*resolution):
                    if j not in self.finished_row:
                        self.ox.append(i/resolution)
                        self.oy.append(9.05 - j*0.475)

        for i in range(-1*resolution, 9*resolution):
            self.ox.append(i/resolution)
            self.oy.append(self.Harvester.get_pos()[1])
        for i in range(-1*resolution, 9*resolution):
            self.ox.append(i/resolution)
            self.oy.append(self.running_chaserin.get_pos()[1])
        
        for i in range(-1*resolution, 9*resolution):
            self.ox.append(i/resolution)
            self.oy.append(self.running_chaserin.get_pos()[1]-0.3)

        if self.is_command_path:
            self.CAcommand = command_set_path(self.idle_chaserin.get_pos()[0], self.idle_chaserin.get_pos()[
                                              1], self.goal[0], self.goal[1], self.idle_chaserin.my_id, self.ox, self.oy)
            self.is_command_path = False
        if self.is_command_exit:
            self.CAcommand = command_set_exit(self.running_chaserin.get_angle(
            ), self.running_chaserin.get_pos(), self.running_chaserin.my_id, self.ox, self.oy)
            self.is_command_exit = False
        self.CAcommand.task.timestamp = self.get_clock().now().to_msg().sec
        self.CAcommand.task.route_id = self.mission_id
        self.publish_CAcommand.publish(self.CAcommand)

        self.get_logger().info("MESSAGE : Command: " + format(self.CAcommand)+"sent")

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
