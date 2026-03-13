#!/usr/bin/env python3

from tkinter import INSIDE
import rclpy
import random
from rclpy.node import Node
from agent_msgs.srv import StandardAction, ControlAvailable, GetControl
from agent_msgs.msg import VehicleStatus, CAcommand, OperationControl, Status
from agent_msgs.msg import Task
from geometry_msgs.msg import Pose
from agent_msgs.srv import StartRoute, SetStatus
import copy
import umsgpack
from agent_msgs.srv import RobotIdle, RouteStarted, RouteFinished, StandardAction
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped

# lely-case


#from lely_msgs.msg import RobotStatus, RouteStatus, RobotPose

#from lely_msgs.msg import StartRouteCommand, SetOperationStatus

# h2-case
from h2trac_msgs.msg import RobotStatus, SetMission, SetMotion, MissionStatus, Point2d

from math import sin, cos, radians
import array
import time
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from rosgraph_msgs.msg import Clock
import numpy as np
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Time
from helper import generate_course_chaser_file, generate_course_harvest_file, calc_distance, generate_path_H, quart_to_rpy, calculate_angle, calc_turn_command, calculate_dth, generate_path_V, calculate_front_wheel_angle, calculate_offset_point, calculate_front_wheel_angle_and_speed

L_ROBOT = 0.6
W_ROBOT = 0.455  # 0.425 for real robot
HARVESTER_ID = "Robot1"

# @class VoaBtHandler
#
#  @brief This class represents a handler for a Robot Operating System 2 (ROS2) node in a robotic harvesting system.
#
#  The `VoaBtHandler` class provides the functionality for a chaser bin robot to follow a harvester robot in a field.
#  The class inherits from the Node class of ROS2, allowing it to interact with other ROS2 nodes in the system.
#
#  It has methods to control the speed and direction of the chaser bin robot based on the position of the harvester robot,
#  using both a simpler chase method and a more complex method involving a proportional-integral controller.
#
#  The class also manages the bin's contents, tracking how long it has been since the bin was last emptied and using this
#  to estimate how full the bin is.
#
#  @note The specific functionalities of this class may vary depending on additional methods and attributes not detailed here.
#


class VoaBtHandler(Node):
    """
    This class is a ROS2 node.
    It subscribes to various robot status topics, provides queries on the status,
    and republishes these statuses combined in a VehicleStatus message.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('voa_bt_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.declare_parameter('use_pre_set_task_list')
        self.use_pre_set_task_list = self.get_parameter('use_pre_set_task_list').value
        self.my_id = self.get_parameter('my_id').value
        self.is_sim = self.get_parameter('use_sim_time').value
        self.get_logger().info("Simulation mode: " + str(self.is_sim))
        # Initialize variables
        self.tasklist = []
        self.tasklist_regular = []
        self.tasklist_takeover = []
        self.robot_status = RobotStatus()
        self.mission_status = MissionStatus()
        self.is_new_robot_status = False

        # tf2 listener
        buffer_time = Time()
        buffer_time.sec = 2
        self.tf_buffer = Buffer(buffer_time)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.on_frame_listener)
        self.Robot_frames = {}

        # message_counter for liveness check
        self.messagecounter = [0] * 9
        self.liveness = False
        self.control_available = False
        #self.nextroute = 0
        self.current_task = Task()

        #self.tasklist_regular = self.create_reagular_timetable(self.my_id)

        # self.Tasklist_ignore_pretask(self.tasklist_regular, self.get_clock().now().to_msg().sec)
        # self.tasklist = self.tasklist_regular

        self.CAcommand = CAcommand()
        self.CAcommand_pre = CAcommand()
        self.is_CAcommand_available = False
        #
        self.is_harvesting = False
        self.is_chasing = False
        # self.is_take_over = False

        # Create subscriber(s)
        # interface
        self.robot_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.on_robot_status_msg, 10)
        self.route_status_sub = self.create_subscription(
            MissionStatus, 'mission_status', self.on_mission_status_msg, 10)
        # agent interface
        self.CAcommand_sub = self.create_subscription(
            CAcommand, 'CA_command', self.on_CA_command_msg, 10)
        self.operation_control_sub = self.create_subscription(
            OperationControl, 'operation_control', self.on_operation_control_msg, 10)

        # Create publisher(s)
        # agent interface
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, 'vehicle_status', 10)
        # lely interface
        self.set_mission_pub = self.create_publisher(
            SetMission, 'set_mission', 10)
        self.set_motion_pub = self.create_publisher(
            SetMotion, 'set_motion', 10)

        # Create service(s) -- conditions
        self.command_available_service = self.create_service(
            ControlAvailable, "control_available", self.on_control_available_srv)

        self.robot_idle_service = self.create_service(
            RobotIdle, "robot_idle", self.on_robot_idle_srv)

        self.robot_available_service = self.create_service(
            StandardAction, "robot_available", self.on_robot_available_srv)

        self.is_time_passed_service = self.create_service(
            StandardAction, "is_time_passed", self.on_is_time_passed_srv)
        self.is_event_get_service = self.create_service(
            StandardAction, "is_event_get", self.on_event_get_srv)

        self.is_CAcommand_get_service = self.create_service(
            StandardAction, "is_CAcommand_get", self.on_CAcommand_get_srv)

        # Create service(s) -- actions
        self.get_command_service = self.create_service(
            GetControl, "get_control", self.on_get_control_srv)

        self.send_vehiclestatus_service = self.create_service(
            StandardAction, "send_vehiclestatus", self.on_send_vehiclestatus_srv)

        self.update_status_service = self.create_service(
            StandardAction, "update_status", self.on_update_status_srv)

        self.set_status_service = self.create_service(
            SetStatus, "set_status", self.on_set_status_srv)
        self.start_route_service = self.create_service(
            StartRoute, "start_route", self.on_start_route_srv)

        self.execute_command_service = self.create_service(
            StandardAction, "Execute_command", self.on_execute_command_srv)

        self.bringup_dir = get_package_share_directory('voa')
        if self.is_sim:

            path = self.bringup_dir + '/config/' + 'harvester_coordinates' + '.xml'
        else:
            path = self.bringup_dir + '/config/' + 'lab' + '.xml'

        self.get_logger().info("Path XML :" + path)
        self.harvest_course = generate_course_harvest_file(path)
        self.chaser_course = generate_course_chaser_file(path)
        ## Robot id for harvester
        if self.my_id == HARVESTER_ID:
            felid_x = [self.harvest_course[0][0]
                       [0], self.harvest_course[0][1][0]]
        else:
            felid_x = [self.chaser_course[0][0]
                       [0], self.chaser_course[0][1][0]]
        self.felid_x_min = min(felid_x)
        self.felid_x_max = max(felid_x)
        self.current_mission = SetMission()


    

        # Finally
        self.get_logger().info("initialized1")

    """
        Subscribers callback ------------------------------------------------------------------------

    """

    def on_operation_control_msg(self, msg: OperationControl):
        """
        Callback function that gets called on receiving a OperationControl message.
        """
        if msg.vehicle_id == self.my_id:
            self.operation_status = msg.status
            self.control_available = True

    def on_robot_status_msg(self, msg: RobotStatus):
        """
        Callback function that gets called on receiving a RobotStatus message.
        """
        self.robot_status = msg
        self.is_new_robot_status = True
        if self.liveness == False:
            self.messagecounter = [0] * 9
            self.liveness == True

        self.messagecounter[0] += 1

    def on_frame_listener(self):
        """
        Callback function that compute the frame to frame relationships
        """
        ##loading sequence for the task list
        if self.use_pre_set_task_list:
            self.get_logger().warn("Loading the preset task list")
            ##load the tasklist for Harvester look up the start time 
            start_time_haverster = self.readTaskListXml(self.bringup_dir + '/config/' + 'task_list_' + HARVESTER_ID + '.xml')[0].timestamp
            self.get_logger().warn("Harvester start time: " + str(start_time_haverster))
            self.get_logger().warn("My start time: " + str(self.get_clock().now().to_msg().sec))
            ##check if the system time for the first Harvester task is passed
            if self.get_clock().now().to_msg().sec < start_time_haverster:
                ## only when its not passed then we load the task list 
                self.tasklist = self.readTaskListXml(self.bringup_dir + '/config/' + 'task_list_' + self.my_id + '.xml')               
            else:
                self.get_logger().error("The preset task list can not load please restart the system")
            
            self.use_pre_set_task_list = False
        # Store frame names in variables that will be used to
        # compute transformations
        # Store frame names in a dictionary that will be used to compute transformations
        frame_relations = [
            "Robot0_base_link", "Robot1_base_link", "Robot2_base_link"
        ]
        to_frame_rel = 'map'

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            time_tf = Time()
            for from_frame_rel in frame_relations:
                try:
                    self.Robot_frames[from_frame_rel] = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        time_tf
                    )
                except BaseException as e:
                    pass
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        #self.get_logger().info( str(self.Robot_frames["Robot1_base_link"]))

    def on_mission_status_msg(self, msg: MissionStatus):
        """
        Callback function that gets called on receiving a MissionStatus message.
        """
        self.mission_status = msg

        #self.get_logger().info("[Node:] %s" %self.MissionStatus)

    def on_CA_command_msg(self, msg: CAcommand):
        """
        Callback function that gets called on receiving a CAcommand message.
        """
        if msg.vehicle_id == self.my_id:
            self.CAcommand = msg
            self.is_CAcommand_available = True

    """
        BT conditions service callback------------------------------------------------------------------------

    """

    def on_CAcommand_get_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a CA command get request.
        """

        response.success = False
        if self.is_CAcommand_available:
            response.success = True
        return response

    def on_robot_idle_srv(self, request: RobotIdle.Request, response: RobotIdle.Response) -> RobotIdle.Response:
        """
        Callback function that gets called on receiving a RobotIdle request.
        """

        response.success = True

        return response

    def on_robot_available_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that checks if the robot are online at all time
        using an array of int to work as a linear buffer and constantly compare the first index with the last 
        give about the (length of the array * the executing interval of the bt) as its expire time. 
        In this case, if the system do not hear message from the lely_collector for 4.5 s, the system will determine the collector as offline.   
        """

        response.success = False
        if self.messagecounter[0] != self.messagecounter[8]:
            response.success = True
        else:
            self.liveness = False

        for i in range(8):
            self.messagecounter[8-i] = self.messagecounter[7-i]

        return response

    def on_is_time_passed_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a is_time_passed request.
        """
        response.success = False
        timestamp = self.get_clock().now().to_msg().sec
        #self.get_logger().info("checking the time!")
        if len(self.tasklist)> 0 :
            for i in range(len(self.tasklist)):
                if self.tasklist[i].timestamp < timestamp and (self.tasklist[i].task_status == Task.TASK_STATUS_TODO or self.tasklist[i].task_status == Task.TASK_STATUS_TODO_TAKEOVER):            
                    ## set your mission based on the task if its a path
                    self.current_task.task_status = "Finished/interrupted"
                    self.current_task = self.tasklist[i]
                    
                    if self.tasklist[i].task_name == Task.TASK_API_HARVEST_CHASING:
                        if self.my_id == HARVESTER_ID:
                            self.is_harvesting = True
                            self.state = -1
                            self.is_ready = True
                        else:
                            self.is_chasing = True
                            if len(self.tasklist[i].task_info) > 0:
                                self.course_index = int(self.tasklist[i].task_info[0])

                            self.state = -1
                            self.is_ready = True

                        
                          
                    if self.tasklist[i].task_name == Task.TASK_API_PATH_FOLLOWING:
                        if self.my_id == HARVESTER_ID:
                            self.is_harvesting = False
                        else:
                            self.is_chasing = False
                        #translate the task to mission
                        
                        command = SetMission()
                        if self.mission_status.current_mission_status == MissionStatus.PATH_FOLLOWING:
                            
                            command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING
                            self.set_mission_pub.publish(command)

                        command.operation = SetMission.OPERATION_START_PATH_FOLLOWING
                        #decode byte from the string load the path to path 
                        #[speed , [path[],[]]]

                        decoded_msg = umsgpack.unpackb( bytes(self.tasklist[i].task_info))
                        ## load the msg in to a point2d array
                        path_=[]
                        for point in decoded_msg[1]:
                            temp = Point2d()
                            temp.cord_x=point[0]
                            temp.cord_y=point[1]
                            path_.append(temp)

                        command.path = path_
                        command.speed = decoded_msg[0]
                        self.current_mission = command

                        response.success = True
                    self.tasklist[i].task_status = Task.TASK_STATUS_STARTED
                    break
                


        return response

    def on_event_get_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a is_event_get request.
        """

        response.success = False

        return response

    def on_control_available_srv(self, request: ControlAvailable.Request, response: ControlAvailable.Response) -> ControlAvailable.Response:
        """
        Callback function that gets called on receiving a CommandAvailable request.
        """
        response.success = self.control_available

        return response

    """
        BT actions service callback------------------------------------------------------------------------

    """

    def on_get_control_srv(self, request: GetControl.Request, response: GetControl.Response) -> GetControl.Response:
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

    def on_send_vehiclestatus_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a send_vehiclestatus request.
        """
        """
        temp for autonomous actions 
        """
        try:
            if self.is_harvesting:
                self.excute_harvesting()
            if self.is_chasing:
                self.excute_Chasing()
        except BaseException as e:
            self.get_logger().warning(str(e))
        cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                   self.robot_status.kinematic_state.state.pose.position.y]
        if cur_pos[0] < -3 and cur_pos[0] > -6:
            if cur_pos[1] < 2 and cur_pos[1] > -2:
                self.clear_current_cabin()
                self.get_logger().info("start_dumping..")

        self.pub_vehicle_status()

        response.success = True
        return response

    def on_execute_command_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a execute_command request.

            1.  The robot receives the command from the CA, and it will first check the operation type.
            2.  If the operation type is SetMission, the robot will check if there is a path following mission ongoing.
            3.  If there is one, it will cancel the path following mission and start the new mission.
            4.  If the operation type is SetMotion, the robot will publish the SetMotion message to the robot.
            5.  If the operation type is StartHarvesting, the robot will start to harvest.
            6.  If the operation type is StopHarvesting, the robot will stop harvesting. 
        """

        timestamp = self.get_clock().now().to_msg().sec
        ## ---This are the functions for directly execute the command 
        if (self.CAcommand.operation == CAcommand.OPERATION_SET_MISSION):

            command = SetMission()
            # cancel the pre-path follow
            if self.mission_status.current_mission_status == MissionStatus.PATH_FOLLOWING and self.CAcommand.mission_operation == SetMission.OPERATION_START_PATH_FOLLOWING:
                self.is_chasing = False
                self.is_harvesting = False
                command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING
                self.set_mission_pub.publish(command)

            command.operation = self.CAcommand.mission_operation
            command.path = self.CAcommand.path
            command.speed = self.CAcommand.acceration
            self.set_mission_pub.publish(command)

        if (self.CAcommand.operation == CAcommand.OPERATION_SET_MOTION):
            command = SetMotion()
            command.angles = self.CAcommand.angles
            command.speeds = self.CAcommand.speeds
            self.set_motion_pub.publish(command)

        if (self.CAcommand.operation == CAcommand.OPERATION_START_HARVESTING):
            if self.my_id == HARVESTER_ID :
                self.is_harvesting = True
                self.state = -1
                self.is_ready = True
            else:
                self.is_chasing = True
                self.course_index = self.CAcommand.mission_operation

                self.state = -1
                self.is_ready = True

        if (self.CAcommand.operation == CAcommand.OPERATION_STOP_HARVESTING):
            if self.my_id == HARVESTER_ID:
                self.is_harvesting = False
            else:
                self.is_chasing = False
        
        ## --This are the functions for schedule 
        ## you can add a task to the current task list
        if (self.CAcommand.operation == CAcommand.OPERATION_ADD_TIMETABLE):
            if self.CAcommand.task.route_id not in (task.route_id for task in self.tasklist):
                self.tasklist.append(self.CAcommand.task)

        ## you can delete a task from the current task list 
        if (self.CAcommand.operation == CAcommand.OPERATION_REMOVE_TIMETABLE):
            if len(self.tasklist) > 0: 
                task_list_ = copy.deepcopy(self.tasklist)
                for task in task_list_:         
                    if task.route_id == self.CAcommand.task.route_id:
                        self.tasklist.remove(task)

        ## you can clear the entire current task lists
        if (self.CAcommand.operation == CAcommand.OPERATION_CLEAR_TIMETABLE):
            self.tasklist.clear()

                  
        ## you can load a full task list from memory or files :TODO 
        ## you can alter/correct a task based on its unique id 
        if (self.CAcommand.operation == CAcommand.OPERATION_ALTER_TIMETABLE):
            if len(self.tasklist) > 0: 
                for i in range (len(self.tasklist)):         
                    if self.tasklist[i].route_id == self.CAcommand.task.route_id:
                        self.tasklist[i] = self.CAcommand.task
        self.CAcommand_pre = copy.deepcopy(self.CAcommand)
        self.is_CAcommand_available = False
        response.success = True
        return response

    def on_update_status_srv(self, request: StandardAction.Request, response: StandardAction.Response) -> StandardAction.Response:
        """
        Callback function that gets called on receiving a update_status request.
        """

        response.success = True
        return response

    def on_set_status_srv(self, request: SetStatus.Request, response: SetStatus.Response) -> SetStatus.Response:
        """
        Callback function that gets called on receiving a SetStatus request.
        """

        response.success = True
        return response

    def on_start_route_srv(self, request: StartRoute.Request, response: StartRoute.Response) -> StartRoute.Response:
        """
        Callback function that gets called on receiving a StartRoute request.
        """
        self.set_mission_pub.publish(self.current_mission)

        response.success = True
        return response

    """
        Standard membership functions------------------------------------------------------------------------

    """

    def readTaskListXml(self,filename):
        task_list = []
        tree = ET.parse(filename)
        root = tree.getroot()
        for task_element in root.findall("task"):
            timestamp = task_element.find("timestamp").text
            task_name = task_element.find("task_name").text
            task_id = task_element.find("task_id").text
            task_info = task_element.find("task_info").text
            # Extract the list of values from the data_string
            try:
                list_start = task_info.find("[")
                list_end = task_info.rfind("]")
                list_contents = task_info[list_start + 1 : list_end]
                byte_values = [int(value) for value in list_contents.split(",")]

                # Create a byte array from the extracted values
                byte_array = array.array('B', byte_values)
            except:
                byte_array = array.array('B', [])
            temp = Task()
            temp.timestamp = int(timestamp)
            temp.task_name = task_name
            temp.route_id = int(task_id)
            temp.task_status = Task.TASK_STATUS_TODO
            temp.task_info = byte_array
            task_list.append(temp)
        return task_list

  
    def timestamp_from_timestring(self, timestring):
        # " 20:28:54"
        """ 
        1. This node is used to calculate the timestamp from the time string
        2. The time string format is " 20:28:54", " " is the key point
        2.1. The first character is " ", which means the time is in the same day as the current time
        2.2. If the first character is not " ", which means the time is in the next day of the current time
        3. The timestamp is used to judge whether the time is expired 
        """
        timestring_full = time.strftime(
            '%Y-%m-%d ', time.localtime(float(self.get_clock().now().to_msg().sec))) + timestring
        timeArray = time.strptime(timestring_full,  "%Y-%m-%d %H:%M:%S")
        timestamp = time.mktime(timeArray)
        if timestamp <= 0:
            timestamp = 0
        return int(timestamp)

    def create_reagular_timetable(self, vehicle_id):
        """     
        1. we get the path of the xml file.
        2. we use the tree to parse the xml file and get the root of the xml file.
        3. we go through the xml file and get the attributes of each task.
        4. we create a Task object for each task and add it to the tasklist. 
        """
        tasklist = []
        bringup_dir = get_package_share_directory('voa')
        path = bringup_dir + '/config/' + 'schedule_' + vehicle_id + "_regular" + '.xml'

        tree = ET.parse(path)
        root = tree.getroot()

        for data in root:
            for child in data:
                onedict = child.attrib
                taskId = onedict['taskId']
                startTime = onedict['startTime']
                task = Task()
                task.timestamp = self.timestamp_from_timestring(startTime)
                task.route_id = int(taskId)
                task.task_status = Task.TASK_STATUS_TODO
                tasklist.append(task)

        return tasklist

    def create_takeover_timetable(self, vehicle_id, tasklist, currtimestamp):
        '''
        1. The function takes in the current timestamp and the tasklist for the current vehicle
        2. It creates a new tasklist that will be returned after the function is done
        3. The function then iterates over the tasklist and adds all tasks that have a timestamp that is smaller than the current timestamp to the new tasklist
        4. The function then parses the xml file of the current vehicle and iterates over all tasks in the xml file
        5. The attributes of the tasks are extracted and added to a dictionary
        6. The dictionary is used to create a new task and the task is added to the new tasklist
        7. The new tasklist is returned 
        '''
        tasklist_takeover = []

        for task in tasklist:
            if task.timestamp <= currtimestamp:
                tasklist_takeover.append(task)

        bringup_dir = get_package_share_directory('voa')
        path = bringup_dir + '/config/' + 'schedule_' + vehicle_id + "_takeover" + '.xml'

        tree = ET.parse(path)
        root = tree.getroot()

        for data in root:
            for child in data:
                onedict = child.attrib
                taskId = onedict['taskId']
                startTime = onedict['startTime']
                task = Task()
                task.timestamp = self.timestamp_from_timestring(startTime)
                task.route_id = int(taskId)
                task.task_status = Task.TASK_STATUS_TODO_TAKEOVER
                if task.timestamp > currtimestamp:
                    tasklist_takeover.append(task)

        return tasklist_takeover

    def Tasklist_unblock_area(self, tasklist):
        """
        This Function is to block the route base on the id list from the CA
        """
        for task in tasklist:
            if (task.task_status == "Blocked"):
                task.task_status = Task.TASK_STATUS_TODO

    def Tasklist_block_area(self, tasklist, BlockedRoutedidList):
        """
        This Function is to block the route base on the id list from the CA
        """
        for task in tasklist:
            for BlockedRoutedid in BlockedRoutedidList:
                if task.route_id == BlockedRoutedid:
                    task.task_status = "Blocked"

    def Tasklist_ignore_pretask(self, tasklist, currtimestamp):
        """
        This Function is to ignore the previous task that has been passed
        """
        for task in tasklist:
            if task.timestamp < currtimestamp and task.task_status == Task.TASK_STATUS_TODO:
                task.task_status = "ignored"

    def Tasklist_reset_task(self, tasklist):
        """
        This Function is to reset all the task in the list to "todo"
        """
        for task in tasklist:
            task.task_status = Task.TASK_STATUS_TODO

    def pub_vehicle_status(self):
        """
        function that publish the vehicle status
        """
        if self.is_new_robot_status:  # Got a RobotStatus msg
            vehicle_status = VehicleStatus()

            timestamp = self.get_clock().now()
            vehicle_status.header.stamp = timestamp.to_msg()
            vehicle_status.header.frame_id = self.robot_status.kinematic_state.header.frame_id

            vehicle_status.vehicle_id = str(self.my_id)

            vehicle_status.pose = self.robot_status.kinematic_state.state.pose

            speed = round(np.sqrt(self.robot_status.kinematic_state.state.lateral_velocity_mps **
                          2+self.robot_status.kinematic_state.state.longitudinal_velocity_mps**2), 6)
            if (self.robot_status.kinematic_state.state.longitudinal_velocity_mps < 0):
                speed = -speed
            vehicle_status.status.current_speed = speed

            vehicle_status.status.charger_connected = self.mission_status.docked
            vehicle_status.status.remaining_power = self.mission_status.battery_status
            if (self.mission_status.current_mission_status == MissionStatus.PATH_FOLLOWING):
                vehicle_status.status.controller_status = Status.CONTROLLER_STATUS_OPERATIONAL
            elif (self.mission_status.current_mission_status == MissionStatus.MANUNAL_CONTROL):
                vehicle_status.status.controller_status = Status.CONTROLLER_STATUS_MANUAL
            if (self.is_harvesting or self.is_chasing):
                vehicle_status.status.controller_status = Status.CONTROLLER_STATUS_HARVESTING
            vehicle_status.status.storage_cabin_status = int(
                self.stoarge_cabin)
            vehicle_status.status.current_action = int(self.course_index)
            if self.mission_status.is_paused:
                vehicle_status.status.operation_status = Status.CONTROLLER_STATUS_PAUSED
            else:
                vehicle_status.status.operation_status = Status.CONTROLLER_STATUS_RUNNING

            vehicle_status.status.route_id = self.state
            vehicle_status.status.route_status = str(self.is_ready)
            vehicle_status.task_list = self.tasklist
            # if self.currenttask.task_status:
            #vehicle_status.status.route_status = self.currenttask.task_status
            self.vehicle_status_pub.publish(vehicle_status)
            self.get_logger().info("[Node]: vehicle_status sent")

    course_index = 0
    pre_course_index = 0
    goal = [float('inf'), float('inf')]
    state = -1
    is_ready = True
    cur_heading = 1
    target_heading = 0
    stoarge_cabin = 0
    thres_d_th = 3
    is_find_next_point = False

    pre_timestamp = 0
    err_i = 0

    def excute_harvesting(self):
        '''
        1. The robot will move in a square course with the length of 28m and width of 2m. The robot will move in a clockwise manner.
        2. For each line, the robot will move to the point that is the farthest from the start point. Then it will move to the point that is the nearest to the start point.
        3. The robot will be set to a fixed speed of 0.16m/s. 
        '''
        self.get_logger().info("[Node]: Harvesting....")

        #course = generate_square_course([-1.55, 9.065], 28, 2, 0.475,10.35)
        #course = generate_square_course(H_START_POINT, H_ROW, H_COL, H_ROW_DIS,H_COL_DIS)
        harvesting_speed = 0.16
        course = copy.deepcopy(self.harvest_course)
        cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                   self.robot_status.kinematic_state.state.pose.position.y]
        roll, pitch, yaw = quart_to_rpy(self.robot_status.kinematic_state.state.pose.orientation.x, self.robot_status.kinematic_state.state.pose.orientation.y,
                                        self.robot_status.kinematic_state.state.pose.orientation.z, self.robot_status.kinematic_state.state.pose.orientation.w)
        cur_angle = math.degrees(yaw)

        if cur_angle < 10 and cur_angle > -10:
            self.cur_heading = 1
        if cur_angle > 170 or cur_angle < -170:
            self.cur_heading = -1
        self.get_logger().info(self.cur_heading .__str__())
        # self.get_logger().info(course.__str__())
        # self.get_logger().info(cur_pos.__str__())
        #course_i = self.inside_out_index_first_field[self.course_index]
        course_i = self.course_index
        # calcu the felid with offset for stop and wait
        felid_offset = 0
        felid_x_max = self.felid_x_max - felid_offset
        felid_x_min = self.felid_x_min + felid_offset
        linked_marging = [1.5, 0.8]  # x, y
        chaserbin_frames = [
            "Robot0_base_link", "Robot2_base_link"
        ]
        # find the linked chaserbin frame and the distance

        linked_frame = None
        closest_vehicle = None

        # for key in self.Robot_frames:
        #     if key != self.robot_status.kinematic_state.header.frame_id:
        #         chaserbin_pos = [ self.Robot_frames[key].transform.translation.x,
        #                 self.Robot_frames[key].transform.translation.y]
        #         distance = np.linalg.norm(np.array(chaserbin_pos) - np.array(cur_pos))
        #         if distance < min_distance:
        #             min_distance = distance
        #             closest_vehicle = chaserbin_pos

        try:

            time_tf = Time()

            for frame in chaserbin_frames:
                to_frame_rel = frame
                from_frame_rel = self.robot_status.vehicle_id+"_base_link"
                try:
                    closest_vehicle = self.tf_buffer.lookup_transform(
                        to_frame_rel,
                        from_frame_rel,
                        time_tf
                    )
                    linked_frame = frame
                    r, p, yaw = quart_to_rpy(closest_vehicle.transform.rotation.x, closest_vehicle.transform.rotation.y,
                                             closest_vehicle.transform.rotation.z, closest_vehicle.transform.rotation.w)
                    # for x both direction and y only left side
                    if abs(closest_vehicle.transform.translation.x) < linked_marging[0] and closest_vehicle.transform.translation.y < linked_marging[1] \
                            and closest_vehicle.transform.translation.y > 0 and abs(yaw) < np.radians(20):
                        break
                    else:
                        linked_frame = None
                        closest_vehicle = None
                except BaseException:
                    pass
            self.get_logger().info(str(closest_vehicle))
            self.get_logger().info(str(linked_frame))
        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        if self.state == -1 and self.is_ready:

            curr_line = course[course_i]
            dis = []
            for point in curr_line:
                dis.append(calc_distance(cur_pos, point))
            self.goal = curr_line[dis.index(max(dis))]

            d_th, steer = calculate_dth(cur_angle, cur_pos, self.goal)
            self.get_logger().info("the d_th is "+d_th.__str__()+"the steer is "+steer.__str__())
            if linked_frame:
                command = SetMission()
                command.operation = SetMission.OPERATION_RESUME
                self.set_mission_pub.publish(command)
                self.get_logger().info("resuming the haverester")

            if d_th >= self.thres_d_th:

                command = calc_turn_command(d_th, steer)

                self.set_motion_pub.publish(command)

            elif linked_frame == None:
                command = SetMission()
                command.operation = SetMission.OPERATION_PAUSE
                self.set_mission_pub.publish(command)
                self.get_logger().info("stopping the haverester")

            else:

                self.get_logger().info(self.goal.__str__())
                distance_ = calc_distance(cur_pos, self.goal)

                path = generate_path_H(cur_pos, self.goal, int(distance_/0.3))

                self.get_logger().info(path.__str__() + ", the length of the path is : " + str(len(path)))
                command = SetMission()
                command.operation = SetMission.OPERATION_START_PATH_FOLLOWING
                for point in path:
                    temp = Point2d()
                    temp.cord_x = point[0]
                    temp.cord_y = point[1]
                    command.path.append(temp)
                command.speed = harvesting_speed
                self.set_mission_pub.publish(command)
                self.is_ready = False

        # stop the myself when its at the starting pos of the route until the one of the chaserbin is in linked pos ,
        # if linked_frame != None:
        #     pass
        # else:
        #     if self.pre_course_index != self.course_index and self.is_ready == False and self.is_find_next_point == False:
        #     #if self.haverster is inside of the felid_x_max
        #         command = SetMission()
        #         command.operation = SetMission.OPERATION_PAUSE
        #         self.set_mission_pub.publish(command)
        #         self.get_logger().info("stoping the haverester")

        if self.state == 1 and self.is_ready:

            self.get_logger().info("state 1")
            curr_line = course[course_i]
            # find the goal
            dis = []
            for point in curr_line:
                dis.append(calc_distance(cur_pos, point))
            self.goal = curr_line[dis.index(min(dis))]

            if cur_pos[1] - self.goal[1] > 0:

                self.goal[1] = self.goal[1] - W_ROBOT
                goal = copy.deepcopy(self.goal)
                goal[1] -= 10
            else:
                self.goal[1] = self.goal[1] + W_ROBOT
                goal = copy.deepcopy(self.goal)
                goal[1] += 10

            d_th, steer = calculate_dth(cur_angle, cur_pos, goal)
            self.get_logger().info("the d_th is "+d_th.__str__()+"the steer is "+steer.__str__())
            if d_th >= self.thres_d_th:
                #steer_ = cur_pos [0] < 5
                command = calc_turn_command(d_th, steer)

                self.set_motion_pub.publish(command)
            else:

                self.is_find_next_point = True
                self.is_ready = False

        if self.is_find_next_point:
            if cur_pos[1] - self.goal[1] > 0:
                goal = copy.deepcopy(self.goal)
                goal[1] -= 10
            else:
                goal = copy.deepcopy(self.goal)
                goal[1] += 10
            target_speed = harvesting_speed
            command = SetMotion()
            target_angle = calculate_front_wheel_angle(
                cur_pos, goal, cur_angle)
            command.speeds = np.array(
                [target_speed, target_speed, target_speed, target_speed], dtype="float")
            if calc_distance(cur_pos, self.goal) > L_ROBOT:
                command.angles = np.array(
                    [target_angle, target_angle, 0.0, 0.0], dtype="float")
            self.set_motion_pub.publish(command)

        if calc_distance(cur_pos, self.goal) <= 0.52 and self.mission_status.current_mission_status == MissionStatus.MANUNAL_CONTROL:

            if self.state == -1:
                self.course_index += 1
            self.state = -self.state
            self.is_find_next_point = False
            self.is_ready = True

        if self.course_index == len(course):
            self.is_harvesting = False
            self.course_index = 0

    def excute_Chasing_v2(self):
        '''
        Beta
        The robot chases a harvester by moving towards a point offset from the harvester's position. 
        The function calculates the position of the chasing point, sets the speed and direction of the robot, 
        and publishes the command to the robot's motion controller. 
        '''
        self.get_logger().info("[Node]: Chasing....")
        timestamp = self.get_clock().now().to_msg().sec

        cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                   self.robot_status.kinematic_state.state.pose.position.y]
        roll, pitch, yaw = quart_to_rpy(self.robot_status.kinematic_state.state.pose.orientation.x, self.robot_status.kinematic_state.state.pose.orientation.y,
                                        self.robot_status.kinematic_state.state.pose.orientation.z, self.robot_status.kinematic_state.state.pose.orientation.w)
        cur_angle = math.degrees(yaw)

        if cur_angle < 30 and cur_angle > -30:
            self.cur_heading = 1
        if cur_angle > 150 or cur_angle < -150:
            self.cur_heading = -1
        self.get_logger().info(self.cur_heading .__str__())
        # self.get_logger().info(course.__str__())
        # self.get_logger().info(cur_pos.__str__())
        offset_x = 0
        offset_y = 0
        target_pos = [self.Robot_frames["Robot1_base_link"].transform.translation.x,
                      self.Robot_frames["Robot1_base_link"].transform.translation.y]

        roll, pitch, yaw_target = quart_to_rpy(self.Robot_frames["Robot1_base_link"].transform.rotation.x, self.Robot_frames["Robot1_base_link"].transform.rotation.y,
                                               self.Robot_frames["Robot1_base_link"].transform.rotation.z, self.Robot_frames["Robot1_base_link"].transform.rotation.w)

        error_x = 0
        error_y = 0

        try:
            to_frame_rel = self.robot_status.vehicle_id+"_base_link"
            from_frame_rel = "Robot1_base_link"
            dis_error = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time()
            )
            self.get_logger().info(
                "The Harvester and Chaserbin distance in transformation: " + str(dis_error))
            error_x = dis_error.transform.translation.x - offset_x
            error_y = dis_error.transform.translation.y - offset_y

        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        scale_rotation_rate = 20.0
        angular_z = scale_rotation_rate * math.atan2(
            error_y,
            error_x)

        scale_forward_speed = 0.2
        linear_x = scale_forward_speed * math.sqrt(
            error_x ** 2 +
            error_y ** 2)

        command = SetMotion()

        target_speed = linear_x
        track_width = 0.16  # Assuming the distance between the wheels is 2.0 units
        target_angle = angular_z * track_width / 2.0
        target_angle = -target_angle
        command.speeds = np.array(
            [target_speed, target_speed, target_speed, target_speed], dtype="float")

        command.angles = np.array(
            [target_angle, target_angle, 0.0, 0.0], dtype="float")
        self.set_motion_pub.publish(command)

    def excute_Chasing(self):
        '''
        1.The same as the harvesting, 
        2.but the robot will get the location from the harvester
        3.base on the location it will use the pi controller to control its own speed
        '''
        self.get_logger().info("[Node]: Chasing....")
        chaserbin_speed = 0.16
        timestamp = self.get_clock().now().to_msg().sec
        course = copy.deepcopy(self.chaser_course)

        #course = generate_square_course([-0.95, 8.55], 28, 2, 0.475, 9.15)
        # course = generate_square_course(C_START_POINT, C_ROW, C_COL, C_ROW_DIS, C_COL_DIS)
        cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                   self.robot_status.kinematic_state.state.pose.position.y]
        roll, pitch, yaw = quart_to_rpy(self.robot_status.kinematic_state.state.pose.orientation.x, self.robot_status.kinematic_state.state.pose.orientation.y,
                                        self.robot_status.kinematic_state.state.pose.orientation.z, self.robot_status.kinematic_state.state.pose.orientation.w)
        cur_angle = math.degrees(yaw)

        if cur_angle < 30 and cur_angle > -30:
            self.cur_heading = 1
        if cur_angle > 150 or cur_angle < -150:
            self.cur_heading = -1
        self.get_logger().info(self.cur_heading .__str__())

        # self.get_logger().info(course.__str__())
        # self.get_logger().info(cur_pos.__str__())
        offset_x = 0.5
        offset_y = 0.475
        target_pos = [self.Robot_frames["Robot1_base_link"].transform.translation.x,
                      self.Robot_frames["Robot1_base_link"].transform.translation.y]

        is_harvester_in = (
            target_pos[0] < self.felid_x_max and target_pos[0] > self.felid_x_min)
        error = ((target_pos[0]-offset_x*self.cur_heading) -
                 cur_pos[0])*self.cur_heading
        # PID position CONTROL
        if self.mission_status.current_mission_status == MissionStatus.PATH_FOLLOWING:

            roll, pitch, yaw_ = quart_to_rpy(self.Robot_frames["Robot1_base_link"].transform.rotation.x, self.Robot_frames["Robot1_base_link"].transform.rotation.y,
                                             self.Robot_frames["Robot1_base_link"].transform.rotation.z, self.Robot_frames["Robot1_base_link"].transform.rotation.w)
            target_angle = math.degrees(yaw_)

            # try:
            #         time_tf = Time()
            #         to_frame_rel = self.robot_status.vehicle_id+"_base_link"
            #         from_frame_rel = "Robot1_base_link"
            #         dis_error = self.tf_buffer.lookup_transform(
            #             to_frame_rel,
            #             from_frame_rel,
            #             time_tf
            #         )
            #         self.get_logger().info("The Harvester and Chaserbin distance in transformation: " + str(dis_error))
            #         self.get_logger().info("The distance between the Harvester and chaserbin :  " + str(math.sqrt(dis_error.transform.translation.x**2 + dis_error.transform.translation.y **2)))
            #         #error = dis_error.transform.translation.x- offset_x
            #         #error = math.sqrt((dis_error.transform.translation.x- offset_x)**2 +(dis_error.transform.translation.y- offset_y)**2 )
            # except TransformException as ex:
            #     self.get_logger().warning(
            #         f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            #     return

            if target_angle < 30 and target_angle > -30:
                self.target_heading = 1  # left to right
            if target_angle > 150 or target_angle < -150:
                self.target_heading = -1  # right to left
            KP = 1.0
            KI = 1e-05
            dt = 0.2  # in s
            self.err_i += error
            control_signal = error * KP + self.err_i * KI * dt

            command = SetMission()
            command.operation = SetMission.OPERATION_CHANGE_SPEED

            command.speed = control_signal
            self.get_logger().info('PI controller, error = ' + error.__str__() +
                                   ", control = " + control_signal.__str__())

            self.set_mission_pub.publish(command)

        course_i = self.course_index

        if self.state == -1 and self.is_ready:

            self.err_i = 0
            curr_line = course[course_i]
            dis = []
            for point in curr_line:
                dis.append(calc_distance(cur_pos, point))
            self.goal = curr_line[dis.index(max(dis))]

            d_th, steer = calculate_dth(cur_angle, cur_pos, self.goal)
            self.get_logger().info("the d_th is "+d_th.__str__()+"the steer is "+steer.__str__())

            if d_th >= self.thres_d_th:
                #steer_ = cur_pos [0] < 5
                command = calc_turn_command(d_th, steer)

                self.set_motion_pub.publish(command)

                self.get_logger().info(target_pos .__str__())
                self.get_logger().info(
                    self.harvest_course[self.course_index] .__str__())

                # find the shortest point
                dis_harvester = []
                for point in self.harvest_course[self.course_index]:
                    dis_harvester.append(calc_distance(target_pos, point))
                self.get_logger().info("release the chaser when its < 0.4 :  " + str(min(dis_harvester)))
                if min(dis_harvester) < 0.4:
                    command = SetMission()
                    command.operation = SetMission.OPERATION_RESUME
                    self.set_mission_pub.publish(command)
                    self.get_logger().info("release the chaser")

            else:

                self.get_logger().info(self.goal.__str__())
                distance_ = calc_distance(cur_pos, self.goal)

                path = generate_path_H(cur_pos, self.goal, int(distance_/0.3))

                self.get_logger().info(path.__str__() + ", the length of the path is : " + str(len(path)))

                command = SetMission()
                command.operation = SetMission.OPERATION_START_PATH_FOLLOWING
                for point in path:
                    temp = Point2d()
                    temp.cord_x = point[0]
                    temp.cord_y = point[1]
                    command.path.append(temp)
                command.speed = chaserbin_speed
                self.set_mission_pub.publish(command)
                self.is_ready = False

        if self.state == 1 and self.is_ready:

            self.get_logger().info("state 1")
            curr_line = course[course_i]
            # find the goal
            dis = []
            for point in curr_line:
                dis.append(calc_distance(cur_pos, point))
            self.goal = curr_line[dis.index(min(dis))]
            if cur_pos[1] - self.goal[1] > 0:

                self.goal[1] = self.goal[1] - W_ROBOT
                goal = copy.deepcopy(self.goal)
                goal[1] -= 10
            else:
                self.goal[1] = self.goal[1] + W_ROBOT
                goal = copy.deepcopy(self.goal)
                goal[1] += 10

            d_th, steer = calculate_dth(cur_angle, cur_pos, goal)

            self.get_logger().info("the d_th is "+d_th.__str__()+"the steer is "+steer.__str__())

            if d_th >= self.thres_d_th:
                #steer_ = cur_pos [0] < 5
                command = calc_turn_command(d_th, steer)

                self.set_motion_pub.publish(command)

                self.get_logger().info(target_pos .__str__())
                self.get_logger().info(
                    self.harvest_course[self.course_index] .__str__())

                # find the shortest point
                dis_harvester = []
                for point in self.harvest_course[self.course_index]:
                    dis_harvester.append(calc_distance(target_pos, point))
                self.get_logger().info("release the chaser when its <0.4 :  " + str(min(dis_harvester)))
                if min(dis_harvester) < 0.40:
                    command = SetMission()
                    command.operation = SetMission.OPERATION_RESUME
                    self.set_mission_pub.publish(command)
                    self.get_logger().info("release the chaser")

            else:

                self.is_find_next_point = True
                self.is_ready = False

        if self.is_find_next_point:
            if cur_pos[1] - self.goal[1] > 0:
                goal = copy.deepcopy(self.goal)
                goal[1] -= 10
            else:
                goal = copy.deepcopy(self.goal)
                goal[1] += 10
            target_speed = chaserbin_speed
            command = SetMotion()
            target_angle = calculate_front_wheel_angle(
                cur_pos, goal, cur_angle)
            command.speeds = np.array(
                [target_speed, target_speed, target_speed, target_speed], dtype="float")
            if calc_distance(cur_pos, self.goal) > L_ROBOT:
                command.angles = np.array(
                    [target_angle, target_angle, 0.0, 0.0], dtype="float")
            self.set_motion_pub.publish(command)

        if calc_distance(cur_pos, self.goal) < 0.55 and self.mission_status.current_mission_status == MissionStatus.MANUNAL_CONTROL:
            if self.state == -1:
                command = SetMission()
                command.operation = SetMission.OPERATION_PAUSE
                self.set_mission_pub.publish(command)
                self.get_logger().info("stopping the chaserbin")
                self.course_index += 1
            self.state = -self.state
            self.is_find_next_point = False
            self.is_ready = True
        # sim cabin for chaserbin
        if self.state == -1 and is_harvester_in and abs(error) < 0.5 and calc_distance(cur_pos, target_pos) < 1:
            if self.stoarge_cabin < 100:
                if (timestamp - self.pre_timestamp) <= 1:
                    self.stoarge_cabin += (timestamp -
                                           self.pre_timestamp) * random.uniform(0.2, 0.9)
        self.pre_timestamp = timestamp

        if self.course_index == len(course):
            self.is_chasing = False
            self.course_index = 0

    def clear_current_cabin(self):
        """ 
        1. We use the current time to minus the previous time, and then get the time difference.
        2. If the time difference is greater than 1 second, we need to subtract the time difference from the current value of the cabin.
        3. If the time difference is less than or equal to 1 second, we need to subtract the time difference multiplied by 5 from the current value of the cabin.
        4. If the value of the cabin is less than 0, then set the value of the cabin to 0.
        5. Update the previous time value to the current time value.
          """
        timestamp = self.get_clock().now().to_msg().sec

        if self.stoarge_cabin > 0:
            if (timestamp - self.pre_timestamp) <= 1:
                self.stoarge_cabin -= (timestamp - self.pre_timestamp) * 5
        else:
            self.stoarge_cabin = 0
        self.pre_timestamp = timestamp


def quaternion_from_yaw(yaw_degrees):
    """
    Converts yaw (degrees) to quaternion
    """
    yaw = radians(yaw_degrees)
    x = 0.0
    y = 0.0
    z = sin(yaw * 0.5)
    w = cos(yaw * 0.5)

    return [x, y, z, w]


def main(args=None):

    rclpy.init(args=args)

    # Create the node
    voabt_handler = VoaBtHandler()

    # Spin the node so the callback functions are called.
    rclpy.spin(voabt_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    voabt_handler.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
