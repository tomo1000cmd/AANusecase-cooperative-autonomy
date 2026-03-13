#!/usr/bin/env python3

from pickle import TRUE
from re import T
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from agent_msgs.msg import VehicleStatus, CAcommand
from agent_msgs.srv import RobotIdle, RouteStarted, RouteFinished,StandardAction
from lely_msgs.msg import RobotStatus, RouteStatus, RobotPose
from agent_msgs.srv import StartRoute, SetStatus
from lely_msgs.msg import StartRouteCommand, SetOperationStatus
from math import sin, cos, radians
from agent_msgs.msg import Task
import time,os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from rosgraph_msgs.msg import Clock




  

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
        self.my_id = self.get_parameter('my_id').value

        # Initialize variables
        self.tasklist = []
        self.tasklist_regular = []
        self.tasklist_takeover = []
        self.robot_status = RobotStatus()
        self.route_status = RouteStatus()
        self.is_new_robot_status = False
        self.is_new_route_status = False

        self.messagecounter = [0] * 9
        self.liveness = False

        self.nextroute = 0
        self.currenttask = Task()

        self.tasklist_regular = self.create_reagular_timetable(self.my_id)


        self.Tasklist_ignore_pretask(self.tasklist_regular, self.get_clock().now().to_msg().sec)
        self.tasklist = self.tasklist_regular


        self.CAcommand = CAcommand()
        self.is_CAcommand_available = False

        self.is_take_over = False

        # Create subscriber(s)
        #lely interface
        self.lely_status_sub = self.create_subscription(
            RobotStatus, 'robot_status', self.on_robot_status_msg, 10)
        self.route_status_sub = self.create_subscription(
            RouteStatus, 'route_status', self.on_route_status_msg, 10)
        #agent interface
        self.CAcommand_sub = self.create_subscription(
            CAcommand, 'CA_command', self.on_CA_command_msg, 10)
    
        
        # Create publisher(s)
        #agent interface
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, 'vehicle_status', 10)
        #lely interface
        self.set_operation_status_pub = self.create_publisher(
            SetOperationStatus, 'set_operation_status', 10)
        self.start_route_command_pub = self.create_publisher(
            StartRouteCommand, 'start_route_command', 10)

        
        

        # Create service(s) -- conditions
        self.robot_idle_service = self.create_service(
            RobotIdle, "robot_idle", self.on_robot_idle_srv)
        self.route_started_service = self.create_service(
            RouteStarted, "route_started", self.on_route_started_srv)
        self.route_finished_service = self.create_service(
            RouteFinished, "route_finished", self.on_route_finished_srv)
        self.robot_available_service = self.create_service(
            StandardAction, "robot_available", self.on_robot_available_srv)

        self.is_time_passed_service = self.create_service(
            StandardAction, "is_time_passed", self.on_is_time_passed_srv)
        self.is_event_get_service = self.create_service(
            StandardAction, "is_event_get", self.on_event_get_srv)

        self.is_CAcommand_get_service = self.create_service(
            StandardAction, "is_CAcommand_get", self.on_CAcommand_get_srv)
            
        # Create service(s) -- actions
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
    


        # Finally
        self.get_logger().info("initialized1")

    """
        Subscribers callback ------------------------------------------------------------------------

    """

    def on_robot_status_msg(self, msg):
        """
        Callback function that gets called on receiving a RobotStatus message.
        """
        self.robot_status = msg
        self.is_new_robot_status = True
        if self.liveness == False:
            self.messagecounter = [0] * 9
            self.liveness == True

        self.messagecounter[0]+=1
        

    def on_route_status_msg(self, msg):
        """
        Callback function that gets called on receiving a RouteStatus message.
        """
        self.route_status = msg
        self.is_new_route_status = True
        self.get_logger().info("[Node:] %s" %self.route_status) 

    
    def on_CA_command_msg(self, msg):
        """
        Callback function that gets called on receiving a CAcommand message.
        """
        if msg.vehicle_id == self.my_id:
            self.CAcommand = msg
            self.is_CAcommand_available = True

    """
        BT conditions service callback------------------------------------------------------------------------

    """


    def on_CAcommand_get_srv(self, request, response):
        """
        Callback function that gets called on receiving a isCA command get request.
        """

        response.success = False
        if self.is_CAcommand_available:
            response.success = True
        return response

    def on_robot_idle_srv(self, request, response):
        """
        Callback function that gets called on receiving a RobotIdle request.
        """

        response.success = False
        if self.is_new_robot_status:
            missionstate = str(self.robot_status.missionControllerState)
            ## if statement to see if the robot is idle
            if ((missionstate == "MANUAL" or 
                                missionstate == "IDLE" or 
                                missionstate == "SERVICE" ) and
                                self.robot_status.chargerConnected == True and
                                self.robot_status.operationStatus == "out" 
                                ):
                response.success = True
            else:
                ## if robot is still not idle when the time has passed a cetarin task ignore the task
                
                self.Tasklist_ignore_pretask(self.tasklist_regular, self.get_clock().now().to_msg().sec)
            self.is_new_robot_status = False
        return response

    def on_route_started_srv(self, request, response):
        """
        Callback function that gets called on receiving a RouteStarted request. #NOT USED YET
        """
        if self.route_status:
            response.success = (self.route_status.status == "started"
                                )
        return response

    def on_route_finished_srv(self, request, response):
        """
        Callback function that gets called on receiving a RouteFinished request. #NOT USED YET
        """

        response.success = False
        if self.route_status:
            success = (self.route_status.status == "finished succesfully")
            failure = (self.route_status.status == "finished with error")
            response.success = success or failure
        return response

    def on_robot_available_srv(self, request, response):
        """
        Callback function that checks if the robot are online at all time
        using a array of int to work as a linear buffer and constantly compare the first index with the last 
        give about the (length of the array * the executing interval of the bt) as its expire time. 
        In this case, if the system do not hear message from the lely_collecter for 4.5 s, the system will determine the collector as offline.   
        """

        response.success = False
        if self.messagecounter[0] != self.messagecounter[8]:
            response.success = True
        else :
            self.liveness = False

        for i in range(8):
            self.messagecounter[8-i] = self.messagecounter[7-i]

        return response

    def on_is_time_passed_srv(self, request, response):
        """
        Callback function that gets called on receiving a is_time_passed request.
        """

        response.success = False
        timestamp = self.get_clock().now().to_msg().sec
        for task in self.tasklist:
            if task.timestamp < timestamp and (task.task_status == Task.TASK_STATUS_TODO or task.task_status == Task.TASK_STATUS_TODO_TAKEOVER):            
                self.currenttask = task
                self.nextroute = task.route_id
                response.success = True
                

        return response
        
    def on_event_get_srv(self, request, response):
        """
        Callback function that gets called on receiving a is_event_get request.
        """
        
        response.success = False
        if self.is_new_route_status:
            response.success = True
        if self.is_new_robot_status:    
            if self.robot_status.missionControllerState == "ERROR": # keep sending 
            #publish error to mlA 
                response.success = True
 
        return response
        

  
  
    """
        BT actions service callback------------------------------------------------------------------------

    """
       
    def on_send_vehiclestatus_srv(self, request, response):
        """
        Callback function that gets called on receiving a send_vehiclestatus request.
        """
        
        self.pub_vehicle_status()
        response.success = True
        return response

    def on_execute_command_srv(self, request, response):
        """
        Callback function that gets called on receiving a execute_command request.
        """

        timestamp = self.get_clock().now().to_msg().sec

        if(self.CAcommand.operation == CAcommand.OPERATION_ADD_TIMETABLE): # 1 add 2 remove 3 alter 4 resume
            new_tasklist = []

            if self.tasklist == self.tasklist_regular:    
                for task in self.tasklist:
                    if task.timestamp <= timestamp:
                        new_tasklist.append(task)
                self.tasklist = new_tasklist
            missionstate = str(self.robot_status.missionControllerState)
            ## if statement to see if the robot is idle
            if ((missionstate == "MANUAL" or 
                                missionstate == "IDLE" or 
                                missionstate == "SERVICE" ) and
                                self.robot_status.chargerConnected == True and
                                self.robot_status.operationStatus == "out" 
                                ):
                newtask = Task()
                newtask.timestamp = timestamp+2*60
                newtask.route_id = self.CAcommand.task.route_id
                newtask.task_status = self.CAcommand.task.task_status
                self.tasklist.append(newtask)
            self.is_take_over = True

        
        if(self.CAcommand.operation == CAcommand.OPERATION_ALTER_TIMETABLE): # 1 add 2 remove 3 alter 4 resume
            self.tasklist_takeover = self.create_takeover_timetable(self.my_id, self.tasklist, timestamp)
            self.Tasklist_block_area(self.tasklist_takeover,self.CAcommand.blockedareas)
            self.tasklist = self.tasklist_takeover
            self.is_take_over = True

        # it could be better if it can keep the previous finished tasks instead of rewriting everthing     
        if(self.CAcommand.operation == CAcommand.OPERATION_RESUME_TIMETABLE): # 1 add 2 remove 3 alter 4 resume
            self.tasklist = self.tasklist_regular
            self.Tasklist_unblock_area(self.tasklist)
            self.Tasklist_ignore_pretask(self.tasklist,timestamp)
            self.is_take_over = False

            
            #block_area(self.tasklist,self.CAcommand.blockedareas)
        self.is_CAcommand_available = False
        response.success = True
        return response

    def on_update_status_srv(self, request, response):
        """
        Callback function that gets called on receiving a update_status request.
        """

        if self.route_status.status == "started":
            for task in reversed(self.tasklist):
                if task.route_id == self.currenttask.route_id and task.timestamp == self.currenttask.timestamp: 
                    task.task_status = Task.TASK_STATUS_STARTED
                    self.currenttask.task_status = Task.TASK_STATUS_STARTED
                    
                    break

        if self.route_status.status == "finished successfully":
            for task in reversed(self.tasklist):
                if task.route_id == self.currenttask.route_id and task.timestamp == self.currenttask.timestamp: 
                    task.task_status = Task.TASK_STATUS_SUCCESS
                    self.currenttask.task_status = Task.TASK_STATUS_SUCCESS
                 
                    break
                    
        if self.route_status.status == "finished with error" or self.robot_status.missionControllerState == "ERROR" :   
            for task in reversed(self.tasklist):
                if  task.route_id == self.currenttask.route_id and task.timestamp == self.currenttask.timestamp:     
                    task.task_status = Task.TASK_STATUS_FAILURE
                    self.currenttask.task_status = Task.TASK_STATUS_FAILURE
                    self.is_new_robot_status = False
                    break
                    #publish error to MLA
        self.is_new_route_status = False
        response.success = True
        return response
    
    def on_set_status_srv(self, request, response):
        """
        Callback function that gets called on receiving a SetStatus request.
        """

        msg = SetOperationStatus()
        msg.status = request.status
        self.set_operation_status_pub.publish(msg)
        self.set_operation_status_pub.publish(msg)
        response.success = True
        return response

    def on_start_route_srv(self, request, response):
        """
        Callback function that gets called on receiving a StartRoute request.
        """

        msg = StartRouteCommand()
        msg.id = self.nextroute
        #self.start_route_command_pub.publish(msg)
        self.start_route_command_pub.publish(msg)
        response.success = True
        return response    


    """
        Standard membership functions------------------------------------------------------------------------

    """


    def timestamp_from_timestring(self,timestring):
        # " 20:28:54"
        timestring_full = time.strftime('%Y-%m-%d ', time.localtime(float(self.get_clock().now().to_msg().sec))) + timestring
        timeArray = time.strptime(timestring_full,  "%Y-%m-%d %H:%M:%S")
        timestamp = time.mktime(timeArray)
        if timestamp <= 0:
            timestamp = 0
        return int(timestamp)

    def create_reagular_timetable(self,vehicle_id):
        tasklist = []
        bringup_dir = get_package_share_directory('voa')
        path= bringup_dir + '/config/' + 'schedule_' + vehicle_id + "_regular" + '.xml'
        
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


    def create_takeover_timetable(self,vehicle_id, tasklist, currtimestamp):
        
        tasklist_takeover = []
        
        for task in tasklist:
            if task.timestamp <= currtimestamp:
                tasklist_takeover.append(task)

            
        bringup_dir = get_package_share_directory('voa')
        path= bringup_dir + '/config/' + 'schedule_' + vehicle_id + "_takeover" + '.xml'
        
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

    def Tasklist_unblock_area(self,tasklist):
        """
        This Function is to block the route base on the id list from the CA
        """
        for task in tasklist:
            if(task.task_status == "Blocked"):
                    task.task_status = Task.TASK_STATUS_TODO

    def Tasklist_block_area(self,tasklist,BlockedRoutedidList):
        """
        This Function is to block the route base on the id list from the CA
        """
        for task in tasklist:
            for BlockedRoutedid in BlockedRoutedidList:
                if task.route_id == BlockedRoutedid:
                    task.task_status = "Blocked"

    def Tasklist_ignore_pretask(self,tasklist, currtimestamp):
        """
        This Function is to ignore the previous task that has been passed
        """
        for task in tasklist:
            if task.timestamp < currtimestamp and task.task_status == Task.TASK_STATUS_TODO:
                task.task_status = "ignored"

    def Tasklist_reset_task(self,tasklist):
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
            vehicle_status.header.frame_id = 'map'

            vehicle_status.vehicle_id = str(self.my_id)

            pose = Pose()
            pose.position.x = self.robot_status.robotPose.x
            pose.position.y = self.robot_status.robotPose.y
            pose.position.z = 0.0

            quat = quaternion_from_yaw(self.robot_status.robotPose.angle)

            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            vehicle_status.pose = pose
            
            vehicle_status.status.charger_connected = self.robot_status.chargerConnected
            vehicle_status.status.remaining_power = self.robot_status.percentageBattery
            vehicle_status.status.controller_status = self.robot_status.missionControllerState
            vehicle_status.status.operation_status = self.robot_status.operationStatus
            vehicle_status.status.route_id = self.robot_status.currentRouteId

            vehicle_status.is_takeover = self.is_take_over
            vehicle_status.task_list = self.tasklist
            if self.currenttask.task_status:
                vehicle_status.status.route_status = self.currenttask.task_status
            self.vehicle_status_pub.publish(vehicle_status)
            self.get_logger().info("[Node]: vehicle_status sent")
            

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
