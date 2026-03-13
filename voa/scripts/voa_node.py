#!/usr/bin/env python3


import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Pose
import copy
import umsgpack
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped

# h2-case
from mas_msgs.msg import VehicleStatus, Task, Status,TaskList,CAcommand
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

class CommandHandler(Node):
    # """
    # This class is a ROS2 node.
    # It handles commands received from the CA.
    # """

    def __init__(self):
     
        # Class constructor to set up the node
       
        super().__init__('voa_node_')

        # Parameters
        self.declare_parameter('my_id',"Robot1")
        self.my_id = self.get_parameter('my_id').value
        self.is_sim = self.get_parameter('use_sim_time').value
        self.robot_namespace = self.get_namespace()
        self.robot_namespace = self.my_id 
        self.get_logger().info("Simulation mode: " + str(self.is_sim))



        # Initialize variables
        self.current_vehicle_status = VehicleStatus()
        self.tasklist = []
        self.current_task = Task()

        # Create subscriber(s)
        self.CAcommand_sub = self.create_subscription(
            CAcommand, '/server/ca_command', self.on_CA_command_msg, 10)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, self.robot_namespace+ '/' +'vehicle_status', self.on_vehicle_status, 10)
        # Create publisher(s)
        self.task_pub = self.create_publisher(
            Task, self.robot_namespace+ '/' +'set_task', 10)
        self.task_list_publisher = self.create_publisher(
            TaskList, self.robot_namespace+ '/' +'task_list', 10)
       



        # Create timer(s)
        self.timer_execute_task = self.create_timer(0.01, self.on_timer)
        self.timer_publish = self.create_timer(0.05, self.on_timer_publish)

        # Finally
        self.get_logger().info("initialized")

    def on_timer_publish(self):
        ## check if the current task is finished or interrupted
        current_task_id = self.current_vehicle_status.status.current_task.uuid
        for i in range(len(self.tasklist)):
            if self.tasklist[i].uuid != current_task_id and self.tasklist[i].task_status == Task.TASK_STATUS_STARTED:
                self.tasklist[i].task_status = "Finished/interrupted"

        task_list = TaskList()
        task_list.header.stamp = self.get_clock().now().to_msg()
        task_list.header.frame_id = self.my_id
        task_list.task_list = self.tasklist
        self.task_list_publisher.publish(task_list)
    
    def on_timer(self):
        timestamp = self.get_clock().now().to_msg().sec
        #self.get_logger().info("checking the time!")
        if len(self.tasklist)> 0 :
            for i in range(len(self.tasklist)):
                if self.tasklist[i].timestamp < timestamp and (self.tasklist[i].task_status == Task.TASK_STATUS_TODO or self.tasklist[i].task_status == Task.TASK_STATUS_TODO_TAKEOVER):            
                    command = self.tasklist[i]
                    self.task_pub.publish(command)
                    break

    def on_CA_command_msg(self, msg:CAcommand):
        if self.my_id == msg.vehicle_id:
            timestamp = self.get_clock().now().to_msg().sec
            self.CAcommand = msg
            ## ---This are the functions for directly execute the command 
            if (self.CAcommand.operation == CAcommand.OPERATION_EXECUTE_NOW):
                command = Task()
                self.task_pub.publish(command)

            ## --This are the functions for schedule 
            ## you can add a task to the current task list
            if (self.CAcommand.operation == CAcommand.OPERATION_ADD_TIMETABLE):
                if self.CAcommand.task.uuid not in (task.uuid for task in self.tasklist):
                    self.tasklist.append(self.CAcommand.task)

            ## you can delete a task from the current task list 
            if (self.CAcommand.operation == CAcommand.OPERATION_REMOVE_TIMETABLE):
                if len(self.tasklist) > 0: 
                    task_list_ = copy.deepcopy(self.tasklist)
                    for task in task_list_:         
                        if task.uuid == self.CAcommand.task.uuid:
                            self.tasklist.remove(task)

            ## you can clear the entire current task lists
            if (self.CAcommand.operation == CAcommand.OPERATION_CLEAR_TIMETABLE):
                self.tasklist.clear()

                    
            ## you can load a full task list msg 
            if (self.CAcommand.operation == CAcommand.OPERATION_REPLACE_ENTIRE_TIMETABLE):
                self.tasklist = self.CAcommand.tasks

            ## you can alter/correct a task based on its unique id 
            if (self.CAcommand.operation == CAcommand.OPERATION_ALTER_TIMETABLE):
                if len(self.tasklist) > 0: 
                    for i in range (len(self.tasklist)):         
                        if self.tasklist[i].uuid == self.CAcommand.task.uuid:
                            self.tasklist[i] = self.CAcommand.task

           

    def on_vehicle_status(self, msg:VehicleStatus):
        self.current_vehicle_status = msg
        task_status = self.current_vehicle_status.status.current_task.task_status
        ## loop the tasklist and update the status of the task
        for i in range(len(self.tasklist)):
            if self.tasklist[i].uuid == self.current_vehicle_status.status.current_task.uuid:
                self.tasklist[i].task_status = task_status



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
