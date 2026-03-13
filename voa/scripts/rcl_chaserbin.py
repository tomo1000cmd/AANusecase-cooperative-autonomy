#!/usr/bin/env python3


#This is a simple version of robot cooperation layer (RCL) for the VOA. This is a way that could achieve some general functions for the DurableCase project.

import rclpy
import random
from rclpy.node import Node
import copy
import umsgpack
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped

# h2-case
from h2trac_msgs.msg import RobotStatus, SetMission, SetMotion, MissionStatus, Point2d

from mas_msgs.msg import VehicleStatus, Task, Status, TaskList
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
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid
import time
from helper import generate_course_chaser_file, generate_course_harvest_file, calc_distance, generate_path_H, quart_to_rpy, command_set_exit, calc_turn_command, calculate_dth, command_set_path, calculate_front_wheel_angle, command_cancel_path, decompose_euclidean_path,calculate_distance_path

SCENARIO = 1 #1 is full simulation 2 is lab scale

L_ROBOT = 0.65
W_ROBOT = 0.455  # 0.425 for real robot
HARVESTER_ID = "Robot1"
if SCENARIO == 1:
    AVERAGE_HARVESTER_RATE = 0.65   # in %/s
    STD_HARVESTER_RATE = 0.3       # in %/s
    DEFAULT_AVERAGE_HARVESTER_SPEED = 0.16  # average harvest speed in m/s
    FIELD_DIAMENTION_X = [-0.75, 8.0]
    FIELD_DIAMENTION_Y = [9.3, -4.0]
    AVERAGE_HARVESTER_RATE = 0.65   # in %/s
elif SCENARIO == 2:
    AVERAGE_HARVESTER_RATE = 2.15    # in %/s
    STD_HARVESTER_RATE = 0.0       # in %/s
    DEFAULT_AVERAGE_HARVESTER_SPEED = 0.08  # average harvest speed in m/s
        # Constants for scaled lab
    FIELD_DIAMENTION_X = [-1.8, 2.0]
    FIELD_DIAMENTION_Y = [0.4125, -1.7875]

HARVESTER_ACC_OFFSET = 0.0      # offset for harvester acceleration in m/s^2
AVERAGE_CHASERBIN_SPEED = 0.4   # average chaserbin speed in m/s
MAX_CHASERBIN_STORAGE = 100     # in %
CHASERBIN_SWITCH_THRESHOLD = 90  # in #

LEFT_ENTER_POINT = FIELD_DIAMENTION_X[0] - 1.0
RIGHT_ENTER_POINT = FIELD_DIAMENTION_X[1] + 1.0

H_CONTROL_MODE = -1
S_CONTROL_MODE = 1

class ControlMode:
    chasing = 1
    idle = 0
    switch = 2
    path_following = 3

class ComposeTask:
    COMPOSED_TASK_TYPE_HARVESTING_CHASING = 0
    COMPOSED_TASK_TYPE_SWITCHING = 1

class MotionTask:
    MOTION_TASK_TYPE_SET_MOTION = 0
    MOTION_TASK_TYPE_PAUSE = 1
    MOTION_TASK_TYPE_RESUME = 2
    MOTION_TASK_TYPE_CHANGE_PATH_SPEED = 3
# this node is only built for H2trac use case the node will be responsible for the following tasks:
# Robot
class RobotCooperationLayer(Node):
    ## First step: bridge the message and service types from the mas_msgs package into the scope of the individual robot supervision layer.


    def __init__(self):

        super().__init__('RobotCooperationLayer_chaserbin_node')
        # simtime =Parameter("use_sim_time", Parameter.Type.BOOL, True)
        # self.set_parameters([simtime])  
        self.is_sim = self.get_parameter('use_sim_time').value
        # Parameters
        self.declare_parameter('my_id',"Robot0")
        self.get_logger().info("Simulation mode: " + str(self.is_sim))
        self.my_id = self.get_parameter('my_id').value
        self.robot_namespace = self.get_namespace()
        self.robot_namespace = self.my_id 
        # Initialize variables
        self.current_status = Status()
        self.robot_status = RobotStatus()
        self.is_linked = False
        self.current_low_level_system_status = MissionStatus()
        self.finished_row = []
        self.ox = []
        self.oy = []
        self.stoarge_cabin = 0
        self.current_harvester_row = 0
        self.course_index = 0
        self.path_following_goal = []
        # tf2 listener
        buffer_time = Time()
        buffer_time.sec = 2
        self.tf_buffer = Buffer(buffer_time)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.Robot_frames = {}
        self.linked_frame = None
        self.chasers_frames = ["Robot0_base_link", "Robot2_base_link"]
        self.control_mode = ControlMode.idle
        self.harvester_course_index = 0
        # Create subscriber(s)
        self.create_subscription(
            RobotStatus, self.robot_namespace+ '/' + 'robot_status', self.on_robot_status, 10)
        
        self.create_subscription(
            MissionStatus, self.robot_namespace+ '/' + 'mission_status', self.on_mission_status, 10)
        # Task execution
        self.create_subscription(
            Task, self.robot_namespace+ '/' + 'set_task', self.on_task, 10)
        ##debug for demo only
        self.create_subscription(VehicleStatus,HARVESTER_ID + '/' + 'vehicle_status', self.on_harvester_status, 10)

        # Create publisher(s)
        self.set_mission_pub = self.create_publisher(
            SetMission, self.robot_namespace+'/set_mission', 10)
        self.set_motion_pub = self.create_publisher(
            SetMotion, self.robot_namespace+'/set_motion', 10)
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, self.robot_namespace+ '/' +'vehicle_status', 10)
        self.publish_updatesMap = self.create_publisher(
            OccupancyGrid, 
            self.robot_namespace+ '/' + 'map', 
            10)
       
        self.bringup_dir = get_package_share_directory('voa')
        if self.is_sim:

            path = self.bringup_dir + '/config/' + 'harvester_coordinates' + '.xml'
            if SCENARIO == 2:
                path = self.bringup_dir + '/config/' + 'lab' + '.xml'

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
        self.nav_to_start_chaserbin = True
        
        self.average_harvester_speed = DEFAULT_AVERAGE_HARVESTER_SPEED
        ##very_debug for demo only
        if SCENARIO == 2:
            if self.my_id == "Robot0":
                self.stoarge_cabin = 80
        
                

        # Create timer(s)
        self.timer = self.create_timer(0.1, self.main_loop)

        # Finally
        self.get_logger().info("initialized")

    def publish_map(self):
        map_data = OccupancyGrid()
        map_data.header.stamp = self.get_clock().now().to_msg()
        map_data.header.frame_id = "map"
        map_data.info.resolution = 0.2 # in meters
        map_data.info.width = 110
        map_data.info.height = 110
        map_data.info.origin.position.x = -11.0
        map_data.info.origin.position.y = -11.0
        map_data.info.origin.position.z = 0.0
        map_data.info.origin.orientation.x = 0.0
        map_data.info.origin.orientation.y = 0.0
        map_data.info.origin.orientation.z = 0.0
        map_data.info.origin.orientation.w = 1.0
        map_data.data = [0] * 110 * 110
        for i in range(len(self.ox)):
            index_x = int((self.ox[i] - map_data.info.origin.position.x) / map_data.info.resolution)
            index_y = int((self.oy[i] - map_data.info.origin.position.y) / map_data.info.resolution)
                
            map_data.data[index_y * map_data.info.width + index_x] = 99 # obstacle
             

        self.publish_updatesMap.publish(map_data) 
    
    def main_loop(self):

        #time the main loop
        start_time = time.time()
        ## the code starts here====================================================================================================== 
        self.on_frame_listener()
        if len(self.Robot_frames)>=2:
            self.linked_frame = self.check_if_chaserbin_is_linked()
            self.dump_the_harvested_cabin()
            other_chasers = self.find_other_chasers()
            self.updata_map()
            self.publish_map()
            self.get_logger().info("self.current_harvester_row : " + str(self.current_harvester_row ))
            if self.control_mode == ControlMode.chasing:
                  ## only for real demo need to be scaled up for simulation
                cur_pos = [self.robot_status.kinematic_state.state.pose.position.x, self.robot_status.kinematic_state.state.pose.position.y]
                # is course_index is even 
                if self.course_index %2 == 0:
                    enter_side_index = 0
                    enter_side_offset = -0.9
                    if cur_pos[0] > 0.0:
                        enter_side_index = 1
                        enter_side_offset = 0.9
                elif self.course_index %2 == 1:
                    enter_side_index = 1
                    enter_side_offset = -0.9
                    if cur_pos[0] > 0.0:
                        enter_side_index = 0
                        enter_side_offset = 0.9
                current_enter_x = self.chaser_course[self.course_index][enter_side_index][0]+enter_side_offset
                current_enter_y = self.chaser_course[self.course_index][enter_side_index][1]
                euclidean_distance_ = calc_distance(cur_pos, [current_enter_x, current_enter_y])
                lateral_distance = abs(cur_pos[1] - current_enter_y)
                path = []
                self.get_logger().info("lateral_distance : " + str(lateral_distance))
                command = SetMission()
                self.get_logger().info("euclidean_distance_ : " + str(euclidean_distance_))
                if euclidean_distance_ < 0.2 or lateral_distance < 0.2:
                    if self.nav_to_start_chaserbin:
                        command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING
                        self.set_mission_pub.publish(command)
                    self.nav_to_start_chaserbin = False
                ## plan a path to the start point of the chaser bin

                
                if self.nav_to_start_chaserbin :

                    self.get_logger().warn("chaser_course : " + str(self.chaser_course[self.course_index]))
                    path.clear()
                    step = int(lateral_distance*4 )
                    path.append([current_enter_x,cur_pos[1]])
                    for i in range(1,step):
                        coord = [current_enter_x,(current_enter_y-cur_pos[1])/step*i+cur_pos[1]]
                        print(coord)
                        path.append(coord)
                    path.append([current_enter_x,current_enter_y])

     
                    command.operation = SetMission.OPERATION_START_PATH_FOLLOWING
                    for point in path:
                        temp = Point2d()
                        temp.cord_x = point[0]
                        temp.cord_y = point[1]
                        command.path.append(temp)
                    command.speed =  0.7
                    if self.current_low_level_system_status.current_mission_status != MissionStatus.PATH_FOLLOWING:
                        self.set_mission_pub.publish(command)

                else:
                    self.execute_Chasing(other_chasers)


            elif self.control_mode == ControlMode.switch and self.linked_frame != None:
                self.execute_switch_chaserbin()


        ## the code ends here========================================================================================================
        #time the main loop
        end_time = time.time()
        self.get_logger().info("Main loop time: " + str((end_time - start_time)* 1000)  + "ms")
    def dump_the_harvested_cabin(self):
        cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                            self.robot_status.kinematic_state.state.pose.position.y]
        if SCENARIO == 1:
            if cur_pos[0] < -3 and cur_pos[0] > -6:
                if cur_pos[1] < 2 and cur_pos[1] > -2:
                    self.clear_current_cabin()
                    self.get_logger().info("start_dumping..")
        elif SCENARIO == 2:       
            if cur_pos[1] > 0.65 and cur_pos[1] < 10:
                    self.clear_current_cabin()
                    self.get_logger().info("start_dumping..")
    def find_other_chasers(self):
        other_chasers = []
        for frame_name in self.chasers_frames:
            if frame_name != self.my_id + "_base_link":
                other_chasers.append(frame_name)
        return other_chasers 
    def calculate_hte_s(self):
        pass
        #  ## calculate the euclidean distance to the harvester
        # result_distance,euclidean_distance_to_harvester = decompose_euclidean_path(look_up_table_entrance[0], [harvester_pos[0] - time_s_to_left*harvester_velocity , harvester_pos[1]])
        # time_s_to_harvester = result_distance / AVERAGE_CHASERBIN_SPEED

        # goal_harvester = FIELD_DIAMENTION_X[0]
        # time_s_harvester_to_end = (abs(harvester_pos[0]-goal_harvester) - (time_s_to_left+time_s_to_harvester) * harvester_velocity)/harvester_velocity  

    switch_state = 0 
    entrance_goal = [0.0, 0.0]
    pre_harvester_pos = None
    pre_harvester_course_index = -99
    switch_strategy = 0 # 0 is the next row switch, 1 is the current row switch
    is_index_diff = False
    def execute_switch_chaserbin(self):
        if self.robot_status.vehicle_id == self.my_id and self.ox != [] and self.oy != []:
            ## find the proper entrance point
           
            cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                                self.robot_status.kinematic_state.state.pose.position.y]
            harvester_status = self.Robot_frames[HARVESTER_ID+"_base_link"]
            harvester_pos = [harvester_status.transform.translation.x, harvester_status.transform.translation.y]
            r,p,harvester_yaw = quart_to_rpy(harvester_status.transform.rotation.x, harvester_status.transform.rotation.y,
                                                                harvester_status.transform.rotation.z, harvester_status.transform.rotation.w)
            harvester_yaw = np.degrees(harvester_yaw)
            
            linkedchaserbin_status = self.Robot_frames[self.linked_frame]
            linkedchaserbin_pos = [linkedchaserbin_status.transform.translation.x, linkedchaserbin_status.transform.translation.y]
            r,p,linkedchaserbin_yaw = quart_to_rpy(linkedchaserbin_status.transform.rotation.x, linkedchaserbin_status.transform.rotation.y,
                                                                linkedchaserbin_status.transform.rotation.z, linkedchaserbin_status.transform.rotation.w)
            linkedchaserbin_yaw = np.degrees(linkedchaserbin_yaw)

            is_face_left =  (harvester_yaw > 170 or harvester_yaw < -170)
            is_face_right = (harvester_yaw < 10 and harvester_yaw > -10)
            midpoint_y = (FIELD_DIAMENTION_Y[0] + FIELD_DIAMENTION_Y[1]) / 2
            midpoint_x = (FIELD_DIAMENTION_X[0] + FIELD_DIAMENTION_X[1]) / 2
            ## only for the full scale scenario
            look_up_table_entrance = [[-2.5 , midpoint_y], #left
                                    [ 9.2 , midpoint_y], #right
                                    ]
            harvester_velocity = DEFAULT_AVERAGE_HARVESTER_SPEED
                        #state 1: go to the entrance point
            self.get_logger().info("switch_state: " + str(self.switch_state))
            self.get_logger().info("switch_strategy: " + str(self.switch_strategy))

            # check if the harvester has finished the row
            if self.is_index_diff == False and self.pre_harvester_course_index != -99:
                self.is_index_diff = self.pre_harvester_course_index != self.harvester_course_index
            self.get_logger().info("is_index_diff: " + str(self.is_index_diff))

            if self.switch_state == 0:
                # calculate the path

                path_to_left_command = command_set_path(cur_pos[0],cur_pos[1],look_up_table_entrance[0][0], look_up_table_entrance[0][1],self.ox, self.oy)
                path_to_right_command = command_set_path(cur_pos[0],cur_pos[1],look_up_table_entrance[1][0], look_up_table_entrance[1][1],self.ox, self.oy)

                time_s_to_left = calculate_distance_path(path_to_left_command.path)/AVERAGE_CHASERBIN_SPEED
                time_s_to_right = calculate_distance_path(path_to_right_command.path)/AVERAGE_CHASERBIN_SPEED

                time_s_harvester_to_end = 0
                HTE_threshold = 2.0 # in s
                if is_face_left:
                    ## when the harvester is facing left, calculate the time where the chaser bin can reach the right side
                    ## calculate the euclidean distance to the harvester
                    tts = time_s_to_right
                    entrance_point = look_up_table_entrance[1]
                    result_distance,euclidean_distance_to_harvester = decompose_euclidean_path(entrance_point, [harvester_pos[0] - tts*harvester_velocity , harvester_pos[1]])
                    time_s_to_harvester = result_distance / AVERAGE_CHASERBIN_SPEED

                    goal_harvester = FIELD_DIAMENTION_X[0]
                    time_s_harvester_to_end = (abs(harvester_pos[0]-goal_harvester) - (tts+time_s_to_harvester) * harvester_velocity)/harvester_velocity  
                elif is_face_right:
                    tts = time_s_to_left
                    entrance_point = look_up_table_entrance[0]
                    result_distance,euclidean_distance_to_harvester = decompose_euclidean_path(entrance_point, [harvester_pos[0] + tts*harvester_velocity , harvester_pos[1]])
                    time_s_to_harvester = result_distance / AVERAGE_CHASERBIN_SPEED

                    goal_harvester = FIELD_DIAMENTION_X[1]
                    time_s_harvester_to_end = (abs(harvester_pos[0]-goal_harvester) - (tts+time_s_to_harvester) * harvester_velocity)/harvester_velocity
                self.get_logger().info("time_s_harvester_to_end: " + str(time_s_harvester_to_end))
                #### strategy 0 : next row switch to the chaser bin 
                #### strategy 1 : current row switch to the chaser bin
                if time_s_harvester_to_end >= HTE_threshold:
                    self.switch_strategy = 1
                else:
                    self.switch_strategy = 0

                if is_face_right:
                    self.get_logger().info("left")
                    if self.switch_strategy == 0:
                        self.execute_path_following_command(path_to_right_command)
                        self.entrance_goal = look_up_table_entrance[1]
                    else:
                        self.execute_path_following_command(path_to_left_command)
                        self.entrance_goal = look_up_table_entrance[0]
                elif is_face_left:
                    self.get_logger().info("right")
                    if self.switch_strategy == 0:
                        self.execute_path_following_command(path_to_left_command)
                        self.entrance_goal = look_up_table_entrance[0]
                    else:
                        self.execute_path_following_command(path_to_right_command)
                        self.entrance_goal = look_up_table_entrance[1]
                
                self.switch_state = 1
            
            #state 1:check if the chaserbin is in the entrance point
            if self.switch_state == 1:
                distance_to_entrance = calc_distance(cur_pos, self.entrance_goal)
                self.get_logger().info("distance_to_entrance: " + str( distance_to_entrance))
                if distance_to_entrance < 0.5:
                    self.switch_state = 2
            #state 2: wait for the harvester to finish the row            
            if self.switch_state == 2:
                if self.pre_harvester_pos != None:
                    if self.switch_strategy == 1:
                        self.switch_state = 3
                    if self.is_index_diff:
                        self.switch_state = 3
           #state 3: wait for the harvester to finish the row            
            if self.switch_state == 3:
                if self.pre_harvester_pos != None:

                    theta = linkedchaserbin_yaw - harvester_yaw
                    if theta > 180:
                        theta = 360 - theta
                    if theta < -180:
                        theta = 360 + theta
                    theta = abs(theta)
                    self.get_logger().info("thelta: " + str(theta))
                    harvester_in_field = harvester_pos[0] < (FIELD_DIAMENTION_X[1]) and harvester_pos[0] > (FIELD_DIAMENTION_X[0])

                    # if self.switch_strategy == 1:
                    #     self.switch_state = 4
                    if theta <= 2  and (is_face_left or is_face_right) and harvester_in_field:
                        self.switch_state = 4
            
            #state 4: switch the chaser bin
            if self.switch_state == 4:
                self.course_index = self.harvester_course_index
                self.clear_chasing_flag()
                self.clear_switching_flag()
                self.control_mode = ControlMode.chasing 
                self.switch_state = 0

            self.pre_harvester_pos = harvester_pos
            self.pre_harvester_course_index = self.harvester_course_index

    def execute_path_following_command(self,command_in):
        if self.robot_status.vehicle_id == self.my_id and self.ox != [] and self.oy != []:
            command = command_cancel_path()
            self.set_mission_pub.publish(command)
            command = command_in
            self.set_mission_pub.publish(command)

    def execute_path_following(self,goal_x, goal_y):
        if self.robot_status.vehicle_id == self.my_id and self.ox != [] and self.oy != []:
            current_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                        self.robot_status.kinematic_state.state.pose.position.y]
            command = command_cancel_path()
            self.set_mission_pub.publish(command)

            command = command_set_path(current_pos[0],current_pos[1],goal_x, goal_y,self.ox, self.oy)
            self.set_mission_pub.publish(command)

         
    def execute_exit_field(self,other_chasers = []):

        thres = 1.38
        current_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                            self.robot_status.kinematic_state.state.pose.position.y]
        r,p,current_angle = quart_to_rpy(self.robot_status.kinematic_state.state.pose.orientation.x, self.robot_status.kinematic_state.state.pose.orientation.y, 
                                                self.robot_status.kinematic_state.state.pose.orientation.z, self.robot_status.kinematic_state.state.pose.orientation.w)
        current_angle = np.degrees(current_angle)
        for key in other_chasers:
            chaser_bin_status = self.Robot_frames[key]
            chaser_bin_pos = [chaser_bin_status.transform.translation.x,
                            chaser_bin_status.transform.translation.y]
            r,p,chaser_bin_angle = quart_to_rpy(chaser_bin_status.transform.rotation.x, chaser_bin_status.transform.rotation.y,
                                            chaser_bin_status.transform.rotation.z, chaser_bin_status.transform.rotation.w)
            chaser_bin_angle = np.degrees(chaser_bin_angle)
            distance_x = abs(current_pos[0] - chaser_bin_pos[0])
            angle_diff = current_angle - chaser_bin_angle
            if angle_diff > 180:
                angle_diff = 360 - angle_diff
            if angle_diff < -180:
                angle_diff = 360 + angle_diff
            angle_diff = abs(angle_diff)
            if distance_x < thres and abs(current_pos[1] - chaser_bin_pos[1])<=0.3 and angle_diff<=20:    
          
                if self.robot_status.vehicle_id == self.my_id and self.ox != [] and self.oy != []:
                    command = command_cancel_path()
                    self.set_mission_pub.publish(command)
                    command = SetMission()
                    command.operation = SetMission.OPERATION_RESUME
                    self.set_mission_pub.publish(command)
                    command = command_set_exit(current_angle, current_pos, self.ox, self.oy)
                    
                    self.set_mission_pub.publish(command)
                    self.get_logger().info("exit the field")
                    self.control_mode = ControlMode.idle
                    self.current_status.current_task.task_status = Task.TASK_STATUS_SUCCESS


    def updata_map(self):

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
        if SCENARIO == 1:
                ## for full scale scenario
                # add obs
                

                for j in range(0, 28):
                        for i in range(-1*resolution, 9*resolution):
                            if j not in self.finished_row:
                                self.ox.append(i/resolution)
                                self.oy.append(9.05 - j*0.475)
                ## dynamic update the row
                try:
                        harvester_status = self.Robot_frames[HARVESTER_ID+"_base_link"]
                        r,p,harvester_yaw = quart_to_rpy(harvester_status.transform.rotation.x, harvester_status.transform.rotation.y,
                                                            harvester_status.transform.rotation.z, harvester_status.transform.rotation.w)
                        harvester_yaw = np.degrees(harvester_yaw)
                        is_harvesting =  (harvester_yaw < 10 and harvester_yaw > -10) or (harvester_yaw > 170 or harvester_yaw < -170)
                    
                        chaser_bin_status = self.Robot_frames[self.linked_frame]
           
                        harveter_start_point_y = 9.0625
                        width_row = 0.475
                        index = (harveter_start_point_y -harvester_status.transform.translation.y)/width_row
                        cur_row = int(round(index,0))
                        self.current_harvester_row = cur_row
                        
                        if (cur_row not in self.finished_row) and  (is_harvesting) :
                                self.finished_row.append(cur_row)
                        for i in range(-1*resolution, 9*resolution):
                            self.ox.append(i/resolution)
                            self.oy.append(harvester_status.transform.translation.y)
                        if chaser_bin_status!= None:
                            for i in range(-1*resolution, 9*resolution):
                                self.ox.append(i/resolution)
                                self.oy.append(chaser_bin_status.transform.translation.y)
                            
                            for i in range(-1*resolution, 9*resolution):
                                self.ox.append(i/resolution)
                                self.oy.append(chaser_bin_status.transform.translation.y-0.3)
                except BaseException as e:
                    pass
        elif SCENARIO == 2:
                #add obs
                for j in range (0,4):
                    # ox.append(-1.5)
                    # oy.append(-1.7875+j*0.5)

                    for i in range (-2*resolution,int(2.5*resolution)):
                        self.ox.append(i/resolution)
                        self.oy.append(-1.7875+j*0.5)

    
    def check_if_chaserbin_is_linked(self):
        linked_marging = [1.5, 0.8]  # x, y
        chaserbin_frames = self.chasers_frames
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
                from_frame_rel = HARVESTER_ID+"_base_link"
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

            return linked_frame
        except TransformException as ex:
            self.get_logger().warning(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return None
        
    def on_frame_listener(self):
        """
        Callback function that compute the frame to frame relationships
        """
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
    ## state machine based chasing algorithm

    pre_course_index = 0
    goal = [float('inf'), float('inf')]
    state = -1
    is_ready = True
    cur_heading = 1
    target_heading = 0
    
    thres_d_th = 5
    goal_for_pause_time = [float('inf'), float('inf')]
    ## for the chaser bin only
    is_find_next_point = False
    is_edge_offset = False
    pre_timestamp = 0
    err_i = 0
    goal_x_chaserbin = 0.0
    pre_harvester_pos = [0.0, 0.0]
    pre_haverster_velocity = 0.0

    def execute_Chasing(self,other_chasers = []):
        '''
        1.The same as the harvesting, 
        2.but the robot will get the location from the harvester
        3.base on the location it will use the pi controller to control its own speed
        '''

        #main 
        self.get_logger().info("[Node]: Chasing....")
        chaserbin_speed = DEFAULT_AVERAGE_HARVESTER_SPEED
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1000000000
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
        offset_x = 0.26

        if self.stoarge_cabin <= 25:
            offset_x = offset_x
        if self.stoarge_cabin > 25 and self.stoarge_cabin <= 50:
            offset_x -= L_ROBOT/4
        if self.stoarge_cabin > 50 and self.stoarge_cabin <= 75:
            offset_x -= L_ROBOT/4*2
        if self.stoarge_cabin > 75 :
            offset_x -= L_ROBOT/4*3
        offset_y = 0.475
        try:
            target_pos = [self.Robot_frames["Robot1_base_link"].transform.translation.x,
                      self.Robot_frames["Robot1_base_link"].transform.translation.y]
        except BaseException as e:
            return
        is_harvester_in = (
            target_pos[0] < self.felid_x_max and target_pos[0] > self.felid_x_min)
        error = ((target_pos[0]-offset_x*self.cur_heading) -
                 cur_pos[0])*self.cur_heading
        roll, pitch, yaw_ = quart_to_rpy(self.Robot_frames["Robot1_base_link"].transform.rotation.x, self.Robot_frames["Robot1_base_link"].transform.rotation.y,
                                             self.Robot_frames["Robot1_base_link"].transform.rotation.z, self.Robot_frames["Robot1_base_link"].transform.rotation.w)
        target_angle = math.degrees(yaw_)
        #convert to 0 -360 degree
        haverster_velocity_map = 0.0
        
        self.get_logger().info("the target_angle is "+target_angle.__str__())
        ptf_flag =  (target_angle < 30 and target_angle > -30) or (target_angle > 150 or target_angle < -150)
        # PID position CONTROL
        if self.current_low_level_system_status.current_mission_status == MissionStatus.PATH_FOLLOWING:


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
            ## assume catersian coordinate in the chaserbin frame
            ## velocity control
            if self.pre_timestamp != 0 and self.pre_timestamp != timestamp:
                dt = timestamp - self.pre_timestamp
                haverster_velocity_map = math.sqrt((target_pos[0] - self.pre_harvester_pos[0])**2 + (target_pos[1] - self.pre_harvester_pos[1])**2)/dt
                ## in exp moving average
                alpha = 0.8
                haverster_velocity_map = (1- alpha) * haverster_velocity_map + alpha * self.pre_haverster_velocity
                self.get_logger().info("the haverster_velocity is "+haverster_velocity_map.__str__())
                command = SetMission()
                command.operation = SetMission.OPERATION_CHANGE_SPEED

                command.speed = haverster_velocity_map
                ## 
                self.set_mission_pub.publish(command)

            ## accleration control
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
            command.operation = SetMission.OPERATION_CHANGE_ACCLERATION

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
                if min(dis_harvester) < 0.4 and ptf_flag:
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
                  # find the shortest point
                dis_harvester = []
                for point in self.harvest_course[self.course_index]:
                    dis_harvester.append(calc_distance(target_pos, point))
                self.get_logger().info("release the chaser when its <0.4 :  " + str(min(dis_harvester)))
                
               
                    
                if min(dis_harvester) < 0.40 and ptf_flag :
                    command = SetMission()
                    command.operation = SetMission.OPERATION_RESUME
                    self.set_mission_pub.publish(command)
                    self.get_logger().info("release the chaser!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                    if abs(target_pos[1] - cur_pos[1]) > W_ROBOT/2 and self.is_edge_offset == False:
                        if self.cur_heading == 1:
                            self.goal_x_chaserbin = self.goal[0] + 0.6
                           
                        else:
                            self.goal_x_chaserbin = self.goal[0] - 0.6

                        self.is_edge_offset = True

                command = calc_turn_command(d_th, steer)

                if abs(cur_pos[0] - self.goal_x_chaserbin) > 0.1 and self.is_edge_offset:
                    command = SetMotion()
                    command.speeds = np.array(
                    [chaserbin_speed, chaserbin_speed, chaserbin_speed, chaserbin_speed], dtype="float")

                self.set_motion_pub.publish(command)
                
                self.get_logger().info(self.goal .__str__())
                self.get_logger().info(self.goal_x_chaserbin .__str__())
                                          
            else:

                self.is_find_next_point = True
                self.is_ready = False
                if self.is_edge_offset:
                    self.goal[0] = self.goal_x_chaserbin


        if self.is_find_next_point:
            if cur_pos[1] - self.goal[1] > 0:
                goal = copy.deepcopy(self.goal)
                goal[1] -= 10
            else:
                goal = copy.deepcopy(self.goal)
                goal[1] += 10
            target_speed = chaserbin_speed
            command = SetMotion()
            target_angle_front_wheel = calculate_front_wheel_angle(
                cur_pos, goal, cur_angle)
            command.speeds = np.array(
                [target_speed, target_speed, target_speed, target_speed], dtype="float")
            if calc_distance(cur_pos, self.goal) > L_ROBOT:
                command.angles = np.array(
                    [target_angle_front_wheel, target_angle_front_wheel, 0.0, 0.0], dtype="float")
            self.set_motion_pub.publish(command)

        if calc_distance(cur_pos, self.goal) < 0.55 and self.current_low_level_system_status.current_mission_status == MissionStatus.MANUNAL_CONTROL:
            if self.state == -1:
                command = SetMission()
                command.operation = SetMission.OPERATION_PAUSE
                self.set_mission_pub.publish(command)
                self.get_logger().info("stopping the chaserbin")
                self.course_index += 1
            self.state = -self.state
            self.is_find_next_point = False
            self.is_ready = True
            self.is_edge_offset = False
            
        # sim cabin for chaserbin
        if self.state == -1 and is_harvester_in and abs(error) < 0.5 and calc_distance(cur_pos, target_pos) < 1:
            if self.stoarge_cabin < 100:
                if (timestamp - self.pre_timestamp) <= 1:
                    self.stoarge_cabin += (timestamp -
                                                      self.pre_timestamp) *  random.uniform(AVERAGE_HARVESTER_RATE-STD_HARVESTER_RATE, AVERAGE_HARVESTER_RATE+STD_HARVESTER_RATE)
        self.pre_timestamp = timestamp
        self.pre_harvester_pos = target_pos
        self.pre_haverster_velocity = haverster_velocity_map
        if self.course_index == len(course):
            self.control_mode = ControlMode.idle
            self.clear_chasing_flag()
            self.course_index = 0
        if self.stoarge_cabin >= 60:
            #check if the chaserbin is in the field if the chaserbin needs to go out of the field
            self.execute_exit_field(other_chasers=other_chasers)
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
        self.stoarge_cabin = np.clip(self.stoarge_cabin, 0, 100)
        self.pre_timestamp = timestamp

    def on_harvester_status(self,msg:VehicleStatus):
        self.harvester_course_index = msg.status.kpis[1]        
    # only update the class member variable
    def on_mission_status(self, msg:MissionStatus):
        self.current_low_level_system_status = msg
        if self.control_mode == ControlMode.path_following and self.current_status.current_task.task_status == Task.TASK_STATUS_STARTED:
            cur_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                   self.robot_status.kinematic_state.state.pose.position.y]
            dis = calc_distance(cur_pos, [self.path_following_goal[0], self.path_following_goal[1]])

            if msg.current_mission_status == MissionStatus.MANUNAL_CONTROL and dis < 2.0:
                self.current_status.current_task.task_status = Task.TASK_STATUS_SUCCESS
                self.control_mode = ControlMode.idle
                self.get_logger().info("Task done")

    def on_robot_status(self, msg:RobotStatus):

        if msg.vehicle_id == self.my_id:
            self.robot_status = msg
            vehicle_status = VehicleStatus()
            #basic status
            
            vehicle_status.status = copy.deepcopy(self.current_status)
            vehicle_status.status.kpis = []

            #current_kinematic_state
            vehicle_status.pose.header.frame_id = msg.kinematic_state.header.frame_id
            vehicle_status.pose.header.stamp = msg.kinematic_state.header.stamp
            vehicle_status.pose.pose = msg.kinematic_state.state.pose
            vehicle_status.twist.linear.x = msg.kinematic_state.state.longitudinal_velocity_mps
            vehicle_status.twist.linear.y = msg.kinematic_state.state.lateral_velocity_mps
            vehicle_status.twist.linear.z = 0.0
            vehicle_status.twist.angular.z = msg.kinematic_state.state.heading_rate_rps
            ## kpi 
            vehicle_status.status.vehicle_id = self.my_id
            ## first kpi is the self.stoarge_cabin
            vehicle_status.status.kpis.append(int(self.stoarge_cabin))
            ## second kpi is the self.course_index
            vehicle_status.status.kpis.append(int(self.course_index))
            ## third kpi is the current mode at rcl
            vehicle_status.status.kpis.append(int(self.control_mode))
            ## fourth kpi is the low level system status
            vehicle_status.status.kpis.append(int(self.current_low_level_system_status.current_mission_status))

            self.vehicle_status_pub.publish(vehicle_status)

    def clear_chasing_flag(self):
        self.pre_course_index = 0
        self.goal = [float('inf'), float('inf')]
        self.state = -1
        self.is_ready = True
        self.cur_heading = 1
        self.target_heading = 0
        
        self.thres_d_th = 5
        self.goal_for_pause_time = [float('inf'), float('inf')]
        ## for the chaser bin only
        self.is_find_next_point = False
        self.is_edge_offset = False
        self.pre_timestamp = 0
        self.err_i = 0
        self.goal_x_chaserbin = 0.0
        self.pre_harvester_pos = [0.0, 0.0]
        self.pre_haverster_velocity = 0.0
        self.nav_to_start_chaserbin = True

    def clear_switching_flag(self):
        self.switch_state = 0 
        self.entrance_goal = [0.0, 0.0]
        self.pre_harvester_pos = None
        self.pre_harvester_course_index = -99
        self.switch_strategy = 0 # 0 is the next row switch, 1 is the current row switch
        self.is_index_diff = False

    # the task execution function       
    def on_task(self,msg:Task):
        if msg.task_api ==Task.TASK_API_STOP_CURRENT_TASK:
            self.control_mode = ControlMode.idle
            self.current_status.current_task.task_status = Task.TASK_STATUS_FAILURE
            self.clear_switching_flag()
            self.get_logger().info("Stop the harvester")
            self.clear_chasing_flag()
            command = SetMission()
            command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING
            self.set_mission_pub.publish(command)

            self.set_motion_pub.publish(SetMotion())  
        # composed task 
        if msg.task_api == Task.TASK_API_COMPOSED_TASK and msg.task_status == Task.TASK_STATUS_TODO:
            task_info = umsgpack.unpackb(msg.task_info)
             ## load the values from the task_info
            composed_task_type = task_info[0]
            if composed_task_type == ComposeTask.COMPOSED_TASK_TYPE_HARVESTING_CHASING:
                self.current_status.current_task = msg

                self.get_logger().info("Harvesting/chasing task")
                self.course_index =np.clip(task_info[2],0,999)
                self.clear_chasing_flag()
                self.control_mode = ControlMode.chasing
                self.current_status.current_task.task_status = Task.TASK_STATUS_STARTED
    

            elif composed_task_type == ComposeTask.COMPOSED_TASK_TYPE_SWITCHING:
                if self.control_mode == ControlMode.chasing:
                    return
                self.current_status.current_task = msg
                self.get_logger().info("Switching task")
                self.clear_switching_flag()
                self.control_mode = ControlMode.switch
                self.current_status.current_task.task_status = Task.TASK_STATUS_STARTED

        if msg.task_api == Task.TASK_API_PATH_FOLLOWING and msg.task_status == Task.TASK_STATUS_TODO:
            self.current_status.current_task = msg
            task_info = umsgpack.unpackb(msg.task_info)
            try:
                ## load the values from the task_info


                self.get_logger().info("Path following task")

                self.execute_path_following(task_info[0], task_info[1])
                self.current_status.current_task.task_status = Task.TASK_STATUS_STARTED
                self.control_mode = ControlMode.path_following
                self.path_following_goal = [task_info[0], task_info[1]]
                speed_setpoint = task_info[3]
                command = SetMission()
                command.operation = SetMission.OPERATION_CHANGE_SPEED
                command.speed = float(speed_setpoint)
                self.set_mission_pub.publish(command)   

            except BaseException as e:
                self.current_status.current_task.task_status = Task.TASK_STATUS_FAILURE
                self.get_logger().error("Error in path following task: " + str(e))

        ## debug motion control related api 
        if msg.task_api == Task.TASK_API_MOTION_CONTROL:
            self.current_status.current_task = msg
            task_info = umsgpack.unpackb(msg.task_info)
            #[task_type, motion_array,speed_setpoint]
            task_type = task_info[0]
            if task_type == MotionTask.MOTION_TASK_TYPE_PAUSE:
                command = SetMission()
                command.operation = SetMission.OPERATION_PAUSE
                self.set_mission_pub.publish(command)   
            if task_type == MotionTask.MOTION_TASK_TYPE_RESUME:
                command = SetMission()
                command.operation = SetMission.OPERATION_RESUME
                self.set_mission_pub.publish(command)  

            if task_type == MotionTask.MOTION_TASK_TYPE_CHANGE_PATH_SPEED:
                command = SetMission()
                command.operation = SetMission.OPERATION_CHANGE_SPEED
                command.speed = float(task_info[2])
                self.set_mission_pub.publish(command)   
            if task_type == MotionTask.MOTION_TASK_TYPE_SET_MOTION:
                commnad = SetMotion()
                commnad.angles =np.array( task_info[1][0],dtype="float")
                commnad.speeds =np.array( task_info[1][1],dtype="float")
                self.set_motion_pub.publish(commnad)  

            self.current_status.current_task.task_status = Task.TASK_STATUS_SUCCESS



def main(args=None):

    rclpy.init(args=args)

    # Create the node
    RobotCooperationLayer_node = RobotCooperationLayer()

    # Spin the node so the callback functions are called.
    rclpy.spin(RobotCooperationLayer_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    RobotCooperationLayer_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
