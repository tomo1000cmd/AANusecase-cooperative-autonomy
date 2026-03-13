#!/usr/bin/env python3


#This is a simple version of robot cooperation layer (RCL) for the VOA. This is a way that could achieve some general functions for the DurableCase project.

import rclpy
import random
from rclpy.node import Node
from agent_msgs.srv import StandardAction
from geometry_msgs.msg import Pose
from agent_msgs.srv import StartRoute, SetStatus
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

from helper import generate_course_chaser_file, generate_course_harvest_file, calc_distance, generate_path_H, quart_to_rpy, command_set_path, calc_turn_command, calculate_dth, command_cancel_path, calculate_front_wheel_angle, calculate_offset_point, calculate_front_wheel_angle_and_speed

SCENARIO = 1 #1 is full simulation 2 is lab scale

L_ROBOT = 0.65
W_ROBOT = 0.455  # 0.425 for real robot
HARVESTER_ID = "Robot1"
if SCENARIO == 1:
    AVERAGE_HARVESTER_RATE = 0.65   # in %/s
    STD_HARVESTER_RATE = 0.3       # in %/s
    DEFAULT_AVERAGE_HARVESTER_SPEED = 0.16  # average harvest speed in m/s
elif SCENARIO == 2:
    AVERAGE_HARVESTER_RATE = 2.15    # in %/s
    STD_HARVESTER_RATE = 0.0       # in %/s
    DEFAULT_AVERAGE_HARVESTER_SPEED = 0.16  # average harvest speed in m/s
HARVESTER_ACC_OFFSET = 0.0      # offset for harvester acceleration in m/s^2
AVERAGE_CHASERBIN_SPEED = 0.5   # average chaserbin speed in m/s
class ControlMode:
    harvesting = 1
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
# For the harvester:
# only harvesting function is needed
class RobotCooperationLayer(Node):
    ## First step: bridge the message and service types from the mas_msgs package into the scope of the individual robot supervision layer.


    def __init__(self):

        super().__init__('RobotCooperationLayer_harvester_node')
        # force simulation mode
        # simtime =Parameter("use_sim_time", Parameter.Type.BOOL, True)
        # self.set_parameters([simtime])  
        # Parameters
        self.declare_parameter('my_id',"Robot1")
        self.my_id = self.get_parameter('my_id').value
        self.is_sim = self.get_parameter('use_sim_time').value
        self.robot_namespace = self.get_namespace()
        self.robot_namespace = self.my_id 
        self.get_logger().info("Simulation mode: " + str(self.is_sim))
        # Initialize variables
        self.current_status = Status()
        self.robot_status = RobotStatus()
        resolution = 2
        self.ox = []
        self.oy = []
        border_size = 10.5
        self.control_mode = ControlMode.idle
            
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
        self.current_low_level_system_status = MissionStatus()

        # tf2 listener
        buffer_time = Time()
        buffer_time.sec = 0
        self.tf_buffer = Buffer(buffer_time)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.02, self.on_frame_listener)
        self.Robot_frames = {}

        # Create subscriber(s)
        self.create_subscription(
            RobotStatus, self.robot_namespace+ '/' + 'robot_status', self.on_robot_status, 10)
        
        self.create_subscription(
            MissionStatus, self.robot_namespace+ '/' + 'mission_status', self.on_mission_status, 10)
        # Task execution
        self.create_subscription(
            Task, self.robot_namespace+ '/' + 'set_task', self.on_task, 10)
        
        # Create publisher(s)
        self.set_mission_pub = self.create_publisher(
            SetMission, self.robot_namespace+ '/set_mission', 10)
        self.set_motion_pub = self.create_publisher(
            SetMotion, self.robot_namespace+ '/set_motion', 10)
        self.vehicle_status_pub = self.create_publisher(
            VehicleStatus, self.robot_namespace+ '/' +'vehicle_status', 10)
       
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
        self.nav_to_start_chaserbin = False
        self.stoarge_cabin = 0
        self.average_harvester_speed = DEFAULT_AVERAGE_HARVESTER_SPEED
        if SCENARIO == 2:
            if self.my_id == "Robot0":
                self.stoarge_cabin = 80

        # Create timer(s)
        self.timer = self.create_timer(0.1, self.main_loop)

        # Finally
        self.get_logger().info("initialized")
        
    def main_loop(self):
        if self.control_mode == ControlMode.harvesting:
            self.excute_harvesting()

    def execute_path_following(self,goal_x, goal_y):
        if self.robot_status.vehicle_id == self.my_id and self.ox != [] and self.oy != []:
            current_pos = [self.robot_status.kinematic_state.state.pose.position.x,
                        self.robot_status.kinematic_state.state.pose.position.y]
            print("current_pos", current_pos)
            command = command_cancel_path()
            self.set_mission_pub.publish(command)
            command = command_set_path(current_pos[0],current_pos[1],goal_x, goal_y,self.ox, self.oy)
            self.set_mission_pub.publish(command)

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
        
        
    ## state machine based harvesting algorithm
    course_index = 0
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
    pre_haverster_pos = [0.0, 0.0]
    pre_haverster_velocity = 0.0

    def excute_harvesting(self):
        '''
        1. The robot will move in a square course with the length of 28m and width of 2m. The robot will move in a clockwise manner.
        2. For each line, the robot will move to the point that is the farthest from the start point. Then it will move to the point that is the nearest to the start point.
        3. The robot will be set to a fixed speed of 0.16m/s. at start 
        '''
        self.get_logger().info("[Node]: Harvesting....")

        #course = generate_square_course([-1.55, 9.065], 28, 2, 0.475,10.35)
        #course = generate_square_course(H_START_POINT, H_ROW, H_COL, H_ROW_DIS,H_COL_DIS)
        timestamp = self.get_clock().now().to_msg().sec + self.get_clock().now().to_msg().nanosec/1000000000
        harvesting_speed = DEFAULT_AVERAGE_HARVESTER_SPEED
        target_speed = harvesting_speed
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
        # calcu the felid with offset for harvesting_chasing_velocityelid_offset
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
            self.goal_for_pause_time = curr_line[dis.index(min(dis))]
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
                if course_i == 1 or course_i == 2 or course_i == 6 or course_i == 7 or course_i == 16 or course_i == 17 :
                    command = SetMission()
                    command.operation = SetMission.OPERATION_PAUSE
                    self.set_mission_pub.publish(command)
                    self.get_logger().info("stopping the haverester")
                #advance to the field for a haverster lengthstep
                command = SetMotion()
                goal = copy.deepcopy(self.goal_for_pause_time )
                goal[0] += self.cur_heading * (L_ROBOT+0.3)
                target_angle = calculate_front_wheel_angle(
                    cur_pos, goal, cur_angle)
                command.speeds = np.array(
                    [target_speed, target_speed, target_speed, target_speed], dtype="float")
                
                command.angles = np.array(
                        [target_angle, target_angle, 0.0, 0.0], dtype="float")
                self.set_motion_pub.publish(command)
                dis_elapse  = calc_distance(cur_pos, goal)
                self.get_logger().info("the goal " + goal.__str__())
                self.get_logger().info("the dis_elapse " + dis_elapse.__str__())
                
                if dis_elapse < 0.2 :
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
                command.speed =  self.average_harvester_speed 
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

        if calc_distance(cur_pos, self.goal) <= 0.52 and self.current_low_level_system_status.current_mission_status == MissionStatus.MANUNAL_CONTROL:

            if self.state == -1:
                self.course_index += 1
            self.state = -self.state
            self.is_find_next_point = False
            self.is_ready = True
        self.pre_timestamp = timestamp

        if self.course_index == len(course):
            self.control_mode = ControlMode.idle
            self.clear_harvesting_flag()
            self.course_index = 0

    # only update the class member variable
    def on_mission_status(self, msg:MissionStatus):
        self.current_low_level_system_status = msg
        if self.control_mode == ControlMode.path_following and msg.current_mission_status == MissionStatus.PATH_FOLLOWING:
            self.current_status.current_task.task_status = Task.TASK_STATUS_STARTED

        if self.control_mode == ControlMode.path_following and self.current_status.current_task.task_status == Task.TASK_STATUS_STARTED:
            if msg.current_mission_status == MissionStatus.MANUNAL_CONTROL:
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
            vehicle_status.status.kpis.append(int(0))
            ## second kpi is the self.course_index
            vehicle_status.status.kpis.append(int(self.course_index))
            ## third kpi is the current mode at rcl
            vehicle_status.status.kpis.append(int(self.control_mode))
            ## fourth kpi is the low level system status
            vehicle_status.status.kpis.append(int(self.current_low_level_system_status.current_mission_status))
            self.vehicle_status_pub.publish(vehicle_status)
    def clear_harvesting_flag(self):
        self.course_index = 0
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
        self.pre_haverster_pos = [0.0, 0.0]
        self.pre_haverster_velocity = 0.0
    # the task execution function       
    def on_task(self,msg:Task):
        # composed task 
        if msg.task_api =="STOP":
            self.control_mode = ControlMode.idle
            self.current_status.current_task.task_status = Task.TASK_STATUS_FAILURE
            self.get_logger().info("Stop the harvester")
            self.clear_harvesting_flag()
            command = SetMission()
            command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING
            self.set_mission_pub.publish(command)

            self.set_motion_pub.publish(SetMotion())  

        if msg.task_api == Task.TASK_API_COMPOSED_TASK and msg.task_status == Task.TASK_STATUS_TODO:
            self.current_status.current_task = msg
            task_info = umsgpack.unpackb(msg.task_info)
            ## load the values from the task_info
            composed_task_type = task_info[0]
            if composed_task_type == ComposeTask.COMPOSED_TASK_TYPE_HARVESTING_CHASING:
                self.clear_harvesting_flag()
                self.get_logger().info("Harvesting/chasing task")
                self.course_index =np.clip(task_info[2],0,999)
                harvesting_velocity = task_info[1]
                self.control_mode = ControlMode.harvesting
                self.current_status.current_task.task_status = Task.TASK_STATUS_STARTED
                command = SetMission()
                command.operation = SetMission.OPERATION_CHANGE_SPEED
                command.speed = float(harvesting_velocity)
                self.set_mission_pub.publish(command)   

            elif composed_task_type == ComposeTask.COMPOSED_TASK_TYPE_SWITCHING:
                self.get_logger().error("Noooooooooooooooooooooooooooooooooooooooo!")
                self.current_status.current_task.task_status = Task.TASK_STATUS_FAILURE

        if msg.task_api == Task.TASK_API_PATH_FOLLOWING and msg.task_status == Task.TASK_STATUS_TODO:
            # if self.control_mode != ControlMode.idle:
            #     self.current_status.current_task.task_status = Task.TASK_STATUS_FAILURE
            self.current_status.current_task = msg
            task_info = umsgpack.unpackb(msg.task_info)
            try:
                ## load the values from the task_info
                self.execute_path_following(task_info[0], task_info[1])
                self.control_mode = ControlMode.path_following
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
