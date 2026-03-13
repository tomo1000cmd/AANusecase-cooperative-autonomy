
from mas_msgs.msg import CAcommand,Task
import numpy as np
import math
import copy
from h2trac_msgs.msg import SetMotion,Point2d,SetMission
from Astar import AStarPlanner
from scipy import interpolate
import uuid
import umsgpack
"""
genearte the course for harvesting base on the input parameter

@input
startpoint: start_point of the harvesting ([x,y])
col_count: number of columns in the field (int)
row_count: number of rows in the field (int)
dis_col: distance between two columns ()
dis_row: distance between two rows

@output
course: the complete list of lines that is in the syntax of course[line[x,y]]
"""

IS_SCHEDULE = True

def generate_square_course(startpoint: list, col_count: int, row_count: int, dis_col: float, dis_row: float) -> list:
    course = []
    for col in range(col_count):
        line = []
        for row in range(row_count):
            x = startpoint[0] + row*dis_row
            y = startpoint[1] - col*dis_col
            line.append([x, y])
        course.append(line)
    return course


"""
calculate the distance between a and b
@input 
point_a : the cordinates of a in list [x,y]
point_b : the cordinates of b in list [x,y]

@output
distance: distance between a and b
"""


def calc_distance(point_a: list, point_b: list) -> float:
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return math.hypot(dx, dy)


def generate_path_H(cur_pos: list, goal_pos: list, step: float):
    path = []
    point_a = copy.deepcopy(cur_pos)

    path.append(cur_pos)
    if (cur_pos[0] < goal_pos[0]):
        while point_a[0] < goal_pos[0]:
            point_a[0] += step
            temp = copy.deepcopy(point_a)
            temp[1] = goal_pos[1]
            path.append(temp)
    if (cur_pos[0] > goal_pos[0]):
        while point_a[0] > goal_pos[0]:
            point_a[0] -= step
            temp = copy.deepcopy(point_a)
            temp[1] = goal_pos[1]
            path.append(temp)

    return path


def generate_path_V(cur_pos: list, goal_pos: list, step: float):
    path = []
    point_a = copy.deepcopy(cur_pos)

    path.append(cur_pos)
    if (cur_pos[1] < goal_pos[1]):
        while point_a[1] < goal_pos[1]:
            point_a[1] += step
            temp = copy.deepcopy(point_a)
            temp[0] = goal_pos[1]
            path.append(temp)
    if (cur_pos[1] > goal_pos[1]):
        while point_a[1] > goal_pos[1]:
            point_a[1] -= step
            temp = copy.deepcopy(point_a)
            temp[0] = goal_pos[1]
            path.append(temp)

    return path


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw


ERROR = -9999


def calculate_angle(point_a, point_b):
    x0 = point_a[0]
    y0 = point_a[1]

    x1 = point_b[0]
    y1 = point_b[1]
    th = 0

    if (y1 < y0) and (x0 == x1):
        th = -90
    elif (y1 > y0) and (x0 == x1):
        th = 90
    elif(y1 == y0) and (x0 == x1):
        th = ERROR
    else:
        th_ = math.atan((y0-y1)/(x0-x1))
        th_ = math.degrees(th_)
        th = th_
        if (x0 > x1):
            th = 180 + th_
            if (y0 > y1):
                th = -180 + th_

    return th


def calc_turn_command(d_th):

    target_speed = d_th * -0.004
    if target_speed > 0.16:
        target_speed = 0.16
    if target_speed < -0.16:
        target_speed = -0.16
    rotation_angle_rad = -np.radians(49)

    command = SetMotion()

    # [-v,v,-v,v] Anti Clock Wise
    speeds = np.array(
        [-target_speed, target_speed, -target_speed, target_speed], dtype="float")
    angles = np.array([-rotation_angle_rad, rotation_angle_rad,
                       rotation_angle_rad, -rotation_angle_rad], dtype="float")
    command.speeds = speeds
    command.angles = angles

    return command


def command_start_harvesting(robotid):
    command = CAcommand()
    if IS_SCHEDULE:
        task_ = Task()
        # task_.timestamp = self.ros2listener.v_status.header.stamp.sec
        

            # Generate a random UUID
        # random_id = uuid.uuid4()

        #     # Convert UUID to int16
        # int16_value_uuid = int(random_id.hex, 16) % 32768 
        # task_.route_id = int16_value_uuid

        task_.task_name = Task.TASK_API_HARVEST_CHASING
        task_.task_status = Task.TASK_STATUS_TODO


            
        command.vehicle_id = robotid
        command.operation = CAcommand.OPERATION_ADD_TIMETABLE
        command.task = task_
    else:
        command.vehicle_id = robotid
        command.operation = CAcommand.OPERATION_START_HARVESTING
    return command

def command_pause_robot(robotid):
    command = CAcommand()
    command.vehicle_id = robotid
    command.operation = CAcommand.OPERATION_SET_MISSION
    command.mission_operation = SetMission.OPERATION_PAUSE
    return command

def command_resume_robot(robotid):
    command = CAcommand()
    command.vehicle_id = robotid
    command.operation = CAcommand.OPERATION_SET_MISSION
    command.mission_operation = SetMission.OPERATION_RESUME
    return command

def command_set_exit( cur_angle, cur_pos, robotid, ox, oy):

    #the edge of the field 
    exit_place = [-2.0, 9.0]
    rx = []
    ry = []
    path = []
    print(cur_angle)
    goal_exit = []
        
        # heading left to right
    if cur_angle < 20 and cur_angle > -20:
            grid_size_ = int(calc_distance(cur_pos,[exit_place[1],cur_pos[1]])/1)
            path.append(cur_pos)
            for i in range(grid_size_-1):
                rx.append(cur_pos[0] + (exit_place[1] - cur_pos[0]) / grid_size_ * i)
                ry.append(cur_pos[1])
            
            # rx.append(exit_place[1])
            # ry.append(cur_pos[1])
            goal_exit = [exit_place[1], cur_pos[1]]

        # heading right to left
    if cur_angle > 160 or cur_angle < -160:
            grid_size_ =int( calc_distance(cur_pos,[exit_place[0],cur_pos[1]]) / 1)
            path.append(cur_pos)
            for i in range(grid_size_-1):
                rx.append(cur_pos[0] + (exit_place[0] - cur_pos[0]) / grid_size_ * i)
                ry.append(cur_pos[1])
            # rx.append(exit_place[0])
            # ry.append(cur_pos[1])
            #path.append([exit_place[0], cur_pos[1]])
            goal_exit = [exit_place[0], cur_pos[1]]

    command = CAcommand()
    # astar
    try:
        sx = goal_exit[0]
        sy = cur_pos[1]
    except BaseException:
        if abs(cur_pos[0] - exit_place[0]) < 2:
            sx = exit_place[0]
        if abs(cur_pos[0] - exit_place[1]) < 2:
            sx = exit_place[1]
        sy = cur_pos[1]
    ## goal location
    gx = -5.0
    gy = -1.0
   

    grid_size = 0.5  # [m]
    robot_radius = 0.5  # [m]
   
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx_a, ry_a = a_star.planning(sx, sy, gx, gy)
     
    path_size = len(rx_a)
    for i in range(path_size):
        rx.append(rx_a[path_size-i-1]) 
        ry.append(ry_a[path_size-1-i])
    tck, u = interpolate.splprep([rx, ry], s=0.05)
    unew = np.arange(-0.00, 1.00, float(1/len(rx)*0.5))
    out = interpolate.splev(unew, tck)
    rx = out[0]
    ry = out[1]
    
    if IS_SCHEDULE:
            path = []
            for i in range(len(rx)):
                
                temp = [float(rx[i]), float(ry[i])]
                path.append(temp)
        
            
            task_ = Task()
            #task_.timestamp = self.ros2listener.v_status.header.stamp.sec
        

            # Generate a random UUID
            # random_id = uuid.uuid4()

            # # Convert UUID to int16
            # int16_value_uuid = int(random_id.hex, 16) % 32768 
            # task_.route_id = int16_value_uuid

            task_.task_name = Task.TASK_API_PATH_FOLLOWING
            task_.task_status = Task.TASK_STATUS_TODO

            task_info_bytes = umsgpack.packb([0.7,path])

            task_.task_info = task_info_bytes
            command.vehicle_id = robotid
            command.operation = CAcommand.OPERATION_ADD_TIMETABLE
            command.task = task_


    else: 
            for i in range(len(rx)):
                temp = Point2d()
                temp.cord_x = float(rx[i])
                temp.cord_y = float(ry[i])
                command.path.append(temp)
     
        

            command.vehicle_id = robotid
            command.operation = CAcommand.OPERATION_SET_MISSION
            
            command.acceration = 0.7
            command.mission_operation = SetMission.OPERATION_START_PATH_FOLLOWING


    return command
  



def command_set_path(sx,sy,gx,gy,robotid,ox,oy):

    command = CAcommand()
    
    grid_size = 0.5  # [m]
    robot_radius = 0.447  # [m]


    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    path = []
    path_size = len(rx)
    for i in range(path_size):
        path.append([rx[path_size-i-1], ry[path_size-1-i]])

    
    if IS_SCHEDULE:
            task_ = Task()
            #task_.timestamp = self.ros2listener.v_status.header.stamp.sec
        

            # Generate a random UUID
            # random_id = uuid.uuid4()

            # # Convert UUID to int16
            # int16_value_uuid = int(random_id.hex, 16) % 32768 
            # task_.route_id = int16_value_uuid

            task_.task_name = Task.TASK_API_PATH_FOLLOWING
            task_.task_status = Task.TASK_STATUS_TODO

            task_info_bytes = umsgpack.packb([0.7,path])

            task_.task_info = task_info_bytes
            command.vehicle_id = robotid
            command.operation = CAcommand.OPERATION_ADD_TIMETABLE
            command.task = task_
    else:

            for point in path:
                temp = Point2d()
                temp.cord_x = point[0]
                temp.cord_y = point[1]
                command.path.append(temp)
        
            

            command.vehicle_id = robotid
            command.operation = CAcommand.OPERATION_SET_MISSION
            
            command.acceration = 0.7 
            command.mission_operation = SetMission.OPERATION_START_PATH_FOLLOWING

    return command

