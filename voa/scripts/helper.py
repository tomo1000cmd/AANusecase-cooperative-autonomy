import numpy as np
import math
import copy
from h2trac_msgs.msg import SetMotion,Point2d,SetMission
from Astar import AStarPlanner
from xml_trajectory_planner_class import trajectory_path_planner
from scipy import interpolate

def generate_course_harvest_file(filepath):

    trajectory_path = trajectory_path_planner(filepath)
    trajectory_path.xml_reader()
    trajectory_path.trajectory_sorter()
    course_harvest, course_chaser = trajectory_path.trajectory_planner()
    course_tf_harvester = []
    for i in range(int(len(course_harvest)/2)):
        course_tf_harvester.append([course_harvest[2*i], course_harvest[2*i+1]])
    return course_tf_harvester

def generate_course_chaser_file(filepath):
    trajectory_path = trajectory_path_planner(filepath)
    trajectory_path.xml_reader()
    trajectory_path.trajectory_sorter()
    course_harvest, course_chaser = trajectory_path.trajectory_planner()
    
    course_tf_chaser = []
    for i in range(int(len(course_chaser)/2)):
        course_tf_chaser.append([course_chaser[2*i], course_chaser[2*i+1]])

    return course_tf_chaser
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
    rx = []
    ry = []
    step_size = (goal_pos[0]-cur_pos[0])/step

    rx.append(cur_pos[0])
    ry.append(goal_pos[1])
    for i in range(1,step):
        rx.append(cur_pos[0]+(goal_pos[0]-cur_pos[0])/step*i)
        ry.append(goal_pos[1])
    rx.append(goal_pos[0])
    ry.append(goal_pos[1])

    tck, u = interpolate.splprep([rx, ry], s=0.01)
    unew = np.arange(-0.00, 1.00, 0.04)
    out = interpolate.splev(unew, tck)
    for i in range(len(rx)):
        path.append([rx[i], ry[i]])

    return path


def generate_path_V(cur_pos: list, goal_pos: list, step: float):
    path = []

    step_size = (goal_pos[0]-cur_pos[0])/step

    path.append(cur_pos)
    for i in range(1,step):
        path.append([goal_pos[0],cur_pos[1]+(goal_pos[1]-cur_pos[1])/step*i])
    path.append(goal_pos)

 

    
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

    th = math.atan2((y1-y0),(x1-x0))*180 / math.pi

    return th

import numpy as np

def calculate_dth(cur_angle,cur_pos,goal):
    x1 = cur_pos[0]
    y1 = cur_pos[1]
    x2 = goal[0]
    y2 = goal[1]
    # Calculate the angle to the goal position
    goal_angle = math.degrees(math.atan2(y2 - y1, x2 - x1))

    # Calculate the difference between the robot's current angle and the goal angle
    angle_diff = goal_angle - cur_angle

    # Normalize the angle difference to the range (-180, 180]
    if angle_diff <= -180:
        angle_diff += 360
    elif angle_diff > 180:
        angle_diff -= 360

    steer = angle_diff > 0
    
    return abs(angle_diff),steer


def calculate_front_wheel_angle(car_pos, end_point, yaw_angle):
    """
    Calculates the front wheel angle in degrees based on the position of the car,
    the end point, and the yaw angle of the car.

    Args:
        car_pos (list): List containing the x, y position of the car [x, y].
        end_point (list): List containing the x, y position of the end point [x, y].
        yaw_angle (float): Yaw angle of the car in degrees.

    Returns:
        float: Front wheel angle in radians.
    """
    # Extract the x, y components of the car position and end point
    car_x, car_y = car_pos
    end_x, end_y = end_point

    # Calculate the displacement vector
    d_x = end_x - car_x
    d_y = end_y - car_y

    # Convert the yaw angle from degrees to radians
    yaw_rad = math.radians(yaw_angle)

    # Calculate the direction vector
    d_dir_x = math.cos(yaw_rad) * d_x + math.sin(yaw_rad) * d_y
    d_dir_y = -math.sin(yaw_rad) * d_x + math.cos(yaw_rad) * d_y

    # Calculate the front wheel angle in radians
    theta_rad = math.atan2(d_dir_y, d_dir_x)

    theta_rad = np.clip(theta_rad,-np.radians(40),np.radians(40))
    return -theta_rad

def calculate_front_wheel_angle_and_speed(car_pos, end_point, yaw_angle):
    """
    Calculates the front wheel angle and speed of the wheels based on the position
    of the car, the end point, and the yaw angle of the car.

    Args:
        car_pos (list): List containing the x, y position of the car [x, y].
        end_point (list): List containing the x, y position of the end point [x, y].
        yaw_angle (float): Yaw angle of the car in degrees.

    Returns:
        tuple: Front wheel angle (in degrees) and speed of the wheels (in meters per second).
    """
    # Extract the x, y components of the car position and end point
    car_x, car_y = car_pos
    end_x, end_y = end_point

    # Calculate the displacement vector
    d_x = end_x - car_x
    d_y = end_y - car_y

    # Convert the yaw angle from degrees to radians
    yaw_rad = math.radians(yaw_angle)

    # Calculate the direction vector
    d_dir_x = math.cos(yaw_rad) * d_x + math.sin(yaw_rad) * d_y
    d_dir_y = -math.sin(yaw_rad) * d_x + math.cos(yaw_rad) * d_y

    # Calculate the front wheel angle in radians
    theta_rad = math.atan2(d_dir_y, d_dir_x)

    theta_rad = np.clip(theta_rad,-np.radians(30),np.radians(30))
    # Calculate the length of the displacement vector
    d_length = math.sqrt(d_x**2 + d_y**2)

    # Estimate the required speed of the wheels based on the length of the displacement vector
    speed = d_length / 2
    if d_length > 0:
        speed = math.copysign(speed, d_dir_x)
    return -theta_rad, speed

def calculate_offset_point(car_position, car_yaw_degrees):
    """
    Calculate the offset point behind and to the left of the car's position.

    Args:
        car_position (list): Current position of the car as [x, y] coordinates.
        car_yaw (float): Yaw angle of the car in radians.

    Returns:
        list: The offset point as [x, y] coordinates.
    """
    # Convert the yaw angle from radians to degrees
    car_yaw = math.radians(car_yaw_degrees)

    # Calculate the offset in the car's local coordinate system
    offset_x_local = -0.5  # 0.5 meters to the left
    offset_y_local = -0.5  # 0.5 meters behind

    # Convert the local offset to the global coordinate system
    offset_x_global = offset_x_local * math.cos(car_yaw) - offset_y_local * math.sin(car_yaw)
    offset_y_global = offset_x_local * math.sin(car_yaw) + offset_y_local * math.cos(car_yaw)

    # Calculate the new point by adding the offset to the car's position
    new_point_x = car_position[0] + offset_x_global
    new_point_y = car_position[1] + offset_y_global

    # Return the new point as [x, y] coordinates
    return [new_point_x, new_point_y]


def calc_turn_command(d_th,steer):
    p = 0.01
    if steer:
        target_speed = d_th * p
    else:
        target_speed = d_th * -p
    if target_speed > 0.16:
        target_speed = 0.16
    if target_speed < -0.16:
        target_speed = -0.16
    rotation_angle_rad = -np.radians(40)

    command = SetMotion()

    # [-v,v,-v,v] Anti Clock Wise
    speeds = np.array(
        [-target_speed, target_speed, -target_speed, target_speed], dtype="float")
    angles = np.array([-rotation_angle_rad, rotation_angle_rad,
                       rotation_angle_rad, -rotation_angle_rad], dtype="float")
    command.speeds = speeds
    command.angles = angles

    return command

def command_set_exit(cur_angle, cur_pos, ox, oy):

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

    command = SetMission()
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
    

    for i in range(len(rx)):
                temp = Point2d()
                temp.cord_x = float(rx[i])
                temp.cord_y = float(ry[i])
                command.path.append(temp)
     
        

    
            
    command.speed = 0.7
    command.operation = SetMission.OPERATION_START_PATH_FOLLOWING 


    return command
  

def command_set_exit_scale_lab( cur_angle, cur_pos, ox, oy):

    #the edge of the field 
    exit_place = [-2.0, 2.5]
    rx = []
    ry = []
    path = []
    print(cur_angle)
    goal_exit = []
        
        # heading left to right
    if cur_angle < 20 and cur_angle > -20:
            grid_size_ = int(calc_distance(cur_pos,[exit_place[1],cur_pos[1]])/0.5)
            path.append(cur_pos)
            for i in range(grid_size_-1):
                rx.append(cur_pos[0] + (exit_place[1] - cur_pos[0]) / grid_size_ * i)
                ry.append(cur_pos[1])
            
            # rx.append(exit_place[1])
            # ry.append(cur_pos[1])
            goal_exit = [exit_place[1], cur_pos[1]]

        # heading right to left
    if cur_angle > 160 or cur_angle < -160:
            grid_size_ =int( calc_distance(cur_pos,[exit_place[0],cur_pos[1]]) / 0.5)
            path.append(cur_pos)
            for i in range(grid_size_-1):
                rx.append(cur_pos[0] + (exit_place[0] - cur_pos[0]) / grid_size_ * i)
                ry.append(cur_pos[1])
            # rx.append(exit_place[0])
            # ry.append(cur_pos[1])
            #path.append([exit_place[0], cur_pos[1]])
            goal_exit = [exit_place[0], cur_pos[1]]

    command = SetMission()
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
    gx = -0.0
    gy = 0.75
   

    grid_size = 0.5  # [m]
    robot_radius = 0.5  # [m]
   
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx_a, ry_a = a_star.planning(sx, sy, gx, gy)
     
    path_size = len(rx_a)
    for i in range(path_size):
        rx.append(rx_a[path_size-i-1]) 
        ry.append(ry_a[path_size-1-i])
    tck, u = interpolate.splprep([rx, ry], s=0.001)
    unew = np.arange(-0.00, 1.00, float(1/len(rx)))
    out = interpolate.splev(unew, tck)
    rx = out[0]
    ry = out[1]
    
  
    for i in range(len(rx)):
                temp = Point2d()
                temp.cord_x = float(rx[i])
                temp.cord_y = float(ry[i])
                command.path.append(temp)
     
        


    command.speed = 0.7
    command.operation = SetMission.OPERATION_START_PATH_FOLLOWING


    return command


def command_set_path(sx,sy,gx,gy,ox,oy):

    command = SetMission()
    
    grid_size = 0.5  # [m]
    robot_radius = 0.447  # [m]
    #if the distance between the start and the goal is less than 1m append the goal to the path and done 
    if calc_distance([sx,sy],[gx,gy]) < 1:
        temp = Point2d()
        temp.cord_x = sx
        temp.cord_y = sy
        command.path.append(temp)
        temp.cord_x = gx
        temp.cord_y = gy
        command.path.append(temp)
        command.speed = 0.7 
        command.operation = SetMission.OPERATION_START_PATH_FOLLOWING
        return command

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)

    path = []
    path_size = len(rx)
    for i in range(path_size):
        path.append([rx[path_size-i-1], ry[path_size-1-i]])

    


    for point in path:
            temp = Point2d()
            temp.cord_x = point[0]
            temp.cord_y = point[1]
            command.path.append(temp)
        
            


            
    command.speed = 0.7 
    command.operation = SetMission.OPERATION_START_PATH_FOLLOWING

    return command


def command_cancel_path():

    command = SetMission()
    

    command.speed = 0.7 
    command.operation = SetMission.OPERATION_CANCEL_PATH_FOLLOWING

    return command



def decompose_euclidean_path(point_a, point_b):
    euclidean_distance = calc_distance(point_a, point_b)
    x1 = point_a[0]
    y1 = point_a[1]
    x2 = point_b[0]
    y2 = point_b[1]
    # Calculate the angle of the triangle
    delta = math.atan2(y2 - y1, x2 - x1)
    #with delta calculate the length of the x and y components
    x = euclidean_distance * math.cos(delta)
    y = euclidean_distance * math.sin(delta)
    result = abs(x) + abs(y)
    return result,euclidean_distance


def calculate_distance_path(path):
    distance = 0
    for i in range(len(path)-1):

        point_a = [path[i].cord_x,path[i].cord_y]
        point_b = [path[i+1].cord_x,path[i+1].cord_y]
        distance += calc_distance(point_a,point_b)

    return distance