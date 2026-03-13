
from agent_msgs.msg import CAcommand,Forecast
import numpy as np
import math
import copy
from h2trac_msgs.msg import SetMotion,Point2d,SetMission


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


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw



def command_set_exit( cur_angle, cur_pos, robotid, ox, oy):


    exit_place = [-1.0, 9.0]
    path = []
    print(cur_angle)
    goal_exit = []
    # heading left to right
    if cur_angle < 20 and cur_angle > -20:

        path.append(cur_pos)
        for i in range(10):
            path.append(
                [cur_pos[0] + (exit_place[1] - cur_pos[0]) / 10 * i, cur_pos[1]])
        #path.append([exit_place[1], cur_pos[1]])
        goal_exit = [exit_place[1], cur_pos[1]]

    # heading right to left
    if cur_angle > 160 or cur_angle < -160:

        path.append(cur_pos)
        for i in range(10):
            path.append(
                [cur_pos[0] + (exit_place[0] - cur_pos[0]) / 10 * i, cur_pos[1]])
        #path.append([exit_place[0], cur_pos[1]])
        goal_exit = [exit_place[0], cur_pos[1]]

    command = CAcommand()
    # astar
    try:
        sx = goal_exit[0]
        sy = cur_pos[1]
    except BaseException:
        sx = cur_pos[0]
        sy = cur_pos[1]
    gx = -5.0
    gy = -1.0
    from Astar import AStarPlanner

    grid_size = 0.5  # [m]
    robot_radius = 0.5  # [m]
   
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)


    path_size = len(rx)
    for i in range(path_size):
        path.append([rx[path_size-i-1], ry[path_size-1-i]])

    for point in path:
        temp = Point2d()
        temp.cord_x = float(point[0])
        temp.cord_y = float(point[1])
        command.path.append(temp)

    command.vehicle_id = robotid
    command.operation = CAcommand.OPERATION_SET_MISSION

    command.acceration = 0.6
    command.mission_operation = SetMission.OPERATION_START_PATH_FOLLOWING

    return command
  



def command_set_path(sx,sy,gx,gy,ox,oy):

    command = Forecast()
    
    from Astar import AStarPlanner

    grid_size = 0.25  # [m]
    robot_radius = 0.447  # [m]


    a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
    rx, ry = a_star.planning(sx, sy, gx, gy)



    path = []
    path_size = len(rx)
    for i in range(path_size):
        path.append([rx[path_size-i-1], ry[path_size-1-i]])

    for point in path:
        temp = Point2d()
        temp.cord_x = float(point[0])
        temp.cord_y = float(point[1])
        command.path.append(temp)

    command.acceration = 0.7

    return command

