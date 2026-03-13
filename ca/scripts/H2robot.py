
from mas_msgs.msg import VehicleStatus
import math
from helper import  quart_to_rpy
class H2_robot(object):
    my_status = VehicleStatus()
    is_alive = False

    def __init__(self, robotid="None"):
        self.my_id = robotid
        self.timestamps = [0] * 21

    def update_status(self, Vehciestatus=VehicleStatus()):
        if self.my_id == Vehciestatus.status.vehicle_id:
            self.my_status = Vehciestatus

    def liveness_check(self):
        self.timestamps[0] = self.my_status.pose.header.stamp.sec
        self.is_alive = (self.timestamps[0] != self.timestamps[19])
        for i in range(20):
            self.timestamps[20-i] = self.timestamps[19-i]
        return self.is_alive

    def get_pos(self):
        cur_pos = [self.my_status.pose.pose.position.x,
                   self.my_status.pose.pose.position.y]

        return cur_pos

    def get_angle(self):
        roll, pitch, yaw = quart_to_rpy(self.my_status.pose.pose.orientation.x, self.my_status.pose.pose.orientation.y,
                                        self.my_status.pose.pose.orientation.z, self.my_status.pose.pose.orientation.w)
        cur_angle = math.degrees(yaw)
        return cur_angle

    def get_operation_status(self):
        return self.my_status.status.current_task.task_status


class Harvester(H2_robot):
    pass


class Chaserbin(H2_robot):
    def check_cabin_remaing(self):
        return self.my_status.status.kpis[0]