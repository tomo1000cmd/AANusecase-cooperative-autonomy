
from agent_msgs.msg import VehicleStatus
import math
from helper import  quart_to_rpy
class H2_robot(object):
    my_status = VehicleStatus()
    is_alive = False

    def __init__(self, robotid="None"):
        self.my_id = robotid
        self.timestamps = [0] * 9

    def update_status(self, Vehciestatus=VehicleStatus()):
        if self.my_id == Vehciestatus.vehicle_id:
            self.my_status = Vehciestatus

    def liveness_check(self):
        self.timestamps[0] = self.my_status.header.stamp.sec
        self.is_alive = (self.timestamps[0] != self.timestamps[4])
        for i in range(8):
            self.timestamps[8-i] = self.timestamps[7-i]
        return self.is_alive

    def get_pos(self):
        cur_pos = [self.my_status.pose.position.x,
                   self.my_status.pose.position.y]

        return cur_pos

    def get_angle(self):
        roll, pitch, yaw = quart_to_rpy(self.my_status.pose.orientation.x, self.my_status.pose.orientation.y,
                                        self.my_status.pose.orientation.z, self.my_status.pose.orientation.w)
        cur_angle = math.degrees(yaw)
        return cur_angle

    def get_operation_status(self):
        return self.my_status.status.controller_status


class Harvester(H2_robot):
    pass


class Chaserbin(H2_robot):
    def check_cabin_remaing(self):
        return self.my_status.status.storage_cabin_status