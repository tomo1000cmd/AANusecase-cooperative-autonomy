#!/usr/bin/env python3
from agent_msgs.msg import VehicleStatus



class LelyCollector(object):
    """
    This class is a LelyCollector;

    """
    my_status = VehicleStatus()
    is_error = False
    is_alive = True
    is_available = True
    is_takeover = False
    is_idle = True

    def __init__(self,robotid = "None"):
        self.my_id = robotid
        self.timestamps = [0] * 9

    def liveness_check(self):
        self.timestamps[0] = self.my_status.header.stamp.sec
        self.is_alive = (self.timestamps[0] != self.timestamps[4])
        for i in range(8):
            self.timestamps[8-i] = self.timestamps[7-i]
        return self.is_alive

    def error_check(self):   
        self.is_error = (self.my_status.status.controller_status == "ERROR")
        return self.is_error

    def availability_check(self):   
        self.is_available = not(self.is_error) and self.is_alive
        return self.is_available

    def update_status(self, Vehciestatus = VehicleStatus()):  
        if self.my_id == Vehciestatus.vehicle_id : 
            self.my_status = Vehciestatus

    def get_current_working_area(self):
        worling_area = 0
        if self.my_status.status.route_id > 0 :
            worling_area = self.my_status.status.route_id
        return worling_area

    def takeover_check(self): 
        self.is_takeover = self.my_status.is_takeover

    def idle_check(self):
        missionstate = str(self.my_status.status.controller_status)
        self.is_idle = False
            ## if statement to see if the robot is idle
        if ((missionstate == "MANUAL" or 
                                missionstate == "IDLE" or 
                                missionstate == "SERVICE" ) and
                                self.my_status.status.charger_connected == True and
                                self.my_status.status.operation_status == "out" 
                                ):
                                self.is_idle = True

            
      
    def resetStatus(self):
       self.my_status = VehicleStatus()
       

    def self_diagnostic(self):  
        self.error_check()
        self.liveness_check()
        self.availability_check()
        self.idle_check()
