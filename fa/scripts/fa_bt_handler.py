#!/usr/bin/env python3

from email.policy import default
from click import command
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agent_msgs.msg import VehicleStatus,FleetStatus,Forecast,CAcommand,Task
from agent_msgs.srv import StandardAction
from bt_msgs.srv import LocationServer
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET

class LelyCollector(object):
    """
    This class is a class for represent lelycollector. 

    """
    my_status = VehicleStatus()
    is_error = False
    is_alive = True
    is_available = True
    is_takeover = False

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
      
    def resetStatus(self):
       self.my_status = VehicleStatus()
       

    def self_diagnostic(self):  
        self.error_check()
        self.liveness_check()
        self.availability_check()


class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('fa_bt_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value
        self.declare_parameter('number_of_collectors', 2)
        self.number_of_collectors = self.get_parameter('number_of_collectors').value
    
        # Initialize variables
        self.robotIDs = [""] * self.number_of_collectors
        for i in range(self.number_of_collectors):
            self.robotIDs[i] = "robot" + (i+1).__str__()

        self.collectors = []

        for robotID in self.robotIDs:
            self.collectors.append(LelyCollector(robotID)) 

    
        self.feetstatus = FleetStatus()
        self.isFleetnewmessage = False

        self.command_ca = CAcommand()
       
        self.FAcommand = Forecast()
        self.isnewcommandCA = False


        # Create subscriber(s)
        self.get_fleetstatus = self.create_subscription(
            FleetStatus,
            'Fleet_status_message',
            self.on_update_fleet_status,
            10) 
            
        self.get_CAcommnad = self.create_subscription(
            CAcommand,
            'CA_command',
            self.on_update_cacommand,
            10) 
        
        # Create publisher(s)
        self.publish_forcast = self.create_publisher(
            Forecast, 
            'forecast_message', 
            10)
        
        # Create service(s) for bt coditions
        
        self.is_request_get_service = self.create_service(
            StandardAction, "is_request_get", self.on_is_request_get_srv)
         
        self.is_fleet_message_get_fa_service = self.create_service(
            StandardAction, "is_fleet_message_get_fa", self.on_is_fleet_message_get_fa_srv)

        # Create service(s) for bt actions
      
        self.compute_send_service = self.create_service(
            StandardAction, "compute_send", self.on_compute_send_srv)


        # Finally
        self.get_logger().info("initialized")

        
    """
        Subscribers callback ------------------------------------------------------------------------

    """
        
    def on_update_fleet_status(self, msg):
        """
        Callback function that gets called on receiving a on_update_fleet_status message.

        """
        self.feetstatus = msg
        self.isFleetnewmessage = True


    def on_update_cacommand(self,msg):
        """
        Callback function that gets called on receiving a GetCommand request.

        """
        self.isnewcommandCA = False
        if msg.operation == CAcommand.OPERATION_ASK_FOR_URGENT_ID:
            self.isnewcommandCA = True
            self.get_logger().info("[Node: ] MESSAGE: OPERATION_ASK_FOR_URGENT_ID GET")

    """
        BT conditions service callback------------------------------------------------------------------------

    """
    def on_is_request_get_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

        """
        response.success = False
        if self.isnewcommandCA:
            response.success = True
      

            

        return response


    def on_is_fleet_message_get_fa_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_fleet_message_get_fa_srv request.

        """
      
        response.success = False
        if self.isFleetnewmessage:
            response.success = True
            for i in range(self.number_of_collectors):
                self.collectors[i].update_status(self.feetstatus.vehicles_status[i])
                
        #perform event check for all the collector
        for i in range(self.number_of_collectors):
                self.collectors[i].self_diagnostic()
        return response


    """
        BT actions service callback------------------------------------------------------------------------

    """
    def on_compute_send_srv(self, request, response):
        """
        Callback function that gets called on receiving a send_Status request.

        """

        # load the master list from the disk
        ltvList = self.load_masterlist("master_list")

        #find the broken collector(s)
        broken_collector = []
        for collector in self.collectors:
            if collector.is_available == False:
                broken_collector.append(collector)

        # concatenate the different tasklist from different robots
        currtasklists = []
        for i in range(self.number_of_collectors):
            currtasklists.append(self.feetstatus.vehicles_status[i].task_list)

        for currtasklist in currtasklists:
            self.updatePlist(ltvList,currtasklist)

        

        #compute the block area 
        blockarea = []
        for collector in broken_collector:
            if collector.get_current_working_area() != 0:
                blockarea.append(collector.get_current_working_area())

        self.get_logger().info("[Node: ] The blocked area is "+ format(blockarea))

        #remove the block area from the last time visit list
        for area in blockarea:
            for ltv in ltvList:
                if area == ltv[1]:
                    ltvList.remove(ltv)
                    self.get_logger().info("[Node: ]"+ format(ltv) + " removed!")

        #sort the array using the first index from the last time vist list
        ltvList.sort(key=(lambda ltvList:ltvList[0]),reverse=True)

        #send out the message
        self.get_logger().info("[Node: ] The most urgent id list is "+ format(ltvList))
        self.FAcommand.most_urgent_routeid = ltvList[0][1]
        self.FAcommand.perfered_robotid = ltvList[0][2]
        
        self.publish_forcast.publish(self.FAcommand)

        self.isFleetnewmessage = False
        self.isnewcommandCA = False
        self.get_logger().info("[Node: ] The most urgent id "+format(self.FAcommand.most_urgent_routeid)+" sent!!")
        response.success = True

        return response


    """
        Standard membership functions------------------------------------------------------------------------

    """
        
        
    def updatePlist(self,LastTimeVistlist,Tasklist):
        """
        membership function that updates the last time vist list by comparing if the task has been finished/started before

        """

        currenttime = int(self.get_clock().now().to_msg().sec)

        for task in Tasklist:
            if (task.task_status == Task.TASK_STATUS_STARTED) or (task.task_status == Task.TASK_STATUS_SUCCESS):
                for LastTimeVist in LastTimeVistlist:
                    if task.route_id == LastTimeVist[1]:
                        LastTimeVist[0] = currenttime - task.timestamp
       
    def load_masterlist(self,filename):
        """
        membership function that loads the masterlist from the disk

        """        
        ltvList = []
        bringup_dir = get_package_share_directory('fa')
        path= bringup_dir + '/config/' + filename + '.xml'
        
        tree = ET.parse(path)
        root = tree.getroot()
        default_last_vist_time = 86400 
        for data in root:
            for child in data:
                onedict = child.attrib   
                taskId = int(onedict['taskId'])   
                preferredRobot = onedict['preferredRobot']  
                last_time_vist = [default_last_vist_time, taskId, preferredRobot]
                ltvList.append(last_time_vist)


        #self.get_logger().info("[load_masterlist]:the configration from the mastlist is: "+format(ltvList))
       


        return ltvList



  


def main(args=None):

    rclpy.init(args=args)

    # Create the node
    Bt_handler = BtHandler()

    # Spin the node so the callback functions are called.
    rclpy.spin(Bt_handler)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Bt_handler.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
