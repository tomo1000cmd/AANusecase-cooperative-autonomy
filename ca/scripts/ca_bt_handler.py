#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from agent_msgs.msg import VehicleStatus,FleetStatus,CAcommand,OperationControl,Task,Forecast
from agent_msgs.srv import StandardAction
from bt_msgs.srv import LocationServer
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
from LelyCollector import LelyCollector

def get_settings():
    static_routing = "false"
    bringup_dir = get_package_share_directory('fa')
    path= bringup_dir + '/config/' + 'setting' + '.xml'
        
    tree = ET.parse(path)
    root = tree.getroot()
    
    for data in root:
        onedict = data.attrib   
        static_routing = onedict['static_routing']   
    
    
    static_routing = eval(static_routing)
    return bool(static_routing)


class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('ca_bt_handler')

        # Parameters
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value

        self.declare_parameter('number_of_collectors', 2)
        self.number_of_collectors = self.get_parameter('number_of_collectors').value
        
        # Initialize variables
        self.robotIDs = [""] * self.number_of_collectors
        for i in range(self.number_of_collectors):
            self.robotIDs[i] = "robot" + (i+1).__str__()

        self.feetstatus = FleetStatus()
        self.isFleetnewmessage = False

        self.is_static_routing = get_settings()
        self.collectors = []

        for robotID in self.robotIDs:
            self.collectors.append(LelyCollector(robotID))

        self.CAcommand = CAcommand()
        self.OperationControl = OperationControl()

        self.forcast = Forecast()
        self.is_new_forcast = False
        self.forcast_id = [int()]*2
        # Create subscriber(s)  
        self.get_fleetstatus = self.create_subscription(
            FleetStatus,
            'Fleet_status_message',
            self.on_update_fleet_status,
            10) 
        self.get_forcast = self.create_subscription(
            Forecast,
            'forecast_message',
            self.on_update_forcast,
            10) 
        
        
        # Create publisher(s)
        self.publish_CAcommand = self.create_publisher(
            CAcommand, 
            'CA_command', 
            10)

        self.publish_OperationControl = self.create_publisher(
            OperationControl, 
            'operation_control', 
            10)
        
        # Create service(s) for bt coditions
        
        self.is_message_get_service = self.create_service(
            StandardAction, "is_fleet_message_get", self.on_is_fleet_message_get_srv)

        self.is_error_service = self.create_service(
            StandardAction, "is_error", self.on_is_error_srv)

        self.is_resume_service = self.create_service(
            StandardAction, "is_resume", self.on_is_resume_srv)
        # Create service(s) for bt actions
      
        self.send_send_command_service = self.create_service(
            StandardAction, "send_command", self.on_send_command_srv)

        #self.send_emgency_service = self.create_service(
         #   StandardAction, "send_emgency_status", self.on_send_emgency_Status_srv)

        

        # Finally
        self.get_logger().info("initialized")
        if self.is_static_routing:
            self.get_logger().info("Static routing on")
        else:
            self.get_logger().info("Dynamic routing on")

        
    """
        Subscribers callback ------------------------------------------------------------------------

    """
    
    def on_update_fleet_status(self, msg):
        """
        Callback function that gets a fleet status

        """
        self.feetstatus = msg
        #self.get_logger().info("[Node: ] " + format(self.feetstatus))
        self.isFleetnewmessage = True


    
    def on_update_forcast(self,msg):
        """
        Callback function that gets a forcast

        """
        self.is_new_forcast = False
        
        self.forcast = msg
        self.forcast_id[0] = self.forcast.most_urgent_routeid 
        if(self.forcast_id[0] != self.forcast_id[1]):
            self.is_new_forcast = True
        self.forcast_id[1] = self.forcast_id[0]


    """
        BT conditions service callback------------------------------------------------------------------------

    """
    def on_is_fleet_message_get_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

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

    def on_is_resume_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

        """
      
        response.success = False
        if (self.collectors[0].is_available and self.collectors[1].is_available):
            if (self.collectors[0].is_takeover or self.collectors[1].is_takeover):
                takeover_id = ""
                for collector in self.collectors:
                    if collector.is_takeover:
                        takeover_id = collector.my_id
                self.CAcommand.vehicle_id = takeover_id
                self.CAcommand.operation = CAcommand.OPERATION_RESUME_TIMETABLE
                self.get_logger().info("MESSAGE :Resume the task list from collector :"+ takeover_id + ". ") 
                self.forcast_id[1] = -1
                response.success = True         

        
        return response
    
    def on_is_error_srv(self, request, response):
        """
        Callback function that gets called on receiving a on_is_message_get request.

        """
        
        if self.is_static_routing:
            self.static_routing(response)
        else:    
            self.dynamic_routing(response)
            

        return response


    """
        BT actions service callback------------------------------------------------------------------------

    """
    def on_send_command_srv(self, request, response):
        """
        Callback function that gets called on receiving a send_Status request.

        """
        self.publish_CAcommand.publish(self.CAcommand)
        
        self.get_logger().info("MESSAGE : Command: "+ format(self.CAcommand)+"sent")
        
        response.success = True
       

        return response


    """
        Standard membership functions------------------------------------------------------------------------

    """
    def static_routing(self,response):
        decision_table = [bool(0)] * ((self.number_of_collectors) * (self.number_of_collectors))


        decision_table[0]   = self.collectors[0].is_available and self.collectors[1].is_available 
        decision_table[1]   = self.collectors[0].is_available and not(self.collectors[1].is_available) 
        decision_table[2]   = not(self.collectors[0].is_available) and self.collectors[1].is_available 
        decision_table[3]   = not(self.collectors[0].is_available) and not(self.collectors[1].is_available) 

        for collector in self.collectors:
            collector.takeover_check()

        response.success = False
        if (decision_table[0]) : 
            #no problem
            response.success = False
            self.get_logger().info("MESSAGE : Both collector is ok!") 


        if (decision_table[1] and (self.collectors[0].is_takeover == False)) : 
            response.success = True
            self.CAcommand.vehicle_id = self.collectors[0].my_id
            self.CAcommand.operation = CAcommand.OPERATION_ALTER_TIMETABLE
            self.CAcommand.blockedareas = [self.collectors[1].get_current_working_area()]
            self.get_logger().info("MESSAGE : collector 2 is down, alter the tasklist for collector 1  ") 

        if (decision_table[2] and (self.collectors[1].is_takeover == False)) : 
            response.success = True
            self.CAcommand.vehicle_id = self.collectors[1].my_id
            self.CAcommand.operation = CAcommand.OPERATION_ALTER_TIMETABLE
            self.CAcommand.blockedareas = [self.collectors[0].get_current_working_area()]
            self.get_logger().info("MESSAGE : collector 1 is down, alter the tasklist for collector 2  ") 

        if (decision_table[3]) : 
            for i in range(self.number_of_collectors):
                self.collectors[i].resetStatus()
            self.CAcommand.blockedareas = [0]
            self.get_logger().error("ERROR : Both collector is down! PLEASE contact the system admin. ") 
            response.success = False

        return response
       # self.get_logger().info(format(self.collectors[0].is_takeover)) 

    def dynamic_routing(self,response):
        response.success = False

        for collector in self.collectors:
            collector.takeover_check()

        #find the working collector(s)
        available_collector = []
        not_available_collector = []
        broken_collector = []
        for collector in self.collectors:
            if collector.is_available and collector.is_idle:
                available_collector.append(collector.my_status.vehicle_id)
            elif collector.is_available == False:
                broken_collector.append(collector.my_status.vehicle_id)
            elif collector.is_idle == False:
                not_available_collector.append(collector.my_status.vehicle_id)

        if broken_collector == []:
            self.get_logger().info("MESSAGE : Every collector is ok") 
        else:
            self.get_logger().info("MESSAGE : The following collector is broken: "+ format(broken_collector))
            self.get_logger().info("MESSAGE : The following collector is at service: "+ format(not_available_collector))

            # only ask the FA when there is collector availble
            if available_collector != []:
            
                Forecast_command = CAcommand()
                Forecast_command.operation = CAcommand.OPERATION_ASK_FOR_URGENT_ID
                self.publish_CAcommand.publish(Forecast_command)
                #see if there is new forcast respond
                if self.is_new_forcast:
                  
                    self.get_logger().info("MESSAGE : The new forcast is : "+ format(self.forcast)+" prepare sending")

                    #select the optimal robot
                    optimal_robot = available_collector[0]
                    for collector in available_collector:
                        if self.forcast.perfered_robotid == collector:
                            optimal_robot = collector

                        #preapare the sending sequnece 
                    self.CAcommand.vehicle_id = optimal_robot
                    self.CAcommand.operation = CAcommand.OPERATION_ADD_TIMETABLE
                    self.CAcommand.task.route_id = self.forcast.most_urgent_routeid
                    self.CAcommand.task.task_status = Task.TASK_STATUS_TODO_TAKEOVER

                    response.success = True
                    self.is_new_forcast = False



        return response
       
       



  


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
