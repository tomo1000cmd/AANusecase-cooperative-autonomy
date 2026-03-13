#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from H2robot import Harvester, Chaserbin
from helper import quart_to_rpy
import umsgpack
import numpy as np
#change it to xml 
import uuid
import time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from builtin_interfaces.msg import Time
from rclpy.parameter import Parameter
from mas_msgs.msg import VehicleStatus, FleetStatus, CAcommand, Task, Forecast, Status


HARVESTER_ID = "Robot1"
CHASERBIN_SWITCH_THRESHOLD = 70
class ControlMode:
    harvesting_chasing = 1
    idle = 0
    switch = 2

class ComposeTask:
    COMPOSED_TASK_TYPE_HARVESTING_CHASING = 0
    COMPOSED_TASK_TYPE_SWITCHING = 1

def num_from_string(s):
    return int(''.join(filter(str.isdigit, s)))
class BtHandler(Node):
    """
    This class is a ROS2 node.

    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('ca_node_test')
        # force simulation mode
        simtime =Parameter("use_sim_time", Parameter.Type.BOOL, True)
        self.set_parameters([simtime])  
        # Parameters
        self.declare_parameter('my_id','server')
        self.my_id = self.get_parameter('my_id').value

        self.declare_parameter('number_of_robots', 3)
        self.number_of_robots = self.get_parameter(
            'number_of_robots').value

        # Initialize variables
        self.robotIDs = [""] * self.number_of_robots
        for i in range(self.number_of_robots):
            self.robotIDs[i] = "Robot" + (i).__str__()

        self.isFleetnewmessage = False

        self.CAcommand = CAcommand()
        self.Harvester = Harvester(self.robotIDs[1])
        self.Chaserbins = [
            Chaserbin(self.robotIDs[0]), Chaserbin(self.robotIDs[2])]

        self.running_chaserbin = None
        self.idle_chaserbin = []


        buffer_time = Time()
        buffer_time.sec = 2
        self.tf_buffer = Buffer(buffer_time)
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.Robot_frames = {}
        self.linked_frame = None
        self.chasers_frames = ["Robot0_base_link", "Robot2_base_link"]

        # set obstacle positions
       
        self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 
        
        # Create subscriber(s)
        self.get_fleetstatus = self.create_subscription(
            FleetStatus,
            self.my_id + '/Fleet_status_message', 
            self.on_update_fleet_status,
            10)

        self.publish_CAcommand = self.create_publisher(
            CAcommand,
            self.my_id + '/ca_command',
            10)


        ## Create timer(s)
        self.timer_main = self.create_timer(0.5, self.on_timer_main)
        self.timer_refresh = self.create_timer(1.0, self.on_refresh_id)

        # Finally
        self.get_logger().info("initialized")


    # """
    #     Subscribers callback ------------------------------------------------------------------------

    # """
    def on_refresh_id(self):
        self.mission_id =  int( uuid.uuid4().hex, 16) % 32768 

    def on_update_fleet_status(self, msg=FleetStatus()):
        # """
        # Callback function that gets a fleet status

        # """

        #self.get_logger().info("[Node: ] " + format(self.feetstatus))
        self.Harvester.update_status(msg.vehicles_status[1])
        self.Harvester.liveness_check()

        self.Chaserbins[0].update_status(msg.vehicles_status[0])
        self.Chaserbins[1].update_status(msg.vehicles_status[2])

        for chaserbin in self.Chaserbins:
            chaserbin.liveness_check()

    def simple_switch(self):
        print("simple switch",self.running_chaserbin.check_cabin_remaing())
        print("simple switch",self.running_chaserbin.my_id)
        if self.running_chaserbin.check_cabin_remaing() > CHASERBIN_SWITCH_THRESHOLD:
            self.send_switch_command(self.idle_chaserbin[0].my_id)

    def send_switch_command(self, chaserbin_id):
        
        task_ = Task()
        # Generate a random UUID
        random_id = uuid.uuid4()

        # Convert UUID to int16
        int16_value_uuid = self.mission_id
        task_.uuid = int16_value_uuid

        task_.task_api = Task.TASK_API_COMPOSED_TASK
        task_.task_status = Task.TASK_STATUS_TODO
        task_.timestamp = self.get_clock().now().to_msg().sec 

        harvester_chasing_index = 0

        harvesting_chasing_velocity = 0.16
        compose_task_type = ComposeTask.COMPOSED_TASK_TYPE_SWITCHING
        task_info_bytes = umsgpack.packb([compose_task_type,harvesting_chasing_velocity,harvester_chasing_index])

        task_.task_info = task_info_bytes

        
        command = CAcommand()
        command.vehicle_id = chaserbin_id    
            
        command.operation = CAcommand.OPERATION_ADD_TIMETABLE
        command.task = task_

        self.get_logger().info("[Node]: Sending switch command to " + chaserbin_id)
        self.publish_CAcommand.publish(command)

    def on_timer_main(self):
        self.on_frame_listener()
        if (self.Harvester.is_alive and self.Chaserbins[0].is_alive and self.Chaserbins[1].is_alive) and len(self.Robot_frames)>=2:
            self.linked_frame = self.check_if_chaserbin_is_linked()
            self.idle_chaserbin.clear()
            ## use the linked frame to know the running chaserbin
            for chaserbin in self.Chaserbins:
                if chaserbin.my_id + '_base_link' == self.linked_frame:
                    self.running_chaserbin = chaserbin
                else:
                    if chaserbin.my_status.status.kpis[2] == ControlMode.idle:
                        self.idle_chaserbin.append(chaserbin)
            
            if self.running_chaserbin is not None and len(self.idle_chaserbin) > 0:
                self.simple_switch()
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
