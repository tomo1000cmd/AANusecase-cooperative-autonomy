#!/usr/bin/env python3

from time import sleep
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from bt_msgs.action import RoleLelyVoa


class VehicleOperatingAgent(Node):
    """
    This class is a ROS2 node.
    It is the central node of the Vehicle Operating Agent.
    As an action client of the voa_bt node it activates the behavior tree execution. 
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        super().__init__('voa_main')

        # Parameters
        self.declare_parameter('executing_interval')  # seconds
        self.executing_interval = self.get_parameter('executing_interval').value
        self.declare_parameter('my_id')
        self.my_id = self.get_parameter('my_id').value

        # Create action client
        self.action_client = ActionClient(self, RoleLelyVoa, 'voa_role')

        # Finally
        self.get_logger().info("vehicle '{}' initialized".format(self.my_id))

    def send_goal(self):
        goal_msg = RoleLelyVoa.Goal()
        goal_msg.role = self.my_id

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)

        self.send_goal_future.add_done_callback(self.goal_response_callback)

        #self.get_logger().info('goal sent')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected :(')
            return

        #self.get_logger().info('goal accepted :)')

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info('action completed')
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            pass
            #self.get_logger().info('goal succeeded!')
        else:
            self.get_logger().info('goal failed with status code: {0}'.format(status))
        


def main(args=None):

    rclpy.init(args=args)
     # Create the node
    voa = VehicleOperatingAgent()

    while 1:
        try:
            voa.send_goal()
            # Spin the node so the callback functions are called.
            rclpy.spin_once(voa)
            sleep(voa.executing_interval)
        except BaseException:
            pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
