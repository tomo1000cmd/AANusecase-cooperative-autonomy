import rclpy
import os
import time
import sys
import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses,Wait,BackUp,DriveOnHeading
from geometry_msgs.msg import PoseStamped, PolygonStamped, Point32,Twist,Point,Polygon,Quaternion
from builtin_interfaces.msg import Duration
from nav_msgs.msg import Odometry,Path
import tkinter as tk
from tkinter import ttk
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
# Docking parameters
approx_docking_location = [28.0, 3.0]
# Coverage parameters
field_start = [2.5, 22.0]
field_size = [10.0, 10.0]
lane_width = 1.7
headland_width = 1.0
# Row parameters (x, y, East/West)
row_starts = [[19.0, 23.5, 'E'],
              [33.0, 25.0, 'W'],
              [19.0, 26.5, 'E']
              ] 

LOG_FILE = "task_log.txt"

def log_to_file(task_name, eta, distance_remaining, progress_percentage):
    """Log ETA and task progress to a file."""
    with open(LOG_FILE, "a") as f:
        f.write(f"{task_name}: ETA = {eta:.2f} sec, Distance Remaining = {distance_remaining:.2f} m, Progress = {progress_percentage:.2f}%\n")


class DockingClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('aan_docking_client')
        

    def start(self, robot_name):

        # Start action client
        self.nav_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')
        

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_docking.xml')

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = approx_docking_location[0]
        pose_msg.pose.position.y = approx_docking_location[1]
        goal_msg.pose = pose_msg

        
        self.get_logger().info('Waiting for Nav2 action server to come online...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available, sending docking goal...')
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Done!")
        self.status = 'done'
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining 
        self.total_distance = 100
        self.cmd_vel= 0.6
              #Distance left to cover in meters
        progress_percentage = 100 - ((distance_remaining / self.total_distance) * 100)
        eta = distance_remaining / self.cmd_vel if self.cmd_vel > 0 else float('inf')
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        self.get_logger().info(f"Progress: {progress_percentage:.2f}%", throttle_duration_sec=1)
        self.get_logger().info(f"ETA: {eta:.2f} seconds", throttle_duration_sec=1)

        log_to_file("Docking Task", eta, distance_remaining, progress_percentage)
        time.sleep(1)

class FieldCoverClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('aan_field_cover_client')

        # Init path
        self.path = []


    def start(self, robot_name):

        # Start action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, robot_name+'/navigate_through_poses')

        # Publishers
        self.field_publisher = self.create_publisher(PolygonStamped, '/field_polygon', 10)
        self.headland_publisher = self.create_publisher(PolygonStamped, '/headland_polygon', 10)
        self.total_distance = 100

    def publish_polygons(self, field_start, field_size, headland_width):

        field_polygon = [[field_start[0], field_start[1]], 
                         [field_start[0], field_start[1] + field_size[1]], 
                         [field_start[0] + field_size[0], field_start[1] + field_size[1]], 
                         [field_start[0] + field_size[0], field_start[1]]]
        
        field_polygon_msg = self.polygon_to_msg(field_polygon)
        self.field_publisher.publish(field_polygon_msg)
        
        headland_polygon = [[field_start[0] - headland_width, field_start[1] - headland_width], 
                            [field_start[0] - headland_width, field_start[1] + field_size[1] + headland_width], 
                            [field_start[0] + field_size[0] + headland_width, field_start[1] + field_size[1] + headland_width], 
                            [field_start[0] + field_size[0] + headland_width, field_start[1] - headland_width]]
        
        headland_polygon_msg = self.polygon_to_msg(headland_polygon)
        self.headland_publisher.publish(headland_polygon_msg)     


    def polygon_to_msg(self, polygon):
        
        msg = PolygonStamped()
        msg.header.frame_id = "map"

        for point in polygon:
            point_msg = Point32()
            point_msg.x = point[0]
            point_msg.y = point[1]
            msg.polygon.points.append(point_msg)

        return msg


    def plan_path(self, field_start, field_size, lane_width, headland_width):

        n_lanes = 0
        self.path = [[field_start[0] + lane_width/2.0, field_start[1] - headland_width]]

        while n_lanes < 6:
            self.path.append([field_start[0] + lane_width/2.0 + n_lanes*lane_width, field_start[1] - headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + n_lanes*lane_width, field_start[1] + field_size[1] + headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + (n_lanes+1)*lane_width, field_start[1] + field_size[1] + headland_width/2.0])
            self.path.append([field_start[0] + lane_width/2.0 + (n_lanes+1)*lane_width, field_start[1] - headland_width/2.0])
            n_lanes += 2
    

    def send_start(self):

        # First send starting position to NavigateToPose Navigator
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_to_pose.xml')
        
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.pose.position.x = self.path[0][0]
        goal_msg.pose.pose.position.y = self.path[0][1]
        goal_msg.pose.pose.orientation.z = 0.7
        goal_msg.pose.pose.orientation.w = 0.7

        self.get_logger().info('Waiting for Nav2 NavigateToPose action server to come online...')
        self.nav_to_pose_client.wait_for_server()
        self.get_logger().info('NavigateToPose action server available, sending start pose as goal...')
        future = self.nav_to_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        # Done callback will send coverage path once start is reached
        future.add_done_callback(self.nav_to_pose_goal_response_callback)


    def nav_to_pose_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_to_pose_get_result_callback)


    def nav_to_pose_get_result_callback(self, future):
        self.get_logger().info(f"Reached start of coverage path")
        self.send_coverage_path()


    def send_coverage_path(self):

        # Send coverage path to NavigateThroughPoses navigator
        goal_msg = NavigateThroughPoses.Goal()
         
        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_exact_path.xml')
        
        for point in self.path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = point[0]
            pose_msg.pose.position.y = point[1]
            pose_msg.pose.orientation.z = -0.7
            pose_msg.pose.orientation.w = 0.7
            goal_msg.poses.append(pose_msg)

        self.get_logger().info('Waiting for Nav2 NavigateThroughPoses action server to come online...')
        self.nav_through_poses_client.wait_for_server()
        self.get_logger().info('NavigateThroughPoses action server available, sending field coverage path...')
        future = self.nav_through_poses_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.nav_through_poses_goal_response_callback)


    def nav_through_poses_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.nav_through_poses_get_result_callback)


    def nav_through_poses_get_result_callback(self, future):
        self.get_logger().info(f"Done!")
        self.status = 'done'
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining 
        #Distance left to cover in meters
        progress_percentage = 100 - ((distance_remaining / self.total_distance) * 100)
        self.cmd_vel= 0.6
        #estimated time of arrival
        eta = (distance_remaining / self.cmd_vel) if self.cmd_vel > 0 else float('inf')

        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        self.get_logger().info(f"Progress: {progress_percentage:.2f}%", throttle_duration_sec=1)
        self.get_logger().info(f"ETA: {eta:.2f} seconds", throttle_duration_sec=1)
        # Publish field polygons
        self.publish_polygons(field_start, field_size, headland_width)
        log_to_file("field cover Task", eta, distance_remaining, progress_percentage)

        time.sleep(1)
        
class RowFollowClient(Node):

    def __init__(self):

        # Start node 
        super().__init__('row_follow_client')

    def start(self, robot_name):

        # Start action client
        self.nav_client = ActionClient(self, NavigateToPose, robot_name+'/navigate_to_pose')

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()

        goal_msg.behavior_tree = os.path.join(
            get_package_share_directory('aan_navigation_clients'), 
            'behavior_trees', 'navigate_row_following.xml')

        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = row_starts[0][0]
        pose_msg.pose.position.y = row_starts[0][1]
        if row_starts[0][2] == 'E':
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
        elif row_starts[0][2] == 'W':
            pose_msg.pose.orientation.z = 1.0
            pose_msg.pose.orientation.w = 0.0
        goal_msg.pose = pose_msg

        self.get_logger().info('Waiting for Nav2 action server to come online...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server available, sending row following goals...')
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2 server')
            return

        self.get_logger().info('Goal accepted by Nav2 server, executing... ')

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        if len(row_starts) > 1:
            self.get_logger().info(f"Row completed, moving to next...")
            row_starts.pop(0)
            self.send_goal()
            return

        self.get_logger().info(f"Done!")
        self.status = 'done'
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance_remaining = feedback.distance_remaining 
        self.total_distance = 100
        self.cmd_vel= 0.6
              #Distance left to cover in meters
        progress_percentage = 100 - ((distance_remaining / self.total_distance) * 100)
        eta = distance_remaining / self.cmd_vel if self.cmd_vel > 0 else float('inf')
        self.get_logger().info(f"Navigating... Distance remaining: {int(feedback.distance_remaining)} meters", 
                               throttle_duration_sec=1)
        self.get_logger().info(f"Progress: {progress_percentage:.2f}%", throttle_duration_sec=1)
        self.get_logger().info(f"ETA: {eta:.2f} seconds", throttle_duration_sec=1)
        log_to_file("Row follow task Task", eta, distance_remaining, progress_percentage)
    
        time.sleep(1)

  
  


def wait_for_task_to_complete(client):
    """Generic wait function for task completion."""
    while rclpy.ok():
        rclpy.spin_once(client)
        if getattr(client, 'status', None) == 'done':  # Check client status
            print(f"{client.get_name()} task completed")
            break


def main(args=None):
    rclpy.init(args=args)  # Single initialization
    
    try:
        robot_name = sys.argv[1] if len(sys.argv) > 1 else "diffbot_1"
        
 
 # --- Row Following Task
        row_follow_client = RowFollowClient()
        row_follow_client.start(robot_name) 
        row_follow_client.send_goal()
       
        wait_for_task_to_complete(row_follow_client)
        row_follow_client.destroy_node()
                  # --- Docking Task ---
        rclpy.init(args=args)
        
        docking_client = DockingClient()
        docking_client.start(robot_name)
        docking_client.send_goal()
     
        wait_for_task_to_complete(docking_client)
        docking_client.destroy_node()
        
          # --- Field Coverage Task ---
        rclpy.init(args=args) 
        field_cover_client = FieldCoverClient()
        field_cover_client.start(robot_name)
        field_cover_client.publish_polygons(field_start, field_size, headland_width)
        field_cover_client.plan_path(field_start, field_size, lane_width, headland_width)
        field_cover_client.send_start()
        
        wait_for_task_to_complete(field_cover_client)
        field_cover_client.destroy_node()
        

      

    except Exception as e:
      print(f"An error occurred: {e}")
    finally:
     if rclpy.ok(): 
        rclpy.shutdown()



if __name__ == '__main__':
    main()

