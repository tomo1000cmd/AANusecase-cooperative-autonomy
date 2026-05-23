from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    supervisor_control_dir = get_package_share_directory('supervisor_control')
    
    # Launch arguments
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='robot_1',
        description='Robot identifier'
    )
    
    policy_file_arg = DeclareLaunchArgument(
        'policy_file',
        default_value='policies.yaml',
        description='Path to policy configuration file'
    )

    # Supervisory control node
    supervisor_node = Node(
        package='supervisor_control',
        executable='supervisory_control_node.py',
        name='supervisor_control',
        namespace=LaunchConfiguration('robot_id'),
        output='screen',
        emulate_tty=True,
        parameters=[
            {'robot_id': LaunchConfiguration('robot_id')},
            {'policy_file': LaunchConfiguration('policy_file')},
            {'health_threshold': 0.3},
            {'centralized_timeout': 5.0},
        ],
    )

    return LaunchDescription([
        robot_id_arg,
        policy_file_arg,
        supervisor_node,
    ])
