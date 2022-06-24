import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

log_level = "info"

def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('merge_point_cloud'),
            'param',
            'config.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file for merge point cloud'),

        Node(
            package='merge_point_cloud',                                                                                             
            executable='merge_point_cloud',
            name='merge_point_cloud',
            parameters=[param_dir],
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],)
    ])
