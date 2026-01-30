import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    this_pkg_name = 'guide_robot_localization'
    this_pkg_dir = get_package_share_directory(this_pkg_name)
    declare_map_path = DeclareLaunchArgument('map', default_value='maps/map.pcd')
    map_path = LaunchConfiguration('map')
    
    goal_pose_remapper = Node(
        package='guide_robot_localization',
        executable='goal_pose_remapper',
        name='goal_pose_remapper',
        parameters=[{
            'map_path':map_path, 
            'verbose': True
        }]
    )

    return LaunchDescription([
        declare_map_path, 
        goal_pose_remapper
    ])