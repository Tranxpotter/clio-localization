import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    fast_lio_dir = get_package_share_directory('fast_lio')

    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'mid360.yaml'}.items()
    )

    tf_body_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['-0.2', '0', '0.4', '3.14159', '-1.047', '0.0', 'base_link', 'livox_frame']
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(fast_lio_dir, 'rviz', 'fast_lio.rviz')]
    )
    return LaunchDescription([
        fast_lio_node,
        tf_body_lidar,
        rviz_node
    ])