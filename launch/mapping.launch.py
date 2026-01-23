import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import math

def generate_launch_description():
    pkg_name = 'guide_robot_localization'
    pkg_dir = get_package_share_directory(pkg_name)
    fast_lio_dir = get_package_share_directory('fast_lio') 
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # --- 1. ROBOT PHYSICAL DESCRIPTION (URDF/TF) ---
    # Instead of manual static TFs, we usually use a URDF. 
    # But sticking to your static TF style, we define the Lidar -> Base relation here.
    # NOTE: This is for Rviz/Nav2 to know where the lidar is. 
    # FAST-LIO needs this SAME info in its YAML file.
    
    # Rotation: Yaw 180 + Pitch -60 (approximate radians)
    # Pitch -60 deg = -1.0472 rad
    # Yaw 180 deg = 3.14159 rad
    tf_base_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_to_lidar',
        # Args: x y z yaw pitch roll parent child
        # Note: static_transform_publisher usually takes: x y z qx qy qz qw OR x y z yaw pitch roll
        arguments=['0', '0', '0', '3.14159', '-1.0472', '0', 'base_footprint', 'livox_frame']
    )

    # --- 2. FAST-LIO ---
    # IMPORTANT: You must edit mid360.yaml to include the extrinsic_R and extrinsic_T
    # so that FAST-LIO publishes "odom -> base_footprint" (or body) directly.
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'mid360.yaml'}.items()
    )

    # --- 3. POINTCLOUD TO LASERSCAN ---
    # We feed it the registered cloud. We let it use TF to transform to base_footprint.
    pc2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # FAST-LIO usually publishes /cloud_registered or /Odometry
            # Check the FAST-LIO output topic name!
            ('cloud_in', '/cloud_registered'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_footprint', # We want the scan flat on the floor
            'transform_tolerance': 0.2,       # Give it some slack
            'min_height': 0.1,                # Height relative to base_footprint
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True
        }]
    )

    # --- 4. SLAM TOOLBOX ---
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml') 
        }.items()
    )

    # --- 5. RVIZ ---
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'nav2_default_view.rviz')]
    )

    return LaunchDescription([
        tf_base_to_lidar,
        fast_lio_launch,
        pc2scan_node,
        slam_toolbox_launch,
        rviz_node
    ])