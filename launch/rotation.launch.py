import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'guide_robot_localization'
    pkg_dir = get_package_share_directory(pkg_name)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio') 

    map_file = os.path.join(pkg_dir, 'maps', 'map.yaml')
    nav2_config_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # FRAMES
    map_frame = 'map'
    odom_frame = 'odom'
    camera_init_frame = 'camera_init'
    body_frame = 'body'
    base_footprint_frame = 'base_footprint'
    lidar_frame = 'lidar_frame'


    # LIDAR ROTATION CALCULATION  
    # Clockwise 60° around Y-axis = -60° in ROS right-hand rule
    y_rotation_deg = 240.0  # Clockwise Y rotation
    y_rotation_rad = y_rotation_deg * (math.pi / 180.0)
    
    # 180° around Z-axis
    z_rotation_deg = 180.0
    z_rotation_rad = z_rotation_deg * (math.pi / 180.0)
    
    lidar_to_base_tf_args = ['0.1', '0', '0.5', '0', str(y_rotation_rad), str(z_rotation_rad), base_footprint_frame, lidar_frame]
    # --- NODES ---

    # 1. FAST-LIO
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'mid360.yaml'}.items()
    )

    # 2. TF GLUE
    # Connect FAST-LIO's world frame to ROS navigation frames
    tf_odom_glue = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', odom_frame, camera_init_frame]
    )
    # Connect FAST-LIO's body frame to standard robot base frame
    tf_base_glue = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', body_frame, base_footprint_frame]
    )
    # LiDAR mounting position transform
    tf_base_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=lidar_to_base_tf_args
    )

    # 3. CORRECTED BODY FRAME PUBLISHER
    # Gets camera_init→body transform from FAST-LIO and applies Y-axis correction
    # to make X/Y axes parallel to ground, publishing as odom→corrected_body
    corrected_body_frame_node = Node(
        package='guide_robot_localization',
        executable='corrected_body_frame_publisher',
        name='corrected_body_frame_publisher',
        parameters=[{
            'source_frame': camera_init_frame,
            'intermediate_frame': body_frame,
            'target_frame': odom_frame,
            'output_frame': 'corrected_body',
            'correction_angle_deg': -60.0,  # Counter-clockwise Y-axis rotation
            'publish_rate': 50.0,
            'tf_timeout': 1.0
        }]
    )

    # 4. ACCUMULATOR (The Sparsity Fix)
    # Collects 10 frames (1.0s) of data to make lines solid
    accumulator_node = Node(
        package='guide_robot_localization',
        executable='accumulator',
        name='pointcloud_accumulator',
        parameters=[{'accumulation_frames': 10}] 
    )

    

    # 4. POINTCLOUD TO LASERSCAN
    pc2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            # LISTEN TO THE ACCUMULATOR NOT FAST-LIO
            ('cloud_in', '/cloud_accumulated'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': lidar_frame, 
            'transform_tolerance': 0.1, # Increased tolerance for processing time
            'min_height': -0.5,
            'max_height': 0.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1, #
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True
        }]
    )

    # 5. NAV2 LOCALIZATION
    localization_launch = TimerAction(
        period=5.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'false',
                    'params_file': nav2_config_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'rviz', 'config.rviz')]
    )

    return LaunchDescription([
        fast_lio_launch,
        tf_odom_glue,
        tf_base_glue,
        tf_base_lidar,
        corrected_body_frame_node,
        accumulator_node,
        pc2scan_node,
        localization_launch,
        rviz_node
    ])