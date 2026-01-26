import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import math

def generate_launch_description():
    # --- CONFIGURATION ---
    # 1. Package Names
    my_pkg = 'guide_robot_localization'
    fast_lio_pkg = 'fast_lio'
    slam_pkg = 'slam_toolbox'

    # 2. Paths
    pkg_path = get_package_share_directory(my_pkg)
    fast_lio_path = get_package_share_directory(fast_lio_pkg)
    slam_path = get_package_share_directory(slam_pkg)

    # Use standard SLAM Toolbox params (or your custom one)
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')


    # FRAMES
    map_frame = 'map'
    odom_frame = 'odom'
    camera_init_frame = 'camera_init'
    body_frame = 'body'
    base_footprint_frame = 'base_footprint'
    lidar_frame = 'lidar'

    # Lidar frame tilt transform calculation (60 pitch, 180 yaw) #TODO: what's the actual lidar frame angles?
    pitch_angle_deg = -60.0
    pitch_angle_rad = pitch_angle_deg * (math.pi / 180.0)
    yaw_angle_deg = 180.0
    yaw_angle_rad = yaw_angle_deg * (math.pi / 180.0)
    lidar_tf_args = ['0.1', '0', '0.4', str(yaw_angle_rad), str(pitch_angle_rad), '0', body_frame, lidar_frame] # (x, y, z, yaw, pitch, roll, parent frame, child )

    '''
        TF TREE (Subject to change): 
        map             -nav2
        odom            -nav2
        camera_init     -fastlio
        body            -nav2 fastlio
        lidar base_footprint
    '''

    # --- NODES ---
    # 1. TF GLUE
    # Connect ROS navigation frames to FAST-LIO's world frame
    tf_odom_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', odom_frame, camera_init_frame]
    )
    # Connect FAST-LIO's body frame to robot base footprint
    tf_body_footprint = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','-0.4','0','0','0', body_frame, base_footprint_frame]
    )
    # LiDAR mounting position transform
    tf_body_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=lidar_tf_args
    )

    # 2. FAST-LIO (Odometry Source)
    # Publishes: camera_init -> body
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_path, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'mid360.yaml'
        }.items()
    )


    # 3. Transform Point Cloud from camera_init to lidar frame
    rotate_pc_node = Node(
        package='guide_robot_localization',
        executable='point_cloud_transformer',
        name='pointcloud_rotator',
        remappings=[
            ('/input_point_cloud', '/Laser_map'),
            ('/output_point_cloud', '/cloud_rotated')
        ],
        parameters=[{
            'input_topic': '/Laser_map',
            'output_topic': '/cloud_rotated',
            'target_frame': lidar_frame,
            'verbose': False
        }]
    )


    # 4. POINTCLOUD TO LASERSCAN
    pc2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_rotated'), 
            ('scan', '/scan') #Output scan topic
        ],
        parameters=[{
            'target_frame': base_footprint_frame, # Slice the point_cloud based on the transformation of the target_frame
            'transform_tolerance': 0.1, # Time tolerance for transform lookups
            'min_height': 0.1,
            'max_height': 1.2,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1, # scan rate in seconds
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True
        }]
    )

    # 4. SLAM TOOLBOX (The Mapper)
    # Publishes: map -> odom
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_path, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': slam_params_file
        }.items()
    )

    # 5. RVIZ
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'mapping.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # Use Nav2 default view or your own
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        tf_odom_camera,
        tf_body_footprint,
        tf_body_lidar,
        fast_lio_node,
        rotate_pc_node, 
        pc2scan_node,
        slam_node,
        rviz_node
    ])