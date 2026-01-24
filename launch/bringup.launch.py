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
    odom_frame = 'odom'
    base_frame = 'base_footprint'
    lidar_frame = 'livox_frame'
    

    # TILT CALCULATION (60 Degrees)
    angle_deg = 60.0 
    angle_rad = angle_deg * (math.pi / 180.0)
    z_rotation_angle = 180.0
    z_rotation_rad = z_rotation_angle * (math.pi / 180.0)
    lidar_tf_args = ['0.1', '0', '0.5', '0', str(angle_rad), str(z_rotation_rad), base_frame, lidar_frame]


                # extrinsic_T: [ 0.1, 0.0, 0.4 ]
            # extrinsic_R: [ -0.5, 0.0, 0.866025,
            #                 0.0, -1.0, 0.0,
            #                 0.866025, 0.0, 0.5]
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
        arguments=['0','0','0','0','0','0', odom_frame, "camera_init"]
    )
    # Connect FAST-LIO's body frame to standard robot base frame
    tf_base_glue = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', 'body', base_frame]
    )
    # LiDAR mounting position transform
    tf_base_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=lidar_tf_args
    )


    # 4. ROTATE POINT CLOUD TO base_frame
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
            'timeout': 1.0
        }]
    )

    # 4. POINTCLOUD TO LASERSCAN
    pc2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_rotated'), 
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': base_frame, 
            'transform_tolerance': 0.1, # Increased tolerance for processing time
            'min_height': 0.1,
            'max_height': 1.2,
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
        rotate_pc_node,
        pc2scan_node,
        localization_launch,
        rviz_node
    ])