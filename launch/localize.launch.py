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
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    fast_lio_dir = get_package_share_directory('fast_lio') 

    default_map_path = os.path.join(this_pkg_dir, 'maps', 'map.yaml')
    default_nav2_config_path = os.path.join(this_pkg_dir, 'config', 'nav2_params.yaml')

    declare_map_path = DeclareLaunchArgument("map_path", default_value=default_map_path)
    declare_nav2_config_path = DeclareLaunchArgument("nav2_config_path", default_value=default_nav2_config_path)

    # FRAMES
    map_frame = 'map'
    odom_frame = 'odom'
    camera_init_frame = 'camera_init'
    body_frame = 'body'
    base_footprint_frame = 'base_footprint'
    lidar_frame = 'lidar'
    

    # Lidar frame tilt transform calculation (60 pitch, 180 yaw) #TODO: what's the actual lidar frame angles?
    angle_deg = 60.0
    angle_rad = angle_deg * (math.pi / 180.0)
    z_rotation_angle = 180.0
    z_rotation_rad = z_rotation_angle * (math.pi / 180.0)
    lidar_tf_args = ['0.1', '0', '0.4', str(z_rotation_rad), str(angle_rad), '0', body_frame, lidar_frame] # (x, y, z, yaw, pitch, roll, parent frame, child )

    # Saved lidar tf from body? or base_footprint
        # extrinsic_T: [ 0.1, 0.0, 0.4 ]
        # extrinsic_R: [ -0.5, 0.0, 0.866025,
        #                 0.0, -1.0, 0.0,
        #                 0.866025, 0.0, 0.5]
    
    '''
        TF TREE (Subject to change): 
        map             -nav2
        odom            -nav2
        camera_init     -fastlio
        body            -nav2 fastlio
        lidar base_footprint
    '''

    # --- NODES ---
    # 1. Launch FAST-LIO (camera_init -> body)
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'mid360.yaml'}.items()
    )

    # 2. TF GLUE
    # Connect ROS navigation frames to FAST-LIO's world frame
    tf_odom_glue = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0', odom_frame, camera_init_frame]
    )
    # Connect FAST-LIO's body frame to robot base footprint
    tf_base_glue = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','-0.4','0','0','0', body_frame, base_footprint_frame]
    )
    # LiDAR mounting position transform
    tf_base_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_base_lidar',
        arguments=lidar_tf_args
    )


    # 4. Transform Point Cloud from camera_init to lidar frame
    rotate_pc_node = Node(
        package='guide_robot_localization',
        executable='point_cloud_transformer',
        name='point_cloud_transformer',
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

    # 5. NAV2 LOCALIZATION
    localization_launch = TimerAction(
        period=5.0, 
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
                ),
                launch_arguments={
                    'map': LaunchConfiguration("map_path"),
                    'use_sim_time': 'false',
                    'params_file': LaunchConfiguration("nav2_config_path"),
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(this_pkg_dir, 'rviz', 'config.rviz')]
    )

    return LaunchDescription([
        declare_map_path, 
        declare_nav2_config_path, 
        fast_lio_launch,
        tf_odom_glue,
        tf_base_glue,
        tf_base_lidar,
        rotate_pc_node,
        pc2scan_node,
        localization_launch,
        rviz_node
    ])