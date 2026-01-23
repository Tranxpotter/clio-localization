import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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

    # 3. Parameter Files
    # Ensure this points to the YAML we fixed earlier
    fast_lio_config = 'mid360.yaml' 
    # Use standard SLAM Toolbox params (or your custom one)
    slam_params_file = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')

    # --- NODES ---

    # 1. STATIC TRANSFORM (Base -> Lidar)
    # Values: Backwards 0.2m, Up 0.5m, Yaw 180 (3.14), Pitch -60 (-1.047)
    tf_base_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_lidar_publisher',
        arguments=['-0.2', '0.0', '0.5', '3.14159', '-1.047', '0.0', 'base_link', 'livox_frame']
    )

    tf_odom_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init',
        arguments=['0','0','0','0','0','0', 'odom', 'camera_init']
    )

    tf_body_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link',
        arguments=['0','0','0','0','0','0', 'body', 'base_link']
    )

    # 2. FAST-LIO (Odometry Source)
    # Publishes: odom -> base_link
    fast_lio_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_path, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': fast_lio_config
        }.items()
    )

    # 3. POINTCLOUD TO LASERSCAN (The Slicer)
    # Converts 3D Livox data to 2D /scan
    pc2scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/cloud_registered'), #Listen to FAST-LIO's registered point cloud output
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'base_link', 
            'transform_tolerance': 0.1,
            'min_height': 0.1,           # Ignore floor (0.0 to 0.1)
            'max_height': 1.0,           # See walls/furniture
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0043,
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 50.0,
            'use_inf': True,
            'qos_overrides./parameter_events.publisher.reliability': 'reliable', #Forcing compatibility
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
        tf_base_lidar,
        tf_odom_bridge,
        tf_body_bridge,
        fast_lio_node,
        pc2scan_node,
        slam_node,
        rviz_node
    ])