# guide_robot_localization

Lightweight ROS 2 package for localizing the guide robot using lidar point clouds. Launch the node with:

```bash
ros2 launch guide_robot_localization bringup.launch.py
```

## Summary
This package receives lidar point clouds (originating in the `camera_init` frame), transforms them into the robot-centric `robot_footprint` frame, applies filtering and accumulation to improve density and robustness, and publishes the processed point cloud for downstream localization and mapping components.

## Features
- Transform incoming PointCloud2 from `camera_init` → `robot_footprint` using tf2.
- Temporal accumulator: combine several consecutive clouds to increase point density and reduce sparsity/occlusions.
- Launch file to start required nodes and parameters.

## How it works (methods)
- Accumulator
    - Incoming transformed clouds are stored in a short ring buffer (configurable number of frames).
    - On each update the buffer contents are concatenated into a single point cloud. Concatenation increases point density and fills gaps caused by sensor motion or occlusion.
- TF-based transform
    - A tf2 buffer/listener is used to lookup the transform between `camera_init` and `robot_footprint` at the cloud timestamp.
    - The package uses `tf2_sensor_msgs::doTransform` (or equivalent tf2 sensor_msgs helper) to transform the `sensor_msgs/PointCloud2` message into the target frame while preserving timestamps and cloud metadata.
- Filtering & cleanup
    - After transformation the cloud is filtered (e.g., passthrough on z) to map onto 2D plane.
    - The resultant cloud is published for localization or mapping nodes.

## Launch
Build and source your workspace, then run:

```bash
colcon build
source install/setup.bash
ros2 launch guide_robot_localization bringup.launch.py
```

The launch file sets up node parameters, remappings, and any required lifecycle or tf relaying nodes.

## Dependencies
The package depends on common ROS 2 and point-cloud libraries:

- ROS 2 Humble
- rclcpp
- sensor_msgs
- tf2
- tf2_ros
- tf2_sensor_msgs
- launch / launch_ros (for the provided launch file)

Add these to package.xml and CMakeLists.txt as appropriate.

## License
Apache 2.0
