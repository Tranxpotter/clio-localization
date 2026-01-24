# Guide Bot Localization Package
This package is used for HKU Clio robot project

Launch localizer with: 
```bash
ros2 launch guide_robot_localization localize.launch.py
```

Launch mapping with: 
```bash
ros2 launch guide_robot_localization mapping.launch.py
```

## Summary
This package receives lidar point cloud, transforms and aligns the point cloud before slicing the point cloud for 2D mapping with slam or 2D localization with nav2. 