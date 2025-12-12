---
sidebar_position: 3
title: Visual SLAM
---

# Visual SLAM with Isaac ROS

## RealSense D435i VSLAM Pipeline

**Launch Isaac ROS Visual SLAM**:

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Publishes:
# /visual_slam/tracking/odometry (nav_msgs/Odometry)
# /visual_slam/tracking/vo_pose (geometry_msgs/PoseStamped)
# /visual_slam/map (sensor_msgs/PointCloud2)
```

**Configuration** (`config/vslam_params.yaml`):

```yaml
visual_slam_node:
  ros__parameters:
    denoise_input_images: true
    rectified_images: true
    enable_image_denoising: true
    enable_imu_fusion: true  # Use D435i IMU
    gyro_noise_density: 0.0003
    accel_noise_density: 0.002
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
```

**Integration with Nav2**:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator

navigator = BasicNavigator()

# Set initial pose from VSLAM
navigator.setInitialPose(vslam_pose)

# Navigate to goal
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 5.0
goal_pose.pose.position.y = 3.0
navigator.goToPose(goal_pose)

while not navigator.isTaskComplete():
    feedback = navigator.getFeedback()
    print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
```

---

**Next**: [Nav2 for Humanoids](./nav2-humanoid)
