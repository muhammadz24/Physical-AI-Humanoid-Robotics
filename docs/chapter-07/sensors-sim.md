---
sidebar_position: 2
title: Advanced Sensor Simulation
---

# Advanced Sensor Simulation

## Introduction

Accurate sensor simulation is the cornerstone of reliable sim-to-real transfer. When perception algorithms trained in simulation fail on real hardware, the culprit is often oversimplified sensor models—perfect noise-free cameras, infinite-range LIDAR, or IMUs without drift. This section teaches you to simulate sensors with realistic imperfections: Gaussian noise, motion blur, temporal delays, and hardware-specific artifacts. By the end, your Gazebo simulations will produce sensor data indistinguishable from the real **Intel RealSense D435i** and professional LIDAR units.

We focus on two sensor categories critical for Physical AI: depth cameras (RGB-D) for manipulation and close-range navigation, and LIDAR for long-range mapping and obstacle avoidance. Understanding their physics, limitations, and simulation parameters enables you to develop robust perception algorithms that work reliably when deployed to hardware.

## Conceptual Foundation

### Depth Camera Fundamentals

**Stereo Depth Cameras** (like RealSense D435i):
- Two infrared cameras + IR projector
- Compute depth via stereo triangulation
- **Strengths**: High resolution (1280×720), low cost
- **Limitations**: Struggles with textureless surfaces, sunlight interference, limited range (0.3m - 3m indoor)

**Time-of-Flight (ToF) Cameras**:
- Measure light travel time
- **Strengths**: Works on textureless surfaces
- **Limitations**: Lower resolution, higher cost

**Structured Light**:
- Project pattern, measure distortion
- **Strengths**: Very accurate indoors
- **Limitations**: Fails outdoors (sunlight washes out pattern)

### LIDAR Fundamentals

**2D LIDAR** (e.g., Hokuyo, SICK):
- Single horizontal plane scan
- **Use**: Floor-level obstacle detection, indoor navigation
- **Range**: 10m - 30m

**3D LIDAR** (e.g., Velodyne VLP-16, Ouster OS1):
- Multiple laser beams (16, 32, 64, 128 channels)
- Rotating or solid-state
- **Use**: 3D mapping, autonomous vehicles
- **Range**: 100m - 200m outdoors

**Key Parameters**:
- **Angular Resolution**: Spacing between rays (0.25° - 1°)
- **Range Resolution**: Precision of distance measurement (±2cm)
- **Update Rate**: 5 Hz - 20 Hz (3D), 10 Hz - 40 Hz (2D)

### Sensor Noise Models

**Gaussian Noise**: Random variations around true value
```
measured_depth = true_depth + N(0, σ²)
```

**Depth-Dependent Noise**: Error increases with distance
```
σ = baseline_noise + (distance * distance_factor)
```

**Outliers**: Invalid readings (reflective surfaces, absorption)
- Replace with NaN or max_range

**Temporal Noise**: Frame-to-frame jitter
- Model as high-frequency noise component

## Technical Details

### Simulating Intel RealSense D435i in Gazebo

**URDF Configuration with Gazebo Plugin**:

```xml
<?xml version="1.0"?>
<robot name="robot_with_realsense">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- RealSense D435i Link -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.025 0.09 0.025"/>  <!-- D435i dimensions -->
      </geometry>
      <material name="aluminum">
        <color rgba="0.7 0.7 0.7 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.025 0.09 0.025"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.072"/>  <!-- D435i actual mass: 72g -->
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Joint: Mount camera on robot front -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.15 0 0.05" rpy="0 0 0"/>  <!-- Front-facing -->
  </joint>

  <!-- Optical Frame (ROS convention: Z forward, Y down) -->
  <link name="camera_optical_frame"/>
  <joint name="camera_optical_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="camera_optical_frame"/>
    <origin xyz="0 0 0" rpy="-1.5708 0 -1.5708"/>  <!-- 90° rotations -->
  </joint>

  <!-- Gazebo Plugin: RGB-D Camera -->
  <gazebo reference="camera_link">
    <sensor name="realsense_d435i" type="depth">
      <update_rate>30.0</update_rate>  <!-- D435i: 30 FPS typical -->

      <camera>
        <!-- RGB Camera (Color sensor) -->
        <horizontal_fov>1.2043</horizontal_fov>  <!-- 69° (D435i spec) -->
        <image>
          <width>1920</width>  <!-- Full HD -->
          <height>1080</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>  <!-- Minimum depth -->
          <far>10.0</far>   <!-- Maximum depth (extend for outdoor) -->
        </clip>

        <!-- Depth Camera -->
        <depth_camera>
          <output>
            <type>depth</type>
          </output>
        </depth_camera>

        <!-- Noise Model (Gaussian) -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>  <!-- 7mm noise at close range -->
        </noise>
      </camera>

      <!-- ROS 2 Plugin -->
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/realsense</namespace>
          <remapping>image_raw:=rgb/image_raw</remapping>
          <remapping>depth/image_raw:=depth/image_raw</remapping>
          <remapping>camera_info:=rgb/camera_info</remapping>
          <remapping>depth/camera_info:=depth/camera_info</remapping>
          <remapping>points:=depth/points</remapping>  <!-- PointCloud2 -->
        </ros>

        <camera_name>realsense_d435i</camera_name>
        <frame_name>camera_optical_frame</frame_name>

        <!-- Generate Point Cloud -->
        <hack_baseline>0.05</hack_baseline>  <!-- Stereo baseline: 50mm -->
        <min_depth>0.3</min_depth>  <!-- D435i minimum depth: 0.3m -->
        <max_depth>3.0</max_depth>  <!-- Indoor range: 3m -->
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Sensor (BMI055) -->
  <link name="imu_link"/>
  <joint name="imu_joint" type="fixed">
    <parent link="camera_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- Co-located with camera -->
  </joint>

  <gazebo reference="imu_link">
    <sensor name="realsense_imu" type="imu">
      <update_rate>200.0</update_rate>  <!-- BMI055: 200 Hz -->

      <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
        <ros>
          <namespace>/realsense</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>

        <frame_name>imu_link</frame_name>

        <!-- Noise Parameters (BMI055 datasheet) -->
        <initial_orientation_as_reference>false</initial_orientation_as_reference>

        <!-- Gyroscope Noise -->
        <gyroscope_noise_density>0.0003</gyroscope_noise_density>  <!-- rad/s/√Hz -->
        <gyroscope_random_walk>0.00004</gyroscope_random_walk>
        <gyroscope_bias_correlation_time>1000.0</gyroscope_bias_correlation_time>

        <!-- Accelerometer Noise -->
        <accelerometer_noise_density>0.002</accelerometer_noise_density>  <!-- m/s²/√Hz -->
        <accelerometer_random_walk>0.0003</accelerometer_random_walk>
        <accelerometer_bias_correlation_time>300.0</accelerometer_bias_correlation_time>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**Key Configuration Details**:

1. **Camera FOV**: `horizontal_fov="1.2043"` (69°) matches D435i RGB camera
2. **Depth Range**: `min_depth="0.3"` to `max_depth="3.0"` matches indoor spec
3. **Resolution**: 1920×1080 for RGB (can reduce to 640×480 for performance)
4. **Noise**: `stddev="0.007"` (7mm) approximates D435i depth accuracy
5. **IMU Rate**: 200 Hz matches BMI055
6. **Point Cloud**: Automatically generated from depth image

### Advanced Depth Noise Model

For more realistic depth-dependent noise:

**Custom Gazebo Plugin** (C++):

```cpp
#include <gazebo/plugins/DepthCameraPlugin.hh>

namespace gazebo
{
  class RealisticDepthNoisePlugin : public DepthCameraPlugin
  {
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
      DepthCameraPlugin::Load(_sensor, _sdf);

      // Read noise parameters
      this->baseline_noise_ = 0.005;  // 5mm base noise
      this->distance_factor_ = 0.01;  // 1% of distance

      if (_sdf->HasElement("baseline_noise"))
        this->baseline_noise_ = _sdf->Get<double>("baseline_noise");
      if (_sdf->HasElement("distance_factor"))
        this->distance_factor_ = _sdf->Get<double>("distance_factor");
    }

    public: virtual void OnNewDepthFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format)
    {
      // Apply depth-dependent noise
      std::vector<float> noisy_image(_width * _height);

      for (unsigned int i = 0; i < _width * _height; ++i)
      {
        float true_depth = _image[i];

        // Noise increases with distance
        double sigma = baseline_noise_ + (true_depth * distance_factor_);

        // Add Gaussian noise
        std::normal_distribution<double> dist(0.0, sigma);
        float noise = dist(random_generator_);

        noisy_image[i] = true_depth + noise;

        // Handle invalid depths
        if (noisy_image[i] < min_depth_ || noisy_image[i] > max_depth_)
          noisy_image[i] = std::numeric_limits<float>::quiet_NaN();
      }

      // Publish noisy depth image
      this->PublishDepthImage(noisy_image.data(), _width, _height);
    }

    private: double baseline_noise_;
    private: double distance_factor_;
    private: double min_depth_ = 0.3;
    private: double max_depth_ = 3.0;
    private: std::default_random_engine random_generator_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(RealisticDepthNoisePlugin)
}
```

**Usage in URDF**:
```xml
<plugin name="depth_noise" filename="librealistic_depth_noise_plugin.so">
  <baseline_noise>0.005</baseline_noise>  <!-- 5mm -->
  <distance_factor>0.01</distance_factor>  <!-- 1% -->
</plugin>
```

### Simulating 3D LIDAR (Velodyne VLP-16)

**URDF Configuration for 16-Channel LIDAR**:

```xml
<robot name="robot_with_lidar">

  <link name="base_link">
    <!-- Robot base (omitted for brevity) -->
  </link>

  <!-- LIDAR Link -->
  <link name="velodyne_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>  <!-- VLP-16 approximate shape -->
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.83"/>  <!-- VLP-16: 830g -->
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="velodyne_joint" type="fixed">
    <parent link="base_link"/>
    <child link="velodyne_link"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>  <!-- Mounted on top -->
  </joint>

  <!-- Gazebo LIDAR Plugin -->
  <gazebo reference="velodyne_link">
    <sensor name="velodyne_vlp16" type="ray">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>  <!-- Set true to see rays in Gazebo GUI -->
      <update_rate>10.0</update_rate>  <!-- VLP-16: 10 Hz rotation -->

      <ray>
        <!-- Vertical Configuration (16 beams) -->
        <scan>
          <horizontal>
            <samples>1800</samples>  <!-- 360° / 0.2° = 1800 points per layer -->
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>  <!-- -180° -->
            <max_angle>3.14159</max_angle>   <!-- +180° -->
          </horizontal>
          <vertical>
            <samples>16</samples>  <!-- 16 laser beams -->
            <resolution>1</resolution>
            <min_angle>-0.2618</min_angle>  <!-- -15° -->
            <max_angle>0.2618</max_angle>   <!-- +15° (30° vertical FOV) -->
          </vertical>
        </scan>

        <!-- Range Configuration -->
        <range>
          <min>0.3</min>    <!-- Minimum range: 0.3m -->
          <max>100.0</max>  <!-- Maximum range: 100m -->
          <resolution>0.03</resolution>  <!-- 3cm accuracy -->
        </range>

        <!-- Noise Model -->
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.02</stddev>  <!-- 2cm standard deviation -->
        </noise>
      </ray>

      <!-- ROS 2 Plugin -->
      <plugin name="gazebo_ros_velodyne" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/velodyne</namespace>
          <remapping>~/out:=points</remapping>  <!-- PointCloud2 -->
        </ros>
        <output_type>sensor_msgs/PointCloud2</output_type>
        <frame_name>velodyne_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

**Key Parameters**:

- **Horizontal Samples**: 1800 (0.2° resolution, VLP-16 spec)
- **Vertical Samples**: 16 beams
- **Vertical FOV**: ±15° (30° total)
- **Range**: 0.3m - 100m
- **Update Rate**: 10 Hz (one full rotation per second)

**Expected Output**: `/velodyne/points` topic publishes `sensor_msgs/PointCloud2` with ~28,800 points per scan (1800 × 16).

## Hands-On Examples

### Example 1: Visualizing RealSense Depth Data in RViz2

**Launch File** (`launch/realsense_sim.launch.py`):

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to robot URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf', 'robot_with_realsense.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch', 'gazebo.launch.py'
        )
    )

    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot', '-topic', 'robot_description'],
        output='screen'
    )

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}]
    )

    # RViz2 with RealSense Configuration
    rviz_config = os.path.join(
        get_package_share_directory('my_robot_description'),
        'config', 'realsense_view.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_pub,
        spawn_robot,
        rviz
    ])
```

**RViz2 Configuration** (`config/realsense_view.rviz`):
- Add **Image** display → Topic: `/realsense/rgb/image_raw`
- Add **DepthCloud** display → Topic: `/realsense/depth/points`
- Add **TF** display → Show coordinate frames

**Launch**:
```bash
ros2 launch my_robot_description realsense_sim.launch.py
```

**Expected Result**: RViz2 shows RGB camera view and 3D point cloud from depth camera.

### Example 2: Processing LIDAR Point Clouds with PCL

**ROS 2 Node to Filter Point Cloud**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        # Subscribe to raw LIDAR
        self.subscription = self.create_subscription(
            PointCloud2,
            '/velodyne/points',
            self.point_cloud_callback,
            10
        )

        # Publish filtered points
        self.publisher = self.create_publisher(
            PointCloud2,
            '/velodyne/points_filtered',
            10
        )

        self.get_logger().info('LIDAR Processor started')

    def point_cloud_callback(self, msg):
        # Convert PointCloud2 to numpy array
        points = []
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append([point[0], point[1], point[2]])

        points = np.array(points)

        if len(points) == 0:
            return

        # Filter 1: Remove ground plane (points below -0.3m in Z)
        above_ground = points[points[:, 2] > -0.3]

        # Filter 2: Crop to region of interest (10m radius)
        distances = np.linalg.norm(above_ground[:, :2], axis=1)
        roi_points = above_ground[distances < 10.0]

        # Filter 3: Downsample (keep every 5th point for performance)
        downsampled = roi_points[::5]

        self.get_logger().info(
            f'Points: {len(points)} → {len(downsampled)} (filtered)'
        )

        # Convert back to PointCloud2 and publish
        filtered_msg = pc2.create_cloud_xyz32(msg.header, downsampled)
        self.publisher.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage**:
```bash
# Terminal 1: Launch simulation
ros2 launch my_robot_description velodyne_sim.launch.py

# Terminal 2: Run processor
ros2 run my_robot_description lidar_processor

# Terminal 3: Visualize in RViz2
rviz2
# Add PointCloud2 display → Topic: /velodyne/points_filtered
```

### Example 3: Depth-to-Obstacle Distance

**Real-Time Obstacle Detection from RealSense Depth**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.bridge = CvBridge()

        # Subscribe to depth image
        self.subscription = self.create_subscription(
            Image,
            '/realsense/depth/image_raw',
            self.depth_callback,
            10
        )

        self.get_logger().info('Obstacle detector started')

    def depth_callback(self, msg):
        # Convert ROS Image to OpenCV (depth in mm)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Define center region (robot's forward path)
        height, width = depth_image.shape
        center_x = width // 2
        center_y = height // 2
        roi_size = 100  # 100x100 pixel region

        roi = depth_image[
            center_y - roi_size//2 : center_y + roi_size//2,
            center_x - roi_size//2 : center_x + roi_size//2
        ]

        # Find minimum distance in ROI
        valid_depths = roi[~np.isnan(roi)]

        if len(valid_depths) == 0:
            self.get_logger().warn('No valid depth readings')
            return

        min_distance = np.min(valid_depths)

        # Alert if obstacle close
        if min_distance < 0.5:  # 50cm threshold
            self.get_logger().error(
                f'⚠️  OBSTACLE at {min_distance:.2f}m - STOP!'
            )
        elif min_distance < 1.0:
            self.get_logger().warn(
                f'Obstacle detected at {min_distance:.2f}m - Slow down'
            )
        else:
            self.get_logger().debug(
                f'Path clear, closest obstacle: {min_distance:.2f}m'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Output**:
```
[INFO] Obstacle detector started
[DEBUG] Path clear, closest obstacle: 2.45m
[DEBUG] Path clear, closest obstacle: 2.31m
[WARN] Obstacle detected at 0.87m - Slow down
[ERROR] ⚠️  OBSTACLE at 0.42m - STOP!
```

### Common Pitfalls

1. **Incorrect Optical Frame**: Depth cameras use Z-forward, Y-down convention. Forgetting the `-1.5708 0 -1.5708` rotation causes point clouds to appear sideways.

2. **Depth Units Confusion**: Some plugins output depth in meters (float), others in millimeters (uint16). Check message encoding.

3. **Point Cloud Overflow**: 3D LIDAR generates massive point clouds (100k+ points). Downsample or filter before processing to avoid lag.

4. **Noise Too Perfect**: Real sensors have non-Gaussian outliers (reflections, multipath). Add manual outlier injection for robustness testing.

5. **Update Rate Mismatch**: If sensor `update_rate` exceeds Gazebo's `max_step_size`, you'll get duplicate frames. Ensure `update_rate * max_step_size ≤ 1.0`.

## Further Resources

### Official Documentation
- **Gazebo Sensors**: [https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins#Camera)
- **RealSense ROS 2**: [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- **PCL Tutorials**: [https://pcl.readthedocs.io/](https://pcl.readthedocs.io/)

### Sensor Datasheets
- **Intel RealSense D435i**: [https://www.intelrealsense.com/depth-camera-d435i/](https://www.intelrealsense.com/depth-camera-d435i/)
- **Velodyne VLP-16**: [https://velodynelidar.com/products/puck/](https://velodynelidar.com/products/puck/)

### Research Papers
- *Depth Camera Noise Modeling for Robotics* (Nguyen et al., 2023)
- *LiDAR Simulation for Autonomous Driving* (Dosovitskiy et al., 2022)

---

**Next**: [Unity Integration with ROS 2](./unity-integration) for photorealistic visualization and synthetic data generation.
