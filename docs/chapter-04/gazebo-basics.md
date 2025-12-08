---
sidebar_position: 2
title: Gazebo Basics
---

# Gazebo Basics

## Introduction

Gazebo is the industry-standard open-source robotics simulator, providing physics-based 3D environments where robots can be tested safely before deployment to real hardware. Unlike simple 2D simulators or kinematic models, Gazebo simulates realistic physics (gravity, friction, inertia), sensor noise, and environmental dynamics—enabling developers to catch bugs, validate algorithms, and train machine learning models in conditions that closely mirror the real world. From autonomous vehicles navigating urban streets to warehouse robots picking items, Gazebo has become an indispensable tool for robotics development.

The power of Gazebo lies in its integration with ROS 2: simulated robots communicate via the same topics, services, and actions as physical robots, meaning code developed in simulation runs on hardware with minimal changes. This "sim-to-real" workflow dramatically reduces development time and cost. Instead of waiting hours for a physical robot to test a navigation algorithm, you can iterate in minutes with Gazebo. When a bug causes a crash in simulation, no hardware is damaged—making it safe to explore edge cases and failure modes that would be too risky or expensive to test in reality.

## Conceptual Foundation

### What is Digital Twin Simulation?

A digital twin is a virtual representation of a physical system that mirrors its behavior, structure, and dynamics. In robotics, a digital twin encompasses:

**Robot Model**: Accurate 3D geometry, mass properties, joint kinematics, and actuation limits that match the physical robot.

**Sensor Models**: Simulated cameras, LIDAR, IMU, GPS, and tactile sensors that produce data with realistic noise, latency, and failure modes.

**Environment**: Virtual worlds with physics—gravity, collision detection, friction, and material properties—plus dynamic objects like pedestrians, vehicles, or moving obstacles.

**The Simulation Loop**:
1. Robot's virtual actuators receive commands (e.g., motor velocities)
2. Physics engine updates robot state (positions, velocities) based on dynamics
3. Sensors observe the virtual environment, producing simulated measurements
4. Sensor data is published to ROS 2 topics (identical to hardware)
5. Your robot control code reads sensor data, computes actions, sends commands
6. Cycle repeats at real-time (or faster/slower as needed)

This closed loop creates a high-fidelity testing environment where robot software interacts with a virtual world exactly as it would with the physical world.

### Why Use Simulation?

**Cost Reduction**: Physical robots cost thousands to millions of dollars. Damaging hardware during testing is expensive. Simulation enables unlimited experimentation at zero marginal cost.

**Safety**: Testing collision avoidance by driving a real autonomous vehicle into obstacles risks injury and property damage. Simulation allows safe exploration of failure modes.

**Speed**: Iterate in minutes instead of hours. No need to charge batteries, reset environments, or repair damaged components. Run hundreds of tests overnight.

**Reproducibility**: Real-world tests are affected by weather, lighting, battery voltage, and sensor wear—making bugs hard to reproduce. Simulations run identically every time, enabling systematic debugging.

**Scalability**: Train reinforcement learning agents for millions of timesteps. Testing this on hardware would take years; simulation completes in days.

**Edge Case Exploration**: Test rare scenarios (sensor failures, extreme weather, adversarial inputs) that are difficult or dangerous to create in reality.

### Gazebo Architecture

Gazebo consists of several components:

**Gazebo Server (gzserver)**: Runs the physics simulation, sensor updates, and plugin logic. Operates headless (no graphics) for maximum performance.

**Gazebo Client (gzclient)**: Provides 3D visualization, camera views, and GUI controls. Can connect to remote servers for distributed simulation.

**Physics Engines**: Pluggable physics backends (ODE, Bullet, DART, Simbody) that compute rigid body dynamics, collisions, and joint constraints.

**Sensor Plugins**: Simulate cameras (RGB, depth), LIDAR, IMU, GPS, force-torque sensors, and contact sensors. Publish data to ROS 2 topics.

**World Files (.world)**: XML descriptions of environments, including ground plane, obstacles, lighting, and robot spawn locations.

**Model Files (SDF)**: Simulation Description Format files defining robot geometry, inertia, joints, sensors, and plugins.

### URDF vs SDF

ROS 2 robots are described using two formats:

**URDF (Unified Robot Description Format)**:
- XML format for robot structure (links, joints, visual geometry, collision geometry)
- Widely used in ROS 1 and ROS 2
- Limited: no support for closed kinematic loops, environments, or some Gazebo features
- Example use: Describe robot arm with 6 revolute joints

**SDF (Simulation Description Format)**:
- Gazebo's native format, superset of URDF
- Supports complex scenes, multiple robots, physics configuration, sensor plugins
- Can include URDF models via `<include>` tags
- Example use: Complete warehouse environment with multiple robots and dynamic obstacles

**Best Practice**: Define robot base in URDF for ROS 2 compatibility, then wrap in SDF for Gazebo-specific features (sensors, plugins).

## Technical Details

### Installing Gazebo and ROS 2 Integration

Gazebo is available in two versions:
- **Gazebo Classic** (formerly Gazebo 11): Mature, widely used, integrates with ROS 2 via `gazebo_ros_pkgs`
- **Gazebo (Ignition)**: Modern rewrite, better performance, improved architecture

For ROS 2 Humble, install Gazebo Classic integration:

```bash
# Install Gazebo Classic and ROS 2 integration packages
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install Gazebo Classic itself (if not already installed)
sudo apt install gazebo

# Verify installation
gazebo --version  # Should show Gazebo 11.x
ros2 pkg list | grep gazebo  # Should show gazebo_ros packages
```

### Launching Gazebo with ROS 2

**Method 1: Launch Gazebo Server and Client Separately**

```bash
# Terminal 1: Start Gazebo server (physics simulation)
ros2 launch gazebo_ros gzserver.launch.py

# Terminal 2: Start Gazebo client (visualization)
ros2 launch gazebo_ros gzclient.launch.py
```

**Method 2: Launch Both Together**

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

**Method 3: Launch with a Specific World**

```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/my_world.world
```

### URDF Robot Description Example

Define a simple two-wheeled robot:

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link (robot body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint: Base to Left Wheel (continuous rotation) -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.175 0" rpy="-1.5708 0 0"/>  <!-- 90° rotation, left side -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (symmetric to left) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.175 0" rpy="-1.5708 0 0"/>  <!-- Right side -->
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Gazebo-specific configurations -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>

  <gazebo reference="left_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>  <!-- Friction coefficient -->
    <mu2>1.0</mu2>
  </gazebo>

  <gazebo reference="right_wheel">
    <material>Gazebo/Black</material>
    <mu1>1.0</mu1>
    <mu2>1.0</mu2>
  </gazebo>

</robot>
```

**Key Elements**:
- `<link>`: Defines a rigid body (visual geometry, collision geometry, inertial properties)
- `<joint>`: Connects links (types: fixed, revolute, continuous, prismatic)
- `<visual>`: How link appears (meshes, primitives, colors)
- `<collision>`: Shape used for collision detection (often simpler than visual)
- `<inertial>`: Mass and inertia tensor (critical for accurate dynamics)
- `<gazebo>`: Gazebo-specific parameters (materials, friction, damping)

### Spawning Robots in Gazebo

Create a launch file to spawn the robot:

```python
# launch/spawn_robot.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('my_robot_description')

    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'simple_robot.urdf')

    # Read URDF content
    with open(urdf_file, 'r') as file:
        robot_desc = file.read()

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_robot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'  # Spawn 0.5m above ground
        ],
        output='screen'
    )

    # Publish robot description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

**Launch the simulation**:
```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

## Hands-On Examples

### Example 1: Adding a Camera Sensor

Add a camera to the robot and publish images to ROS 2:

```xml
<!-- Add this inside the <robot> tag in URDF -->

<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0"
             izz="0.001"/>
  </inertial>
</link>

<!-- Mount camera on front of base -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.25 0 0.1" rpy="0 0 0"/>  <!-- Front center, 10cm up -->
</joint>

<!-- Gazebo Camera Plugin -->
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_link</frame_name>
      <topic_name>camera/image_raw</topic_name>
      <camera_info_topic_name>camera/camera_info</camera_info_topic_name>
    </plugin>
  </sensor>
</gazebo>
```

**View camera images**:
```bash
# In one terminal, launch simulation
ros2 launch my_robot_description spawn_robot.launch.py

# In another terminal, view images
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Example 2: Adding a LIDAR Sensor

Add a 2D LIDAR for obstacle detection:

```xml
<!-- LIDAR Link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
    <material name="black">
      <color rgba="0.1 0.1 0.1 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.15"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0"
             izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0 0 0.15" rpy="0 0 0"/>  <!-- Top center of base -->
</joint>

<!-- Gazebo LIDAR Plugin -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>  <!-- 360-degree scan -->
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>  <!-- -180 degrees -->
          <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**Visualize LIDAR scan**:
```bash
# Launch RViz2 to visualize
ros2 run rviz2 rviz2

# In RViz2, add "LaserScan" display, set topic to /scan
```

### Example 3: Controlling Robot with ROS 2

Add differential drive plugin for wheel control:

```xml
<!-- Gazebo Differential Drive Plugin -->
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">

    <!-- Wheel joints -->
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>

    <!-- Wheel separation and diameter -->
    <wheel_separation>0.35</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>

    <!-- Limits -->
    <max_wheel_torque>20</max_wheel_torque>
    <max_wheel_acceleration>1.0</max_wheel_acceleration>

    <!-- Topics -->
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>

    <!-- Publishing -->
    <publish_odom>true</publish_odom>
    <publish_odom_tf>true</publish_odom_tf>
    <publish_wheel_tf>true</publish_wheel_tf>

    <odometry_source>world</odometry_source>
    <update_rate>50.0</update_rate>
  </plugin>
</gazebo>
```

**Control robot from keyboard**:
```bash
# Terminal 1: Launch simulation
ros2 launch my_robot_description spawn_robot.launch.py

# Terminal 2: Teleoperate robot
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel
```

Use arrow keys to drive the robot!

### Common Pitfalls

1. **Missing Inertial Properties**: Forgetting `<inertial>` tags causes unrealistic physics (infinite mass). Always specify mass and inertia.

2. **Collision vs Visual Mismatch**: If collision geometry doesn't match visual, robot may appear to float or penetrate surfaces. Keep them similar or collision simpler.

3. **Joint Axis Errors**: Wrong joint axis (e.g., `<axis xyz="1 0 0"/>` instead of `<axis xyz="0 0 1"/>`) causes joints to rotate in unintended directions.

4. **Not Sourcing Workspace**: Gazebo needs to find models. After building packages, source workspace: `source install/setup.bash`.

5. **Plugin Library Names**: Plugin filenames are case-sensitive and version-specific. Use exact names from documentation (e.g., `libgazebo_ros_diff_drive.so`).

## Further Resources

### Official Documentation
- **Gazebo Classic Tutorials**: [https://classic.gazebosim.org/tutorials](https://classic.gazebosim.org/tutorials)
- **ROS 2 Gazebo Integration**: [https://github.com/ros-simulation/gazebo_ros_pkgs](https://github.com/ros-simulation/gazebo_ros_pkgs)
- **URDF Tutorial**: [https://docs.ros.org/en/humble/Tutorials/URDF.html](https://docs.ros.org/en/humble/Tutorials/URDF.html)
- **SDF Format**: [http://sdformat.org/](http://sdformat.org/)

### Example Robots
- **TurtleBot3**: [https://github.com/ROBOTIS-GIT/turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- **ROSbot 2.0**: [https://github.com/husarion/rosbot_description](https://github.com/husarion/rosbot_description)

### Tools
- **Onshape-to-robot**: Convert CAD models to URDF ([https://github.com/Rhoban/onshape-to-robot](https://github.com/Rhoban/onshape-to-robot))
- **phobos**: Blender addon for robot modeling ([https://github.com/dfki-ric/phobos](https://github.com/dfki-ric/phobos))

---

**Next**: Continue to [NVIDIA Isaac Sim](./isaac-sim) for GPU-accelerated simulation, or test your understanding with [Self-Assessment](./self-assessment).
