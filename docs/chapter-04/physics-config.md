---
sidebar_position: 3
title: Physics Configuration
---

# Physics Configuration in Gazebo

## Introduction

Accurate physics simulation is the foundation of reliable digital twins. While Gazebo provides default physics settings suitable for many scenarios, understanding and configuring physics parameters—gravity, solver iterations, friction, and inertia tensors—is essential for creating simulations that accurately represent real-world robot behavior. This section covers the physics engine configuration, material properties, and best practices for ensuring your simulated robot behaves like its physical counterpart.

Poor physics configuration leads to unrealistic behaviors: robots that float, slide uncontrollably, or vibrate chaotically. Conversely, well-tuned physics produces simulations where walking gaits transfer to hardware, grasps succeed with similar forces, and collision responses match real-world tests. Mastering physics configuration transforms Gazebo from a visualization tool into a true predictive simulator.

## Conceptual Foundation

### The Physics Engine

Gazebo supports multiple physics engines, each with different strengths:

**ODE (Open Dynamics Engine)** - Default in Gazebo Classic:
- Fast, stable, widely tested
- Good for wheeled robots, manipulators
- Adequate contact modeling
- Limited soft-body support

**Bullet** - Alternative high-performance engine:
- Fast collision detection
- Good for complex scenes with many objects
- Better soft-body physics than ODE
- Used in games, VR applications

**DART (Dynamic Animation and Robotics Toolkit)**:
- Advanced constraint handling
- Excellent for humanoid robots (balance, contacts)
- Precise inverse dynamics
- Slower than ODE but more accurate

**Simbody** - Biomechanics-focused:
- High-accuracy muscle/tendon simulation
- Medical robotics, human motion
- Slower, specialized use cases

**Choosing an Engine**:
- **Wheeled robots**: ODE (default)
- **Humanoid walking**: DART (better contact stability)
- **Large scenes**: Bullet (performance)
- **Biomechanics**: Simbody

### Gravity Configuration

Gravity is the most fundamental physics parameter. Earth's standard gravity is -9.81 m/s² in the Z-axis (downward). However, you may need to modify gravity for:

**Testing Edge Cases**:
- Zero gravity (space robotics)
- Reduced gravity (lunar: -1.62 m/s², Martian: -3.71 m/s²)
- Increased gravity (stress testing robot strength)

**Debugging**:
- Disable gravity temporarily (`0 0 0`) to isolate kinematic issues from dynamic effects

### Inertia Tensors

The inertia tensor describes how mass is distributed in a rigid body, affecting rotational dynamics. Incorrect inertia causes:
- Unrealistic spinning
- Incorrect torque requirements
- Unstable controllers

**Key Properties**:
- **Ixx, Iyy, Izz**: Moments of inertia around X, Y, Z axes
- **Ixy, Ixz, Iyz**: Products of inertia (coupling terms, often zero for symmetric bodies)

**Calculation Guidelines**:
```
Box: Ixx = (m/12) * (h² + d²), Iyy = (m/12) * (w² + d²), Izz = (m/12) * (w² + h²)
Cylinder: Ixx = Iyy = (m/12) * (3r² + h²), Izz = (m/2) * r²
Sphere: Ixx = Iyy = Izz = (2/5) * m * r²
```

Where: m = mass, w = width, h = height, d = depth, r = radius

### Friction and Damping

**Friction Coefficients** (`mu1`, `mu2`):
- `mu1`: Primary friction direction (typically along surface)
- `mu2`: Secondary friction direction (perpendicular)
- Typical values: 0.0 (ice) to 2.0 (rubber on concrete)

**Damping**:
- **Linear damping**: Opposes translational velocity (air resistance)
- **Angular damping**: Opposes rotational velocity
- Prevents unrealistic perpetual motion
- Typical values: 0.01 to 0.1

## Technical Details

### Configuring Physics in World Files

Physics parameters are set in `.world` files:

```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Physics Configuration -->
    <physics name="default_physics" default="true" type="ode">

      <!-- Gravity vector: [x, y, z] in m/s² -->
      <gravity>0 0 -9.81</gravity>

      <!-- Simulation time step -->
      <max_step_size>0.001</max_step_size>  <!-- 1ms, 1000 Hz -->

      <!-- Real-time update rate -->
      <real_time_factor>1.0</real_time_factor>  <!-- 1.0 = real-time -->
      <real_time_update_rate>1000</real_time_update_rate>  <!-- Hz -->

      <!-- ODE-specific parameters -->
      <ode>
        <solver>
          <type>quick</type>  <!-- Options: quick, world -->
          <iters>50</iters>   <!-- Solver iterations (higher = more accurate, slower) -->
          <sor>1.3</sor>      <!-- Successive Over-Relaxation parameter -->
        </solver>

        <constraints>
          <cfm>0.0</cfm>      <!-- Constraint Force Mixing (softness) -->
          <erp>0.2</erp>      <!-- Error Reduction Parameter (stiffness) -->
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>

    </physics>

    <!-- Ground plane with friction -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>   <!-- Friction coefficient -->
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
      </link>
    </model>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

  </world>
</sdf>
```

**Key Parameters Explained**:

- **max_step_size**: Smaller = more accurate but slower. 0.001s (1ms) is standard for robots.
- **real_time_factor**: Target speed (1.0 = real-time, 0.5 = half-speed, 2.0 = double-speed).
- **iters**: Solver iterations. Higher values improve contact stability (important for walking robots).
- **cfm** (Constraint Force Mixing): Makes constraints "soft". Higher values = springier contacts.
- **erp** (Error Reduction Parameter): How quickly constraints are corrected. 0.2 is typical.

### Computing Inertia Tensors for Complex Shapes

For simple primitives, use analytical formulas. For complex meshes, use CAD software or approximation tools:

**Python Script for Box Inertia**:

```python
import numpy as np

def compute_box_inertia(mass, width, height, depth):
    """
    Compute inertia tensor for a box.

    Args:
        mass: Mass in kg
        width: X dimension in meters
        height: Y dimension in meters
        depth: Z dimension in meters

    Returns:
        Dictionary with inertia components
    """
    ixx = (mass / 12.0) * (height**2 + depth**2)
    iyy = (mass / 12.0) * (width**2 + depth**2)
    izz = (mass / 12.0) * (width**2 + height**2)

    return {
        'ixx': ixx,
        'iyy': iyy,
        'izz': izz,
        'ixy': 0.0,  # Zero for symmetric box
        'ixz': 0.0,
        'iyz': 0.0
    }

# Example: Robot base link
base_mass = 15.0  # kg
base_width = 0.4  # meters
base_height = 0.3
base_depth = 0.5

inertia = compute_box_inertia(base_mass, base_width, base_height, base_depth)
print(f"Inertia tensor for robot base:")
print(f"  ixx: {inertia['ixx']:.4f}")
print(f"  iyy: {inertia['iyy']:.4f}")
print(f"  izz: {inertia['izz']:.4f}")
```

**Output**:
```
Inertia tensor for robot base:
  ixx: 0.5250
  iyy: 0.5125
  izz: 0.3125
```

**Using in URDF**:
```xml
<link name="base_link">
  <inertial>
    <mass value="15.0"/>
    <inertia ixx="0.5250" ixy="0.0" ixz="0.0"
             iyy="0.5125" iyz="0.0"
             izz="0.3125"/>
  </inertial>
  <!-- visual and collision geometry -->
</link>
```

### Material Properties and Friction

Define material properties for realistic contact behavior:

**URDF with Gazebo Material Tags**:

```xml
<robot name="mobile_robot">

  <link name="wheel_left">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0" ixz="0"
               iyy="0.001" iyz="0"
               izz="0.001"/>
    </inertial>
  </link>

  <!-- Gazebo-specific friction and damping -->
  <gazebo reference="wheel_left">
    <mu1>1.2</mu1>  <!-- High friction (rubber on asphalt) -->
    <mu2>1.2</mu2>
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>1.0</kd>        <!-- Contact damping -->
    <material>Gazebo/Black</material>
  </gazebo>

</robot>
```

**Common Material Friction Values**:
- Ice on metal: 0.02 - 0.05
- Wood on wood: 0.25 - 0.5
- Rubber on concrete: 0.6 - 1.0
- Rubber on dry asphalt: 1.0 - 1.5
- Metal on metal (dry): 0.15 - 0.3

### Testing Physics Accuracy

Validate your physics configuration with simple tests:

**Test 1: Free Fall (Gravity Check)**:

```python
# ROS 2 Node to measure free fall time
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import time

class FreeRallTest(Node):
    def __init__(self):
        super().__init__('free_fall_test')
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.state_callback,
            10
        )
        self.start_z = None
        self.start_time = None

    def state_callback(self, msg):
        try:
            idx = msg.name.index('test_sphere')
            z_pos = msg.pose[idx].position.z

            if self.start_z is None:
                self.start_z = z_pos
                self.start_time = time.time()
                self.get_logger().info(f'Starting fall from z={z_pos:.3f}m')

            elif z_pos < 0.1:  # Hit ground
                fall_time = time.time() - self.start_time
                fall_distance = self.start_z - z_pos

                # Physics: d = 0.5 * g * t²
                # Expected: t = sqrt(2*d/g)
                expected_time = (2 * fall_distance / 9.81) ** 0.5

                self.get_logger().info(f'Fall complete!')
                self.get_logger().info(f'  Distance: {fall_distance:.3f}m')
                self.get_logger().info(f'  Time: {fall_time:.3f}s')
                self.get_logger().info(f'  Expected: {expected_time:.3f}s')
                self.get_logger().info(f'  Error: {abs(fall_time - expected_time):.3f}s')

                rclpy.shutdown()
        except ValueError:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FreeRallTest()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Expected Output** (for 2m drop):
```
[INFO] Starting fall from z=2.000m
[INFO] Fall complete!
[INFO]   Distance: 2.000m
[INFO]   Time: 0.639s
[INFO]   Expected: 0.639s
[INFO]   Error: 0.001s
```

**Test 2: Friction Coefficient (Inclined Plane)**:

Spawn a box on an inclined plane. Measure the angle at which it starts sliding:
```
tan(θ_slip) ≈ μ_static

For μ = 0.5: θ_slip ≈ 26.6°
For μ = 1.0: θ_slip ≈ 45°
```

## Hands-On Examples

### Example 1: Lunar Gravity Simulation

Simulate robot operations on the Moon (gravity = -1.62 m/s²):

**lunar_world.world**:
```xml
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="lunar_surface">

    <physics name="lunar_physics" type="ode">
      <gravity>0 0 -1.62</gravity>  <!-- Moon gravity -->
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <model name="lunar_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>  <!-- Regolith friction -->
                <mu2>0.8</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**Launch lunar simulation**:
```bash
ros2 launch gazebo_ros gazebo.launch.py world:=/path/to/lunar_world.world
```

**Observation**: Robot jumps higher, falls slower. Adjust controller gains for reduced gravity.

### Example 2: High-Friction Gripper

Configure gripper fingers with high friction for secure grasping:

```xml
<robot name="gripper">

  <link name="finger_left">
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0"
               iyy="0.0001" iyz="0"
               izz="0.0001"/>
    </inertial>
    <collision>
      <geometry>
        <box size="0.02 0.05 0.01"/>
      </geometry>
    </collision>
    <visual>
      <geometry>
        <box size="0.02 0.05 0.01"/>
      </geometry>
    </visual>
  </link>

  <gazebo reference="finger_left">
    <mu1>2.0</mu1>   <!-- Very high friction (rubber pad) -->
    <mu2>2.0</mu2>
    <kp>10000000.0</kp>  <!-- Stiff contact -->
    <kd>10.0</kd>
    <minDepth>0.001</minDepth>  <!-- Contact threshold -->
    <material>Gazebo/Red</material>
  </gazebo>

</robot>
```

**Use Case**: Grasping smooth objects (glass, metal). High `mu` prevents slipping even with low grip force.

### Example 3: Switching Physics Engines at Runtime

Compare ODE vs DART for humanoid stability:

**Launch with DART**:
```bash
ros2 launch gazebo_ros gazebo.launch.py \
    world:=/path/to/humanoid_world.world \
    extra_gazebo_args:="--physics dart"
```

**Observation**: DART typically provides more stable contacts for bipedal walking due to better constraint handling.

### Common Pitfalls

1. **Zero or Missing Inertia**: If `<inertial>` is omitted, Gazebo assumes infinite mass. Links don't move. Always specify mass and inertia.

2. **Unrealistic Inertia Values**: Copy-pasting inertia without scaling for actual mass causes bizarre spinning. Use formulas or CAD tools.

3. **Time Step Too Large**: `max_step_size > 0.01` (10ms) causes instability, especially for fast-moving or small objects. Use 0.001s (1ms) for robots.

4. **Insufficient Solver Iterations**: Low `iters` (< 20) causes contact jittering, especially for humanoids. Increase to 50-100 for walking robots.

5. **Ignoring Real-Time Factor**: If simulation runs slower than `real_time_factor`, controller timings are wrong. Reduce scene complexity or lower RTF.

## Further Resources

### Official Documentation
- **Gazebo Physics Parameters**: [http://gazebosim.org/tutorials?tut=physics_params](http://gazebosim.org/tutorials?tut=physics_params)
- **SDF Physics Specification**: [http://sdformat.org/spec?elem=physics](http://sdformat.org/spec?elem=physics)
- **ODE Solver Parameters**: [https://www.ode.org/ode-latest-userguide.html](https://www.ode.org/ode-latest-userguide.html)

### Tools
- **MeshLab**: Compute inertia from 3D meshes ([https://www.meshlab.net/](https://www.meshlab.net/))
- **SolidWorks/Fusion 360**: Export URDF with accurate inertia from CAD models

### Research Papers
- Ivaldi et al. (2015). "Tools for Dynamic Simulation of Robots: A Survey Based on User Feedback"
- Erez et al. (2015). "Simulation Tools for Model-Based Robotics: Comparison of Bullet, Havok, MuJoCo, ODE and PhysX"

---

**Next**: Return to [Gazebo Basics](./gazebo-basics) or proceed to [NVIDIA Isaac Sim](./isaac-sim) for GPU-accelerated simulation.
