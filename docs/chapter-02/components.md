---
sidebar_position: 2
title: Key Components
---

# Key Components of Humanoid Robots

## Introduction

A humanoid robot is a complex mechatronic system integrating mechanical structures, electronic sensors, actuators, and intelligent control systems. Understanding these components is essential for appreciating how humanoid robots achieve human-like mobility and interaction. From the motors that drive each joint to the sensors that maintain balance, every component plays a critical role in creating a functional bipedal robot.

This section examines the hardware architecture of modern humanoid robots, focusing on the mechanical design, actuation systems, sensing modalities, and computational infrastructure that enable these machines to walk, manipulate objects, and interact with their environment.

## Conceptual Foundation

### Humanoid Robot Architecture

A typical humanoid robot consists of several subsystems:

**1. Mechanical Structure (Skeleton)**:
- **Rigid Links**: Limbs and body segments (thighs, shins, torso, arms)
- **Joints**: Rotational or prismatic connections between links (hips, knees, ankles, shoulders, elbows)
- **Materials**: Aluminum alloys, carbon fiber, 3D-printed plastics (balancing strength and weight)

**2. Actuation System (Muscles)**:
- **Electric Motors**: DC motors, brushless motors, servo motors
- **Gearboxes**: Reduction gears to increase torque
- **Hydraulics/Pneumatics**: High-power actuators for heavy-duty robots (e.g., Boston Dynamics Atlas)

**3. Sensing System (Nervous System)**:
- **Proprioceptive Sensors**: Joint encoders (position), torque sensors (force), IMUs (orientation and acceleration)
- **Exteroceptive Sensors**: Cameras (vision), LIDAR (depth), force/torque sensors in feet (ground contact)

**4. Control System (Brain)**:
- **Onboard Computer**: Processes sensor data, runs control algorithms
- **Power System**: Batteries, power distribution, voltage regulation

**5. Communication Infrastructure**:
- **Internal Bus**: Communication between sensors, actuators, and computer (CAN bus, EtherCAT)
- **Wireless**: Remote monitoring, teleoperation interfaces

### Degrees of Freedom (DOF)

The number of independently controllable joints determines a robot's dexterity:

- **Lower Body** (Legs): Typically 12 DOF total
  - 6 per leg: 3 (hip: roll, pitch, yaw) + 1 (knee: pitch) + 2 (ankle: pitch, roll)
- **Upper Body** (Arms): Typically 14 DOF total
  - 7 per arm: 3 (shoulder) + 1 (elbow) + 3 (wrist)
- **Torso**: 1-3 DOF (waist rotation, bending)
- **Head**: 2 DOF (pan, tilt for vision)

**Example**: Honda ASIMO has 57 DOF total, enabling highly dexterous movements.

### Actuation: Electric vs Hydraulic

**Electric Actuation** (Most common in research humanoids):
- **Advantages**: Precise control, energy-efficient, quiet, easier to program
- **Disadvantages**: Lower power-to-weight ratio, limited torque
- **Examples**: NAO, Pepper, ASIMO

**Hydraulic Actuation** (Heavy-duty applications):
- **Advantages**: Very high power-to-weight ratio, massive torque output
- **Disadvantages**: Complex (pumps, valves, fluid), noisy, potential leaks
- **Examples**: Boston Dynamics Atlas, HRP-series

## Technical Details

### Actuation System Deep Dive

**Servo Motors**:

A servo motor combines a DC motor, gearbox, position sensor (encoder), and control circuit:

```python
class ServoMotor:
    """
    Simplified servo motor model with position control.
    """
    def __init__(self, max_angle=180, max_torque=10.0):
        self.current_angle = 0.0  # Current position (degrees)
        self.target_angle = 0.0   # Desired position
        self.max_angle = max_angle
        self.max_torque = max_torque  # Nm

    def set_target(self, angle):
        """Set target angle (command from controller)"""
        self.target_angle = max(-self.max_angle, min(self.max_angle, angle))

    def update(self, dt, load_torque=0.0):
        """
        Simulate servo response over time step dt.

        Args:
            dt: Time step (seconds)
            load_torque: External load on the joint (Nm)
        """
        # Simple PID control (proportional only for simplicity)
        error = self.target_angle - self.current_angle
        control_torque = 0.5 * error  # Proportional gain = 0.5

        # Limit to max torque
        control_torque = max(-self.max_torque, min(self.max_torque, control_torque))

        # Net torque considering load
        net_torque = control_torque - load_torque

        # Update position (simplified dynamics: torque directly changes angle)
        # Real servos have inertia, damping, etc.
        angular_velocity = net_torque * 10  # Simplified dynamics
        self.current_angle += angular_velocity * dt

        return self.current_angle

# Example: Knee joint servo tracking a target
knee_servo = ServoMotor(max_angle=150, max_torque=50.0)
knee_servo.set_target(90)  # Target: 90 degrees (bent knee)

for step in range(20):
    current = knee_servo.update(dt=0.01, load_torque=5.0)  # 5 Nm from leg weight
    print(f"Step {step}: Angle = {current:.2f}°, Error = {knee_servo.target_angle - current:.2f}°")
```

**Output**: Servo gradually moves from 0° to 90°, demonstrating position control against a load.

### Sensor Integration

**Inertial Measurement Unit (IMU)**:

IMUs combine accelerometers and gyroscopes to measure orientation and acceleration—critical for balance:

```python
import numpy as np

class SimpleIMU:
    """
    Simplified IMU for robot orientation estimation.
    """
    def __init__(self):
        self.orientation = np.array([0.0, 0.0, 0.0])  # Roll, pitch, yaw (radians)

    def read_gyroscope(self):
        """Simulated gyroscope reading (angular velocities)"""
        # In reality, read from hardware
        return np.random.normal(0, 0.01, 3)  # Small noise around zero

    def read_accelerometer(self):
        """Simulated accelerometer reading (m/s²)"""
        # Gravity vector: should be [0, 0, 9.81] when robot upright
        gravity = np.array([0.0, 0.0, 9.81])
        noise = np.random.normal(0, 0.1, 3)
        return gravity + noise

    def estimate_orientation(self, dt):
        """
        Update orientation estimate using sensor fusion.

        Args:
            dt: Time step (seconds)
        """
        # Read sensors
        gyro = self.read_gyroscope()  # rad/s
        accel = self.read_accelerometer()  # m/s²

        # Integrate gyroscope for orientation change
        self.orientation += gyro * dt

        # Complementary filter: trust accelerometer for long-term, gyro for short-term
        # Accelerometer-based pitch estimate
        accel_pitch = np.arctan2(accel[0], accel[2])

        # Fuse estimates (simple complementary filter)
        alpha = 0.98  # Trust gyro 98%, accel 2%
        self.orientation[1] = alpha * self.orientation[1] + (1 - alpha) * accel_pitch

        return self.orientation

# Example: IMU tracking robot tilt
imu = SimpleIMU()
for step in range(50):
    orientation = imu.estimate_orientation(dt=0.01)
    roll, pitch, yaw = np.degrees(orientation)
    print(f"Step {step}: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")
```

**Why This Matters**: Humanoid robots must constantly monitor their orientation to prevent falls. A tilt of more than 10-15 degrees typically triggers emergency stabilization maneuvers.

**Force/Torque Sensors in Feet**:

Measure ground reaction forces to determine contact and weight distribution:

```python
class FootForceSensor:
    """Force sensor in robot foot for ground contact detection."""

    def __init__(self, num_sensors=4):
        """
        Args:
            num_sensors: Number of force sensors in foot (e.g., corners)
        """
        self.num_sensors = num_sensors
        self.readings = np.zeros(num_sensors)

    def read(self):
        """Simulated force sensor readings (Newtons)"""
        # In reality, read from hardware (load cells)
        # Simulate robot standing: weight distributed across foot
        self.readings = np.random.uniform(20, 30, self.num_sensors)
        return self.readings

    def is_foot_in_contact(self, threshold=5.0):
        """
        Determine if foot is on ground.

        Args:
            threshold: Minimum force (N) to consider contact
        """
        return np.any(self.readings > threshold)

    def get_center_of_pressure(self, sensor_positions):
        """
        Compute center of pressure (CoP) for balance control.

        Args:
            sensor_positions: List of (x, y) positions of each sensor in foot frame
        """
        total_force = np.sum(self.readings)
        if total_force < 1.0:
            return None  # No contact

        weighted_pos = np.zeros(2)
        for i, (x, y) in enumerate(sensor_positions):
            weighted_pos[0] += self.readings[i] * x
            weighted_pos[1] += self.readings[i] * y

        cop = weighted_pos / total_force
        return cop

# Example: Detecting foot contact
left_foot = FootForceSensor(num_sensors=4)
sensor_positions = [(0.05, 0.05), (0.05, -0.05), (-0.05, 0.05), (-0.05, -0.05)]  # Corners

forces = left_foot.read()
in_contact = left_foot.is_foot_in_contact()
cop = left_foot.get_center_of_pressure(sensor_positions)

print(f"Force readings: {forces} N")
print(f"Foot in contact: {in_contact}")
print(f"Center of Pressure: {cop}")
```

**Application**: Center of Pressure (CoP) must stay within the support polygon (footprint area) for stable standing. If CoP moves outside, the robot will tip over.

### Power and Energy Management

Humanoid robots face strict energy constraints:

- **Battery Capacity**: Typical humanoid carries 1-2 kWh battery (similar to power tools)
- **Power Consumption**:
  - Walking: 100-300W
  - Standing: 50-100W (motors holding position against gravity)
  - Running: 500-1000W
- **Operating Time**: 30 minutes to 2 hours depending on activity

**Energy-Efficient Design Strategies**:
1. **Lightweight Materials**: Carbon fiber, magnesium alloys reduce weight
2. **Regenerative Braking**: Recapture energy during downward motions
3. **Passive Dynamics**: Design joints that naturally swing (like human walking) to reduce actuation energy

## Hands-On Examples

### Example: Simulating a Robot Joint with Load

```python
class RobotJoint:
    """
    Simulates a single robot joint with motor, encoder, and gravity load.
    """
    def __init__(self, mass_kg=2.0, length_m=0.3, motor_max_torque=20.0):
        """
        Args:
            mass_kg: Mass of link attached to joint
            length_m: Length from joint to center of mass
            motor_max_torque: Maximum motor torque (Nm)
        """
        self.mass = mass_kg
        self.length = length_m
        self.max_torque = motor_max_torque

        self.angle = 0.0  # Current angle (radians)
        self.angular_velocity = 0.0  # rad/s
        self.moment_of_inertia = self.mass * self.length**2  # Simplified

    def gravity_torque(self):
        """Compute torque due to gravity."""
        g = 9.81  # m/s²
        # Torque = m * g * L * sin(theta)
        return self.mass * g * self.length * np.sin(self.angle)

    def apply_motor_torque(self, commanded_torque, dt):
        """
        Apply motor torque and update joint state.

        Args:
            commanded_torque: Desired torque from controller (Nm)
            dt: Time step (s)
        """
        # Limit torque to motor maximum
        actual_torque = np.clip(commanded_torque, -self.max_torque, self.max_torque)

        # Net torque = motor - gravity
        net_torque = actual_torque - self.gravity_torque()

        # Angular acceleration = torque / inertia
        angular_accel = net_torque / self.moment_of_inertia

        # Update velocity and position
        self.angular_velocity += angular_accel * dt
        self.angle += self.angular_velocity * dt

        # Add damping (friction)
        self.angular_velocity *= 0.99

        return self.angle, self.angular_velocity

# Example: Lifting a robot arm (shoulder joint)
shoulder = RobotJoint(mass_kg=1.5, length_m=0.25, motor_max_torque=15.0)

print("Attempting to lift arm to 90 degrees (horizontal)...")
target_angle = np.pi / 2  # 90 degrees

for step in range(100):
    # Simple proportional controller
    error = target_angle - shoulder.angle
    commanded_torque = 5.0 * error  # P-gain = 5.0

    angle, velocity = shoulder.apply_motor_torque(commanded_torque, dt=0.01)

    if step % 20 == 0:
        print(f"Step {step}: Angle={np.degrees(angle):.1f}°, Velocity={velocity:.2f} rad/s")

print(f"Final angle: {np.degrees(shoulder.angle):.2f}° (target: 90°)")
```

**Key Takeaway**: Even holding a robot arm horizontal requires continuous motor torque to counteract gravity. This is why humanoid robots consume significant power even when stationary.

## Further Resources

### Technical Specifications
- **NAO Humanoid** (SoftBank Robotics): Technical datasheet - accessible research platform
- **Atlas Robot** (Boston Dynamics): Specifications and videos - state-of-the-art dynamic humanoid
- **ASIMO** (Honda, retired 2018): Historical documentation - pioneering humanoid design

### Research Papers
- Hirose, M., & Ogawa, K. (2007). *Honda Humanoid Robots Development*. Philosophical Transactions of the Royal Society A.
- Sakagami, Y., et al. (2002). *The Intelligent ASIMO: System Overview and Integration*. IEEE/RSJ Intelligent Robots and Systems.

### Books
- *Humanoid Robotics: A Reference* by Ambarish Goswami and Prahlad Vadakkepat
- *Introduction to Humanoid Robotics* by Shuuji Kajita et al.

### Video Resources
- IEEE Spectrum Robotics Channel - Humanoid robot demonstrations
- Boston Dynamics YouTube - Atlas robot capabilities

---

**Next**: Dive into [Kinematics](./kinematics) to learn the mathematics of robot motion, or test your knowledge with [Self-Assessment](./self-assessment).
