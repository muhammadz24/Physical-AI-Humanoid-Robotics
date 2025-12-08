---
sidebar_position: 3
title: Kinematics
---

# Kinematics of Humanoid Robots

## Introduction

Kinematics is the mathematical study of motion without considering forces. For humanoid robots, kinematics answers fundamental questions: "If I move joint 3 by 30 degrees, where will the hand end up?" (forward kinematics) and "What joint angles do I need to reach this cup?" (inverse kinematics). Mastering kinematics is essential for programming robots to manipulate objects, walk, and interact with their environment.

This section covers the mathematical frameworks used to describe and control humanoid robot motion, from basic coordinate transformations to solving inverse kinematics for multi-joint systems.

## Conceptual Foundation

### Forward vs. Inverse Kinematics

**Forward Kinematics (FK)**:
- Input: Joint angles (θ₁, θ₂, ..., θₙ)
- Output: End-effector position and orientation in world coordinates
- **Example**: "If shoulder is at 45° and elbow at 90°, where is the hand?"
- **Complexity**: Straightforward—apply transformation matrices sequentially

**Inverse Kinematics (IK)**:
- Input: Desired end-effector position and orientation
- Output: Joint angles that achieve that pose
- **Example**: "What joint angles put the hand at coordinates (0.5, 0.3, 1.2)?"
- **Complexity**: Challenging—may have multiple solutions or no solution

### Coordinate Frames and Transformations

Each robot link has its own coordinate frame. We use homogeneous transformation matrices to convert between frames:

A transformation matrix combines rotation and translation:
```
T = [R | t]  where R is 3x3 rotation, t is 3x1 translation
    [0 | 1]
```

Denavit-Hartenberg (DH) parameters provide a standard way to define these transformations for serial robots.

## Technical Details

### Forward Kinematics Implementation

```python
import numpy as np

class SimpleRobotArm:
    """
    2-link planar robot arm for demonstrating forward kinematics.
    """
    def __init__(self, link1_length=0.3, link2_length=0.25):
        self.L1 = link1_length  # meters
        self.L2 = link2_length

    def forward_kinematics(self, theta1, theta2):
        """
        Compute end-effector position for given joint angles.

        Args:
            theta1: First joint angle (radians)
            theta2: Second joint angle (radians)

        Returns:
            (x, y) end-effector position in world frame
        """
        # Position of joint 2 (elbow)
        x1 = self.L1 * np.cos(theta1)
        y1 = self.L1 * np.sin(theta1)

        # Position of end-effector (hand)
        x2 = x1 + self.L2 * np.cos(theta1 + theta2)
        y2 = y1 + self.L2 * np.sin(theta1 + theta2)

        return (x2, y2)

# Example: Robot arm with both joints at 45 degrees
arm = SimpleRobotArm(link1_length=0.3, link2_length=0.25)
theta1 = np.radians(45)
theta2 = np.radians(45)

x, y = arm.forward_kinematics(theta1, theta2)
print(f"Joint angles: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°")
print(f"End-effector position: ({x:.3f}, {y:.3f}) m")
```

**Output**: End-effector position computed from joint angles.

### Inverse Kinematics: Analytical Solution

For simple 2-link arms, IK has a closed-form solution:

```python
class SimpleRobotArm:
    # ... (previous code)

    def inverse_kinematics(self, target_x, target_y):
        """
        Compute joint angles to reach target position (2D planar arm).

        Args:
            target_x, target_y: Desired end-effector position

        Returns:
            (theta1, theta2) joint angles, or None if unreachable
        """
        # Distance from base to target
        d = np.sqrt(target_x**2 + target_y**2)

        # Check if target is reachable
        if d > (self.L1 + self.L2) or d < abs(self.L1 - self.L2):
            return None  # Target out of reach

        # Law of cosines to find theta2
        cos_theta2 = (d**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Numerical stability

        # Two solutions: elbow up or elbow down
        theta2 = np.arccos(cos_theta2)  # Elbow up solution

        # Solve for theta1
        k1 = self.L1 + self.L2 * np.cos(theta2)
        k2 = self.L2 * np.sin(theta2)
        theta1 = np.arctan2(target_y, target_x) - np.arctan2(k2, k1)

        return (theta1, theta2)

# Example: Reach position (0.4, 0.3)
arm = SimpleRobotArm(link1_length=0.3, link2_length=0.25)
result = arm.inverse_kinematics(target_x=0.4, target_y=0.3)

if result:
    theta1, theta2 = result
    print(f"Solution: θ1={np.degrees(theta1):.1f}°, θ2={np.degrees(theta2):.1f}°")

    # Verify with forward kinematics
    x, y = arm.forward_kinematics(theta1, theta2)
    print(f"Verification: Reached ({x:.3f}, {y:.3f}) m")
    print(f"Error: {np.sqrt((x-0.4)**2 + (y-0.3)**2):.6f} m")
else:
    print("Target unreachable")
```

**Key Insight**: For 2-link arms, analytical IK is straightforward. For humanoid arms with 7+ joints, numerical methods (Jacobian-based, optimization) are required.

### Jacobian Matrix for Velocity Control

The Jacobian relates joint velocities to end-effector velocity:
```
ẋ = J(θ) * θ̇
```
Where:
- ẋ: End-effector linear velocity (m/s)
- θ̇: Joint angular velocities (rad/s)
- J(θ): Jacobian matrix (depends on current joint configuration)

```python
def compute_jacobian_2d(arm, theta1, theta2):
    """
    Compute Jacobian matrix for 2-link planar arm.

    Returns:
        2x2 Jacobian matrix J
    """
    L1, L2 = arm.L1, arm.L2

    # Partial derivatives of end-effector position w.r.t. joint angles
    J = np.array([
        [-L1*np.sin(theta1) - L2*np.sin(theta1+theta2), -L2*np.sin(theta1+theta2)],
        [ L1*np.cos(theta1) + L2*np.cos(theta1+theta2),  L2*np.cos(theta1+theta2)]
    ])

    return J

# Example: Compute required joint velocities to move end-effector
arm = SimpleRobotArm(link1_length=0.3, link2_length=0.25)
theta1, theta2 = np.radians(45), np.radians(45)

# Desired end-effector velocity: move right at 0.1 m/s
desired_velocity = np.array([0.1, 0.0])  # [vx, vy]

J = compute_jacobian_2d(arm, theta1, theta2)
joint_velocities = np.linalg.pinv(J) @ desired_velocity  # Pseudo-inverse

print(f"Desired end-effector velocity: {desired_velocity} m/s")
print(f"Required joint velocities: θ̇1={np.degrees(joint_velocities[0]):.2f} deg/s, θ̇2={np.degrees(joint_velocities[1]):.2f} deg/s")
```

**Application**: Velocity control is used for smooth motions and real-time trajectory tracking.

### Walking Kinematics: Zero Moment Point (ZMP)

For bipedal locomotion, the Zero Moment Point (ZMP) criterion ensures dynamic stability:

**ZMP Definition**: The point on the ground where the net moment of all forces (gravity, inertia) is zero.

**Stability Criterion**: If ZMP is inside the support polygon (footprint), the robot is stable.

```python
class BipedalWalkingController:
    """
    Simplified ZMP-based walking controller.
    """
    def __init__(self, robot_mass=50.0, robot_height=1.0):
        self.mass = robot_mass  # kg
        self.height = robot_height  # m (height of center of mass)
        self.g = 9.81  # m/s²

    def compute_zmp(self, com_position, com_acceleration):
        """
        Compute Zero Moment Point position.

        Args:
            com_position: [x, y, z] center of mass position (m)
            com_acceleration: [ax, ay, az] CoM acceleration (m/s²)

        Returns:
            [zmp_x, zmp_y] ZMP position on ground
        """
        x_com, y_com, z_com = com_position
        ax, ay, az = com_acceleration

        # ZMP formula (simplified, assuming flat ground at z=0)
        zmp_x = x_com - (z_com / (self.g + az)) * ax
        zmp_y = y_com - (z_com / (self.g + az)) * ay

        return np.array([zmp_x, zmp_y])

    def is_stable(self, zmp, support_polygon):
        """
        Check if robot is stable based on ZMP.

        Args:
            zmp: [x, y] ZMP position
            support_polygon: List of (x, y) vertices of foot/feet contact area

        Returns:
            True if ZMP inside support polygon
        """
        # Simplified: check if ZMP within bounding box of support polygon
        xs = [p[0] for p in support_polygon]
        ys = [p[1] for p in support_polygon]

        return (min(xs) <= zmp[0] <= max(xs)) and (min(ys) <= zmp[1] <= max(ys))

# Example: Robot standing on one foot
controller = BipedalWalkingController(robot_mass=50, robot_height=0.9)

com_pos = [0.0, 0.0, 0.9]  # CoM directly above foot
com_accel = [0.5, 0.0, 0.0]  # Accelerating forward

zmp = controller.compute_zmp(com_pos, com_accel)
print(f"ZMP position: ({zmp[0]:.3f}, {zmp[1]:.3f}) m")

# Support polygon: single foot (20cm x 10cm)
foot_polygon = [(-0.1, -0.05), (-0.1, 0.05), (0.1, 0.05), (0.1, -0.05)]
stable = controller.is_stable(zmp, foot_polygon)

print(f"Robot stable: {stable}")
```

**Result**: ZMP moves forward during acceleration. If it leaves the foot area, robot will tip over.

## Hands-On Examples

### Example: Trajectory Planning for Smooth Motion

```python
def linear_trajectory(start_pos, end_pos, duration, dt):
    """
    Generate smooth linear trajectory from start to end.

    Args:
        start_pos: [x, y] starting position
        end_pos: [x, y] ending position
        duration: Total time (seconds)
        dt: Time step (seconds)

    Yields:
        (t, [x, y]) at each time step
    """
    num_steps = int(duration / dt)
    for step in range(num_steps + 1):
        t = step * dt
        # Linear interpolation
        alpha = t / duration
        x = start_pos[0] + alpha * (end_pos[0] - start_pos[0])
        y = start_pos[1] + alpha * (end_pos[1] - start_pos[1])
        yield (t, np.array([x, y]))

# Example: Move robot hand from (0.3, 0.2) to (0.5, 0.4) in 2 seconds
print("Trajectory waypoints:")
for t, pos in linear_trajectory([0.3, 0.2], [0.5, 0.4], duration=2.0, dt=0.5):
    print(f"  t={t:.1f}s: position=({pos[0]:.2f}, {pos[1]:.2f})")

    # At each waypoint, solve IK to get joint angles
    arm = SimpleRobotArm()
    angles = arm.inverse_kinematics(pos[0], pos[1])
    if angles:
        print(f"    Joint angles: θ1={np.degrees(angles[0]):.1f}°, θ2={np.degrees(angles[1]):.1f}°")
```

**Application**: Trajectory planning generates smooth paths for robot arms when picking, placing, or avoiding obstacles.

## Further Resources

### Textbooks
- *Robot Modeling and Control* by Mark W. Spong - Comprehensive kinematics and dynamics
- *Modern Robotics* by Kevin Lynch and Frank Park - Contemporary approach with code examples

### Online Courses
- *Robotic Systems* (ETH Zurich, edX) - Covers kinematics, control, and planning
- *Robot Mechanics* (Georgia Tech, Coursera) - Mathematical foundations

### Software Tools
- **ROS MoveIt**: Motion planning framework with IK solvers
- **Peter Corke Robotics Toolbox** (Python/MATLAB): Kinematics visualization and computation
- **PyBullet**: Physics simulation for testing kinematic solutions

### Research Papers
- Kajita, S., et al. (2003). *Biped Walking Pattern Generation by using Preview Control of ZMP*. IEEE Robotics and Automation.
- Vukobratović, M., & Borovac, B. (2004). *Zero-Moment Point: Thirty Five Years of its Life*. International Journal of Humanoid Robotics.

---

**Next**: Test your understanding with [Self-Assessment](./self-assessment), or proceed to [Chapter 3: ROS 2 Fundamentals](/chapter-03/) to learn the software framework powering modern robots.
