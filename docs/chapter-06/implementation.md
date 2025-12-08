---
sidebar_position: 3
title: Implementation Guide
---

# Step-by-Step Implementation

## Phase 1: Simulation Environment

### Create Gazebo World

```xml
<!-- config/robot_world.world -->
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="robot_workspace">
    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
    </light>

    <!-- Ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Red cube -->
    <model name="red_cube">
      <pose>0.5 0.2 0.025 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.05 0.05 0.05</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.05 0.05 0.05</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Blue cylinder -->
    <model name="blue_cylinder">
      <pose>0.4 -0.2 0.025 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder><radius>0.025</radius><length>0.05</length></cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder><radius>0.025</radius><length>0.05</length></cylinder>
          </geometry>
          <material>
            <ambient>0 0 1 1</ambient>
            <diffuse>0 0 1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Launch Gazebo

```python
# launch/gazebo_world.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', 'config/robot_world.world'],
            output='screen'
        )
    ])
```

## Phase 2: Perception Module

### Object Detector

```python
# src/perception/object_detector.py
import cv2
import numpy as np

class ColorObjectDetector:
    """Detects colored objects in RGB images."""

    def __init__(self):
        # HSV color ranges
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'green': ([40, 100, 100], [80, 255, 255])
        }

    def detect(self, image, color='red'):
        """
        Detect objects of specified color.

        Args:
            image: RGB image (numpy array)
            color: Color to detect ('red', 'blue', 'green')

        Returns:
            List of detected object centroids [(x, y), ...]
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)

        # Create mask
        lower, upper = self.color_ranges[color]
        mask = cv2.inRange(hsv, np.array(lower), np.array(upper))

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detections = []
        for contour in contours:
            if cv2.contourArea(contour) > 100:  # Filter noise
                M = cv2.moments(contour)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    detections.append((cx, cy))

        return detections

# Example usage
# detector = ColorObjectDetector()
# detections = detector.detect(camera_image, color='red')
```

### ROS 2 Camera Node

```python
# src/perception/camera_processor.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np

class CameraProcessorNode(Node):
    def __init__(self):
        super().__init__('camera_processor')

        self.bridge = CvBridge()
        self.detector = ColorObjectDetector()

        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish detections
        self.detection_pub = self.create_publisher(
            Point,
            '/detected_objects',
            10
        )

        self.target_color = 'red'  # Default

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Detect objects
        detections = self.detector.detect(cv_image, self.target_color)

        # Publish first detection
        if detections:
            cx, cy = detections[0]

            # Convert pixel coordinates to 3D point (simplified)
            # In real system, use camera calibration
            point = Point()
            point.x = 0.5  # Estimated depth
            point.y = (cx - 320) * 0.001  # Pixel to meters
            point.z = (240 - cy) * 0.001

            self.detection_pub.publish(point)
            self.get_logger().info(f'Detected {self.target_color} object at ({point.x}, {point.y}, {point.z})')

def main():
    rclpy.init()
    node = CameraProcessorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Phase 3: Planning Module

### Grasp Planner

```python
# src/planning/grasp_planner.py
import numpy as np

class GraspPlanner:
    """Compute grasp poses using IK."""

    def __init__(self, robot_arm):
        self.robot = robot_arm  # From Chapter 2

    def plan_grasp(self, object_position):
        """
        Compute joint angles to grasp object.

        Args:
            object_position: (x, y, z) in world frame

        Returns:
            Joint angles or None if unreachable
        """
        # Approach from above
        grasp_pose = np.array(object_position)
        grasp_pose[2] += 0.1  # 10cm above object

        # Solve IK (using Chapter 2 methods)
        joint_angles = self.robot.inverse_kinematics(grasp_pose[:2])  # 2D for simplicity

        if joint_angles is None:
            return None

        # Add vertical offset
        return joint_angles

    def plan_trajectory(self, start_joints, end_joints, duration=2.0, dt=0.1):
        """Generate smooth trajectory."""
        num_steps = int(duration / dt)
        trajectory = []

        for step in range(num_steps + 1):
            alpha = step / num_steps
            # Linear interpolation
            joints = start_joints + alpha * (end_joints - start_joints)
            trajectory.append(joints)

        return trajectory
```

## Phase 4: Pick-and-Place Action Server

```python
# src/control/pick_place_server.py
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class PickPlaceServer(Node):
    def __init__(self):
        super().__init__('pick_place_server')

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robot_arm/follow_joint_trajectory',
            self.execute_callback
        )

        self.get_logger().info('Pick-and-place server ready')

    def execute_callback(self, goal_handle):
        """Execute pick-and-place sequence."""
        self.get_logger().info('Executing pick-and-place...')

        # 1. Move to pre-grasp
        # 2. Descend to grasp
        # 3. Close gripper
        # 4. Lift
        # 5. Move to place location
        # 6. Open gripper

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

def main():
    rclpy.init()
    server = PickPlaceServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
```

## Phase 5: Command Parser

```python
# src/language/command_parser.py
import re

class CommandParser:
    """Parse natural language commands."""

    def __init__(self):
        self.colors = ['red', 'blue', 'green']
        self.objects = ['cube', 'cylinder', 'block']
        self.actions = ['pick', 'grab', 'get']

    def parse(self, command):
        """
        Extract action, color, and object from command.

        Args:
            command: String like "pick up the red cube"

        Returns:
            {'action': 'pick', 'color': 'red', 'object': 'cube'}
        """
        command_lower = command.lower()

        parsed = {
            'action': None,
            'color': None,
            'object': None
        }

        # Find action
        for action in self.actions:
            if action in command_lower:
                parsed['action'] = 'pick'
                break

        # Find color
        for color in self.colors:
            if color in command_lower:
                parsed['color'] = color
                break

        # Find object
        for obj in self.objects:
            if obj in command_lower:
                parsed['object'] = obj
                break

        return parsed

# Example
# parser = CommandParser()
# result = parser.parse("Pick up the red cube")
# print(result)  # {'action': 'pick', 'color': 'red', 'object': 'cube'}
```

## Integration and Testing

### Main Launch File

```python
# launch/robot_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='capstone_robot',
            executable='camera_processor',
            name='camera_processor'
        ),
        Node(
            package='capstone_robot',
            executable='pick_place_server',
            name='pick_place_server'
        )
    ])
```

### Test Command

```bash
# Terminal 1: Launch Gazebo
ros2 launch capstone_robot gazebo_world.launch.py

# Terminal 2: Launch robot system
ros2 launch capstone_robot robot_system.launch.py

# Terminal 3: Send command
ros2 topic pub /language_command std_msgs/String "data: 'pick up the red cube'"
```

## Debugging Tips

1. **Visualization**: Use RViz to see detected objects, planned trajectories
2. **Logging**: Add detailed logging at each step
3. **Step-by-step**: Test each module independently before integration
4. **Simulation Speed**: Reduce Gazebo real-time factor if too slow

---

**Next**: [Self-Assessment](./self-assessment) to evaluate your understanding
