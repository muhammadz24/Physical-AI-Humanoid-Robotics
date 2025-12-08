---
sidebar_position: 3
title: Integration with Robot Systems
---

# Integrating VLA Models with Robot Systems

## Introduction

While VLA models provide powerful language-to-action capabilities, integrating them with real robot systems requires bridging the gap between high-level model predictions and low-level robot control. This section covers practical integration patterns for deploying VLA models in ROS 2-based robot systems, handling real-time constraints, and ensuring safe operation.

## Integration Architecture

### System Components

```
User → Voice/Text → VLA Model → Action Commands → ROS 2 → Robot Hardware
                        ↓
                   Camera Feed (Vision)
```

**Data Flow**:
1. Camera publishes images to ROS 2 topic
2. VLA node subscribes to images and text commands
3. VLA predicts actions
4. Actions published as joint/gripper commands
5. Robot controllers execute actions

## ROS 2 Integration Example

### VLA as a ROS 2 Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from cv_bridge import CvBridge
import numpy as np
from PIL import Image as PILImage

class VLARobotNode(Node):
    """ROS 2 node running VLA model."""

    def __init__(self):
        super().__init__('vla_robot_node')

        # Initialize VLA model (from previous section)
        # self.vla_model = SimpleVLAModel()

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_sub = self.create_subscription(
            String,
            '/vla/text_command',
            self.command_callback,
            10
        )

        # Publisher for robot commands
        self.action_pub = self.create_publisher(
            JointState,
            '/robot/joint_commands',
            10
        )

        self.get_logger().info('VLA Robot Node initialized')

    def image_callback(self, msg):
        """Receive camera images."""
        # Convert ROS Image to PIL Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.latest_image = PILImage.fromarray(cv_image)

    def command_callback(self, msg):
        """Receive text commands and execute."""
        command = msg.data
        self.get_logger().info(f'Received command: {command}')

        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            return

        # Predict actions using VLA
        # actions = self.vla_model.predict_action(self.latest_image, command)

        # For demo: simulate actions
        actions = np.array([0.1, 0.2, -0.1, 0.0, 0.1, 0.0, 1.0])  # 7 joints

        # Publish as JointState message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        joint_msg.position = actions.tolist()

        self.action_pub.publish(joint_msg)
        self.get_logger().info('Published joint commands')

def main(args=None):
    rclpy.init(args=args)
    node = VLARobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File

```python
# launch/vla_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_robot',
            executable='vla_node',
            name='vla_robot_node',
            output='screen',
            parameters=[{
                'model_path': '/path/to/vla/model.pth',
                'confidence_threshold': 0.8
            }]
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node',
            name='camera',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30
            }]
        )
    ])
```

## Safety and Validation

### Action Validation Layer

```python
class SafetyValidator:
    """Validate VLA actions before execution."""

    def __init__(self, joint_limits):
        """
        Args:
            joint_limits: Dict of {joint_name: (min, max)}
        """
        self.joint_limits = joint_limits
        self.max_velocity = 0.5  # rad/s
        self.previous_position = None

    def validate_action(self, joint_positions, dt=0.1):
        """
        Check if action is safe to execute.

        Returns:
            (is_safe, modified_action)
        """
        safe_action = np.array(joint_positions).copy()
        is_safe = True

        # Check joint limits
        for i, (name, pos) in enumerate(zip(self.joint_limits.keys(), safe_action)):
            min_limit, max_limit = self.joint_limits[name]

            if pos < min_limit or pos > max_limit:
                safe_action[i] = np.clip(pos, min_limit, max_limit)
                is_safe = False
                print(f"Warning: {name} exceeds limits. Clipped to {safe_action[i]}")

        # Check velocity limits
        if self.previous_position is not None:
            velocities = (safe_action - self.previous_position) / dt

            if np.any(np.abs(velocities) > self.max_velocity):
                # Scale down to max velocity
                scale = self.max_velocity / np.max(np.abs(velocities))
                safe_action = self.previous_position + (safe_action - self.previous_position) * scale
                is_safe = False
                print("Warning: Velocity limit exceeded. Scaled down action.")

        self.previous_position = safe_action.copy()
        return is_safe, safe_action

# Example usage
joint_limits = {
    'joint1': (-np.pi, np.pi),
    'joint2': (-np.pi/2, np.pi/2),
    # ... more joints
}

validator = SafetyValidator(joint_limits)
predicted_action = np.array([0.5, 0.3, -0.2, 0.1, 0.0, 0.1, 1.0])
is_safe, safe_action = validator.validate_action(predicted_action)

if is_safe:
    print("Action is safe to execute")
else:
    print(f"Action modified for safety: {safe_action}")
```

## Further Resources

### Tutorials
- ROS 2 Python API: https://docs.ros.org/en/humble/Tutorials.html
- CV Bridge: http://wiki.ros.org/cv_bridge

### Example Projects
- Manipulation demos: https://github.com/ros-planning/moveit2_tutorials
- Vision pipelines: https://github.com/ros-perception

---

**Next**: [Self-Assessment](./self-assessment) | [Chapter 6: Capstone Project](/chapter-06/)
