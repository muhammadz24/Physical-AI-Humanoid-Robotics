---
sidebar_position: 2
title: Isaac ROS Setup
---

# Isaac ROS: GPU-Accelerated Perception

## Installation (Ubuntu 22.04 + RTX GPU)

```bash
# Install Isaac ROS dependencies
sudo apt-get install -y ros-humble-isaac-ros-base

# Clone Isaac ROS repos
mkdir -p ~/isaac_ros_ws/src
cd ~/isaac_ros_ws/src

git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_object_detection.git

# Build
cd ~/isaac_ros_ws
colcon build --symlink-install

source install/setup.bash
```

## GPU-Accelerated YOLOv8

**Launch Isaac ROS DetectNet**:

```python
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Subscribe to Isaac ROS detections
        self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            label = detection.results[0].id
            score = detection.results[0].score
            bbox = detection.bbox.center

            self.get_logger().info(
                f"Detected: {label}, Confidence: {score:.2f}, Position: ({bbox.x}, {bbox.y})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Performance**: 30-60 FPS on RTX 4070 Ti, 15-30 FPS on Jetson Orin Nano.

---

**Next**: [Visual SLAM](./vslam) for real-time mapping.
