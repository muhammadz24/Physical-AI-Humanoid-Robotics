---
sidebar_position: 2
title: ROS 2 Architecture
---

# ROS 2 Architecture

## Introduction

The Robot Operating System 2 (ROS 2) represents a fundamental reimagining of robotics middleware, built from the ground up to address the limitations of its predecessor while meeting the demands of production robotics systems. Unlike ROS 1, which was designed primarily for research environments, ROS 2 targets mission-critical applications where real-time performance, security, and reliability are paramount. From autonomous vehicles navigating busy city streets to industrial cobots working alongside humans, ROS 2 provides the robust foundation necessary for deploying robots in the real world.

ROS 2's significance lies in its distributed, modular architecture that enables complex robot systems to be composed from independent, reusable components. This design philosophy allows teams to develop perception, planning, and control subsystems independently, integrate third-party libraries seamlessly, and scale from single-robot prototypes to multi-robot fleets. Whether you're building a laboratory research platform or a commercial product, ROS 2 provides the tools, libraries, and community support to accelerate development while maintaining production-grade quality.

## Conceptual Foundation

### What is ROS 2?

ROS 2 is not an operating system in the traditional sense, but rather a middleware framework that provides:

**1. Communication Infrastructure**: Standardized message passing between processes, whether on the same machine or across networks. This enables distributed computing where sensor processing, decision-making, and control can run on separate processors, each optimized for its task.

**2. Hardware Abstraction**: Uniform interfaces to sensors, actuators, and algorithms. A camera driver publishes images in a standard format regardless of manufacturer, allowing perception algorithms to work with any camera without modification.

**3. Package Ecosystem**: Over 3,000 community-contributed packages covering navigation, manipulation, perception, simulation, and visualization. This ecosystem accelerates development by providing battle-tested solutions to common robotics problems.

**4. Development Tools**: Command-line tools (ros2), visualization (RViz2), simulation (Gazebo), logging, debugging, and testing frameworks that streamline the development workflow.

### ROS 2 vs ROS 1: Why the Redesign?

ROS 1 served robotics research well for over a decade, but several fundamental limitations necessitated a ground-up redesign:

**Network Dependency**: ROS 1 required a central master node for discovery and connection management. If the master failed, the entire robot system collapsed. ROS 2 eliminates this single point of failure with distributed discovery using DDS (Data Distribution Service).

**Real-Time Performance**: ROS 1's TCP-based communication introduced unpredictable latency, problematic for control loops requiring millisecond-level responsiveness. ROS 2 supports real-time communication with deterministic timing guarantees.

**Security**: ROS 1 had no built-in security—any process could publish, subscribe, or control any part of the system. ROS 2 integrates DDS-Security for authentication, encryption, and access control, essential for commercial deployments.

**Multi-Robot Systems**: ROS 1's single-master architecture struggled with multiple robots. ROS 2's distributed discovery naturally supports robot swarms and multi-robot coordination without architectural hacks.

**Platform Support**: ROS 1 was Linux-centric. ROS 2 supports Linux, Windows, macOS, and real-time operating systems (RTOS), enabling deployment on embedded platforms and integration with non-Linux toolchains.

### The DDS Foundation

At ROS 2's core is DDS (Data Distribution Service), an OMG (Object Management Group) standard for real-time, peer-to-peer communication. DDS provides:

**Quality of Service (QoS)**: Fine-grained control over reliability, latency, bandwidth usage, and resource limits. For example, you can configure high-frequency sensor data to use "best effort" delivery (fast, occasional drops acceptable) while command messages use "reliable" delivery (guaranteed, ordered).

**Discovery**: Automatic detection of publishers and subscribers without central coordination. When a camera node starts, nodes needing camera data automatically discover and subscribe to it.

**Data-Centric Communication**: DDS manages data flows, not connections. Nodes publish data types; interested nodes subscribe. The middleware handles all connectivity, buffering, and delivery.

This foundation enables ROS 2's scalability and robustness. However, ROS 2 abstracts DDS complexity, so developers can focus on robot behavior rather than middleware configuration.

### ROS 2 Architecture Layers

A complete ROS 2 system consists of four architectural layers:

**Application Layer**: Your robot's custom logic—perception nodes processing camera images, planning nodes computing paths, control nodes sending motor commands. This is where robot-specific intelligence resides.

**ROS Client Library (rcl)**: The API you interact with (`rclpy` for Python, `rclcpp` for C++). This layer provides node creation, topic publishing/subscribing, service calls, parameter management, and lifecycle management.

**ROS Middleware (rmw)**: An abstraction layer that allows swapping DDS implementations (FastDDS, CycloneDDS, RTI Connext) without changing application code. This vendor independence prevents lock-in and allows choosing implementations optimized for specific use cases.

**DDS Layer**: The actual communication implementation handling message serialization, network transport, quality of service, and discovery.

This layered design separates concerns: application developers work at the client library level, system integrators configure middleware and QoS, and ROS 2 maintainers optimize the rmw and DDS layers.

## Technical Details

### Core ROS 2 Concepts

**Nodes**: The fundamental computation unit in ROS 2. Each node is an independent process performing a specific function: reading sensors, computing trajectories, controlling actuators. Nodes run concurrently and communicate via topics, services, and actions.

**Design Principle**: Keep nodes focused and single-purpose. A camera driver node publishes raw images. A separate image processing node detects objects. A third node tracks detected objects. This modularity enables reuse, testing, and parallel development.

**Topics**: Named buses for streaming data. Nodes publish messages to topics; other nodes subscribe to topics of interest. Topics implement one-to-many, asynchronous communication ideal for sensor data, state updates, and continuous control signals.

**Example**: A `/camera/image_raw` topic carries images from a camera node. Multiple subscribers (object detector, image display, data logger) receive each published image independently.

**Messages**: Strongly-typed data structures defining topic content. ROS 2 provides standard message types (`sensor_msgs/Image`, `geometry_msgs/Twist`, `nav_msgs/Odometry`) and tools to define custom messages.

**Services**: Synchronous request-response communication. A client sends a request and blocks until the server responds. Services suit operations with clear inputs and outputs: triggering a gripper close, requesting a robot's battery level, or resetting a sensor.

**Actions**: Asynchronous request-response for long-running tasks. Like services, but with feedback during execution and the ability to cancel. Actions are ideal for navigation to a goal, picking an object, or executing a planned trajectory—tasks that take seconds or minutes.

**Parameters**: Runtime-configurable values that modify node behavior without recompilation. Parameters set thresholds, select algorithms, tune controllers, and enable/disable features.

### ROS 2 Graph Architecture

A running ROS 2 system forms a computational graph:

```
┌─────────────┐        /camera/image_raw        ┌──────────────────┐
│   Camera    │ ────────────────────────────────> │ Object Detector  │
│   Driver    │                                   │                  │
└─────────────┘                                   └────────┬─────────┘
                                                           │ /detected_objects
                                                           │
                                                           v
┌─────────────┐        /cmd_vel                 ┌──────────────────┐
│   Planner   │ ────────────────────────────────> │  Motor Control   │
│             │ <──────────────────────────────── │                  │
└─────────────┘        /odom                     └──────────────────┘
```

**Nodes** (rectangles) communicate via **topics** (arrows). This graph is dynamic: nodes can start, stop, publish, and subscribe at runtime. The ROS 2 middleware handles connection management automatically.

### Quality of Service (QoS) Policies

QoS policies fine-tune communication behavior for different data types:

**Reliability**:
- `RELIABLE`: Every message is delivered, retransmitted if lost (like TCP). Use for commands, configuration.
- `BEST_EFFORT`: Messages may be lost if network is congested (like UDP). Use for high-frequency sensor streams where old data is useless.

**Durability**:
- `VOLATILE`: Only send to subscribers connected when published. Standard for sensor streams.
- `TRANSIENT_LOCAL`: New subscribers receive the last N messages published before they connected. Useful for configuration data, map data.

**History**:
- `KEEP_LAST(N)`: Buffer the last N messages. Old messages are discarded. Prevents unbounded memory growth.
- `KEEP_ALL`: Buffer all messages. Risky—can exhaust memory if subscribers are slow.

**Lifespan**: Maximum time a message is valid. After expiration, the middleware discards it. Prevents stale sensor data from being processed.

**Example**: Camera images (30 Hz) use `BEST_EFFORT` + `VOLATILE` + `KEEP_LAST(1)` because only the latest frame matters. Navigation goals use `RELIABLE` + `TRANSIENT_LOCAL` + `KEEP_LAST(10)` to ensure commands aren't lost and new planners see recent goals.

### ROS 2 Build System: Colcon and Ament

ROS 2 uses **colcon** (collective construction) to build workspaces containing multiple packages:

**Workspace Structure**:
```
ros2_ws/
├── src/              # Source code packages
│   ├── my_robot_description/
│   ├── my_robot_bringup/
│   └── my_robot_control/
├── build/            # Intermediate build files (generated)
├── install/          # Installed binaries and libraries (generated)
└── log/              # Build logs (generated)
```

**Ament**: ROS 2's build system extensions for CMake and Python packages. Ament handles:
- Package dependency resolution
- Message and service code generation
- Install rules for sharing packages

**Build Command**:
```bash
cd ros2_ws
colcon build --symlink-install
```

The `--symlink-install` flag symlinks Python files instead of copying, allowing code changes without rebuilding.

## Hands-On Examples

### Example 1: Setting Up a ROS 2 Workspace

Before writing ROS 2 code, create a workspace:

```bash
# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 installation (Humble distribution)
source /opt/ros/humble/setup.bash

# Build empty workspace (creates build/, install/, log/)
colcon build

# Source workspace overlay
source install/setup.bash

# Verify workspace
echo $ROS_DISTRO  # Should output: humble
```

**What This Does**: Creates a catkin-style workspace where ROS 2 packages will live. Sourcing `setup.bash` adds workspace packages to the ROS 2 environment.

### Example 2: Understanding ROS 2 Graph Tools

ROS 2 provides introspection tools to visualize and debug running systems:

```bash
# List all active nodes
ros2 node list

# Get information about a specific node
ros2 node info /camera_driver

# List all topics
ros2 topic list

# Show message type and publishers/subscribers for a topic
ros2 topic info /camera/image_raw

# Echo messages being published to a topic
ros2 topic echo /camera/image_raw --once

# Measure topic publishing rate
ros2 topic hz /camera/image_raw

# Show ROS 2 graph visually (requires rqt)
rqt_graph
```

**Expected Output** (for `ros2 node info /camera_driver`):
```
/camera_driver
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /camera/image_raw: sensor_msgs/msg/Image
    /camera/camera_info: sensor_msgs/msg/CameraInfo
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Service Servers:
    /camera_driver/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /camera_driver/get_parameters: rcl_interfaces/srv/GetParameters
    ...
  Service Clients:

  Action Servers:

  Action Clients:
```

This shows what the camera driver publishes, subscribes to, and what services it provides.

### Example 3: Inspecting Message Definitions

Understanding message structure is critical for using topics:

```bash
# Show structure of Image message
ros2 interface show sensor_msgs/msg/Image
```

**Output**:
```
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

This tells you:
- Images have a header with timestamp and coordinate frame
- Image data is in a `uint8[]` array
- Encoding specifies format (e.g., "rgb8", "bgr8", "mono8")

### Example 4: Creating a Custom Package

Create a Python package to hold custom ROS 2 nodes:

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python --node-name my_first_node my_robot_package

# Directory structure created:
# my_robot_package/
# ├── my_robot_package/
# │   ├── __init__.py
# │   └── my_first_node.py
# ├── resource/
# ├── test/
# ├── package.xml
# ├── setup.py
# └── setup.cfg
```

**Key Files**:
- `package.xml`: Package metadata (dependencies, version, maintainer)
- `setup.py`: Python package configuration and entry points
- `my_robot_package/my_first_node.py`: Skeleton node code

Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_package
source install/setup.bash
```

Run the node:
```bash
ros2 run my_robot_package my_first_node
```

### Example 5: Configuring QoS Policies in Python

Demonstrate setting QoS for different communication patterns:

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Define QoS for high-frequency sensor data (best effort)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Allow message loss for speed
            durability=DurabilityPolicy.VOLATILE,        # No historical data
            history=HistoryPolicy.KEEP_LAST,             # Keep only last N messages
            depth=1                                       # Buffer size: 1 (only latest)
        )

        # Create subscription with custom QoS
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            sensor_qos
        )

        self.get_logger().info('Camera subscriber started with BEST_EFFORT QoS')

    def image_callback(self, msg):
        self.get_logger().info(f'Received image: {msg.width}x{msg.height}, encoding: {msg.encoding}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What This Demonstrates**:
- Custom QoS configuration for high-frequency data streams
- Trade-off: `BEST_EFFORT` is faster but may drop messages under load
- Appropriate for sensor data where latest value is most important

### Common Pitfalls

1. **Forgetting to Source Workspace**: After building, you must source `install/setup.bash` to use packages. Common error: `Package 'my_package' not found`.

2. **QoS Mismatch**: Publishers and subscribers must have compatible QoS. A `RELIABLE` subscriber won't receive from a `BEST_EFFORT` publisher. Check QoS compatibility with `ros2 topic info <topic_name> --verbose`.

3. **Message Type Mismatch**: Ensure publisher and subscriber use the same message type. Typos like `sensor_msgs/msg/Image` vs `sensor_msgs/Image` cause silent failures.

4. **Not Calling `rclpy.init()` and `rclpy.shutdown()`**: These initialize and clean up the ROS 2 runtime. Omitting them causes obscure errors or resource leaks.

5. **Blocking in Callbacks**: Callbacks should return quickly (under 10 ms). Heavy computation blocks message processing. Use separate threads or asynchronous processing for long tasks.

## Further Resources

### Official Documentation
- **ROS 2 Documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/) - Comprehensive ROS 2 guides and tutorials
- **DDS Specification**: [https://www.omg.org/spec/DDS/](https://www.omg.org/spec/DDS/) - Underlying middleware standard
- **ROS 2 Design**: [https://design.ros2.org](https://design.ros2.org) - Architectural decisions and rationale

### Tutorials
- ROS 2 Tutorials (Beginner): [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)
- ROS 2 Migration Guide (from ROS 1): [https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html](https://docs.ros.org/en/humble/How-To-Guides/Migrating-from-ROS1.html)

### Books
- *A Concise Introduction to Robot Programming with ROS 2* by Francisco Rico (2023)
- *Programming Robots with ROS 2* by Morgan Quigley et al. (2024 update)

### Video Resources
- The Construct - ROS 2 Basics Course (YouTube)
- Articulated Robotics - ROS 2 Tutorial Series

### Community
- ROS Discourse: [https://discourse.ros.org](https://discourse.ros.org) - Official ROS community forum
- ROS 2 GitHub: [https://github.com/ros2](https://github.com/ros2) - Source code and issue tracking
- ROS Answers: [https://answers.ros.org](https://answers.ros.org) - Q&A for ROS users

---

**Next**: Continue to [Nodes and Topics](./nodes-topics) to learn publish-subscribe communication, or test your knowledge with [Self-Assessment](./self-assessment).
