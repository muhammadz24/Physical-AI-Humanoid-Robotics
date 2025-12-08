---
sidebar_position: 3
title: Nodes and Topics
---

# Nodes and Topics

## Introduction

Nodes and topics form the backbone of ROS 2's communication architecture, enabling distributed robot systems to exchange information seamlessly. Understanding these concepts is essential for building any ROS 2 application, from simple sensor readers to complex multi-robot systems. While nodes represent independent computational processes performing specific tasks, topics provide the communication channels through which these nodes share data asynchronously—creating a flexible, scalable architecture that mirrors how modern robots are designed.

The publish-subscribe pattern implemented through nodes and topics offers significant advantages over traditional function-call architectures. Publishers and subscribers are decoupled: they don't need to know about each other's existence, timing, or location. This loose coupling enables hot-swapping components (replacing a camera driver without restarting the system), parallel development (teams work on perception and control independently), and runtime reconfiguration (adding new sensors or algorithms to a running robot). Mastering nodes and topics gives you the foundation to build sophisticated robotic systems that are modular, testable, and maintainable.

## Conceptual Foundation

### What is a Node?

A node is a process that performs a specific computation within a ROS 2 system. Nodes are the fundamental unit of modularity: each node should have a single, well-defined purpose. This single-responsibility principle makes nodes reusable, testable, and easier to understand.

**Examples of Nodes**:
- **Camera Driver Node**: Interfaces with camera hardware, publishes raw images at 30 Hz
- **Object Detector Node**: Subscribes to images, runs YOLO detection, publishes detected objects
- **Path Planner Node**: Subscribes to map and goal, computes collision-free path, publishes trajectory
- **Motor Controller Node**: Subscribes to velocity commands, sends PWM signals to motor drivers
- **Data Logger Node**: Subscribes to multiple topics, records messages to disk for later analysis

**Node Design Philosophy**: Keep nodes small and focused. Instead of one monolithic "perception" node, create separate nodes for image acquisition, preprocessing, object detection, and tracking. This modularity allows:
- **Reuse**: Camera driver works with any image processing algorithm
- **Testing**: Test object detection independently from camera hardware
- **Parallelism**: Run nodes on different CPU cores or machines
- **Debugging**: Isolate problems to specific nodes

### What is a Topic?

A topic is a named channel for transmitting messages between nodes. Topics implement the publish-subscribe pattern:
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic
- The middleware delivers each published message to all subscribers

**Topic Characteristics**:
- **One-to-Many**: One publisher, multiple subscribers (common for sensor data)
- **Many-to-One**: Multiple publishers, one subscriber (common for distributed sensing)
- **Many-to-Many**: Multiple publishers and subscribers (message fusion, distributed control)
- **Asynchronous**: Publishers don't wait for subscribers; messages are queued and delivered independently
- **Typed**: Topics carry messages of a specific type (e.g., `sensor_msgs/Image`); type mismatches are rejected

**Topic Naming Convention**: Topics use hierarchical names with forward slashes:
- `/camera/image_raw` - Raw images from camera
- `/camera/image_processed` - Processed images
- `/robot_1/cmd_vel` - Velocity commands for robot 1
- `/diagnostics` - System diagnostic messages

This hierarchy organizes topics logically, especially in multi-robot or multi-sensor systems.

### The Publish-Subscribe Pattern

Traditional function calls create tight coupling: the caller must know the callee's address, signature, and timing. Publish-subscribe decouples producers and consumers:

**Decoupling Dimensions**:
1. **Space**: Publisher doesn't know who (if anyone) is subscribed; subscriber doesn't know where data originates
2. **Time**: Publisher and subscriber don't need to exist simultaneously; messages can be buffered
3. **Synchronization**: Publishing is non-blocking; subscribers process messages independently

**Real-World Analogy**: A newspaper publisher prints articles without knowing who subscribes. Subscribers receive papers without knowing the printing press location. New subscribers can join anytime; old subscribers can unsubscribe. The distribution system (middleware) handles delivery.

This pattern is ideal for robot systems:
- **Sensor Streams**: Cameras publish images; any number of algorithms can subscribe
- **State Updates**: Localization publishes robot position; planners, displays, loggers all subscribe
- **Commands**: High-level planners publish goals; low-level controllers execute them

### Message Types in ROS 2

Messages define the data structure for topic communication. ROS 2 provides:

**Standard Messages** (`std_msgs`):
- `Bool`, `Int32`, `Float64`, `String` - Basic types
- `Header` - Timestamp and coordinate frame (used in other messages)

**Sensor Messages** (`sensor_msgs`):
- `Image` - Camera images with encoding and dimensions
- `LaserScan` - 2D LIDAR scans with ranges and angles
- `PointCloud2` - 3D point clouds from depth sensors
- `Imu` - Inertial measurement unit (acceleration, angular velocity)
- `JointState` - Robot joint positions, velocities, efforts

**Geometry Messages** (`geometry_msgs`):
- `Point`, `Vector3`, `Quaternion` - Basic geometric primitives
- `Pose` - Position and orientation (Point + Quaternion)
- `Twist` - Linear and angular velocity
- `PoseStamped`, `TwistStamped` - Stamped versions with Header

**Navigation Messages** (`nav_msgs`):
- `Odometry` - Robot position, orientation, and velocities
- `Path` - Sequence of poses forming a trajectory
- `OccupancyGrid` - 2D map with obstacle information

**Custom Messages**: You can define application-specific messages in `.msg` files, combining standard types and other messages.

## Technical Details

### Creating and Managing Nodes

A basic ROS 2 node in Python inherits from `rclpy.node.Node`:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')  # Node name
        self.get_logger().info('Minimal node has started')

def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python client library
    node = MinimalNode()
    rclpy.spin(node)  # Keep node alive, processing callbacks
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key Components**:
- `rclpy.init()`: Initializes ROS 2 context (must be called once)
- `Node('minimal_node')`: Creates node with specified name
- `self.get_logger()`: Access node's logger for info, warn, error messages
- `rclpy.spin(node)`: Blocks and processes callbacks (timers, subscriptions)
- `node.destroy_node()`: Clean up resources
- `rclpy.shutdown()`: Shutdown ROS 2 context

### Publishers: Sending Data

Publishers write messages to topics:

```python
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Create publisher: message type, topic name, queue size
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create timer: callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2: {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.counter += 1
```

**Publisher Parameters**:
- **Message Type**: `String` (from `std_msgs.msg`)
- **Topic Name**: `'chatter'` (subscribers must use same name)
- **Queue Size**: `10` (buffer 10 messages if network is slow)

**Timer Callback**: `create_timer(period, callback)` calls `timer_callback` every `period` seconds, enabling periodic publishing.

### Subscribers: Receiving Data

Subscribers read messages from topics:

```python
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscription: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')
```

**Subscription Parameters**:
- **Message Type**: Must match publisher's type
- **Topic Name**: Must match publisher's topic
- **Callback**: Function called when message arrives
- **Queue Size**: Buffer size for incoming messages

**Callback Execution**: When a message arrives, ROS 2 calls `listener_callback(msg)` with the message object. Process data quickly; long computations should be offloaded to separate threads.

### Complete Publisher-Subscriber Example

Let's build a realistic example: a simulated laser scanner publishing distance measurements, and a safety monitor node that triggers warnings if obstacles are too close.

**Publisher (Simulated Laser Scanner)**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import random

class LaserScannerSimulator(Node):
    def __init__(self):
        super().__init__('laser_scanner_simulator')

        self.publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)  # 10 Hz

        self.get_logger().info('Laser scanner simulator started')

    def publish_scan(self):
        msg = LaserScan()

        # Header with timestamp and frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_frame'

        # Laser scanner specifications
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2   # +90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree resolution
        msg.range_min = 0.1  # Minimum valid range (meters)
        msg.range_max = 10.0  # Maximum valid range (meters)

        # Simulate distance measurements (180 points, -90 to +90 degrees)
        num_readings = int((msg.angle_max - msg.angle_min) / msg.angle_increment)
        msg.ranges = []

        for i in range(num_readings):
            # Simulate distances: 2-8 meters with some random variation
            # Occasionally place close obstacles (< 0.5m) to trigger warnings
            if random.random() < 0.05:  # 5% chance of close obstacle
                distance = random.uniform(0.2, 0.4)
            else:
                distance = random.uniform(2.0, 8.0)

            msg.ranges.append(distance)

        self.publisher.publish(msg)
        self.get_logger().debug(f'Published {len(msg.ranges)} laser scan points')

def main(args=None):
    rclpy.init(args=args)
    node = LaserScannerSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber (Safety Monitor)**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.danger_threshold = 0.5  # meters
        self.warning_threshold = 1.0  # meters

        self.get_logger().info('Safety monitor started')

    def scan_callback(self, msg):
        # Find minimum distance in scan
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        if not valid_ranges:
            self.get_logger().warn('No valid scan data')
            return

        min_distance = min(valid_ranges)

        # Check thresholds and log warnings
        if min_distance < self.danger_threshold:
            self.get_logger().error(f'DANGER! Obstacle at {min_distance:.2f}m - STOP IMMEDIATELY')
        elif min_distance < self.warning_threshold:
            self.get_logger().warn(f'WARNING! Obstacle at {min_distance:.2f}m - Reduce speed')
        else:
            self.get_logger().debug(f'Clear path, closest obstacle: {min_distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the Example**:

Terminal 1 (Publisher):
```bash
python3 laser_scanner_simulator.py
```

Terminal 2 (Subscriber):
```bash
python3 safety_monitor.py
```

**Expected Output** (Safety Monitor):
```
[INFO] [safety_monitor]: Safety monitor started
[DEBUG] [safety_monitor]: Clear path, closest obstacle: 4.32m
[DEBUG] [safety_monitor]: Clear path, closest obstacle: 5.67m
[WARN] [safety_monitor]: WARNING! Obstacle at 0.87m - Reduce speed
[DEBUG] [safety_monitor]: Clear path, closest obstacle: 3.21m
[ERROR] [safety_monitor]: DANGER! Obstacle at 0.34m - STOP IMMEDIATELY
```

**What This Demonstrates**:
- Realistic use of `sensor_msgs/LaserScan` message
- Timestamp and frame_id in message headers
- Subscriber processing sensor data with safety logic
- Logging at different severity levels

### Topic Introspection and Debugging

ROS 2 command-line tools help debug pub-sub communication:

```bash
# List all active topics
ros2 topic list

# Show details about a topic (publishers, subscribers, message type)
ros2 topic info /scan

# Display message type definition
ros2 interface show sensor_msgs/msg/LaserScan

# Echo messages in real-time
ros2 topic echo /scan

# Measure publishing frequency
ros2 topic hz /scan

# Publish a message manually (for testing subscribers)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}"
```

**Example Output** (`ros2 topic info /scan`):
```
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 2
```

This tells you one node is publishing, two nodes are subscribed—useful for diagnosing missing connections.

## Hands-On Examples

### Example 1: Building a Robot Velocity Publisher

Control a robot's motion by publishing velocity commands:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')

        # Publish to /cmd_vel topic (standard for robot velocity commands)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz

        # Circle parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 0.5  # rad/s (produces 1m radius circle)

        self.get_logger().info('Circle controller started - Robot will move in circles')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = self.linear_speed  # Forward velocity
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_speed  # Rotational velocity

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on Ctrl+C
        stop_msg = Twist()  # All zeros
        node.publisher.publish(stop_msg)
        node.get_logger().info('Stopping robot')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What This Does**: Publishes constant linear and angular velocities, causing a robot to drive in circles. The `Twist` message is the standard for velocity commands in ROS 2.

**To Test with TurtleBot3 Simulation**:
```bash
# Terminal 1: Launch TurtleBot3 simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py

# Terminal 2: Run circle controller
python3 circle_controller.py
```

### Example 2: Multi-Subscriber Node (Sensor Fusion)

Subscribe to multiple topics to fuse sensor data:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)

        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Storage for latest sensor data
        self.latest_image = None
        self.latest_imu = None
        self.latest_odom = None

        self.get_logger().info('Sensor fusion node started')

    def camera_callback(self, msg):
        self.latest_image = msg
        self.get_logger().debug(f'Camera: {msg.width}x{msg.height}')
        self.fuse_data()

    def imu_callback(self, msg):
        self.latest_imu = msg
        accel = msg.linear_acceleration
        self.get_logger().debug(f'IMU: accel=({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f})')
        self.fuse_data()

    def odom_callback(self, msg):
        self.latest_odom = msg
        pos = msg.pose.pose.position
        self.get_logger().debug(f'Odom: pos=({pos.x:.2f}, {pos.y:.2f})')
        self.fuse_data()

    def fuse_data(self):
        # Only fuse when all sensors have reported at least once
        if self.latest_image and self.latest_imu and self.latest_odom:
            self.get_logger().info('All sensors available - performing fusion')
            # Here you would implement actual sensor fusion logic
            # (e.g., Extended Kalman Filter combining odometry and IMU)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**What This Demonstrates**:
- Subscribing to multiple topics with different message types
- Storing latest data from each sensor
- Triggering fusion logic when all data is available

### Example 3: Dynamic Topic Remapping

Run nodes with remapped topic names for flexibility:

```bash
# Remap /cmd_vel to /robot1/cmd_vel (useful for multi-robot systems)
ros2 run my_package circle_controller --ros-args -r /cmd_vel:=/robot1/cmd_vel

# Remap multiple topics
ros2 run my_package sensor_fusion \
    --ros-args \
    -r /camera/image_raw:=/robot1/camera/image_raw \
    -r /imu/data:=/robot1/imu/data \
    -r /odom:=/robot1/odom
```

This allows running the same node code with different robot namespaces without code changes.

### Common Pitfalls

1. **Forgetting to Spin**: If you don't call `rclpy.spin(node)`, callbacks never execute. Subscribers appear to do nothing.

2. **Callback Blocking**: Long-running computations in callbacks block message processing. Use timers or separate threads for heavy tasks:
   ```python
   def camera_callback(self, msg):
       # BAD: Heavy processing blocks other callbacks
       result = heavy_processing(msg)  # Takes 500ms

       # GOOD: Queue for processing elsewhere
       self.image_queue.put(msg)
   ```

3. **Message Mutation**: Don't modify received messages. Make a copy if you need to change data:
   ```python
   def callback(self, msg):
       # BAD: Modifies original message
       msg.data = "changed"

       # GOOD: Create new message
       new_msg = String()
       new_msg.data = "changed"
   ```

4. **Ignoring Timestamps**: Always check message timestamps, especially when fusing sensors. Old data can cause incorrect decisions.

5. **Not Handling Exceptions**: Exceptions in callbacks crash the node. Wrap risky code in try-except:
   ```python
   def callback(self, msg):
       try:
           result = process(msg)
       except Exception as e:
           self.get_logger().error(f'Processing failed: {e}')
   ```

## Further Resources

### Official Documentation
- **ROS 2 Python Node API**: [https://docs.ros2.org/latest/api/rclpy/](https://docs.ros2.org/latest/api/rclpy/)
- **Standard Messages**: [https://github.com/ros2/common_interfaces](https://github.com/ros2/common_interfaces)
- **Understanding Topics**: [https://docs.ros.org/en/humble/Tutorials/Topics.html](https://docs.ros.org/en/humble/Tutorials/Topics.html)

### Tutorials
- ROS 2 Publisher-Subscriber Tutorial: [https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- Custom Message Definition: [https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html)

### Tools
- **rqt_graph**: Visualize node-topic graph
- **PlotJuggler**: Plot topic data in real-time ([https://github.com/facontidavide/PlotJuggler](https://github.com/facontidavide/PlotJuggler))
- **Foxglove Studio**: Advanced visualization and debugging ([https://foxglove.dev](https://foxglove.dev))

---

**Next**: Continue to [Services and Actions](./services-actions) for request-response communication patterns, or test your understanding with [Self-Assessment](./self-assessment).
