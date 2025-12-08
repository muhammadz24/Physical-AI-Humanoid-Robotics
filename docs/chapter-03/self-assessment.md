---
sidebar_position: 5
title: Self-Assessment
---

# Chapter 3: Self-Assessment

Test your understanding of ROS 2 fundamentals, nodes, topics, services, and actions. Try to answer each question before revealing the answer.

---

## Question 1: ROS 2 vs ROS 1 Improvements

**What are three major improvements in ROS 2 over ROS 1 that make it suitable for production robotics systems?**

<details>
<summary>Click to reveal answer</summary>

Three major improvements:

1. **Distributed Discovery (No Single Master)**:
   - ROS 1 required a central rosmaster node for discovery and connection management
   - If rosmaster failed, the entire robot system collapsed (single point of failure)
   - ROS 2 uses DDS for distributed discovery—nodes discover each other peer-to-peer
   - Eliminates single point of failure, making systems more robust

2. **Real-Time Support**:
   - ROS 1 used TCP-based communication with unpredictable latency
   - ROS 2 supports real-time communication with deterministic timing guarantees
   - Critical for control loops requiring millisecond-level responsiveness
   - Enables deployment on real-time operating systems (RTOS)

3. **Built-in Security**:
   - ROS 1 had no security—any process could publish/subscribe to any topic
   - ROS 2 integrates DDS-Security for authentication, encryption, and access control
   - Essential for commercial deployments and safety-critical applications
   - Prevents unauthorized access and man-in-the-middle attacks

**Other Valid Answers**:
- Multi-platform support (Linux, Windows, macOS, embedded RTOS)
- Better multi-robot system support
- Improved build system (colcon vs catkin)
- Quality of Service (QoS) policies for fine-grained communication control

</details>

---

## Question 2: Quality of Service (QoS) Policies

**When would you use `BEST_EFFORT` reliability versus `RELIABLE` reliability? Provide an example for each.**

<details>
<summary>Click to reveal answer</summary>

**BEST_EFFORT Reliability**:
- **Use Case**: High-frequency sensor data where only the latest value matters
- **Behavior**: Messages may be lost if network is congested (like UDP)
- **Advantage**: Lower latency, less CPU/network overhead
- **Example**: Camera images at 30 Hz
  - If one frame is lost, the next frame (33ms later) provides up-to-date information
  - Processing an old frame is useless, so guaranteed delivery isn't necessary
  - Configuration: `reliability=ReliabilityPolicy.BEST_EFFORT`

**RELIABLE Reliability**:
- **Use Case**: Commands, configuration data, or any message that must not be lost
- **Behavior**: Every message is delivered, retransmitted if lost (like TCP)
- **Advantage**: Guaranteed delivery and ordering
- **Example**: Robot velocity commands (`/cmd_vel`)
  - If a "stop" command is lost, the robot continues moving (dangerous!)
  - Commands must be delivered reliably to ensure safe operation
  - Configuration: `reliability=ReliabilityPolicy.RELIABLE`

**QoS Configuration Example**:
```python
# Sensor data (best effort)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# Commands (reliable)
command_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)
```

</details>

---

## Question 3: Node Design Principles

**Why is it better to create multiple small, focused nodes rather than one large, monolithic node? Provide three benefits.**

<details>
<summary>Click to reveal answer</summary>

Three benefits of small, focused nodes:

1. **Modularity and Reusability**:
   - A camera driver node that only publishes images can work with any image processing algorithm
   - You can swap object detection algorithms (YOLO → Faster R-CNN) without touching the camera driver
   - Nodes can be reused across different robot projects
   - **Example**: Camera driver node works with TurtleBot, robotic arm, or humanoid robot

2. **Parallel Development and Testing**:
   - Different teams can work on perception, planning, and control simultaneously
   - Each node can be tested independently with mock inputs/outputs
   - Easier to isolate bugs to specific nodes rather than searching through monolithic code
   - **Example**: Test object detector with recorded images, no need for physical camera

3. **Performance and Fault Isolation**:
   - Nodes run as separate processes, can execute on different CPU cores
   - If one node crashes (e.g., experimental perception algorithm), other nodes continue
   - Can restart failed nodes without restarting the entire system
   - Resource limits can be applied per-node (memory, CPU)
   - **Example**: If object detector crashes, camera driver and motor control keep running

**Additional Benefits**:
- Clearer code organization (single responsibility principle)
- Easier to add/remove features by starting/stopping nodes
- Better for distributed systems (run nodes on different machines)

</details>

---

## Question 4: Topic vs Service vs Action

**Match each scenario to the correct communication pattern (Topic, Service, or Action):**

A. Streaming camera images at 30 Hz
B. Requesting current battery level
C. Navigating robot to a goal position with progress updates
D. Publishing robot odometry continuously
E. Closing a gripper (completes in 200ms)
F. Executing a 30-second pick-and-place manipulation sequence

<details>
<summary>Click to reveal answer</summary>

**Correct Matches**:

**A. Streaming camera images at 30 Hz** → **Topic**
- Continuous data stream
- Multiple nodes may need images (perception, recording, display)
- Asynchronous, one-to-many communication

**B. Requesting current battery level** → **Service**
- Query-response pattern
- Completes quickly (under 1 second)
- Synchronous request, single response
- Example: `std_srvs/srv/Trigger` or custom `GetBatteryLevel` service

**C. Navigating robot to a goal position with progress updates** → **Action**
- Long-running task (seconds to minutes)
- Requires feedback (distance remaining, obstacles encountered)
- Needs cancellation capability (new goal, emergency stop)
- Example: `nav2_msgs/action/NavigateToPose`

**D. Publishing robot odometry continuously** → **Topic**
- Continuous state updates
- Multiple nodes need odometry (planners, localization, visualizers)
- Asynchronous, high-frequency stream
- Example: `nav_msgs/msg/Odometry` on `/odom` topic

**E. Closing a gripper (completes in 200ms)** → **Service**
- Quick operation (under 1 second)
- Simple request (close/open) and response (success/failure)
- No need for continuous feedback
- Example: Custom `SetGripper` service

**F. Executing a 30-second pick-and-place sequence** → **Action**
- Long-running, multi-step task
- Needs feedback (current step: approaching, grasping, lifting, placing)
- May fail at various stages (grasp failure, collision)
- Should be cancellable if conditions change
- Example: `moveit_msgs/action/MoveGroup`

**Decision Rule**:
- Continuous data → Topic
- Quick query/command (under 1s) → Service
- Long task with feedback → Action

</details>

---

## Question 5: Message Headers and Timestamps

**Why do sensor messages (Image, LaserScan, etc.) include a Header with timestamp and frame_id? What problems does this solve?**

<details>
<summary>Click to reveal answer</summary>

The Header provides critical metadata for sensor data:

**Header Structure**:
```python
std_msgs/Header header
  builtin_interfaces/Time stamp
    int32 sec
    uint32 nanosec
  string frame_id
```

**Purpose of Timestamp** (`stamp`):
- **Synchronization**: When fusing data from multiple sensors (camera + LIDAR), timestamps let you match measurements from the same moment in time
- **Latency Detection**: Compare timestamp to current time to detect delayed data (e.g., network congestion, slow processing)
- **Temporal Ordering**: Process data in correct sequence, especially important when replaying logged data
- **Staleness Detection**: Discard old data that's no longer relevant

**Example Problem Solved**: A robot fuses camera (30 Hz) and LIDAR (10 Hz). Without timestamps, you might match a current camera frame with a 100ms-old LIDAR scan, causing misalignment in obstacle detection.

**Purpose of Frame ID** (`frame_id`):
- **Coordinate Frame**: Identifies which coordinate system the data is in (e.g., "camera_frame", "laser_frame", "base_link")
- **Transform Lookup**: Enables automatic coordinate transformation between sensors
- **Spatial Relationships**: ROS 2 uses TF2 (transform library) to maintain coordinate frame tree

**Example Problem Solved**: Camera sees an object at (x=1.0, y=0.0) in camera_frame. The robot needs this position in base_link frame to navigate. Using frame_id="camera_frame", TF2 automatically transforms coordinates.

**Best Practice**:
Always populate headers with:
- Current timestamp: `msg.header.stamp = self.get_clock().now().to_msg()`
- Correct frame_id: `msg.header.frame_id = 'camera_frame'`

</details>

---

## Question 6: Service Callback Implementation

**In the provided `AddTwoInts` service example, what would happen if the callback didn't return the `response` object? How would the client behave?**

<details>
<summary>Click to reveal answer</summary>

**If Callback Doesn't Return Response**:

```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Computed: {response.sum}')
    # MISSING: return response
```

**Client Behavior**:
- Client calls `client.call_async(request)` and waits for response
- Server processes request but doesn't send response back
- Client times out after default timeout period (implementation-dependent)
- Client receives no response or timeout exception
- Client logs error: "Service call failed" or "Service call timed out"

**Correct Implementation**:
```python
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Computed: {response.sum}')
    return response  # MUST return response object
```

**Why This Matters**:
- Service callbacks are synchronous—client blocks waiting for response
- Forgetting `return response` is a common beginner mistake
- No error message from server (callback executes successfully)
- Only the client experiences timeout, making debugging difficult

**Debugging Tip**: If service clients are timing out but server logs show successful processing, check that callbacks return the response object.

</details>

---

## Question 7: Action Feedback and Cancellation

**In the `FibonacciActionServer` example, what happens when a goal is canceled? Trace the execution flow and explain what the server does.**

<details>
<summary>Click to reveal answer</summary>

**Cancellation Execution Flow**:

1. **Client Sends Cancel Request**:
   ```python
   cancel_future = goal_handle.cancel_goal_async()
   ```
   - Client calls `cancel_goal_async()` on the goal handle
   - ROS 2 sends cancellation request to action server

2. **Server Checks for Cancellation** (in execute loop):
   ```python
   if goal_handle.is_cancel_requested:
       goal_handle.canceled()
       self.get_logger().info('Goal canceled')
       return Fibonacci.Result()
   ```
   - `is_cancel_requested` becomes `True`
   - Server detects cancellation in next iteration of computation loop

3. **Server Marks Goal as Canceled**:
   ```python
   goal_handle.canceled()
   ```
   - Notifies ROS 2 that the goal was successfully canceled
   - Sets goal status to `STATUS_CANCELED`

4. **Server Returns Empty or Partial Result**:
   ```python
   return Fibonacci.Result()
   ```
   - Returns empty result (could return partial sequence if desired)
   - Control returns to ROS 2, ending the action execution callback

5. **Client Receives Cancellation Confirmation**:
   ```python
   def cancel_callback(self, future):
       cancel_response = future.result()
       if len(cancel_response.goals_canceling) > 0:
           self.get_logger().info('Goal successfully canceled')
   ```
   - Client's cancel callback is invoked
   - Confirms goal was canceled

**Why This Matters**:
- Actions must check `is_cancel_requested` periodically (every iteration/step)
- Failing to check means goal can't be canceled (unresponsive server)
- Server should clean up resources before returning (e.g., stop motors, release locks)

**Best Practice**:
```python
def execute_callback(self, goal_handle):
    for step in range(num_steps):
        # Check cancellation at each step
        if goal_handle.is_cancel_requested:
            self.cleanup_resources()  # Stop motors, etc.
            goal_handle.canceled()
            return PartialResult()  # Return what was accomplished

        # Do work
        do_computation()
        goal_handle.publish_feedback(feedback)
```

</details>

---

## Question 8: Command-Line Introspection

**You're debugging a robot system and the object detector node isn't receiving camera images. What command-line tools and commands would you use to diagnose the problem?**

<details>
<summary>Click to reveal answer</summary>

**Systematic Debugging Steps**:

**Step 1: Verify Nodes Are Running**
```bash
ros2 node list
```
- Check if both `/camera_driver` and `/object_detector` nodes appear
- If camera_driver is missing, it hasn't started

**Step 2: Check Topic Existence**
```bash
ros2 topic list
```
- Verify `/camera/image_raw` topic exists
- If missing, camera driver isn't publishing

**Step 3: Inspect Topic Publishers and Subscribers**
```bash
ros2 topic info /camera/image_raw
```
**Output**:
```
Type: sensor_msgs/msg/Image
Publisher count: 1
Subscription count: 0  ← PROBLEM: No subscribers!
```
- Confirms camera is publishing, but object detector isn't subscribing
- **Diagnosis**: Object detector topic name mismatch or not started

**Step 4: Verify Message Type**
```bash
ros2 topic info /camera/image_raw --verbose
```
**Output shows QoS**:
```
QoS profile:
  Reliability: BEST_EFFORT
  Durability: VOLATILE
  ...
```
- Check if publisher and subscriber QoS are compatible
- **Common Issue**: Publisher uses `BEST_EFFORT`, subscriber requires `RELIABLE` (incompatible!)

**Step 5: Check If Data Is Actually Being Published**
```bash
ros2 topic hz /camera/image_raw
```
**Output**:
```
average rate: 29.987
  min: 0.033s max: 0.034s std dev: 0.00012s window: 30
```
- Confirms camera is publishing at ~30 Hz
- If no output, camera driver is running but not publishing

**Step 6: Inspect Actual Message Content**
```bash
ros2 topic echo /camera/image_raw --once
```
- Displays one message to verify format
- Check image dimensions, encoding, etc.

**Step 7: Check Node Details**
```bash
ros2 node info /object_detector
```
**Output**:
```
/object_detector
  Subscribers:
    /camera/image_wrong_name: sensor_msgs/msg/Image  ← PROBLEM: Wrong topic name!
```
- Shows object detector is subscribing to wrong topic
- **Diagnosis**: Topic name typo in code or launch file

**Common Problems Diagnosed**:
1. **No publisher**: Camera driver not started
2. **No subscriber**: Object detector not started or wrong topic name
3. **QoS mismatch**: Incompatible reliability/durability settings
4. **Topic name typo**: `/camera/image_raw` vs `/camera/image_raw_`
5. **Message type mismatch**: Publisher sends Image, subscriber expects CompressedImage

**Fix**: Correct topic name or QoS in code, restart nodes, re-check with `ros2 topic info`.

</details>

---

## Question 9: Action Feedback Frequency

**Why is it important to limit the frequency of action feedback messages? What could go wrong if feedback is published too frequently?**

<details>
<summary>Click to reveal answer</summary>

**Importance of Limiting Feedback Frequency**:

**Problems with Too Frequent Feedback**:

1. **Network Congestion**:
   - Feedback messages consume bandwidth
   - Publishing at 1 kHz (every 1ms) creates thousands of messages per second
   - Can saturate network, delaying critical messages (e.g., emergency stop commands)
   - **Example**: Navigation action publishing position updates at 100 Hz floods network

2. **Client Overwhelmed**:
   - Client callbacks process each feedback message
   - Too many callbacks slow down client processing
   - Clients may fall behind, creating backlog
   - **Example**: GUI displaying feedback at 50 Hz freezes trying to update 1000 times/second

3. **Unnecessary Computation**:
   - Publishing feedback requires CPU (serialization, network stack)
   - Checking for cancellation every microsecond wastes cycles
   - Server spends more time publishing feedback than doing actual work
   - **Example**: Fibonacci server checks cancellation every iteration—wasteful for fast loops

**Best Practices**:

**Rate-Limit Feedback** (example: navigation action):
```python
def execute_callback(self, goal_handle):
    last_feedback_time = time.time()
    feedback_rate = 2.0  # 2 Hz (every 0.5 seconds)

    while not_at_goal:
        # Do work (may run at 100 Hz)
        navigate_step()

        # Publish feedback only at limited rate
        if time.time() - last_feedback_time >= (1.0 / feedback_rate):
            goal_handle.publish_feedback(feedback_msg)
            last_feedback_time = time.time()

        # Check cancellation less frequently
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Result()
```

**Recommended Feedback Rates**:
- **Short actions** (under 5 seconds): 5-10 Hz
- **Medium actions** (5-60 seconds): 1-5 Hz
- **Long actions** (>60 seconds): 0.5-1 Hz

**Exception**: If feedback contains critical safety information (e.g., collision detection), higher rates may be justified, but use topics for real-time safety data instead.

</details>

---

## Question 10: Building a Complete System

**Design a ROS 2 system for a mobile robot that navigates to a goal while avoiding obstacles. Specify:**
- **Nodes** (name and purpose)
- **Topics** (name, message type, QoS)
- **Services** (name, service type)
- **Actions** (name, action type)

<details>
<summary>Click to reveal answer</summary>

**System Architecture**:

**Nodes**:

1. **lidar_driver**
   - Publishes: `/scan` (sensor_msgs/LaserScan, BEST_EFFORT)
   - Interfaces with LIDAR hardware, publishes distance measurements at 10 Hz

2. **camera_driver**
   - Publishes: `/camera/image_raw` (sensor_msgs/Image, BEST_EFFORT)
   - Publishes: `/camera/camera_info` (sensor_msgs/CameraInfo, RELIABLE)
   - Interfaces with camera, publishes images at 30 Hz

3. **obstacle_detector**
   - Subscribes: `/scan` (sensor_msgs/LaserScan, BEST_EFFORT)
   - Publishes: `/obstacles` (sensor_msgs/PointCloud2, RELIABLE)
   - Processes LIDAR data to identify obstacles as point cloud

4. **path_planner**
   - Action Server: `/navigate_to_goal` (nav2_msgs/action/NavigateToPose)
   - Subscribes: `/obstacles` (sensor_msgs/PointCloud2, RELIABLE)
   - Subscribes: `/odom` (nav_msgs/Odometry, RELIABLE)
   - Publishes: `/cmd_vel` (geometry_msgs/Twist, RELIABLE)
   - Computes collision-free path and sends velocity commands

5. **motor_controller**
   - Subscribes: `/cmd_vel` (geometry_msgs/Twist, RELIABLE)
   - Publishes: `/odom` (nav_msgs/Odometry, RELIABLE)
   - Service: `/emergency_stop` (std_srvs/Trigger)
   - Controls motors, publishes odometry, provides emergency stop

6. **battery_monitor**
   - Publishes: `/battery_status` (sensor_msgs/BatteryState, BEST_EFFORT, 1 Hz)
   - Service: `/get_battery_level` (custom GetBatteryLevel.srv)
   - Monitors battery, publishes status, responds to queries

**Communication Patterns**:

**Topics (Continuous Data)**:
| Topic | Type | QoS | Purpose |
|-------|------|-----|---------|
| `/scan` | sensor_msgs/LaserScan | BEST_EFFORT, KEEP_LAST(1) | LIDAR measurements |
| `/camera/image_raw` | sensor_msgs/Image | BEST_EFFORT, KEEP_LAST(1) | Camera images |
| `/obstacles` | sensor_msgs/PointCloud2 | RELIABLE, KEEP_LAST(5) | Detected obstacles |
| `/odom` | nav_msgs/Odometry | RELIABLE, KEEP_LAST(10) | Robot position |
| `/cmd_vel` | geometry_msgs/Twist | RELIABLE, KEEP_LAST(10) | Velocity commands |
| `/battery_status` | sensor_msgs/BatteryState | BEST_EFFORT, KEEP_LAST(1) | Battery state |

**Services (Quick Queries/Commands)**:
| Service | Type | Purpose |
|---------|------|---------|
| `/emergency_stop` | std_srvs/Trigger | Immediately stop robot |
| `/get_battery_level` | custom GetBatteryLevel.srv | Query current battery % |

**Actions (Long-Running Tasks)**:
| Action | Type | Purpose |
|--------|------|---------|
| `/navigate_to_goal` | nav2_msgs/action/NavigateToPose | Navigate to (x, y, theta) with feedback |

**Execution Flow**:
1. User sends navigation goal via `/navigate_to_goal` action
2. `path_planner` receives goal, subscribes to `/obstacles` and `/odom`
3. `path_planner` computes path avoiding obstacles
4. `path_planner` publishes velocity commands to `/cmd_vel`
5. `motor_controller` executes commands, publishes updated `/odom`
6. `path_planner` publishes feedback (distance remaining) to action client
7. Robot reaches goal, `path_planner` returns success result
8. If emergency occurs, call `/emergency_stop` service

**Why This Design**:
- Modular: Each node has single responsibility
- Flexible: Can swap obstacle detector (LIDAR → camera) without changing planner
- Testable: Can test planner with recorded `/scan` data, no hardware needed
- Scalable: Can add visual odometry node to improve localization

</details>

---

## Scoring Guide

- **8-10 correct**: Excellent! You have a strong understanding of ROS 2 architecture and communication patterns.
- **6-7 correct**: Good! Review sections on QoS, actions, and system design.
- **4-5 correct**: Fair. Revisit nodes/topics and services/actions chapters, work through code examples.
- **0-3 correct**: Consider re-reading the chapter and implementing the hands-on examples in a real ROS 2 workspace.

---

## Next Steps

1. **Hands-On Practice**: Set up a ROS 2 workspace and implement the publisher-subscriber, service, and action examples
2. **Experiment with TurtleBot3**: Use TurtleBot3 simulation to test navigation and control
3. **Build a Project**: Create a multi-node system (e.g., sensor fusion, autonomous navigation)
4. **Proceed to Chapter 4**: [Digital Twin Simulation](/chapter-04/) to learn about Gazebo and Isaac Sim

---

**Congratulations on completing Chapter 3!** You now understand ROS 2's architecture, communication patterns, and how to build modular robot systems. Ready to simulate robots? Continue to [Chapter 4](/chapter-04/).
