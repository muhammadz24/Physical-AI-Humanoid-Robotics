---
sidebar_position: 4
title: Services and Actions
---

# Services and Actions

## Introduction

While topics excel at streaming continuous data, many robot tasks require request-response interactions: "Close the gripper and tell me if you succeeded," "What's the battery level?", or "Navigate to this goal and update me on progress." Services and actions provide these synchronous and goal-oriented communication patterns, complementing the asynchronous publish-subscribe model of topics. Together, these three communication primitives—topics, services, and actions—give you a complete toolkit for building sophisticated robot behaviors.

Services implement simple request-response patterns suitable for quick operations that complete in milliseconds to seconds. Actions extend this with support for long-running tasks (seconds to minutes), providing continuous feedback, partial results, and the ability to cancel ongoing operations. Understanding when to use each pattern is crucial: topics for continuous data streams, services for queries and quick commands, actions for goal-oriented behaviors. Mastering services and actions enables you to build robust, responsive robot systems that handle both immediate requests and complex, multi-step tasks.

## Conceptual Foundation

### What are Services?

Services implement synchronous request-response communication, similar to function calls but across process boundaries. A service consists of:

**Service Server**: A node that provides a service, waiting for requests and sending responses.

**Service Client**: A node that calls a service, sending a request and blocking until it receives a response.

**Service Type**: Defines the request and response message structures (e.g., `AddTwoInts` has `int64 a, int64 b` request and `int64 sum` response).

**Key Characteristics**:
- **Synchronous**: Client blocks waiting for response (though async clients exist)
- **One-to-One**: Each request gets exactly one response from one server
- **Reliable**: Uses reliable transport; requests and responses are guaranteed to arrive
- **Transient**: No message history; service calls aren't recorded (unlike topics)

**Use Cases for Services**:
- Querying robot state: battery level, joint positions, current mode
- Triggering actions: activate gripper, reset odometry, switch controllers
- Configuration: set parameters, load/save maps, calibrate sensors
- Computation requests: inverse kinematics, path validation, collision checking

**Example**: A gripper service receives a request with target position (open/closed) and responds with success status. The client waits for confirmation before proceeding.

### What are Actions?

Actions extend services for long-running, goal-oriented tasks. An action provides:

**Goal**: What you want to achieve (e.g., navigate to coordinates (x, y))

**Feedback**: Periodic updates during execution (e.g., current distance to goal)

**Result**: Final outcome when task completes (e.g., success/failure, final position)

**Cancellation**: Ability to abort an ongoing action

**Key Characteristics**:
- **Asynchronous with Feedback**: Client doesn't block; receives periodic updates
- **Long-Running**: Suitable for tasks taking seconds to minutes (navigation, manipulation)
- **Cancellable**: Client can abort task if conditions change
- **Preemptable**: New goals can override current goals

**Use Cases for Actions**:
- Navigation: drive to a goal pose, avoid obstacles, report progress
- Manipulation: pick an object, place it, report grasp status
- Trajectory Execution: follow a planned path, report waypoint completion
- Complex Behaviors: multi-step tasks like "inspect area" or "deliver package"

**Example**: A "NavigateToGoal" action takes a target (x, y, theta), sends feedback (distance remaining, time elapsed), and returns a result (success, failure reason, final pose).

### Service vs Action vs Topic Decision Tree

Choose the right communication pattern:

```
Is it continuous data (sensor streams, state updates)?
  YES → Use TOPIC
  NO ↓

Does it complete in under 1 second with simple yes/no result?
  YES → Use SERVICE
  NO ↓

Does it require feedback during execution or take >1 second?
  YES → Use ACTION
  NO → Reconsider: might still be SERVICE with async client
```

**Examples**:
- **Camera images** → Topic (continuous stream)
- **Get battery percentage** → Service (quick query, single response)
- **Close gripper** → Service (completes quickly)
- **Navigate to goal** → Action (long duration, need progress feedback)
- **Pick and place object** → Action (multi-step, need status updates, might fail)

### Service and Action Types

Like topics, services and actions use strongly-typed message definitions.

**Standard Service Types**:
- `std_srvs/SetBool` - Enable/disable a feature (request: bool, response: success, message)
- `std_srvs/Trigger` - Trigger an action with no parameters (request: empty, response: success, message)
- `nav_msgs/GetMap` - Request current map (response: OccupancyGrid)
- `sensor_msgs/SetCameraInfo` - Set camera calibration

**Standard Action Types**:
- `nav2_msgs/NavigateToPose` - Navigate to a goal pose with obstacle avoidance
- `control_msgs/FollowJointTrajectory` - Execute a planned joint trajectory
- `moveit_msgs/MoveGroup` - Plan and execute arm motion to target pose

**Custom Types**: Define `.srv` files (services) or `.action` files (actions) for application-specific needs.

## Technical Details

### Creating and Using Services

**Service Definition** (`.srv` file):

`example_interfaces/srv/AddTwoInts.srv`:
```
int64 a
int64 b
---
int64 sum
```

The `---` separator divides request (above) and response (below).

**Service Server in Python**:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service: service type, service name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('AddTwoInts service ready')

    def add_two_ints_callback(self, request, response):
        # Process request
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')

        return response  # Must return response object

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client in Python**:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client: service type, service name
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (blocks until response)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {a} + {b} = {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None

def main(args=None):
    rclpy.init(args=args)
    client = AddTwoIntsClient()
    result = client.send_request(5, 7)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the Example**:

Terminal 1 (Server):
```bash
python3 add_two_ints_server.py
```

Terminal 2 (Client):
```bash
python3 add_two_ints_client.py
```

**Expected Output** (Client):
```
[INFO] [add_two_ints_client]: Result: 5 + 7 = 12
```

### Creating and Using Actions

**Action Definition** (`.action` file):

`example_interfaces/action/Fibonacci.action`:
```
# Goal: compute Fibonacci sequence up to order n
int32 order
---
# Result: the computed sequence
int32[] sequence
---
# Feedback: intermediate sequences as they're computed
int32[] partial_sequence
```

**Action Server in Python**:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci action server ready')

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: Fibonacci({goal_handle.request.order})')

        # Initialize Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal has been cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')

            time.sleep(0.5)  # Simulate computation time

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Goal succeeded! Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client in Python**:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__('fibonacci_action_client')

        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        self.get_logger().info(f'Sending goal: Fibonacci({order})')

        # Wait for action server
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send goal with feedback callback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = FibonacciActionClient()
    client.send_goal(10)
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

**Expected Output** (Client receives feedback during execution):
```
[INFO] [fibonacci_action_client]: Sending goal: Fibonacci(10)
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2, 3]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

### Service and Action Introspection

Command-line tools for debugging:

```bash
# List all services
ros2 service list

# Show service type
ros2 service type /add_two_ints

# Call service from command line
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 5}"

# List all actions
ros2 action list

# Show action type
ros2 action info /fibonacci

# Send action goal from command line
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" --feedback
```

## Hands-On Examples

### Example 1: Robot Gripper Service

Create a service to control a robot gripper:

**Service Definition** (`robot_interfaces/srv/SetGripper.srv`):
```
bool close  # true = close, false = open
---
bool success
string message
float32 final_position  # Final gripper position (0.0 = closed, 1.0 = open)
```

**Gripper Service Server**:

```python
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SetGripper
import random

class GripperServer(Node):
    def __init__(self):
        super().__init__('gripper_server')

        self.gripper_service = self.create_service(
            SetGripper,
            '/robot/set_gripper',
            self.set_gripper_callback
        )

        self.gripper_position = 1.0  # Start open
        self.get_logger().info('Gripper service ready. Gripper is OPEN')

    def set_gripper_callback(self, request, response):
        target_state = "CLOSE" if request.close else "OPEN"
        self.get_logger().info(f'Gripper command received: {target_state}')

        # Simulate gripper motion (would control real hardware here)
        if request.close:
            self.gripper_position = 0.0
            # Simulate occasional grasp failures (5% chance)
            if random.random() < 0.05:
                response.success = False
                response.message = "Grasp failed - object slipped"
                self.gripper_position = 0.3  # Partially closed
            else:
                response.success = True
                response.message = "Gripper closed successfully"
        else:
            self.gripper_position = 1.0
            response.success = True
            response.message = "Gripper opened successfully"

        response.final_position = self.gripper_position
        self.get_logger().info(f'Gripper state: {response.message}, position: {response.final_position:.2f}')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Usage from Command Line**:
```bash
# Close gripper
ros2 service call /robot/set_gripper robot_interfaces/srv/SetGripper "{close: true}"

# Open gripper
ros2 service call /robot/set_gripper robot_interfaces/srv/SetGripper "{close: false}"
```

### Example 2: Navigation Action (Simplified)

Create an action for robot navigation with progress feedback:

**Action Definition** (`robot_interfaces/action/NavigateToGoal.action`):
```
# Goal: target position
float32 x
float32 y
float32 theta
---
# Result: final position and success status
bool success
string message
float32 final_x
float32 final_y
float32 final_theta
---
# Feedback: current position and distance remaining
float32 current_x
float32 current_y
float32 distance_remaining
float32 time_elapsed
```

**Navigation Action Server**:

```python
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from robot_interfaces.action import NavigateToGoal
import time
import math

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')

        self._action_server = ActionServer(
            self,
            NavigateToGoal,
            '/robot/navigate_to_goal',
            self.execute_callback
        )

        # Simulated robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0

        self.get_logger().info('Navigation action server ready')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'Navigating to: ({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})')

        start_time = time.time()
        feedback_msg = NavigateToGoal.Feedback()

        # Simulate navigation (move gradually toward goal)
        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Navigation canceled')
                return NavigateToGoal.Result()

            # Calculate distance to goal
            dx = goal.x - self.robot_x
            dy = goal.y - self.robot_y
            distance = math.sqrt(dx**2 + dy**2)

            # Check if reached goal (within 0.1m tolerance)
            if distance < 0.1:
                break

            # Move toward goal (simplified motion model)
            step_size = min(0.1, distance)  # Max 0.1m per step
            self.robot_x += (dx / distance) * step_size
            self.robot_y += (dy / distance) * step_size

            # Publish feedback
            feedback_msg.current_x = self.robot_x
            feedback_msg.current_y = self.robot_y
            feedback_msg.distance_remaining = distance
            feedback_msg.time_elapsed = time.time() - start_time

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(
                f'Distance remaining: {distance:.2f}m, Time: {feedback_msg.time_elapsed:.1f}s'
            )

            time.sleep(0.5)  # 2 Hz update rate

        # Reached goal
        self.robot_theta = goal.theta
        goal_handle.succeed()

        result = NavigateToGoal.Result()
        result.success = True
        result.message = "Navigation succeeded"
        result.final_x = self.robot_x
        result.final_y = self.robot_y
        result.final_theta = self.robot_theta

        self.get_logger().info(f'Goal reached! Final position: ({result.final_x:.2f}, {result.final_y:.2f})')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavigationActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Testing with Command Line**:
```bash
# Send navigation goal with feedback display
ros2 action send_goal /robot/navigate_to_goal robot_interfaces/action/NavigateToGoal "{x: 5.0, y: 3.0, theta: 1.57}" --feedback
```

**Expected Output**:
```
Waiting for action server...
Sending goal...
Goal accepted
Feedback: current_x=0.1, current_y=0.06, distance_remaining=5.82, time_elapsed=0.5
Feedback: current_x=0.2, current_y=0.12, distance_remaining=5.72, time_elapsed=1.0
...
Feedback: current_x=4.9, current_y=2.94, distance_remaining=0.12, time_elapsed=28.5
Result: success=True, message="Navigation succeeded", final_x=5.0, final_y=3.0
```

### Example 3: Canceling an Action

Demonstrate canceling a long-running action:

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_interfaces.action import NavigateToGoal
import threading
import time

class NavigationClientWithCancel(Node):
    def __init__(self):
        super().__init__('navigation_client_with_cancel')
        self._action_client = ActionClient(self, NavigateToGoal, '/robot/navigate_to_goal')
        self._goal_handle = None

    def send_goal(self, x, y, theta):
        self._action_client.wait_for_server()

        goal_msg = NavigateToGoal.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.theta = theta

        self.get_logger().info(f'Sending goal: ({x}, {y}, {theta})')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if self._goal_handle.accepted:
            self.get_logger().info('Goal accepted')
        else:
            self.get_logger().info('Goal rejected')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback: position=({feedback.current_x:.2f}, {feedback.current_y:.2f}), '
            f'remaining={feedback.distance_remaining:.2f}m'
        )

    def cancel_goal(self):
        if self._goal_handle:
            self.get_logger().info('Canceling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_callback)

    def cancel_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

def main(args=None):
    rclpy.init(args=args)
    client = NavigationClientWithCancel()

    # Send goal
    client.send_goal(10.0, 10.0, 0.0)

    # Schedule cancellation after 3 seconds
    def cancel_after_delay():
        time.sleep(3.0)
        client.cancel_goal()

    cancel_thread = threading.Thread(target=cancel_after_delay)
    cancel_thread.start()

    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

This demonstrates real-world scenarios: starting a navigation task, then canceling if conditions change (e.g., new higher-priority goal, obstacle detected, user intervention).

### Common Pitfalls

1. **Not Waiting for Service/Action Server**: Calling a service or sending a goal before the server is ready causes timeouts. Always use `wait_for_service()` or `wait_for_server()`.

2. **Blocking in Service Callbacks**: Service callbacks should complete quickly. Long computations block other requests. Offload heavy work to separate threads.

3. **Forgetting to Return Response**: Service callbacks must return a response object. Forgetting causes the client to timeout.

4. **Not Handling Action Cancellation**: Always check `goal_handle.is_cancel_requested` in action servers and clean up resources when canceled.

5. **Ignoring Action Feedback**: Clients should process feedback to monitor progress. Ignoring feedback means you can't detect stalls or problems.

## Further Resources

### Official Documentation
- **ROS 2 Services Tutorial**: [https://docs.ros.org/en/humble/Tutorials/Services.html](https://docs.ros.org/en/humble/Tutorials/Services.html)
- **ROS 2 Actions Tutorial**: [https://docs.ros.org/en/humble/Tutorials/Actions.html](https://docs.ros.org/en/humble/Tutorials/Actions.html)
- **Custom Service/Action Definitions**: [https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html](https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html)

### Standard Interfaces
- **std_srvs**: [https://github.com/ros2/common_interfaces/tree/humble/std_srvs](https://github.com/ros2/common_interfaces/tree/humble/std_srvs)
- **nav2_msgs**: [https://github.com/ros-planning/navigation2/tree/humble/nav2_msgs](https://github.com/ros-planning/navigation2/tree/humble/nav2_msgs)
- **control_msgs**: [https://github.com/ros-controls/control_msgs/tree/humble](https://github.com/ros-controls/control_msgs/tree/humble)

### Tools
- **rqt_action**: Visualize and interact with actions
- **rqt_service_caller**: Call services from GUI

---

**Next**: Test your understanding with [Self-Assessment](./self-assessment), or proceed to [Chapter 4: Digital Twin Simulation](/chapter-04/) to learn about simulating robots in Gazebo and Isaac Sim.
