---
sidebar_position: 4
title: Nav2 for Humanoids
---

# Nav2 for Humanoid Robots

## Configuring Nav2 for Bipedal/Quadrupedal Locomotion

**Key Differences from Wheeled Robots**:
- Footstep planning (discrete foot placements)
- Center of Mass (CoM) constraints
- Slower acceleration/deceleration
- Higher computational cost

**Nav2 Configuration** (`nav2_params_humanoid.yaml`):

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 10.0  # Lower for humanoids (vs 20 Hz for wheeled)

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.1  # Humanoid minimum speed
      max_vel_x: 0.5  # Conservative max speed
      max_vel_theta: 0.3  # Slower turning
      acc_lim_x: 0.2  # Gentle acceleration
      acc_lim_theta: 0.3

local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"  # Humanoid footprint
      inflation_radius: 0.5  # Larger safety margin
```

**Footstep Planning (Advanced)**:

For true humanoid walking, use dedicated footstep planners:

```bash
# Install humanoid_locomotion (example)
sudo apt install ros-humble-humanoid-nav

# Launch with footstep planner
ros2 launch humanoid_nav humanoid_navigation.launch.py
```

**Unitree Go2/G1 Integration**:

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from unitree_go2_interfaces.msg import LowCmd

class HumanoidNavController(Node):
    def __init__(self):
        super().__init__('humanoid_nav')

        # Subscribe to Nav2 path
        self.create_subscription(Path, '/plan', self.path_callback, 10)

        # Publish to Unitree low-level controller
        self.cmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)

    def path_callback(self, path_msg):
        # Convert Nav2 path to footstep sequence
        for pose in path_msg.poses:
            foot_cmd = self.compute_footstep(pose)
            self.cmd_pub.publish(foot_cmd)

def main():
    rclpy.init()
    node = HumanoidNavController()
    rclpy.spin(node)
```

---

✅ **CHAPTER 9 COMPLETE**: Isaac ROS, VSLAM, Nav2 for humanoids!

**Next**: [Chapter 10: Reinforcement Learning & Sim-to-Real](/docs/chapter-10/)
