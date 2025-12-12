---
sidebar_position: 4
title: HRI Design
---

# Human-Robot Interaction Design

## Safety Protocols

Physical human-robot collaboration demands fail-safe mechanisms: emergency stop buttons, collision detection (unexpected force spikes), and speed/force limiting in human-proximity zones. ISO 15066 defines safety requirements for collaborative robots.

**Summary:** Implements ROS 2 safety monitors that halt motion when collision forces exceed thresholds (10N), establish virtual safety zones around detected humans (LiDAR/camera fusion), and provide user-controlled speed scaling. Covers certification considerations and risk assessment methodologies for collaborative manipulation tasks.

## Teleoperation Interfaces

When autonomous grasping fails (novel objects, cluttered scenes), teleoperation allows human operators to guide the robot. Interfaces range from joystick control to VR immersion (Quest 2) enabling intuitive 6-DOF manipulation.

**Summary:** Develops multi-modal teleoperation systems: keyboard velocity control, SpaceMouse 6-DOF input, and Unity VR interface with hand tracking. Implements shared autonomy where the robot provides grasp suggestions and collision avoidance while the human retains high-level control. Demonstrates telepresence applications for remote inspection and manipulation tasks.
