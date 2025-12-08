---
sidebar_position: 2
title: Project Overview
---

# Capstone Project Overview

## Project Specification

### Functional Requirements

**FR-1**: System shall accept text commands like "pick up the red cube"

**FR-2**: System shall detect colored objects in camera view using computer vision

**FR-3**: System shall compute valid grasp poses using inverse kinematics

**FR-4**: System shall execute pick-and-place motions in Gazebo simulation

**FR-5**: System shall validate successful completion (object moved to target)

### System Components

**1. Perception Module** (Chapter 1, Chapter 5)
- RGB camera in Gazebo
- Object detection using color-based segmentation
- Pose estimation (object position relative to robot)

**2. Planning Module** (Chapter 2)
- Inverse kinematics for grasp pose
- Trajectory generation for smooth motion
- Collision checking (basic)

**3. Control Module** (Chapter 3)
- ROS 2 action server for pick-and-place
- Joint trajectory controller
- Gripper control

**4. Simulation Environment** (Chapter 4)
- Gazebo world with robot arm
- Colored objects (cubes, cylinders)
- Camera sensor

**5. Language Interface** (Chapter 5)
- Simple command parser (not full VLA for simplicity)
- Maps "red cube" → object detection parameters

## Setup Requirements

### Software Prerequisites

```bash
# ROS 2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Additional packages
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-robot-state-publisher

# Python dependencies
pip install opencv-python numpy
```

### Hardware Requirements

- **Minimum**: Laptop with integrated GPU
- **Recommended**: Discrete GPU for faster Gazebo rendering
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 10GB for ROS 2 + Gazebo

## Project Structure

```
capstone_robot/
├── src/
│   ├── perception/
│   │   ├── object_detector.py       # Vision processing
│   │   └── camera_processor.py      # ROS 2 camera interface
│   ├── planning/
│   │   ├── grasp_planner.py         # IK and grasp computation
│   │   └── trajectory_planner.py    # Motion planning
│   ├── control/
│   │   ├── pick_place_server.py     # Main action server
│   │   └── gripper_controller.py    # Gripper control
│   └── language/
│       └── command_parser.py        # Text command processing
├── urdf/
│   └── robot_arm.urdf               # Robot description
├── launch/
│   ├── gazebo_world.launch.py       # Spawn Gazebo
│   └── robot_system.launch.py       # Launch all nodes
└── config/
    ├── gazebo_world.world           # Simulation environment
    └── joint_limits.yaml            # Robot constraints
```

## Development Phases

### Phase 1: Simulation Setup (30 minutes)
- Create Gazebo world
- Spawn robot arm (6-DOF)
- Add camera sensor
- Place colored objects

### Phase 2: Perception (45 minutes)
- Implement color-based object detection
- Camera image subscriber
- Publish detected object poses

### Phase 3: Planning (60 minutes)
- Inverse kinematics solver
- Grasp pose computation
- Trajectory generation

### Phase 4: Control (45 minutes)
- Pick-and-place action server
- Joint trajectory execution
- Gripper open/close

### Phase 5: Integration (60 minutes)
- Command parser
- End-to-end testing
- Debug and refine

## Success Criteria

**Minimum Viable Project**:
- ✓ Robot picks up one colored object
- ✓ Moves it to a predefined location
- ✓ Accepts at least one command variant

**Full Success**:
- ✓ Picks up multiple object types (cube, cylinder)
- ✓ Distinguishes colors (red, blue, green)
- ✓ Places objects at commanded locations
- ✓ Handles failures gracefully (object out of reach)

## Risk Mitigation

**Risk 1**: Gazebo crashes or runs slowly
- **Mitigation**: Use headless mode, reduce sensor frequency

**Risk 2**: IK solver fails to find solution
- **Mitigation**: Check object is in workspace, add fallback positions

**Risk 3**: Object detection unreliable
- **Mitigation**: Use solid colors, good lighting, tune HSV thresholds

---

**Next**: Proceed to [Implementation](./implementation) to start building!
