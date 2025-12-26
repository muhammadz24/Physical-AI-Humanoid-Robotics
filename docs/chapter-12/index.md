---
sidebar_position: 12
title: Chapter 12 - Manipulation & Interaction
---

# Manipulation & Human-Robot Interaction

## Chapter Overview

Humanoid manipulation extends beyond simple pick-and-place tasks to encompass dexterous grasping, bimanual coordination, and safe human-robot collaboration. This chapter bridges perception (RealSense D435i depth sensing) with action (7-DOF arm control) to enable robots like Unitree G1 to manipulate objects in unstructured environments. You'll implement end-to-end grasping pipelines using MoveIt 2, integrate force-torque feedback for compliant control, and design intuitive human-robot interfaces for teleoperation and collaboration.

The capstone builds toward conversational robotics where verbal commands ("pick up the red cube") trigger autonomous manipulation sequences, combining computer vision, motion planning, and natural language understanding.

## Learning Objectives

1. **Arm Kinematics**: 7-DOF inverse kinematics for redundant manipulators
2. **Grasp Planning**: Analytical and learning-based grasp synthesis
3. **MoveIt 2 Integration**: Collision-free motion planning and execution
4. **Vision-Based Grasping**: RealSense depth + segmentation for object pose estimation
5. **HRI Design**: Safety protocols and user interface patterns

## Hardware Pipeline

**Perception → Planning → Execution:**
1. **RealSense D435i**: RGB-D capture (1280×720 @ 30 FPS)
2. **Segmentation**: YOLO/SAM object detection on RTX 4070 Ti
3. **Pose Estimation**: PCA-based 6D pose from point clouds
4. **Motion Planning**: MoveIt 2 RRT-Connect trajectory optimization
5. **Execution**: Unitree G1 arm (7 DOF, 3kg payload)

**Latency Budget:**
- Vision: 50ms (GPU inference)
- Planning: 200-500ms (MoveIt 2)
- Execution: 2-5s (arm motion)

---

**Next Sections:**
- [Manipulation Theory](./manipulation-theory)
- [Grasping Pipeline](./grasping-pipeline)
- [HRI Design](./hri-design)
