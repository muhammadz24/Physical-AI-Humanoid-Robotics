---
sidebar_position: 3
title: Grasping Pipeline
---

# Vision-Based Grasping Pipeline

## Object Detection and Pose Estimation

The grasping pipeline begins with RealSense D435i capturing RGB-D frames, followed by object segmentation (YOLOv8 bounding boxes + SAM masks) and 6D pose estimation from aligned point clouds. PCA determines object principal axes for grasp orientation planning.

**Summary:** Step-by-step implementation of perception pipeline: camera calibration, depth-RGB alignment, GPU-accelerated segmentation (Isaac ROS YOLO), point cloud extraction, and RANSAC plane fitting for table removal. Outputs object poses as ROS 2 TF transforms for downstream motion planning.

## MoveIt 2 Motion Planning

Given target object pose, MoveIt 2 computes collision-free trajectories from current arm configuration to pre-grasp → grasp → lift → place waypoints. RRT-Connect planner searches configuration space while respecting joint limits and scene geometry (table, obstacles).

**Summary:** Covers MoveIt 2 setup (SRDF configuration, planning scene management), Cartesian path planning for straight-line approaches, gripper action integration, and execution monitoring with error recovery. Demonstrates complete pick-and-place example with cube objects, including grasp quality metrics (force closure, wrench space analysis).
