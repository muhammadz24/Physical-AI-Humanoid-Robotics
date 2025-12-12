---
sidebar_position: 2
title: Kinematics Basics
---

# Humanoid Kinematics Fundamentals

## Forward Kinematics

Forward kinematics maps joint angles to end-effector (foot) positions using Denavit-Hartenberg (DH) parameters. For a 6-DOF leg (hip yaw/roll/pitch, knee pitch, ankle pitch/roll), the transformation chain computes the foot position in world coordinates.

**Summary:** This section covers DH parameter tables for Unitree G1 legs, homogeneous transformation matrices, and Python implementations using the robotics toolbox. Students learn to compute foot trajectories given joint angle sequences and validate results in RViz visualization.

## Inverse Kinematics

Inverse kinematics solves the reverse problem: given a desired foot position, compute the joint angles required to reach it. For humanoid walking, IK ensures feet follow planned trajectories while maintaining kinematic constraints (joint limits, singularities).

**Summary:** Introduces analytical IK solutions for 6-DOF legs, numerical optimization methods (Jacobian pseudoinverse), and ROS 2 integration with MoveIt 2 IK solvers. Includes practical examples of stepping motion planning with collision avoidance.
