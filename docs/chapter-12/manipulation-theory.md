---
sidebar_position: 2
title: Manipulation Theory
---

# Manipulation Fundamentals

## 7-DOF Inverse Kinematics

Unitree G1 arms have 7 degrees of freedom (shoulder pitch/roll/yaw, elbow pitch, wrist pitch/roll/yaw), providing redundancy for obstacle avoidance. Unlike 6-DOF arms with unique IK solutions, 7-DOF systems have infinite solutions—requiring optimization criteria to select preferred configurations.

**Summary:** Covers null-space optimization for elbow positioning, singularity avoidance strategies, and analytical IK formulations for anthropomorphic arms. Demonstrates ROS 2 KDL and MoveIt 2 IK solvers with joint limit handling. Includes comparative analysis of computational efficiency (analytical: under 1ms vs numerical: 10-50ms).

## Force Control and Compliance

Rigid position control causes damage when contacts occur unexpectedly. Impedance control allows compliant behavior—arms yield to external forces while maintaining desired trajectories. Hybrid force/position control enables tasks like surface wiping (force-controlled normal direction, position-controlled tangent).

**Summary:** Introduces impedance control theory (virtual spring-damper systems), Cartesian stiffness matrices, and force-torque sensor integration. Provides ROS 2 controllers that switch between position/force modes based on contact detection, with safety limits to prevent excessive forces during human interaction.
