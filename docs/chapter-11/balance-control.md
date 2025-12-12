---
sidebar_position: 4
title: Balance Control
---

# Dynamic Balance Control

## Whole-Body Control

Whole-body controllers optimize joint torques across all 23 DOF to achieve desired CoM/ZMP trajectories while respecting joint limits and torque constraints. Quadratic programming (QP) formulations balance multiple objectives (tracking, stability, energy efficiency).

**Summary:** Introduces task-space control frameworks, priority hierarchies for competing objectives, and real-time QP solvers (qpOASES). Demonstrates ROS 2 implementation of whole-body controllers that handle external pushes and uneven terrain by dynamically adjusting foot placements and CoM trajectories.

## Deployment to Unitree G1

Deploying walking controllers to real hardware requires integrating high-level planners with low-level motor drivers, handling communication latency, and tuning gains for physical compliance.

**Summary:** Covers Unitree G1 SDK setup, real-time ROS 2 control loops (1kHz servo rate), and safety protocols (emergency stop, fall detection). Includes step-by-step deployment workflow from Isaac Sim validation to hardware execution, with troubleshooting guides for common issues like foot slipping and torque saturation.
