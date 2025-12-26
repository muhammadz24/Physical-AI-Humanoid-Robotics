---
sidebar_position: 11
title: Chapter 11 - Humanoid Kinematics & Locomotion
---

# Humanoid Kinematics & Locomotion

## Chapter Overview

Humanoid locomotion represents one of the most challenging frontiers in robotics, requiring precise coordination of multiple degrees of freedom to achieve stable bipedal walking. Unlike quadrupeds (Unitree Go2) that distribute weight across four contact points, humanoids (Unitree G1) must continuously manage dynamic balance on two legs while maintaining forward motion. This chapter explores the mathematical foundations of bipedal kinematics, Zero Moment Point (ZMP) theory for stability analysis, and practical gait generation strategies for real-world deployment.

You'll implement walking controllers using ROS 2 control framework, analyze center-of-mass trajectories, and deploy gaits to the Unitree G1 humanoid platform. By mastering these techniques, you'll understand how robots like Atlas, Optimus, and G1 achieve human-like mobility in complex environments.

## Learning Objectives

1. **Understand Kinematic Chains**: Forward/inverse kinematics for multi-DOF humanoid legs
2. **ZMP Stability Theory**: Mathematical conditions for balanced bipedal walking
3. **Gait Pattern Generation**: Trajectory planning for swing/stance phases
4. **Hardware Deployment**: Unitree G1 walking controller implementation
5. **Comparative Analysis**: Humanoid vs quadruped locomotion tradeoffs

## Hardware Context

**Unitree G1 Humanoid Specifications:**
- Height: 1.3m, Weight: 35kg
- DOF: 23 joints (6 per leg, 7 per arm, 3 waist)
- Walking Speed: 0.3-1.0 m/s
- Onboard Computer: Jetson Orin NX (100 TOPS)

**Locomotion Comparison:**
| Feature | Humanoid (G1) | Quadruped (Go2) |
|---------|---------------|-----------------|
| Stability | Dynamic (ZMP) | Static (4 points) |
| Speed | 1.0 m/s | 3.5 m/s |
| Manipulation | ✅ Hands free | ❌ Limited |
| Terrain | Stairs, ladders | Rough terrain |

---

**Next Sections:**
- [Kinematics Basics](./kinematics-basics)
- [Walking Gait Generation](./walking-gait)
- [Balance Control](./balance-control)
