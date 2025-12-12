---
sidebar_position: 3
title: Walking Gait Generation
---

# Bipedal Gait Pattern Generation

## Swing and Stance Phases

Humanoid walking alternates between swing phase (leg lifting/moving forward) and stance phase (leg supporting body weight). Coordinating these phases while maintaining balance requires precise trajectory planning for center of mass (CoM) and zero moment point (ZMP).

**Summary:** This section explains the double support and single support phases, foot trajectory generation using cubic splines, and temporal synchronization of left/right legs. Covers practical gait parameters like step length (0.2-0.4m), step height (0.05-0.1m), and walking frequency (0.5-1.0 Hz) for Unitree G1.

## ZMP-Based Stability

The Zero Moment Point (ZMP) must remain inside the support polygon (foot contact area) to prevent tipping. Cart-table model simplifies humanoid dynamics to a point mass on an inverted pendulum, enabling real-time ZMP computation.

**Summary:** Derives ZMP equations from moment balance, implements preview control for ZMP tracking, and demonstrates ROS 2 controllers that adjust CoM trajectories to maintain stability. Includes Isaac Sim simulations showing stable vs unstable walking gaits with ZMP visualization overlays.
