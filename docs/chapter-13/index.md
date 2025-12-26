---
sidebar_position: 13
title: Chapter 13 - Conversational Robotics (Capstone)
---

# Conversational Robotics: The Autonomous Humanoid

## Chapter Overview

This capstone chapter integrates every skill from the textbook—ROS 2 communication, perception, navigation, manipulation, and reinforcement learning—into a unified conversational robotics system. You'll build an autonomous humanoid that accepts natural language commands ("Go to the kitchen and bring me the red cup"), breaks them into actionable subtasks using large language models (GPT-4o/Claude), and executes them through coordinated locomotion and manipulation. This represents the frontier of embodied AI: robots that understand human intent and act intelligently in unstructured environments.

The final project deploys to Unitree G1 hardware, demonstrating real-world voice-to-action pipelines with speech recognition (Whisper), LLM reasoning, ROS 2 action orchestration, and visual feedback loops.

## Learning Objectives

1. **Vision-Language-Action (VLA) Architecture**: LLM as task planner for robot actions
2. **Speech Integration**: OpenAI Whisper for real-time voice commands
3. **Action Orchestration**: BehaviorTree or state machine for multi-step tasks
4. **End-to-End Pipeline**: Voice → LLM → ROS 2 → Hardware execution
5. **Failure Recovery**: Detecting and correcting execution errors (e.g., grasp failures)

## System Architecture

**Voice-to-Action Pipeline:**
```
[Microphone] → [Whisper STT] → [GPT-4o/Claude] → [ROS 2 Action Server] → [Unitree G1]
     ↓              ↓                  ↓                    ↓                    ↓
  Audio PCM    Text Transcript    Action Sequence    nav/manip/speak      Hardware
```

**Hardware Stack:**
- **Compute**: Jetson Orin Nano (VLA inference, ROS 2 control)
- **Sensors**: RealSense D435i (vision), ReSpeaker Mic Array (audio)
- **Actuators**: Unitree G1 (23 DOF locomotion + manipulation)
- **Network**: 5G/WiFi for cloud LLM API calls (GPT-4o latency: 500-2000ms)

---

**Next Sections:**
- [VLA Fundamentals](./intro-vla)
- [Voice Integration](./voice-integration)
- [Final Project](./final-project)
