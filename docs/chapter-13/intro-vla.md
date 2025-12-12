---
sidebar_position: 2
title: VLA Fundamentals
---

# Vision-Language-Action Models

## LLM as Robot Task Planner

Large language models (GPT-4o, Claude 3.5 Sonnet) excel at decomposing high-level instructions into executable subtasks. Given "Bring me coffee," the LLM generates a plan: navigate to kitchen → detect mug → grasp mug → navigate to user → hand over. Each subtask maps to ROS 2 actions (`/navigate_to_pose`, `/pick_object`, `/move_arm`).

**Summary:** Explains prompt engineering for robot control (few-shot examples, tool use formatting), parsing LLM outputs into structured action sequences (JSON), and error handling when LLM produces invalid commands. Demonstrates integration with OpenAI API and Anthropic Claude API, comparing latency (GPT-4o: 800ms vs Claude Opus: 1200ms) and reasoning quality for spatial tasks.

## Grounding Language in Perception

LLMs must ground spatial references ("the red cup on the left") in visual perception. This requires passing RealSense RGB frames + detected object labels to the LLM, which reasons about scene geometry and selects target objects.

**Summary:** Implements multimodal prompts combining text instructions with base64-encoded images, demonstrates GPT-4 Vision API for object selection, and integrates with YOLO bounding boxes to map linguistic descriptions to pixel coordinates. Covers spatial reasoning challenges (e.g., "left" from robot vs human perspective) and coordinate frame transformations (camera → world).
