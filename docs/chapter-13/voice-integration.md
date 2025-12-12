---
sidebar_position: 3
title: Voice Integration
---

# Speech-to-Text with OpenAI Whisper

## Real-Time Audio Capture

Capturing clean audio in noisy robotics environments requires microphone arrays (ReSpeaker 6-mic) with beamforming to isolate human voice from motor noise. Audio streams at 16kHz PCM format feed into Whisper API for transcription.

**Summary:** Covers ROS 2 audio capture nodes (`audio_common` package), VAD (voice activity detection) to trigger transcription only when speech detected (saves API costs), and noise cancellation preprocessing. Demonstrates integration with ReSpeaker hardware, achieving under 100ms audio latency to Whisper API endpoint.

## Whisper API Integration

OpenAI Whisper transcribes speech to text with high accuracy across accents and background noise. The `whisper-1` model handles 100+ languages, outputting timestamped transcripts for command parsing.

**Summary:** Implements Python ROS 2 node that streams audio chunks to Whisper API, handles API errors/retries, and publishes transcribed text to `/voice_command` topic. Covers cost optimization (batching requests, local Whisper deployment on Jetson for offline operation), and compares cloud (100ms latency) vs edge (500ms on Orin) tradeoffs.

## Text-to-Speech Feedback

Robots should confirm commands audibly ("Navigating to kitchen...") using text-to-speech (TTS). OpenAI TTS or ElevenLabs generates natural-sounding voice responses synchronized with robot actions.

**Summary:** Integrates TTS API calls within ROS 2 action servers, plays audio through robot speakers (pyaudio), and implements status narration for transparency ("Grasping object...", "Navigation complete"). Demonstrates full duplex interaction where robot listens, acts, and speaks simultaneously without cross-talk.
