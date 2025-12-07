---
id: 002
title: Add Docusaurus Best Practices Principle
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "docusaurus", "standards", "amendment"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

# Project Name: Physical AI & Humanoid Robotics — Essentials

## Purpose

Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.

The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI

and a fully integrated free-tier RAG chatbot.

## Scope
- 6 short chapters:
  1. Introduction to Physical AI
  2. Basics of Humanoid Robotics
  3. ROS 2 Fundamentals
  4. Digital Twin Simulation (Gazebo + Isaac)
  5. Vision-Language-Action Systems
  6. Capstone: Simple AI-Robot Pipeline
- Clean UI
- Free-tier friendly
- Lightweight embeddings

## Core Principles
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text
- **Tech Standard: Strictly follow Context 7 (https://docusaurus.io/docs) for all UI/UX patterns.**

## Key Features
- Docusaurus textbook (Best practices from Context 7)
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

## Constraints
- No heavy GPU usage
- Minimal embeddings

## Success Criteria
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

## Response snapshot

Updated constitution to version 1.1.0 with new principle:

**IV. Docusaurus Best Practices Compliance (NON-NEGOTIABLE)**
- Authoritative source: https://docusaurus.io/docs
- All UI/UX decisions verified against official documentation
- Use built-in features before custom solutions
- Follow recommended project structure, configuration, plugin patterns
- Leverage Docusaurus theming (no custom CSS frameworks)
- MDX best practices for content

**Core Docusaurus Features Required:**
- Built-in search (Algolia DocSearch or local)
- Dark mode (built-in theme toggle)
- Sidebar navigation (auto-generated)
- Code block features (syntax highlighting, line numbers, copy)
- Admonitions (:::note, :::tip, :::warning, :::danger)
- Tabs and interactive elements

**Renumbered Principles:**
- IV. Docusaurus Best Practices Compliance (NEW)
- V. Minimalism in Technology Stack (updated to specify Docusaurus v3.x)
- VI. Fast Build & Iteration Cycles (formerly V)
- VII. Content-First Development (formerly VI)
- VIII. RAG Chatbot Guardrails (formerly VII)

**Technology Stack Updates:**
- Specified Docusaurus v3.x
- Added "official best practices" emphasis
- Forbidden custom CSS frameworks (use Docusaurus theming)
- Specified all-MiniLM-L6-v2 for embeddings

## Outcome

- ✅ Impact: Constitution now mandates strict adherence to Docusaurus official documentation for all frontend decisions, ensuring best practices and avoiding custom implementations
- 🧪 Tests: none (constitution amendment)
- 📁 Files: Updated .specify/memory/constitution.md (principle IV added, principles renumbered, version 1.0.0 → 1.1.0)
- 🔁 Next prompts: Initialize Docusaurus project following official documentation, create feature spec for textbook structure
- 🧠 Reflection: Adding Docusaurus compliance as a NON-NEGOTIABLE principle ensures the project leverages the framework's full capabilities and avoids reinventing built-in features. This will speed development and ensure professional UI/UX quality.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution successfully amended with Docusaurus best practices principle
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin Docusaurus project initialization with official scaffolding command
