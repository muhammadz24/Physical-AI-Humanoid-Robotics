---
id: 001
title: Generate Project Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "project-setup", "governance", "principles"]
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

Project Name: Physical AI & Humanoid Robotics — Essentials

Purpose:
Create a short, clean, professional AI-Native textbook based on the Physical AI & Humanoid Robotics course.
The book must serve as a fast, simple, high-quality learning resource built with a modern Docusaurus UI
and a fully integrated free-tier RAG chatbot.

Scope:
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

Core Principles:
- Simplicity
- Accuracy
- Minimalism
- Fast builds
- Free-tier architecture
- RAG answers ONLY from book text

Key Features:
- Docusaurus textbook
- RAG chatbot (Qdrant + Neon + FastAPI)
- Select-text → Ask AI
- Optional Urdu / Personalize features

Constraints:
- No heavy GPU usage
- Minimal embeddings

Success Criteria:
- Build success
- Accurate chatbot
- Clean UI
- Smooth GitHub Pages deployment

Generate full constitution.

## Response snapshot

Created comprehensive constitution for Physical AI & Humanoid Robotics — Essentials project with:

**7 Core Principles:**
1. Simplicity-First Design - clarity and ease of understanding
2. Accuracy & Source Fidelity - technical accuracy, RAG grounded in textbook only
3. Free-Tier Architecture (NON-NEGOTIABLE) - Qdrant, Neon, GitHub Pages/Vercel limits
4. Minimalism in Technology Stack - Docusaurus, FastAPI, Sentence Transformers
5. Fast Build & Iteration Cycles - <60s builds, <10s dev server
6. Content-First Development - 6 chapters with structured requirements
7. RAG Chatbot Guardrails - >90% accuracy, <3s response, no hallucinations

**Additional Sections:**
- Content Standards (chapter structure, code quality)
- Performance Standards (frontend >90 Lighthouse, backend <2s queries)
- Security & Privacy (minimal data collection, dependency scanning)
- Deployment & Operations (GitHub Actions, monitoring, rollback)
- Feature Roadmap (3 phases: MVP, Enhanced UX, Advanced)
- Quality Gates (content merge, deployment, production)
- Governance (amendment process, compliance verification)

**Technology Stack:**
- Frontend: Docusaurus
- Backend: FastAPI
- Vector DB: Qdrant Cloud (free tier)
- Database: Neon Postgres (free tier)
- Embeddings: Sentence Transformers (all-MiniLM-L6-v2, 384 dims)
- Deployment: GitHub Pages or Vercel

**Key Constraints Enforced:**
- No paid APIs or services
- No GPU-dependent production operations
- <1000 vector chunks total
- All builds/deploys within free-tier limits

## Outcome

- ✅ Impact: Full project constitution established with 7 core principles, content standards, performance benchmarks, and governance model
- 🧪 Tests: none (constitution document)
- 📁 Files: Created .specify/memory/constitution.md (257 lines)
- 🔁 Next prompts: Create feature specs for Phase 1 MVP (6 chapters + RAG chatbot), initialize Docusaurus project, set up project structure
- 🧠 Reflection: Constitution successfully balances educational quality, technical constraints (free-tier), and simplicity principles. Clear governance and quality gates will ensure project stays on track.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS - Constitution complete with all required sections
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Begin Phase 1 implementation with Docusaurus setup and chapter 1 content creation
