---
id: 003
title: Add Zero-Edit Deployment Principle
stage: constitution
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5
feature: none
branch: main
user: system
command: /sp.constitution
labels: ["deployment", "environment-variables", "cors", "production-ready"]
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

@CLAUDE_STRICT_MODE
CONTEXT:
1. The user wants a "Zero Manual Edit" policy for the future.
2. Current CORS setup fails on localhost.
3. Code must be "Production Ready" now—meaning it should automatically accept the Live Domain via Environment Variables without changing Python code later.

INSTRUCTIONS:
Execute this workflow using SpecKit Plus (`sp`):

1. INITIALIZE:
   - Run `sp constitution` and `sp spec`.

2. BACKEND PLAN & TASK (`backend/main.py`):
   - Run `sp plan` to refactor CORS in `main.py`.
   - IMPERATIVE: Do not hardcode the production domain.
   - LOGIC TO IMPLEMENT:
     import os
     origins = [
         "http://localhost:3000",
         "http://127.0.0.1:3000"
     ]
     # Auto-add production domain from Environment Variable
     prod_origin = os.getenv("ALLOWED_ORIGIN")
     if prod_origin:
         origins.append(prod_origin)
   - Apply this logic using `sp task`.

3. FRONTEND PLAN & TASK (`src/api` or `src/config`):
   - Locate where the Backend Base URL is defined (e.g., `axios.js`, `config.js`, or `.env`).
   - Ensure it uses a dynamic approach: `process.env.NEXT_PUBLIC_API_URL` (or equivalent for the framework) with a fallback to `http://localhost:8000`.
   - Run `sp task` to fix any hardcoded "localhost" strings that might break in production.

4. FINAL VERIFICATION:
   - Restart the backend server.
   - Output the summary of how to set the "ALLOWED_ORIGIN" variable on the hosting platform (e.g., Vercel/Render) so the user knows what to do *outside* the code.

GOAL: Clean code, no vibe, pure functionality.

## Response snapshot

Constitution updated from v1.1.0 to v1.2.0 with new Principle IX: Zero-Edit Deployment Configuration.

Key additions:
- Added comprehensive deployment configuration requirements
- Backend CORS must use environment-driven origins pattern
- Frontend API URLs must use framework-native environment variables
- Documentation requirements for deployment (README, .env.example)
- Quality gates updated to enforce Principle IX compliance
- Integration with twelve-factor app methodology

The principle is marked NON-NEGOTIABLE to ensure consistent deployment practices across all environments without manual code modifications.

## Outcome

- ✅ Impact: MINOR version bump (1.1.0 → 1.2.0), added new Principle IX covering environment-driven configuration for CORS and API URLs
- 🧪 Tests: None (constitution amendment)
- 📁 Files: .specify/memory/constitution.md
- 🔁 Next prompts: /sp.specify to create feature spec for Zero-Edit CORS refactoring, /sp.plan for implementation architecture
- 🧠 Reflection: This principle codifies production-readiness requirements and eliminates deployment friction by enforcing environment variable usage from the start

## Evaluation notes (flywheel)

- Failure modes observed: Previous hardcoded CORS origins caused localhost connection failures and would require manual code changes for production deployment
- Graders run and results (PASS/FAIL): PASS - Constitution v1.2.0 validates with no unresolved placeholders, version incremented correctly (MINOR for new principle addition)
- Prompt variant (if applicable): Standard constitution amendment workflow
- Next experiment (smallest change to try): Implement Principle IX in backend/main.py and frontend ChatWidget, verify zero-edit deployment to staging environment
