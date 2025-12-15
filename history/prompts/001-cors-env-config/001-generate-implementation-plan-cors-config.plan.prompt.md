---
id: 001
title: Generate Implementation Plan for CORS Configuration
stage: plan
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5
feature: 001-cors-env-config
branch: 001-cors-env-config
user: system
command: /sp.plan
labels: ["cors", "environment-variables", "configuration", "zero-edit-deployment", "principle-ix"]
links:
  spec: specs/001-cors-env-config/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-cors-env-config/plan.md
 - specs/001-cors-env-config/research.md
 - specs/001-cors-env-config/data-model.md
 - specs/001-cors-env-config/quickstart.md
tests:
 - None (planning phase)
---

## Prompt

Execute `/sp.plan` workflow to generate implementation plan for feature `001-cors-env-config`.

**Spec Input**: `specs/001-cors-env-config/spec.md`

**Plan Requirements**:
1. **Backend (backend/main.py)**: Replace hardcoded CORS origins `["*"]` with `os.getenv("ALLOWED_ORIGINS")` pattern using existing `settings.allowed_origins_list` property from config.py
2. **Environment (backend/.env)**: Verify `.env.example` documents `ALLOWED_ORIGINS` with examples
3. **Frontend (src/components/ChatWidget/index.js)**: Remove hardcoded `'http://localhost:8000/api/chat'`, use `process.env.REACT_APP_API_URL || 'http://localhost:8000'` with fallback
4. **Verification**: Include server restart steps in implementation plan

**Output Goal**: Create `specs/001-cors-env-config/plan.md` with full implementation architecture, passing all sp.plan quality checks.

## Response snapshot

Generated complete implementation plan with Phase 0 (Research), Phase 1 (Design Artifacts), and Phase 2 (Task Generation) sections.

### Key Artifacts Created:

1. **plan.md** (394 lines):
   - Technical Context: Python 3.11+, FastAPI 0.109.0, Docusaurus 3.x, pydantic-settings
   - Constitution Check: ✅ PASS on all 4 principles (IX, V, III, I)
   - Phase 0 Research: 4 research questions resolved (pydantic-settings support, Docusaurus env vars, CORS security, fallback behavior)
   - Phase 1 Design: Implementation steps for backend CORS (lines 172-209), frontend API URL (lines 211-235), env documentation (lines 237-275)
   - Verification Plan: 4 test scenarios (local dev, production simulation, unauthorized origin, wildcard prevention)
   - ADR Suggestion: Environment-Variable Driven Configuration (3 alternatives considered)

2. **research.md** (185 lines):
   - 4 research decisions documented with rationale and alternatives
   - Best practices: FastAPI CORS, pydantic-settings, Docusaurus env vars, twelve-factor app
   - Risk mitigation table (4 risks identified)
   - Technical feasibility: HIGH for all changes (backend 10 lines, frontend 2 lines)

3. **data-model.md** (195 lines):
   - Environment variables schema for backend and frontend
   - Configuration lifecycle diagrams
   - Example configurations: local dev, production, multi-environment staging
   - Validation constraints and error handling
   - Migration path from hardcoded to environment-driven

4. **quickstart.md** (277 lines):
   - Local development guide (3 steps, no config needed)
   - Production deployment guides: Vercel, Railway, Render, GitHub Pages
   - Environment variable reference
   - Testing checklist (3 test types)
   - Troubleshooting section (4 common issues)

### Research Findings:

**Q1**: Does pydantic-settings support ALLOWED_ORIGINS?
**A**: YES - config.py:36 already has `allowed_origins: str` field with `allowed_origins_list` property (lines 56-59)

**Q2**: Does Docusaurus support REACT_APP_* env vars?
**A**: YES - Standard React pattern, build-time substitution via `process.env.REACT_APP_*`

**Q3**: What happens with wildcard ["*"] + credentials=True?
**A**: RuntimeError in FastAPI/Starlette. Current code (main.py:89) VIOLATES this (FR-003)

**Q4**: Fallback behavior if env vars missing?
**A**: Backend defaults to "http://localhost:3000" (config.py:36), frontend should default to localhost:8000

### Implementation Strategy:

**Backend Changes** (backend/main.py:87-93):
- Current: `allow_origins=["*"]` (INSECURE)
- After: Validate `settings.allowed_origins_list`, filter by protocol (http/https), log allowed origins, fallback to localhost if all invalid

**Frontend Changes** (src/components/ChatWidget/index.js):
- Add constant: `const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';`
- Update fetch: `fetch(\`\${API_BASE_URL}/api/chat\`, ...)`

**Documentation**:
- backend/.env.example: Already correct (lines 31-35)
- Create root/.env.example with REACT_APP_API_URL

## Outcome

- ✅ Impact: Complete implementation plan generated with 4 design artifacts (plan.md, research.md, data-model.md, quickstart.md)
- 🧪 Tests: None (planning phase - tests defined in verification plan for implementation phase)
- 📁 Files: specs/001-cors-env-config/{plan.md, research.md, data-model.md, quickstart.md}
- 🔁 Next prompts: `/sp.tasks` to generate actionable task list, `/sp.adr environment-variable-configuration` to document architectural decision
- 🧠 Reflection: Discovered existing pydantic-settings infrastructure already supports ALLOWED_ORIGINS pattern - only need to wire it up in main.py (10 lines of code). Frontend change is minimal (2 lines). High feasibility, low risk.

## Evaluation notes (flywheel)

- Failure modes observed: Current hardcoded CORS `["*"]` with credentials=True violates CORS security spec (FR-003) and would crash FastAPI if not for permissive middleware. Backend/frontend both have hardcoded localhost URLs requiring manual code changes for production deployment (violates Constitution Principle IX).
- Graders run and results (PASS/FAIL): PASS - All constitution checks passed (Principle IX: Zero-Edit Deployment ✅, Principle V: Minimalism ✅, Principle III: Free-Tier ✅, Principle I: Simplicity ✅). Plan includes all required sections (Technical Context, Constitution Check, Phase 0 Research, Phase 1 Design). No unresolved NEEDS CLARIFICATION markers.
- Prompt variant (if applicable): Standard `/sp.plan` workflow execution per SpecKit Plus command template
- Next experiment (smallest change to try): Implement Step 1 (backend CORS configuration in main.py) as minimal viable change - 10 lines of code to validate pattern works, then proceed with Step 2 (frontend) and Step 3-4 (documentation)
