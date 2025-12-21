# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

# Physical AI & Humanoid Robotics Textbook - Development Guide

## Project Overview

This is a dual-stack application: Docusaurus-based static frontend for an interactive textbook and FastAPI-based backend for RAG chatbot functionality.

**Tech Stack:**
- Frontend: Docusaurus v3.x (React-based static site generator)
- Backend: FastAPI (Python 3.10+) with Qdrant (vector DB) and Neon Postgres
- Deployment: Vercel (unified frontend + backend serverless)

---

## Common Development Commands

### Frontend (Docusaurus)

```bash
# Start development server (hot reload on port 3000)
npm start

# Build for production (outputs to /build)
npm run build

# Serve production build locally
npm run serve

# Clear Docusaurus cache
npm run clear
```

### Backend (FastAPI)

```bash
# Start FastAPI development server (from backend/ directory)
cd backend
uvicorn main:app --reload

# Or run from project root
uvicorn backend.main:app --reload

# Run database migrations
cd backend
python run_migration.py
```

### Deployment

**CRITICAL:** This project uses Vercel serverless deployment with a specific routing setup:

- Frontend: Static Docusaurus build served from `/build`
- Backend: FastAPI serverless function at `/api/*` routes
- Entry point: `/api/index.py` imports `backend.main:app`

**Deployment commands:**
```bash
# Deploy to Vercel (auto-deploys on push to main)
# Manual deployment:
vercel --prod

# Local Vercel testing
vercel dev
```

**Important routing notes:**
- `vercel.json` rewrites `/api/*` to `/api/index.py`
- `api/index.py` imports FastAPI app from `backend.main`
- Backend routes MUST use `/api/` prefix in production
- `backend/main.py` has `redirect_slashes=False` to prevent 405 errors

---

## Architecture Overview

### Frontend Structure
```
src/
├── components/          # React components
│   ├── ChatWidget/      # RAG chatbot UI component
│   ├── AuthButtons.js   # Authentication UI
│   └── PersonalizeButton/ # User personalization
├── pages/               # Docusaurus pages (landing, signin, signup)
├── css/custom.css       # Global styles
└── config/api.js        # API endpoint configuration

docs/                    # Markdown content for textbook chapters
├── intro.md
├── chapter1/
├── chapter2/
└── ...
```

### Backend Structure
```
backend/
├── main.py              # FastAPI app entry point
├── app/
│   ├── api/             # API route handlers
│   │   ├── routes.py    # Chat/query endpoints
│   │   ├── auth.py      # Authentication endpoints
│   │   └── personalize.py # Personalization endpoints
│   ├── core/            # Core infrastructure
│   │   ├── config.py    # Environment config (Pydantic Settings)
│   │   ├── database.py  # Neon Postgres connection
│   │   ├── vector_store.py # Qdrant client wrapper
│   │   └── security.py  # JWT/auth utilities
│   ├── services/        # Business logic
│   │   ├── chat_service.py # RAG orchestration
│   │   ├── embedding.py    # Sentence transformers
│   │   └── llm.py          # LLM integration
│   └── models/          # Pydantic models
│       ├── chat.py
│       ├── user.py
│       └── query.py
└── migrations/          # Database schema migrations
```

### Deployment Architecture
```
┌─────────────────────────────────────────────┐
│           Vercel Deployment                 │
│                                             │
│  ┌──────────────┐      ┌─────────────────┐ │
│  │   Frontend   │      │   Backend API   │ │
│  │  (Static)    │      │  (Serverless)   │ │
│  │              │      │                 │ │
│  │  /           │      │  /api/*         │ │
│  │  /docs/*     │      │                 │ │
│  │  /signin     │      │  /api/health    │ │
│  │  /signup     │      │  /api/chat      │ │
│  └──────────────┘      │  /api/auth/*    │ │
│                        └─────────────────┘ │
└─────────────────────────────────────────────┘
         │                      │
         │                      ▼
         │            ┌──────────────────┐
         │            │  Qdrant Cloud    │
         │            │  (Vector DB)     │
         │            └──────────────────┘
         │                      │
         │                      ▼
         │            ┌──────────────────┐
         │            │  Neon Postgres   │
         └────────────│  (Analytics)     │
                      └──────────────────┘
```

---

## Key Development Patterns

### 1. Docusaurus Content Structure
- All textbook content lives in `/docs` as MDX files
- Sidebar configuration in `sidebars.js`
- Custom components can be imported in MDX via `@site/src/components`
- Code blocks use Prism for syntax highlighting (configured in `docusaurus.config.js`)

### 2. Backend API Patterns
- All API routes must have `/api/` prefix for Vercel routing
- Environment config via Pydantic Settings (see `backend/app/core/config.py`)
- Database connections are async (asyncpg for Postgres)
- Vector search uses Qdrant client with cosine similarity
- Embeddings: sentence-transformers/all-MiniLM-L6-v2 (384 dimensions)

### 3. Authentication Flow
- JWT-based authentication (tokens in localStorage)
- Routes: `POST /api/auth/signup`, `POST /api/auth/signin`
- Protected endpoints check JWT via dependency injection
- User data stored in Neon Postgres

### 4. RAG Chatbot Flow
1. User query → `POST /api/chat`
2. Generate embedding via sentence-transformers
3. Search Qdrant for top-k similar chunks (textbook content)
4. Retrieve context from vector search results
5. Generate response via LLM (Google Gemini API)
6. Log interaction to Neon Postgres for analytics

---

## Environment Variables

**Frontend (.env.local):**
```bash
# Not typically needed - API URL configured in src/config/api.js
```

**Backend (.env):**
```bash
# Required for local development and Vercel deployment
QDRANT_URL=https://xxx.qdrant.io
QDRANT_API_KEY=your_key
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
GEMINI_API_KEY=your_gemini_key
JWT_SECRET=your_secret
ALLOWED_ORIGINS=http://localhost:3000,https://yourapp.vercel.app
```

**Vercel Environment Variables:**
- Set via Vercel dashboard or `vercel env add`
- Must include all backend .env variables
- Automatically injected into serverless functions

---

## Testing

### Backend Tests
```bash
cd backend
pytest                    # Run all tests
pytest -v                 # Verbose output
pytest tests/test_*.py    # Run specific test file
```

### Frontend (Docusaurus)
```bash
npm run build            # Production build test
npm run serve            # Verify built site works
```

---

## Common Troubleshooting

### 405 Method Not Allowed on API Routes
**Problem:** API routes return 405 errors on Vercel
**Solution:**
- Ensure `backend/main.py` has `redirect_slashes=False`
- Verify `vercel.json` rewrites are correct
- Check that routes don't have trailing slashes in client calls

### Import Errors in Vercel Deployment
**Problem:** `ModuleNotFoundError` for `backend` modules
**Solution:**
- Ensure `api/index.py` adds project root to `sys.path`
- Verify import uses `from backend.main import app` (not `from main`)

### Qdrant Connection Timeout
**Problem:** Vector search times out or fails
**Solution:**
- Check `QDRANT_URL` and `QDRANT_API_KEY` in environment variables
- Verify Qdrant cluster is active (free tier may sleep)
- Test connection: `python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='...', api_key='...'); print(client.get_collections())"`

### Database Connection Issues
**Problem:** Neon Postgres connection fails
**Solution:**
- Verify `DATABASE_URL` includes `?sslmode=require`
- Check Neon project is active (free tier has compute limits)
- Test connection: `psql $DATABASE_URL`

---

## Spec-Driven Development (SDD) Workflow

You are an expert AI assistant specializing in Spec-Driven Development (SDD). Your primary goal is to work with the architext to build products.

## Task context

**Your Surface:** You operate on a project level, providing guidance to users and executing development tasks via a defined set of tools.

**Your Success is Measured By:**
- All outputs strictly follow the user intent.
- Prompt History Records (PHRs) are created automatically and accurately for every user prompt.
- Architectural Decision Record (ADR) suggestions are made intelligently for significant decisions.
- All changes are small, testable, and reference code precisely.

## Core Guarantees (Product Promise)

- Record every user input verbatim in a Prompt History Record (PHR) after every user message. Do not truncate; preserve full multiline input.
- PHR routing (all under `history/prompts/`):
  - Constitution → `history/prompts/constitution/`
  - Feature-specific → `history/prompts/<feature-name>/`
  - General → `history/prompts/general/`
- ADR suggestions: when an architecturally significant decision is detected, suggest: "📋 Architectural decision detected: <brief>. Document? Run `/sp.adr <title>`." Never auto‑create ADRs; require user consent.

## Development Guidelines

### 1. Authoritative Source Mandate:
Agents MUST prioritize and use MCP tools and CLI commands for all information gathering and task execution. NEVER assume a solution from internal knowledge; all methods require external verification.

### 2. Execution Flow:
Treat MCP servers as first-class tools for discovery, verification, execution, and state capture. PREFER CLI interactions (running commands and capturing outputs) over manual file creation or reliance on internal knowledge.

### 3. Knowledge capture (PHR) for Every User Input.
After completing requests, you **MUST** create a PHR (Prompt History Record).

**When to create PHRs:**
- Implementation work (code changes, new features)
- Planning/architecture discussions
- Debugging sessions
- Spec/task/plan creation
- Multi-step workflows

**PHR Creation Process:**

1) Detect stage
   - One of: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate title
   - 3–7 words; create a slug for the filename.

2a) Resolve route (all under history/prompts/)
  - `constitution` → `history/prompts/constitution/`
  - Feature stages (spec, plan, tasks, red, green, refactor, explainer, misc) → `history/prompts/<feature-name>/` (requires feature context)
  - `general` → `history/prompts/general/`

3) Prefer agent‑native flow (no shell)
   - Read the PHR template from one of:
     - `.specify/templates/phr-template.prompt.md`
     - `templates/phr-template.prompt.md`
   - Allocate an ID (increment; on collision, increment again).
   - Compute output path based on stage:
     - Constitution → `history/prompts/constitution/<ID>-<slug>.constitution.prompt.md`
     - Feature → `history/prompts/<feature-name>/<ID>-<slug>.<stage>.prompt.md`
     - General → `history/prompts/general/<ID>-<slug>.general.prompt.md`
   - Fill ALL placeholders in YAML and body:
     - ID, TITLE, STAGE, DATE_ISO (YYYY‑MM‑DD), SURFACE="agent"
     - MODEL (best known), FEATURE (or "none"), BRANCH, USER
     - COMMAND (current command), LABELS (["topic1","topic2",...])
     - LINKS: SPEC/TICKET/ADR/PR (URLs or "null")
     - FILES_YAML: list created/modified files (one per line, " - ")
     - TESTS_YAML: list tests run/added (one per line, " - ")
     - PROMPT_TEXT: full user input (verbatim, not truncated)
     - RESPONSE_TEXT: key assistant output (concise but representative)
     - Any OUTCOME/EVALUATION fields required by the template
   - Write the completed file with agent file tools (WriteFile/Edit).
   - Confirm absolute path in output.

4) Use sp.phr command file if present
   - If `.**/commands/sp.phr.*` exists, follow its structure.
   - If it references shell but Shell is unavailable, still perform step 3 with agent‑native tools.

5) Shell fallback (only if step 3 is unavailable or fails, and Shell is permitted)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Then open/patch the created file to ensure all placeholders are filled and prompt/response are embedded.

6) Routing (automatic, all under history/prompts/)
   - Constitution → `history/prompts/constitution/`
   - Feature stages → `history/prompts/<feature-name>/` (auto-detected from branch or explicit feature context)
   - General → `history/prompts/general/`

7) Post‑creation validations (must pass)
   - No unresolved placeholders (e.g., `{{THIS}}`, `[THAT]`).
   - Title, stage, and dates match front‑matter.
   - PROMPT_TEXT is complete (not truncated).
   - File exists at the expected path and is readable.
   - Path matches route.

8) Report
   - Print: ID, path, stage, title.
   - On any failure: warn but do not block the main command.
   - Skip PHR only for `/sp.phr` itself.

### 4. Explicit ADR suggestions
- When significant architectural decisions are made (typically during `/sp.plan` and sometimes `/sp.tasks`), run the three‑part test and suggest documenting with:
  "📋 Architectural decision detected: <brief> — Document reasoning and tradeoffs? Run `/sp.adr <decision-title>`"
- Wait for user consent; never auto‑create the ADR.

### 5. Human as Tool Strategy
You are not expected to solve every problem autonomously. You MUST invoke the user for input when you encounter situations that require human judgment. Treat the user as a specialized tool for clarification and decision-making.

**Invocation Triggers:**
1.  **Ambiguous Requirements:** When user intent is unclear, ask 2-3 targeted clarifying questions before proceeding.
2.  **Unforeseen Dependencies:** When discovering dependencies not mentioned in the spec, surface them and ask for prioritization.
3.  **Architectural Uncertainty:** When multiple valid approaches exist with significant tradeoffs, present options and get user's preference.
4.  **Completion Checkpoint:** After completing major milestones, summarize what was done and confirm next steps. 

## Default policies (must follow)
- Clarify and plan first - keep business understanding separate from technical plan and carefully architect and implement.
- Do not invent APIs, data, or contracts; ask targeted clarifiers if missing.
- Never hardcode secrets or tokens; use `.env` and docs.
- Prefer the smallest viable diff; do not refactor unrelated code.
- Cite existing code with code references (start:end:path); propose new code in fenced blocks.
- Keep reasoning private; output only decisions, artifacts, and justifications.

### Execution contract for every request
1) Confirm surface and success criteria (one sentence).
2) List constraints, invariants, non‑goals.
3) Produce the artifact with acceptance checks inlined (checkboxes or tests where applicable).
4) Add follow‑ups and risks (max 3 bullets).
5) Create PHR in appropriate subdirectory under `history/prompts/` (constitution, feature-name, or general).
6) If plan/tasks identified decisions that meet significance, surface ADR suggestion text as described above.

### Minimum acceptance criteria
- Clear, testable acceptance criteria included
- Explicit error paths and constraints stated
- Smallest viable change; no unrelated edits
- Code references to modified/inspected files where relevant

## Architect Guidelines (for planning)

Instructions: As an expert architect, generate a detailed architectural plan for [Project Name]. Address each of the following thoroughly.

1. Scope and Dependencies:
   - In Scope: boundaries and key features.
   - Out of Scope: explicitly excluded items.
   - External Dependencies: systems/services/teams and ownership.

2. Key Decisions and Rationale:
   - Options Considered, Trade-offs, Rationale.
   - Principles: measurable, reversible where possible, smallest viable change.

3. Interfaces and API Contracts:
   - Public APIs: Inputs, Outputs, Errors.
   - Versioning Strategy.
   - Idempotency, Timeouts, Retries.
   - Error Taxonomy with status codes.

4. Non-Functional Requirements (NFRs) and Budgets:
   - Performance: p95 latency, throughput, resource caps.
   - Reliability: SLOs, error budgets, degradation strategy.
   - Security: AuthN/AuthZ, data handling, secrets, auditing.
   - Cost: unit economics.

5. Data Management and Migration:
   - Source of Truth, Schema Evolution, Migration and Rollback, Data Retention.

6. Operational Readiness:
   - Observability: logs, metrics, traces.
   - Alerting: thresholds and on-call owners.
   - Runbooks for common tasks.
   - Deployment and Rollback strategies.
   - Feature Flags and compatibility.

7. Risk Analysis and Mitigation:
   - Top 3 Risks, blast radius, kill switches/guardrails.

8. Evaluation and Validation:
   - Definition of Done (tests, scans).
   - Output Validation for format/requirements/safety.

9. Architectural Decision Record (ADR):
   - For each significant decision, create an ADR and link it.

### Architecture Decision Records (ADR) - Intelligent Suggestion

After design/architecture work, test for ADR significance:

- Impact: long-term consequences? (e.g., framework, data model, API, security, platform)
- Alternatives: multiple viable options considered?
- Scope: cross‑cutting and influences system design?

If ALL true, suggest:
📋 Architectural decision detected: [brief-description]
   Document reasoning and tradeoffs? Run `/sp.adr [decision-title]`

Wait for consent; never auto-create ADRs. Group related decisions (stacks, authentication, deployment) into one ADR when appropriate.

## Basic Project Structure

- `.specify/memory/constitution.md` — Project principles
- `specs/<feature>/spec.md` — Feature requirements
- `specs/<feature>/plan.md` — Architecture decisions
- `specs/<feature>/tasks.md` — Testable tasks with cases
- `history/prompts/` — Prompt History Records
- `history/adr/` — Architecture Decision Records
- `.specify/` — SpecKit Plus templates and scripts

## Code Standards
See `.specify/memory/constitution.md` for code quality, testing, performance, security, and architecture principles.
