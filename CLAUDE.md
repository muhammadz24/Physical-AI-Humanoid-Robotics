# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

# Physical AI & Humanoid Robotics Textbook - Development Guide

## Project Overview

This is a dual-stack application: Docusaurus-based static frontend for an interactive textbook and FastAPI-based backend for RAG chatbot functionality.

**Tech Stack:**
- Frontend: Docusaurus v3.x (React-based static site generator)
- Backend: FastAPI (Python 3.12+) with Qdrant (vector DB) and Neon Postgres
- Deployment: Vercel (unified frontend + backend serverless)
- Dependency Management: Flexible version constraints (>= operators) to allow UV/pip to resolve compatible versions

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
# Install backend dependencies
cd backend
pip install -r requirements.txt
# Note: Use flexible version constraints (>=) to allow dependency resolver to find compatible versions

# Start FastAPI development server (from backend/ directory)
cd backend
uvicorn main:app --reload

# Or run from project root
uvicorn backend.main:app --reload

# Run database migrations
cd backend
python run_migration.py

# Populate Qdrant vector database with textbook content
cd backend
python scripts/ingest.py
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

**CRITICAL routing architecture (FINAL - prevents 405 errors):**

```
User request: /api/chat
    ↓
vercel.json rewrites: /api/(.*) → /api/index.py
    ↓
api/index.py imports: from backend.main import app
    ↓
backend/main.py routers: prefix="/chat" (NO /api prefix)
    ↓
Final route: /chat (within app context)
    ↓
Vercel serves at: /api/chat ✅
```

**Key files:**
- `vercel.json`: Single rewrite rule `/api/(.*)` → `/api/index.py`
- `api/index.py`: Clean import of `backend.main:app` (no decorators, no modifications)
- `backend/main.py`: Routers WITHOUT `/api/` prefix:
  - `app.include_router(chat_router, prefix="/chat")`
  - `app.include_router(auth_router, prefix="/auth")`
  - `app.include_router(personalize_router, prefix="/personalize")`
- `backend/main.py`: Has `redirect_slashes=False` (REQUIRED)

**Final URLs:**
- `/api/health`
- `/api/chat`
- `/api/auth/signup`, `/api/auth/signin`
- `/api/personalize`

**Why this architecture:**
- Vercel's rewrite adds `/api` context at the serverless function level
- Adding `/api` prefix in `backend/main.py` causes double-prefix issues
- The app sees routes as `/chat`, `/auth`, etc., but Vercel serves them at `/api/chat`, `/api/auth`
- Frontend MUST call `/api/chat` (no trailing slash)

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
- **Router pattern**: Individual routers define routes WITHOUT any prefix, main.py adds resource prefix:
  ```python
  # In backend/app/api/routes.py
  @router.post("")  # Empty string for exact prefix match

  # In backend/main.py
  app.include_router(chat_router, prefix="/chat")  # NO /api here!
  # Vercel rewrite adds /api context → Final URL: POST /api/chat
  ```
- Routers in `backend/main.py` use resource-level prefix ONLY (e.g., `/chat`, `/auth`)
- The `/api` prefix is added by Vercel's rewrite rule, NOT in Python code
- Environment config via Pydantic Settings (see `backend/app/core/config.py`)
- Database connections are async (asyncpg for Postgres)
- Vector search uses Qdrant client with cosine similarity
- **CRITICAL**: Embeddings use Google Gemini `models/text-embedding-004` (768 dimensions)
  - Config: `backend/app/core/config.py` sets `embedding_dimension = 768`
  - Service: `backend/app/services/embedding.py` uses `genai.embed_content()`
  - Collection: Qdrant collection "textbook" MUST be configured for 768-dim vectors
  - **If you change embedding model or dimensions, you MUST recreate the Qdrant collection and re-ingest data**

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

## Data Ingestion & Vector Database Setup

**CRITICAL**: The RAG chatbot requires textbook content to be ingested into Qdrant before it can answer questions.

### Ingestion Script
Located at: `backend/scripts/ingest.py`

**What it does:**
1. Parses all Markdown files in `/docs` directory
2. Chunks content into semantic blocks
3. Generates 768-dimensional embeddings using Google Gemini API
4. Uploads chunks with metadata to Qdrant collection "textbook"

**When to run:**
- Initial setup (after cloning repo)
- After adding/modifying textbook content in `/docs`
- After changing embedding model or dimensions
- After recreating Qdrant collection

**How to run:**
```bash
cd backend
# Ensure .env has QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY
python scripts/ingest.py
```

**Verification:**
```bash
# Check collection exists and has correct dimensions
python -c "from qdrant_client import QdrantClient; import os; client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY')); print(client.get_collection('textbook'))"
```

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

### 500 Internal Server Error After Deployment
**Problem:** API returns 500 errors after successful deployment
**Common Causes:**
1. **Environment variables not synced**: Check Vercel dashboard for all required env vars
2. **Embedding dimension mismatch**: Qdrant collection configured for wrong dimensions
   - Current config: 768 dimensions (Google Gemini text-embedding-004)
   - Verify with: Check `backend/app/core/config.py:43`
   - If changed: Must recreate Qdrant collection and re-run `backend/scripts/ingest.py`
3. **Missing API keys**: `GEMINI_API_KEY` must be set in Vercel environment
4. **Qdrant collection not populated**: Run ingestion script to populate vector database

**Quick Fix:**
```bash
# Force Vercel rebuild to pick up environment changes
git commit --allow-empty -m "Force rebuild to sync environment"
git push origin main
```

### Dependency Conflicts (LangChain + Python 3.12)
**Problem:** Dependency resolution failures with langchain-core and langchain-google-genai
**Solution:**
- Use flexible version constraints (`>=`) in `requirements.txt` instead of pinned versions (`==`)
- This allows pip/UV to find compatible versions across the dependency tree
- Current working constraints:
  - `langchain-google-genai>=2.0.0`
  - `langchain-core>=0.3.0`
- If you update LangChain packages, maintain flexible constraints to avoid version lock conflicts

---

## Project Workflow & Development Practices

This project follows **Spec-Driven Development (SDD)** with the following key principles:

### Core Practices
- **Specification-First**: Create detailed specs before implementation (see `.specify/memory/constitution.md`)
- **Prompt History Records (PHRs)**: Document AI interactions in `history/prompts/` for traceability
- **Architecture Decision Records (ADRs)**: Document significant architectural choices in `history/adr/`
- **Small, Testable Changes**: Prefer minimal viable diffs over large refactors

### Project Structure
```
.specify/memory/constitution.md  # Project principles and code standards
specs/<feature>/                 # Feature specifications
  ├── spec.md                    # Requirements
  ├── plan.md                    # Architecture decisions
  └── tasks.md                   # Testable tasks
history/
  ├── prompts/                   # AI interaction logs (PHRs)
  └── adr/                       # Architecture decision records
```

### Development Workflow
1. **Planning**: Create spec → plan → tasks before implementation
2. **Implementation**: Make smallest viable changes with code references
3. **Documentation**: Create PHRs for significant interactions; suggest ADRs for architectural decisions
4. **Code Quality**: Follow principles in `.specify/memory/constitution.md`

### Available Skills (Slash Commands)
The project uses SpecKit Plus with the following commands:
- `/sp.plan` - Execute implementation planning workflow
- `/sp.specify` - Create/update feature specifications
- `/sp.tasks` - Generate dependency-ordered task lists
- `/sp.implement` - Execute tasks from tasks.md
- `/sp.adr` - Create Architecture Decision Records
- `/sp.phr` - Record Prompt History (auto-triggered)
- `/sp.constitution` - Create/update project constitution

**Note**: See `.specify/` directory for templates and scripts.
