# .claude/ - Claude Code Project Configuration
**Physical AI & Humanoid Robotics Textbook**

This directory contains all Claude Code configuration, agents, commands, and automated skills for the project.

---

## Directory Structure

```
.claude/
├── README.md                    # This file - Master documentation
│
├── agents/                      # Custom Claude Code agents
│   ├── hackathon-judge.md       # Judge perspective evaluator
│   ├── hackathon-spec-guard.md  # SDD workflow enforcer (formerly todo-spec-enforcer)
│   ├── python-cli-specialist.md # Python/FastAPI expert
│   └── robotics-expert.md       # Robotics domain expert (formerly todo-domain-expert)
│
├── commands/                    # SP Framework slash commands
│   ├── sp.adr.md               # Create Architecture Decision Records
│   ├── sp.analyze.md           # Cross-artifact analysis
│   ├── sp.checklist.md         # Generate feature checklists
│   ├── sp.clarify.md           # Ask clarification questions
│   ├── sp.constitution.md      # Create/update project constitution
│   ├── sp.git.commit_pr.md     # Git workflow automation
│   ├── sp.implement.md         # Execute implementation tasks
│   ├── sp.phr.md               # Record Prompt History
│   ├── sp.plan.md              # Generate implementation plans
│   ├── sp.specify.md           # Create feature specifications
│   └── sp.tasks.md             # Generate dependency-ordered tasks
│
├── skills/                      # Automated verification skills (⭐ SINGLE SOURCE OF TRUTH)
│   ├── README.md               # Skills documentation
│   │
│   ├── INFRASTRUCTURE HEALTH (Python)
│   │   ├── verify_vector_density.py     # Qdrant database audit
│   │   ├── gemini_model_autodetect.py   # LLM model selection
│   │   ├── auth_heartbeat.py            # Auth system monitor
│   │   └── translation_integrity.py     # i18n/RTL verification
│   │
│   ├── QA & TESTING (Markdown)
│   │   ├── qa.quality-audit.md          # Quality audit procedures
│   │   ├── qa.requirement-verification.md
│   │   └── qa.test-execution.md
│   │
│   └── GENERATED REPORTS (JSON - auto-created)
│       ├── vector_density_report.json
│       ├── gemini_model_detection.json
│       ├── auth_heartbeat_report.json
│       └── translation_integrity_report.json
│
└── settings.local.json          # Claude Code local settings
```

---

## Agents

### 1. hackathon-spec-guard.md
**Role:** Primary controlling agent and architectural guardian

**Responsibilities:**
- Enforce SDD workflow (Constitution → Specify → Plan → Tasks → Implement)
- Guard free-tier architecture boundaries (Constitution Principle III)
- Ensure technical accuracy and traceability
- Block scope creep and over-engineering
- Coordinate with domain experts and specialists

**When to Use:**
- Starting any new work
- Creating foundational documents (`/sp.constitution`, `/sp.specify`, `/sp.plan`, `/sp.tasks`)
- Reviewing architecture decisions
- Approving implementation before `/sp.implement`

**Formerly:** `todo-spec-enforcer.md` (rebranded for Physical AI & Robotics context)

---

### 2. robotics-expert.md
**Role:** Domain expert for Physical AI, Humanoid Robotics, ROS 2, and pedagogy

**Responsibilities:**
- Define robotics concepts and knowledge structures
- Validate technical accuracy against ROS 2 documentation
- Design learning experiences and progressions
- Define RAG chatbot behavior and scope limits
- Identify technical edge cases and gotchas

**When to Use:**
- Creating textbook content or new chapters
- Validating technical accuracy during `/sp.specify`
- Designing learning exercises and assessments
- Defining RAG chatbot scope and behavior

**Formerly:** `todo-domain-expert.md` (rebranded for Robotics domain)

---

### 3. hackathon-judge.md
**Role:** Evaluates project from hackathon judge perspective

**Responsibilities:**
- Assess against hackathon rubric criteria
- Evaluate demonstrability and presentation quality
- Check for completeness and polish
- Identify areas that impress vs confuse judges

**When to Use:**
- Final project review before submission
- Evaluating feature priorities
- Assessing documentation completeness

---

### 4. python-cli-specialist.md
**Role:** Python/FastAPI technical expert

**Responsibilities:**
- Review backend architecture and patterns
- Validate FastAPI best practices
- Ensure Python code quality
- Guide API design decisions

**When to Use:**
- Backend implementation planning
- Code review for Python modules
- API endpoint design decisions

---

## SP Framework Commands

The SpecKit Plus (SP) framework provides structured workflows for spec-driven development:

### Core Workflow
```bash
/sp.constitution    # Create project constitution (principles & constraints)
/sp.specify         # Create feature specification (requirements)
/sp.plan            # Generate implementation plan (architecture)
/sp.tasks           # Generate dependency-ordered tasks
/sp.implement       # Execute tasks from tasks.md
```

### Supporting Commands
```bash
/sp.adr             # Create Architecture Decision Record
/sp.phr             # Record Prompt History for learning
/sp.analyze         # Cross-artifact consistency analysis
/sp.clarify         # Ask clarifying questions
/sp.checklist       # Generate feature checklist
/sp.git.commit_pr   # Git workflow automation
```

---

## Skills - Automated Verification

All skills are located in `.claude/skills/` (single source of truth).

### Infrastructure Health Skills

#### verify_vector_density.py
**Purpose:** Deep audit of Qdrant vector database

```bash
python .claude/skills/verify_vector_density.py
```

**Checks:**
- Collection existence and accessibility
- Vector density grading (A-F scale)
- Embedding dimension validation (768D for Gemini)
- Data distribution across chapters

---

#### gemini_model_autodetect.py
**Purpose:** Intelligent Gemini model detection and selection

```bash
python .claude/skills/gemini_model_autodetect.py
```

**Features:**
- Auto-fetch available models from Google API
- Prioritize by stability (Flash > Pro > Experimental)
- Test models and measure latency
- Auto-select fastest working model

**Latest Result:**
- Best Model: `gemini-flash-latest`
- Latency: 907ms
- Status: ✅ WORKING

---

#### auth_heartbeat.py
**Purpose:** Comprehensive auth system health monitoring

```bash
python .claude/skills/auth_heartbeat.py
```

**7-Step Health Check:**
1. Neon Postgres connection test
2. Required tables verification (users, chatlogs)
3. Users table schema validation
4. JWT token generation & validation
5. User count statistics
6. ChatLog table functionality
7. Connection pool health

**Latest Result:**
- Health Score: 33/100
- Status: ⚠️ CRITICAL (Database unreachable - localhost)
- JWT System: ✅ WORKING

---

#### translation_integrity.py
**Purpose:** Verify Urdu translation and RTL layout integrity

```bash
python .claude/skills/translation_integrity.py
```

**6-Category Checks:**
1. Urdu translation toggle component detection
2. RTL CSS class verification
3. Docusaurus i18n configuration audit
4. Translation file existence checks
5. Urdu font support validation
6. Sample Urdu content detection

**Expected Status:** Most checks will fail (Phase 3 feature, not yet implemented)

---

## Quick Start

### Run All Health Checks

```bash
# Run infrastructure test suite
python tests/test_infra.py

# Run individual skills
python .claude/skills/verify_vector_density.py
python .claude/skills/gemini_model_autodetect.py
python .claude/skills/auth_heartbeat.py
python .claude/skills/translation_integrity.py
```

### Check Generated Reports

```bash
# List all skill reports
ls -lh .claude/skills/*_report.json

# View specific report (pretty-printed)
cat .claude/skills/auth_heartbeat_report.json | python -m json.tool
```

### Use SP Framework

```bash
# Create new feature specification
/sp.specify urdu-translation-toggle

# Review and plan implementation
/sp.plan

# Generate tasks
/sp.tasks

# Execute implementation
/sp.implement
```

---

## Current System Health (as of last check)

### ✅ WORKING COMPONENTS

1. **Gemini API**
   - Model: `gemini-flash-latest`
   - Latency: 907ms
   - Status: ✅ FULLY OPERATIONAL

2. **Qdrant Cloud**
   - Connection: ✅ SUCCESSFUL
   - Collection: ❌ MISSING (needs creation + ingestion)

3. **JWT Authentication**
   - Token Generation: ✅ WORKING
   - Token Validation: ✅ WORKING

### ❌ BLOCKED COMPONENTS

1. **Neon Postgres Database**
   - Status: ❌ NOT CONFIGURED
   - Issue: DATABASE_URL still pointing to localhost
   - Action Required: Update `backend/.env` with Neon connection string

2. **Qdrant Collection**
   - Status: ❌ MISSING
   - Action Required:
     ```bash
     # Create collection and ingest textbook data
     python backend/scripts/ingest.py
     ```

---

## Environment Variables Checklist

**backend/.env** (Required for 100/100 health score):

```bash
# Google Gemini API
GEMINI_API_KEY=AIza...                          # ✅ SET (working)
GOOGLE_API_KEY=${GEMINI_API_KEY}               # ✅ SET

# Qdrant Vector Database
QDRANT_URL=https://YOUR_CLUSTER.qdrant.io      # ✅ SET (connected)
QDRANT_API_KEY=...                             # ✅ SET (connected)

# Neon Postgres Database
DATABASE_URL=postgresql://...neon.tech/...     # ❌ NEEDS UPDATE (still localhost)

# JWT Auth
JWT_SECRET_KEY=...                             # ✅ SET (working)
```

---

## Troubleshooting

### Skills Fail to Import

**Error:** `ModuleNotFoundError: No module named 'backend'`

**Solution:**
```bash
# Ensure you're running from project root
cd "E:\IT\GIAIC\GIAIC - QUARTER 4\HACKATHONS\Physical AI & Humanoid Robotics"

# Skills add project root to sys.path automatically
python .claude/skills/verify_vector_density.py
```

### Database Connection Failures

**Error:** Auth heartbeat shows "Database Unreachable"

**Solution:**
1. Check `backend/.env` has valid Neon DATABASE_URL
2. Ensure Neon project is active (not suspended)
3. Verify connection string includes `?sslmode=require`

### Qdrant Collection Missing

**Error:** Vector density check shows "Collection Missing"

**Solution:**
```bash
# Run ingestion script to create collection and populate data
cd backend
python scripts/ingest.py
```

---

## Best Practices

### 1. Always Use Skills for Verification

Before deploying, run all health checks:
```bash
python tests/test_infra.py
python .claude/skills/verify_vector_density.py
python .claude/skills/auth_heartbeat.py
```

### 2. Follow SDD Workflow

Don't skip steps:
1. `/sp.constitution` - Establish principles
2. `/sp.specify` - Define requirements
3. `/sp.plan` - Design architecture
4. `/sp.tasks` - Break into tasks
5. `/sp.implement` - Execute

### 3. Document Significant Decisions

Use `/sp.adr` for:
- Technology choices
- Architecture patterns
- Trade-off decisions
- Scope changes

### 4. Maintain Traceability

- Every task should reference a spec
- Every code change should reference a task ID
- Every decision should have an ADR

---

## Related Documentation

- **Project Constitution:** `CLAUDE.md` (project root)
- **Infrastructure Tests:** `tests/test_infra.py`
- **Skills Documentation:** `.claude/skills/README.md`
- **Backend README:** `backend/README.md`

---

**Last Updated:** 2026-01-05
**Agents:** 4 (all rebranded from Todo context to Robotics context)
**Skills:** 4 Python + 3 Markdown = 7 total
**Health Status:** See `.claude/skills/*_report.json` for current status

---

**Note:** All "Todo" references have been eliminated and rebranded to Physical AI & Humanoid Robotics context as of 2026-01-05.
