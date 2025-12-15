---
id: 002
title: Plan Qdrant API Migration
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: 003-fix-qdrant-attribute-error
branch: 003-fix-qdrant-attribute-error
user: MUHAMMAD
command: /sp.plan
labels: ["plan", "qdrant", "api-migration", "research"]
links:
  spec: specs/003-fix-qdrant-attribute-error/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/003-fix-qdrant-attribute-error/plan.md
 - specs/003-fix-qdrant-attribute-error/research.md
tests:
 - (none - planning stage)
---

## Prompt

$ARGUMENTS

WORKFLOW INSTRUCTIONS:

1. EXECUTE: `sp spec` (completed in PHR 001)
2. EXECUTE: `sp plan`
   - Action 1: Run `pip show qdrant-client` to check version.
   - Action 2: Read `backend/app/services/vector_store.py` (or where `search` is called).
   - Action 3: Plan the syntax correction.

GOAL: Create implementation plan for fixing the Qdrant AttributeError.

## Response snapshot

Completed comprehensive implementation planning with Phase 0 research:

**Key Discovery - Root Cause Identified**:
- The `search()` method was deprecated in qdrant-client v1.13.0
- Completely removed in v1.16.0
- Installed version (v1.16.1) requires `query_points()` API
- This explains the AttributeError: 'QdrantClient' object has no attribute 'search'

**Research Findings** (documented in research.md):
- Verified API changes through official Qdrant documentation and changelogs
- Old API: `client.search(query_vector=...)`
- New API: `client.query_points(query=...)`
- Migration requires 2-line code change:
  1. Method name: `search()` → `query_points()`
  2. Parameter: `query_vector` → `query`
- Return structure unchanged (backward compatible)

**Implementation Plan** (documented in plan.md):

1. **Migrate vector_store.py to new API**:
   - Line 127: Change method from `search()` to `query_points()`
   - Line 129: Change parameter from `query_vector=` to `query=`

2. **Update requirements.txt**:
   - Line 8: Update `qdrant-client==1.7.3` to `qdrant-client==1.16.1`

3. **Testing strategy**:
   - Integration test via POST `/api/chat`
   - Verify no AttributeError
   - Verify 200 OK responses

**Constitution Check**: PASSED ✅
- No new technologies
- Free-tier compliant
- Fast iteration (2-line fix)
- Zero deployment config changes

**Complexity Assessment**:
- Implementation: LOW (2 lines of code)
- Risk: LOW (direct API replacement)
- Testing: Simple (existing endpoint)
- Estimated time: <10 minutes

## Outcome

- ✅ Impact: Root cause identified via research; migration plan documented with specific line numbers
- 🧪 Tests: Integration testing strategy defined for /api/chat endpoint
- 📁 Files: plan.md (196 lines), research.md (comprehensive API analysis)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks
- 🧠 Reflection: Web research was critical - discovered API removal that wasn't obvious from version mismatch alone

## Evaluation notes (flywheel)

- Failure modes observed: None - research and planning completed successfully
- Graders run and results (PASS/FAIL): Constitution check PASSED (all gates satisfied)
- Prompt variant (if applicable): Standard /sp.plan workflow with Phase 0 research
- Next experiment (smallest change to try): Execute implementation via /sp.tasks, then test backend restart
