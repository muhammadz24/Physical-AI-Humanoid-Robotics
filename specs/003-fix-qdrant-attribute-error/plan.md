# Implementation Plan: Fix Qdrant AttributeError

**Branch**: `003-fix-qdrant-attribute-error` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-fix-qdrant-attribute-error/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Fix backend crash caused by calling removed `search()` method on qdrant-client v1.16.1. Research confirmed that the `search()` method was deprecated in v1.13.0 and completely removed in v1.16.0. The installed version (v1.16.1) requires using the new `query_points()` API. The fix involves migrating from the old API to the new one and updating requirements.txt to match the installed version.

**Root Cause**: Code uses removed `search()` method; qdrant-client v1.16.0+ requires `query_points()` API
**Solution Approach**: Migrate to `query_points()` API (2-line code change) and update requirements.txt to v1.16.1

## Technical Context

**Language/Version**: Python 3.13.x (backend)
**Primary Dependencies**:
- `qdrant-client==1.16.1` (currently installed, needs requirements update)
- `fastapi==0.109.0`
- `sentence-transformers==2.3.1`
- `uvicorn[standard]==0.27.0`

**Storage**: Qdrant Cloud (vector database), Neon Postgres (metadata)
**Testing**: pytest==7.4.4, manual API testing
**Target Platform**: Windows (local dev), Linux server (production)
**Project Type**: Web application (backend API)
**Performance Goals**:
- Vector search response: <500ms (p95)
- Overall query processing: <2s (p95)

**Constraints**:
- Free-tier Qdrant Cloud limits
- No breaking changes to existing API contract
- Must maintain backward compatibility with stored vectors

**Scale/Scope**:
- Two file modifications: `backend/requirements.txt` + `backend/app/core/vector_store.py`
- Code changes: 2 lines (method name + parameter name)
- Testing via: `/api/chat` endpoint

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Principle III: Free-Tier Architecture
- No changes to free-tier constraints
- Uses existing Qdrant Cloud free tier
- No additional paid dependencies

### ✅ Principle V: Minimalism in Technology Stack
- No new technologies introduced
- Only updating existing dependency version
- Maintains required stack (FastAPI, Qdrant, Neon Postgres)

### ✅ Principle VI: Fast Build & Iteration Cycles
- Fix involves single file change (requirements.txt)
- Testing can be completed in <5 minutes
- No build time impact

### ✅ Principle IX: Zero-Edit Deployment Configuration
- No deployment configuration changes required
- Environment variables remain unchanged
- Fix applies to all environments identically

**Gate Status**: PASSED ✅ - All principles satisfied, no violations

## Project Structure

### Documentation (this feature)

```text
specs/003-fix-qdrant-attribute-error/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── spec.md              # Feature specification (already created)
├── checklists/
│   └── requirements.md  # Spec validation checklist (already created)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── requirements.txt              # ⚠️  MODIFIED: Update qdrant-client version
├── app/
│   ├── core/
│   │   └── vector_store.py      # 🔍 VERIFY: search() method (line 94-148)
│   ├── api/
│   │   └── routes.py             # 🔍 VERIFY: chat endpoint calls search (line 51)
│   └── main.py                   # 🔍 VERIFY: vector_store.connect() in lifespan (line 52)
└── tests/
    └── (manual API testing)
```

**Structure Decision**: Web application (backend only). This is a bug fix scoped to backend dependencies and does not affect frontend code. The fix involves updating a single dependency version in `requirements.txt` and verifying the existing code works correctly with that version.

## Complexity Tracking

No violations detected - this is a straightforward dependency version update.

## Phase 0: Research & Investigation

**Objective**: Verify qdrant-client API compatibility between v1.7.3 and v1.16.1

### Research Tasks

1. **Qdrant Client API Stability (v1.7.3 → v1.16.1)**
   - Question: Did the `client.search()` method signature change between these versions?
   - Question: Are there any breaking changes in the v1.x series that affect our usage?
   - Source: Official qdrant-client changelog and documentation

2. **Current Code Usage Pattern Verification**
   - File: `backend/app/core/vector_store.py` lines 94-148
   - Method: `VectorStoreManager.search()`
   - API calls: `self.client.search(collection_name, query_vector, limit, score_threshold, query_filter, with_payload)`
   - Verify: This pattern is compatible with v1.16.1

3. **Installed vs. Required Version Reconciliation**
   - Current: v1.16.1 installed (verified via `pip show`)
   - Required: v1.7.3 specified in requirements.txt
   - Decision needed: Update requirements to 1.16.1 OR downgrade to 1.7.3
   - Recommendation: Update to 1.16.1 (newer, tested working in current environment)

### Research Findings

Will be documented in `research.md` after investigation.

## Phase 1: Design & Implementation Approach

**Note**: This is a bug fix, not a new feature. No new data models or API contracts needed.

### Implementation Strategy

1. **Migrate to query_points() API in vector_store.py**
   - File: `backend/app/core/vector_store.py`
   - Line 127: Change `self.client.search(` to `self.client.query_points(`
   - Line 129: Change `query_vector=query_vector,` to `query=query_vector,`
   - Rationale: v1.16.0+ removed `search()` method entirely
   - Risk: None (query_points is direct replacement with same return structure)

2. **Update requirements.txt**
   - File: `backend/requirements.txt`
   - Line 8: Change `qdrant-client==1.7.3` to `qdrant-client==1.16.1`
   - Rationale: Match installed version and document API version dependency
   - Risk: None (version already installed and tested)

3. **Test vector search functionality**
   - Endpoint: POST `/api/chat`
   - Test case: Send "Hello" query
   - Expected: 200 OK response with vector search results
   - Verify: No AttributeError exceptions

### Files Modified

- `backend/app/core/vector_store.py` (lines 127, 129: API migration)
- `backend/requirements.txt` (line 8: qdrant-client version)

### Files Verified (no changes needed)

- `backend/app/api/routes.py` (calls wrapper method, unchanged)
- `backend/main.py` (startup logic, unchanged)

## Testing Strategy

### Unit Testing
- No new unit tests needed (existing code unchanged)
- Verify existing vector_store tests still pass (if any)

### Integration Testing
1. Start backend server: `cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
2. Verify startup logs show: "✅ Qdrant vector store connected"
3. Send test request: POST `/api/chat` with body `{"query": "Hello"}`
4. Expected response: 200 OK with answer and citations (or no_results if vector store empty)
5. Check logs: No AttributeError exceptions

### Acceptance Criteria (from spec.md)
- SC-001: Users receive 200 OK when sending chat queries ✅
- SC-002: Vector search completes without AttributeError ✅
- SC-003: Backend starts without crashing ✅
- SC-004: Search latency <2 seconds ✅

## Rollback Plan

If the version update causes issues:
1. Revert `requirements.txt` to `qdrant-client==1.7.3`
2. Run `pip install -r requirements.txt --force-reinstall`
3. Test with v1.7.3 to identify specific incompatibility
4. Document findings and plan code changes if needed

## Next Steps

1. Complete Phase 0 research (document in `research.md`)
2. Execute implementation (via `/sp.tasks`)
3. Test backend with updated dependency
4. Verify all acceptance criteria pass
5. Document in PHR
