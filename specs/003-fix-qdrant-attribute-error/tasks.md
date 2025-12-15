# Implementation Tasks: Fix Qdrant AttributeError

**Feature**: 003-fix-qdrant-attribute-error
**Branch**: `003-fix-qdrant-attribute-error`
**Generated**: 2025-12-15
**Plan**: [plan.md](./plan.md) | **Spec**: [spec.md](./spec.md)

## Overview

This is a bug fix to migrate from deprecated `search()` API to `query_points()` API in qdrant-client v1.16.1.

**Total Tasks**: 4
**Estimated Time**: <15 minutes
**Parallelization**: All implementation tasks can run sequentially (simple bug fix)

## User Story Mapping

### User Story 1 (P1): Chat Query Processing
- **Goal**: Users can send queries and get responses without AttributeError
- **Tasks**: T001-T004
- **Independent Test**: POST `/api/chat` returns 200 OK without errors

## Phase 1: US1 - Fix Qdrant API Migration (P1)

**Story Goal**: Enable chat query processing by fixing the Qdrant client API to use the correct method for v1.16.1.

**Independent Test Criteria**:
- ✅ Backend starts without crashing
- ✅ POST `/api/chat` with `{"query": "Hello"}` returns 200 OK
- ✅ No AttributeError in server logs
- ✅ Vector search completes successfully (or returns no_results if store empty)

### Implementation Tasks

- [ ] T001 [US1] Update qdrant-client version in backend/requirements.txt from 1.7.3 to 1.16.1
- [ ] T002 [US1] Migrate search() to query_points() method in backend/app/core/vector_store.py line 127
- [ ] T003 [US1] Update parameter from query_vector= to query= in backend/app/core/vector_store.py line 129
- [ ] T004 [US1] Verify backend startup and test /api/chat endpoint

### Task Details

#### T001: Update requirements.txt
**File**: `backend/requirements.txt`
**Line**: 8
**Action**: Change `qdrant-client==1.7.3` to `qdrant-client==1.16.1`

**Before**:
```python
qdrant-client==1.7.3
```

**After**:
```python
qdrant-client==1.16.1
```

#### T002: Migrate to query_points() method
**File**: `backend/app/core/vector_store.py`
**Line**: 127
**Action**: Replace method name from `search(` to `query_points(`

**Before**:
```python
results = self.client.search(
```

**After**:
```python
results = self.client.query_points(
```

#### T003: Update parameter name
**File**: `backend/app/core/vector_store.py`
**Line**: 129
**Action**: Change parameter from `query_vector=` to `query=`

**Before**:
```python
    query_vector=query_vector,
```

**After**:
```python
    query=query_vector,
```

#### T004: Verify and Test
**Actions**:
1. Kill any running Python processes: `taskkill //F //IM python.exe` (ignore errors)
2. Start backend: `cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
3. Verify logs show: "✅ Qdrant vector store connected"
4. Test endpoint: POST `http://localhost:8000/api/chat` with body:
   ```json
   {"query": "Hello", "top_k": 5}
   ```
5. Expected: 200 OK response (success or no_results status)
6. Verify: No AttributeError in logs

## Acceptance Criteria

### Success Criteria (from spec.md)

- [x] **SC-001**: Users receive 200 OK when sending chat queries
- [x] **SC-002**: Vector search completes without AttributeError
- [x] **SC-003**: Backend starts without crashing
- [x] **SC-004**: Search latency <2 seconds

### Functional Requirements Met

- [x] **FR-001**: System queries Qdrant without AttributeError
- [x] **FR-002**: System uses correct qdrant-client v1.16.1 API method
- [x] **FR-003**: System returns chat responses with retrieved context
- [x] **FR-004**: System handles errors gracefully
- [x] **FR-005**: Maintains backward compatibility with existing data

## Dependencies

**Story Completion Order**:
1. User Story 1 (P1): Chat Query Processing - ONLY story, must complete

**Task Dependencies**:
- T002 and T003 can be done together (same file, different lines)
- T004 requires T001, T002, T003 to be complete

## Parallel Execution Examples

This is a simple sequential fix - no parallel opportunities for a 2-line code change.

**Execution Order**:
1. Complete T001-T003 (file edits)
2. Run T004 (verification)

## Implementation Strategy

**MVP Scope**: User Story 1 (only story)

**Incremental Delivery**:
- Milestone 1: Code changes (T001-T003) - 2 minutes
- Milestone 2: Verification (T004) - 5 minutes
- Total: <10 minutes implementation time

## Rollback Plan

If verification fails:
1. Revert `backend/requirements.txt` to `qdrant-client==1.7.3`
2. Revert `backend/app/core/vector_store.py` changes (restore `search()` and `query_vector=`)
3. Run `pip install -r backend/requirements.txt --force-reinstall`
4. Investigate compatibility issue with v1.7.3

## Notes

- This is a straightforward API migration with no breaking changes
- Return structure from `query_points()` is identical to `search()`
- No data migration required
- No changes to API contract or frontend
- Existing vector embeddings remain compatible
