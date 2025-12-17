# Research: Advanced Chatbot with History Management

**Feature**: 008-advanced-chatbot
**Date**: 2025-12-16
**Phase**: 0 (Research & Discovery)

## Overview

This document captures technical research and decisions made during the planning phase for advanced chatbot functionality with history management.

## Research Areas

### 1. sessionStorage vs localStorage for Guest History

**Question**: Which browser storage API should we use for guest chat history?

**Research Findings**:
- **sessionStorage**: Persists only for browser tab session, cleared on tab close
- **localStorage**: Persists indefinitely across sessions, cleared only manually
- **Spec Requirement**: Guest history must "survive refresh, clear on tab close"

**Decision**: Use sessionStorage

**Rationale**: Matches spec requirement exactly - persists across page refreshes within same tab, automatically clears when user closes tab (privacy-friendly)

**Alternatives Considered**:
- localStorage: Rejected - would persist across sessions, violating privacy expectation for guests
- IndexedDB: Rejected - over-engineered for simple key-value storage
- Cookies: Rejected - size limits too restrictive, sent with every request (wasteful)

---

### 2. Guest→User History Synchronization Strategy

**Question**: How to safely merge guest sessionStorage into user database on login?

**Research Findings**:
- **Atomic transaction required**: Must be all-or-nothing to prevent data corruption
- **Session ID tracking**: Need to identify which messages belong to same conversation
- **Timestamp preservation**: Original message timestamps must be maintained
- **Error handling**: If sync fails, guest messages must remain in sessionStorage

**Decision**: Use database transaction with rollback on failure

**Implementation**:
```python
async def sync_guest_messages(user_id: UUID, guest_messages: List[dict], db_pool):
    async with db_pool.acquire() as conn:
        async with conn.transaction():  # Atomic transaction
            for msg in guest_messages:
                await conn.execute("""
                    INSERT INTO chat_messages (user_id, role, content, timestamp)
                    VALUES ($1, $2, $3, $4)
                """, user_id, msg['role'], msg['content'], msg['timestamp'])
    # Only clear sessionStorage if transaction succeeds
```

**Rationale**: Database transactions ensure atomicity - either all messages sync or none do, preventing partial/corrupted sync

**Alternatives Considered**:
- Optimistic sync (no transaction): Rejected - risk of partial sync leaving inconsistent state
- Frontend-only storage: Rejected - doesn't meet "permanent storage" requirement for authenticated users

---

### 3. Database Indexing for Efficient History Queries

**Question**: How to optimize database queries for chat history retrieval?

**Research Findings**:
- **Primary query pattern**: Retrieve all messages for a user_id, ordered by timestamp
- **Secondary pattern**: Delete old messages (auto-delete job)
- **Expected volume**: 100-1000 messages per active user

**Decision**: Create composite index on (user_id, timestamp)

**Implementation**:
```sql
CREATE INDEX idx_chat_messages_user_timestamp
ON chat_messages (user_id, timestamp DESC);
```

**Rationale**: Supports both history retrieval (WHERE user_id = X ORDER BY timestamp) and auto-delete (WHERE user_id = X AND timestamp < Y) efficiently

**Performance Impact**: Query time O(log N) instead of O(N) for full table scan

**Alternatives Considered**:
- Single-column index on user_id: Rejected - requires secondary sort on timestamp
- Single-column index on timestamp: Rejected - doesn't filter by user efficiently
- No index: Rejected - unacceptable performance for large tables

---

### 4. Background Job Scheduling for Auto-Delete

**Question**: What scheduler should we use for daily auto-delete cleanup job?

**Research Findings**:
- **APScheduler**: Lightweight, in-process Python scheduler
- **Celery**: Distributed task queue, requires message broker (Redis/RabbitMQ)
- **Cron/SystemD**: OS-level schedulers

**Decision**: Use APScheduler for POC, recommend Celery for production

**Rationale**:
- APScheduler: Simple integration, no extra dependencies, good for single-server deployment
- Celery: Better for production (separate worker processes, failure recovery), but adds complexity

**Implementation (APScheduler)**:
```python
from apscheduler.schedulers.asyncio import AsyncIOScheduler

scheduler = AsyncIOScheduler()

@scheduler.scheduled_job('cron', hour=2)  # Run daily at 2 AM UTC
async def cleanup_old_messages():
    # Query users with auto_delete_enabled
    # Delete messages older than auto_delete_days
    pass

scheduler.start()
```

**Production Migration Path**: Replace APScheduler with Celery when scaling to multiple backend instances

**Alternatives Considered**:
- SystemD timer: Rejected - requires OS-level config, less portable
- Manual cron job: Rejected - harder to deploy consistently
- Celery immediately: Rejected - over-engineering for POC, adds Redis dependency

---

### 5. Chat Message Data Model Design

**Question**: What fields and structure should the chat_messages table have?

**Research Findings**:
- **Core fields**: user_id (nullable for guests), role (user|assistant), content, timestamp
- **Session tracking**: session_id for grouping related messages (future threading)
- **Metadata**: created_at for auditing

**Decision**: Minimal schema with session_id for future extensibility

**Schema**:
```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,  -- NULL for guests
    session_id UUID,  -- Optional: for future conversation threading
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),
    created_at TIMESTAMP NOT NULL DEFAULT NOW()
);
```

**Rationale**:
- user_id nullable allows storing guest messages temporarily (if needed for sync debugging)
- session_id future-proofs for conversation threading
- created_at separate from timestamp allows preserving original message time during sync

**Alternatives Considered**:
- Separate tables for guest vs user messages: Rejected - adds complexity, sync harder
- Store full conversation context: Rejected - redundant with retrieval system, wastes storage
- JSONB metadata column: Rejected - no current use case, adds query complexity

---

### 6. UI Component Architecture

**Question**: How to structure React components for the overhauled chat interface?

**Research Findings**:
- **Component hierarchy**: ChatWidget → ChatHistory → ChatMessage
- **State management**: React hooks (useState, useEffect) sufficient, no Redux needed
- **Scroll management**: Auto-scroll to bottom, scroll-to-top button when scrolled up

**Decision**: Component-based architecture with custom hooks

**Structure**:
```
ChatWidget/
├── index.js              # Main container, orchestrates components
├── ChatHistory.js        # Message list, scroll management
├── ChatMessage.js        # Individual message bubble (user vs assistant styling)
├── ChatInput.js          # Message input with send button
├── TypingIndicator.js    # Animated dots during LLM response
├── useSessionStorage.js  # Hook for guest storage operations
├── useChatHistory.js     # Hook for user database operations
└── ChatWidget.module.css # Scoped styles
```

**Rationale**:
- Separation of concerns (display vs state management)
- Custom hooks isolate storage logic, easier to test
- Small, focused components easier to maintain

**Alternatives Considered**:
- Monolithic single component: Rejected - hard to test and maintain
- Redux for state: Rejected - overkill for local chat state
- Context API for chat state: Rejected - only needed for auth, not chat messages

---

### 7. Rate Limiting and Spam Prevention

**Question**: How to prevent users from spamming chat messages?

**Research Findings**:
- **Attack vector**: Users could flood backend with requests, exhaust Gemini quota
- **User expectation**: Normal users send 1-5 messages per minute
- **Rate limit recommendation**: 10 messages per minute per user

**Decision**: Implement rate limiting on POST /api/chat endpoint

**Implementation**: Use SlowAPI or FastAPI-Limiter
```python
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@app.post("/api/chat")
@limiter.limit("10/minute")
async def chat(request: Request, message: ChatRequest):
    # Handle chat message
    pass
```

**Rationale**: Protects Gemini quota, prevents abuse, doesn't impact normal users

**Alternatives Considered**:
- No rate limiting: Rejected - vulnerable to abuse
- Per-user limit (requires auth): Considered for future, but per-IP sufficient for POC
- Captcha: Rejected - poor UX, not needed if rate limit is reasonable

---

## Technology Stack Decisions

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| Guest Storage | sessionStorage API | Native | Built-in browser API, no library needed |
| User Storage | Neon Postgres | Free tier | Existing database, already integrated |
| Backend Framework | FastAPI | 0.109.0 | Existing infrastructure |
| Frontend | React + Docusaurus | v3 | Existing infrastructure |
| State Management | React Hooks | Built-in | Sufficient for local chat state |
| Background Jobs | APScheduler | 3.x | Lightweight, in-process scheduler |
| Rate Limiting | SlowAPI | Latest | Simple FastAPI integration |

**New Dependencies**:
- APScheduler (backend) - for auto-delete job
- SlowAPI (backend) - for rate limiting

Both are lightweight and align with Principle V (Minimalism).

---

## Open Questions (Resolved)

1. **Q**: Should we implement conversation threading (multiple chat sessions)?
   **A**: Out of scope for POC - add session_id field for future support, but don't implement threading yet

2. **Q**: Should we store message edit history?
   **A**: No - not in spec, adds complexity

3. **Q**: Should we implement real-time multi-device sync?
   **A**: Out of scope - history syncs on page load only

4. **Q**: Should we compress old messages for storage efficiency?
   **A**: No - adds complexity, free tier storage sufficient for POC

---

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| sessionStorage quota exceeded (5-10MB) | Limit guest history to last 100 messages, show warning |
| Database storage grows unbounded | Implement auto-delete, encourage users to enable it |
| Sync failure loses guest history | Keep guest messages in sessionStorage until confirmed |
| Auto-delete job fails | Idempotent design, logs errors, retries next run |
| User accidentally deletes all history | Confirmation dialog with clear warning |
| Rate limit blocks legitimate users | 10/min limit is generous, show clear error message |

---

## Next Steps

1. Proceed to Phase 1: Design (data models, API contracts)
2. Create chat_messages table schema
3. Write OpenAPI spec for history endpoints
4. Generate tasks.md for implementation

---

## References

- [MDN sessionStorage](https://developer.mozilla.org/en-US/docs/Web/API/Window/sessionStorage)
- [PostgreSQL Transactions](https://www.postgresql.org/docs/current/tutorial-transactions.html)
- [FastAPI Background Tasks](https://fastapi.tiangolo.com/tutorial/background-tasks/)
- [APScheduler Documentation](https://apscheduler.readthedocs.io/)
