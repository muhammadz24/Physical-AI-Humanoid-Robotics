# Data Model: Ultimate Fix Release

**Feature**: 009-ultimate-fix
**Date**: 2025-12-17
**Purpose**: Define database schema changes and data entities

## Overview

This feature primarily involves infrastructure, configuration, and code refactoring. The only new data entity is the **chat_messages** table for persisting conversation history.

## Entities

### 1. Chat Message

**Purpose**: Store user and assistant messages for chat history persistence

**Schema**:

```sql
CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    session_id UUID,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id ON chat_messages(session_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at ON chat_messages(created_at DESC);
```

**Fields**:
- `id` (SERIAL): Auto-incrementing primary key
- `role` (VARCHAR): Message author - 'user', 'assistant', or 'system'
- `content` (TEXT): Message text content (unlimited length)
- `user_id` (INTEGER): Foreign key to users table (optional, NULL for anonymous users)
- `session_id` (UUID): Groups messages from the same conversation session
- `created_at` (TIMESTAMP): When message was created
- `updated_at` (TIMESTAMP): Last modification timestamp

**Validation Rules**:
- `role` must be one of: 'user', 'assistant', 'system'
- `content` cannot be empty (enforced at application level)
- `user_id` must reference valid user if provided
- `created_at` defaults to current timestamp

**Relationships**:
- Many-to-One with `users` table (if authentication is enabled)
- Grouped by `session_id` for conversation threads

**State Transitions**: None (messages are append-only)

---

### 2. Environment Variable Configuration

**Purpose**: Store application configuration outside codebase

These are not database entities but configuration values stored as environment variables:

**Backend (.env)**:
```bash
# Database
DATABASE_URL=postgresql://user:password@host:5432/dbname

# API Keys (examples)
OPENAI_API_KEY=sk-...
QDRANT_API_KEY=...

# CORS
ALLOWED_ORIGIN=https://yourapp.vercel.app

# Optional
DEBUG=false
PORT=8000
```

**Frontend (.env.local for local dev)**:
```bash
NEXT_PUBLIC_API_URL=http://localhost:8000
```

**Frontend (Vercel environment variables for production)**:
```bash
NEXT_PUBLIC_API_URL=https://yourapp.vercel.app/api
```

**Validation**:
- Backend validates required variables at startup (DATABASE_URL)
- Frontend uses fallback for missing NEXT_PUBLIC_API_URL
- Documented in `.env.example` file

---

### 3. Vercel Configuration

**Purpose**: Define routing and deployment settings

**File**: `vercel.json` (root directory)

```json
{
  "version": 2,
  "builds": [
    {
      "src": "backend/api/index.py",
      "use": "@vercel/python"
    }
  ],
  "rewrites": [
    {
      "source": "/api/(.*)",
      "destination": "/backend/api/index.py"
    }
  ],
  "env": {
    "DATABASE_URL": "@database-url",
    "ALLOWED_ORIGIN": "@allowed-origin"
  }
}
```

**Structure**:
- `builds`: Specifies Python serverless function for backend
- `rewrites`: Routes `/api/*` requests to FastAPI app
- `env`: References Vercel environment variables (set in dashboard)

---

## Migration Files

### Migration 002: Create chat_messages Table

**File**: `backend/app/db/migrations/002_create_chat_messages.sql`

```sql
-- Migration 002: Chat Messages Table
-- Purpose: Store chat conversation history
-- Date: 2025-12-17

CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    session_id UUID,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_chat_messages_user_id
    ON chat_messages(user_id);

CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
    ON chat_messages(session_id);

CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at
    ON chat_messages(created_at DESC);

-- Comments for documentation
COMMENT ON TABLE chat_messages IS 'Stores chat conversation history';
COMMENT ON COLUMN chat_messages.role IS 'Message author: user, assistant, or system';
COMMENT ON COLUMN chat_messages.content IS 'Message text content';
COMMENT ON COLUMN chat_messages.user_id IS 'Foreign key to users table (NULL for anonymous)';
COMMENT ON COLUMN chat_messages.session_id IS 'Groups messages from same conversation';
```

### Migration 003: Auto-Delete Old Messages

**File**: `backend/app/db/migrations/003_add_auto_delete.sql`

```sql
-- Migration 003: Auto-Delete Old Chat Messages
-- Purpose: Cleanup messages older than 30 days for data retention
-- Date: 2025-12-17

-- Function to delete old messages
CREATE OR REPLACE FUNCTION delete_old_chat_messages()
RETURNS void AS $$
BEGIN
    DELETE FROM chat_messages
    WHERE created_at < NOW() - INTERVAL '30 days';

    RAISE NOTICE 'Deleted old chat messages (>30 days)';
END;
$$ LANGUAGE plpgsql;

-- Optionally create a manual cleanup procedure
-- Note: Automatic scheduling requires pg_cron extension or external cron
-- For Neon Postgres free tier, run this manually or via API cron job

COMMENT ON FUNCTION delete_old_chat_messages()
    IS 'Deletes chat messages older than 30 days';
```

**Idempotency**: Both migrations use `IF NOT EXISTS` and `CREATE OR REPLACE` to be safely re-runnable.

---

## Data Flow Diagrams

### Chat Message Persistence Flow

```
User types message in frontend
    ↓
Frontend sends POST /api/chat
    ↓
Backend receives request
    ↓
Save user message to chat_messages table
    ↓
Generate AI response
    ↓
Save assistant message to chat_messages table
    ↓
Return response to frontend
    ↓
Frontend displays message
```

### Environment Variable Loading Flow

```
Application startup
    ↓
Load .env file (local) or environment variables (production)
    ↓
Validate required variables (DATABASE_URL, etc.)
    ↓
    ├─ If missing: Raise error with clear message → Exit
    └─ If present: Initialize services → Continue
```

---

## Storage Estimates

### Chat Messages Table

**Assumptions**:
- Average message length: 200 characters
- Messages per user per session: 20
- Active users per day: 100
- Retention period: 30 days

**Calculation**:
```
Per message: ~250 bytes (content + metadata)
Per session: 20 messages × 250 bytes = 5 KB
Per day: 100 users × 5 KB = 500 KB
Per month: 500 KB × 30 days = 15 MB
```

**Neon Postgres Free Tier**: 10 GB storage → Chat messages will use <1% of storage

---

## Constraints and Trade-offs

### Trade-offs

| Decision | Benefit | Cost |
|----------|---------|------|
| Store all messages | Complete history, better UX | Storage grows over time |
| 30-day retention | Manages storage, privacy-friendly | Lose old conversations |
| Session-based grouping | Efficient conversation retrieval | Requires session ID management |
| Optional user_id | Works for anonymous users | Harder to track per-user history without login |

### Constraints

- **Neon Postgres Free Tier**: 10 GB storage, 3 GB data transfer/month
- **No real-time sync**: Messages fetched on page load, not live updates
- **No encryption at rest**: Neon provides this, but not explicitly configured in schema
- **No message editing**: Append-only design (no UPDATE operations)

---

## Summary

This feature adds one primary data entity (chat_messages table) with two supporting migrations. The design prioritizes simplicity, efficient querying, and alignment with free-tier constraints. No complex relationships or state machines are required.

**Ready for**: Contract generation and quickstart guide creation.
