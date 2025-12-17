# Data Model: Advanced Chatbot with History Management

**Feature**: 008-advanced-chatbot
**Date**: 2025-12-16
**Phase**: 1 (Design)

## Overview

This document defines the data structures and models used in the advanced chatbot history management feature.

## Database Schema

### chat_messages Table

**Purpose**: Store chat conversation history for authenticated users

**File**: `backend/migrations/002_create_chat_messages.sql`

```sql
-- Migration: Create chat_messages table for history management
-- Feature: 008-advanced-chatbot
-- Created: 2025-12-16

CREATE TABLE IF NOT EXISTS chat_messages (
    -- Primary key
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

    -- User reference (NULL for guest messages during sync)
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,

    -- Optional session grouping for future conversation threading
    session_id UUID,

    -- Message metadata
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL CHECK (length(content) > 0 AND length(content) <= 10000),

    -- Timestamps
    timestamp TIMESTAMP NOT NULL DEFAULT NOW(),  -- Original message time
    created_at TIMESTAMP NOT NULL DEFAULT NOW()  -- Database insert time
);

-- Composite index for efficient user history queries
CREATE INDEX idx_chat_messages_user_timestamp
ON chat_messages (user_id, timestamp DESC);

-- Index for auto-delete queries
CREATE INDEX idx_chat_messages_timestamp
ON chat_messages (timestamp);

-- Index for session-based queries (future use)
CREATE INDEX idx_chat_messages_session
ON chat_messages (session_id);

-- Verify table creation
DO $$
BEGIN
    IF EXISTS (
        SELECT FROM information_schema.tables
        WHERE table_schema = 'public'
        AND table_name = 'chat_messages'
    ) THEN
        RAISE NOTICE 'chat_messages table created successfully';
    ELSE
        RAISE EXCEPTION 'chat_messages table creation failed';
    END IF;
END $$;
```

**Indexing Strategy**:
- Primary index: `(user_id, timestamp DESC)` - supports history retrieval sorted by time
- Secondary index: `(timestamp)` - supports auto-delete job queries
- Future index: `(session_id)` - supports conversation threading (not implemented in POC)

---

### users Table Extension

**Purpose**: Add auto-delete settings to existing users table

**File**: `backend/migrations/003_add_auto_delete_to_users.sql`

```sql
-- Migration: Add auto-delete settings to users table
-- Feature: 008-advanced-chatbot
-- Created: 2025-12-16

ALTER TABLE users
ADD COLUMN IF NOT EXISTS auto_delete_enabled BOOLEAN DEFAULT FALSE;

ALTER TABLE users
ADD COLUMN IF NOT EXISTS auto_delete_days INTEGER DEFAULT 30
    CHECK (auto_delete_days IN (7, 30, 90));

-- Verify columns added
DO $$
BEGIN
    IF EXISTS (
        SELECT FROM information_schema.columns
        WHERE table_name = 'users'
        AND column_name = 'auto_delete_enabled'
    ) THEN
        RAISE NOTICE 'auto_delete columns added successfully';
    ELSE
        RAISE EXCEPTION 'auto_delete columns addition failed';
    END IF;
END $$;
```

---

## Backend Models (Pydantic)

### ChatMessage

**Purpose**: Represent a single chat message

**File**: `backend/app/models/chat.py`

```python
from pydantic import BaseModel, Field, validator
from typing import Literal, Optional
from datetime import datetime
from uuid import UUID

MessageRole = Literal["user", "assistant"]

class ChatMessage(BaseModel):
    """
    Model for a single chat message.

    Used for both storing in database and returning in API responses.
    """
    id: UUID
    user_id: Optional[UUID] = None  # NULL for guest messages
    session_id: Optional[UUID] = None
    role: MessageRole
    content: str = Field(..., min_length=1, max_length=10000)
    timestamp: datetime
    created_at: datetime

    class Config:
        json_schema_extra = {
            "example": {
                "id": "123e4567-e89b-12d3-a456-426614174000",
                "user_id": "789e4567-e89b-12d3-a456-426614174001",
                "session_id": None,
                "role": "user",
                "content": "What is ROS 2?",
                "timestamp": "2025-12-16T10:30:00Z",
                "created_at": "2025-12-16T10:30:00Z"
            }
        }
```

---

### ChatRequest

**Purpose**: Input model for POST /api/chat endpoint

**File**: `backend/app/models/chat.py`

```python
class ChatRequest(BaseModel):
    """
    Request model for sending a chat message.

    Sent from frontend to backend.
    """
    message: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="User's message to the chatbot"
    )

    @validator('message')
    def validate_not_empty(cls, v):
        if not v.strip():
            raise ValueError("Message cannot be empty or whitespace-only")
        return v.strip()

    class Config:
        json_schema_extra = {
            "example": {
                "message": "Explain what Physical AI is"
            }
        }
```

---

### ChatResponse

**Purpose**: Output model for POST /api/chat endpoint

**File**: `backend/app/models/chat.py`

```python
class ChatResponse(BaseModel):
    """
    Response model for chat message.

    Contains the LLM-generated answer and metadata.
    """
    answer: str = Field(..., description="Chatbot's response")
    confidence: float = Field(..., description="Confidence score 0-1")
    sources: list[str] = Field(default=[], description="Source chapters/sections")
    saved: bool = Field(default=False, description="Whether message was saved to database")

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "Physical AI refers to artificial intelligence systems that interact with the physical world...",
                "confidence": 0.92,
                "sources": ["Chapter 1: Introduction to Physical AI"],
                "saved": True
            }
        }
```

---

### ChatHistoryResponse

**Purpose**: Output model for GET /api/chat/history endpoint

**File**: `backend/app/models/chat.py`

```python
class ChatHistoryResponse(BaseModel):
    """
    Response model for chat history retrieval.

    Returns list of messages for authenticated user.
    """
    messages: list[ChatMessage] = Field(..., description="User's chat history")
    total_count: int = Field(..., description="Total number of messages")
    page: int = Field(default=1, description="Current page number")
    page_size: int = Field(default=100, description="Messages per page")

    class Config:
        json_schema_extra = {
            "example": {
                "messages": [
                    {
                        "id": "123e4567-e89b-12d3-a456-426614174000",
                        "user_id": "789e4567-e89b-12d3-a456-426614174001",
                        "role": "user",
                        "content": "What is ROS 2?",
                        "timestamp": "2025-12-16T10:30:00Z"
                    }
                ],
                "total_count": 42,
                "page": 1,
                "page_size": 100
            }
        }
```

---

### SyncGuestHistoryRequest

**Purpose**: Input model for POST /api/chat/sync endpoint

**File**: `backend/app/models/chat.py`

```python
class GuestMessage(BaseModel):
    """Single guest message for synchronization."""
    role: MessageRole
    content: str = Field(..., min_length=1, max_length=10000)
    timestamp: datetime

class SyncGuestHistoryRequest(BaseModel):
    """
    Request model for syncing guest sessionStorage history to user database.

    Sent from frontend after successful login/signup.
    """
    messages: list[GuestMessage] = Field(
        ...,
        max_items=100,
        description="Guest messages from sessionStorage to sync"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "messages": [
                    {
                        "role": "user",
                        "content": "What is Physical AI?",
                        "timestamp": "2025-12-16T10:25:00Z"
                    },
                    {
                        "role": "assistant",
                        "content": "Physical AI is...",
                        "timestamp": "2025-12-16T10:25:03Z"
                    }
                ]
            }
        }
```

---

### UserSettingsUpdate

**Purpose**: Input model for updating auto-delete settings

**File**: `backend/app/models/user.py` (extend existing)

```python
class UserSettingsUpdate(BaseModel):
    """
    Model for updating user settings.

    Includes auto-delete configuration.
    """
    auto_delete_enabled: Optional[bool] = None
    auto_delete_days: Optional[Literal[7, 30, 90]] = None

    class Config:
        json_schema_extra = {
            "example": {
                "auto_delete_enabled": True,
                "auto_delete_days": 30
            }
        }
```

---

## Frontend Models (TypeScript/JSDoc)

### Message Interface

**Purpose**: TypeScript interface for chat messages

**File**: `frontend/src/components/ChatWidget/types.js`

```javascript
/**
 * @typedef {'user' | 'assistant'} MessageRole
 */

/**
 * @typedef {Object} ChatMessage
 * @property {string} id - Unique message ID
 * @property {MessageRole} role - Who sent the message
 * @property {string} content - Message content
 * @property {string} timestamp - ISO 8601 timestamp
 */

/**
 * @typedef {Object} ChatState
 * @property {ChatMessage[]} messages - All messages in current conversation
 * @property {boolean} isLoading - Whether waiting for LLM response
 * @property {boolean} isAuthenticated - Whether user is logged in
 * @property {string|null} error - Error message if any
 */
```

---

### sessionStorage Format

**Purpose**: Structure for guest chat history in browser storage

**Key**: `chatbot_history_guest`

**Format**:
```json
{
  "messages": [
    {
      "role": "user",
      "content": "What is ROS 2?",
      "timestamp": "2025-12-16T10:30:00.000Z"
    },
    {
      "role": "assistant",
      "content": "ROS 2 is the next generation...",
      "timestamp": "2025-12-16T10:30:03.456Z"
    }
  ],
  "session_id": "abc123-session-uuid",
  "created_at": "2025-12-16T10:25:00.000Z"
}
```

**Size Limit**: ~5-10MB (browser dependent)
**Lifespan**: Current browser tab session only

---

## Data Flow Diagrams

### Guest User Flow

```
1. User (not logged in) opens chatbot
   ↓
2. Frontend checks sessionStorage for key `chatbot_history_guest`
   ↓
3. If found → Load messages into UI
   If not found → Show empty chat
   ↓
4. User sends message
   ↓
5. Frontend adds message to state & sessionStorage
   ↓
6. Backend receives POST /api/chat (no save, just LLM response)
   ↓
7. Frontend adds assistant response to state & sessionStorage
   ↓
8. User refreshes page → Messages restored from sessionStorage
   ↓
9. User closes tab → sessionStorage cleared automatically
```

### Authenticated User Flow

```
1. User (logged in) opens chatbot
   ↓
2. Frontend calls GET /api/chat/history
   ↓
3. Backend queries chat_messages WHERE user_id = {authenticated_user_id}
   ↓
4. Frontend loads history into UI
   ↓
5. User sends message
   ↓
6. Frontend adds message to state
   ↓
7. Backend receives POST /api/chat
   ↓
8. Backend saves message to chat_messages table (user & assistant)
   ↓
9. Backend returns LLM response
   ↓
10. Frontend adds assistant response to state
   ↓
11. User refreshes page → Messages reloaded from database
```

### Guest→User Sync Flow

```
1. Guest user has active conversation in sessionStorage
   ↓
2. User clicks "Sign Up" or "Log In"
   ↓
3. Frontend completes authentication
   ↓
4. AuthProvider triggers sync on successful auth
   ↓
5. Frontend reads sessionStorage `chatbot_history_guest`
   ↓
6. Frontend calls POST /api/chat/sync with guest messages
   ↓
7. Backend starts database transaction
   ↓
8. For each guest message:
      INSERT INTO chat_messages (user_id, role, content, timestamp)
   ↓
9. Transaction commits (all messages saved)
   ↓
10. Frontend clears sessionStorage
   ↓
11. Frontend calls GET /api/chat/history to reload from database
```

---

## Validation Rules

### Backend Validation

**ChatMessage**:
- `content`: 1 ≤ len ≤ 10,000 characters
- `role`: Must be "user" or "assistant"
- `timestamp`: Must be valid ISO 8601 datetime

**SyncGuestHistoryRequest**:
- `messages`: Max 100 messages per sync request
- Each message must pass ChatMessage validation

**UserSettingsUpdate**:
- `auto_delete_days`: Must be 7, 30, or 90 (if provided)

### Frontend Validation

**Pre-Send**:
- Message not empty after trim
- Message length ≤ 1000 characters (user input limit)

**sessionStorage**:
- Total size ≤ 5MB (approximate, show warning at 4MB)
- Max 100 messages stored (oldest messages dropped if exceeded)

---

## Error Models

### Backend Error Responses

```python
class ChatErrorResponse(BaseModel):
    """Standard error response for chat endpoints."""
    detail: str
    error_code: Optional[str] = None
```

**Error Codes**:
- `UNAUTHORIZED`: Missing or invalid authentication
- `INVALID_INPUT`: Message validation failed
- `STORAGE_FULL`: Database quota exceeded (rare)
- `SYNC_FAILED`: Guest history sync transaction failed
- `RATE_LIMITED`: User exceeded message rate limit
- `INTERNAL_ERROR`: Unexpected server error

---

## Performance Considerations

**Database**:
- chat_messages table size: ~1KB per message
- Expected: 100-1000 messages per active user
- Index size: ~10% of table size
- Query performance: O(log N) with indexes

**sessionStorage**:
- Guest history: ~100 messages × ~200 bytes = ~20KB
- Well within 5MB browser limit
- JSON parse/stringify: <10ms overhead

**API Response Times**:
- GET /api/chat/history: <500ms (100 messages)
- POST /api/chat/sync: <1s (50 messages)
- DELETE /api/chat/history: <500ms (async deletion)

---

## Next Steps

1. Implement database migrations (002_create_chat_messages.sql, 003_add_auto_delete_to_users.sql)
2. Create Pydantic models in backend/app/models/chat.py
3. Write API endpoints (GET, POST /api/chat/history, POST /api/chat/sync)
4. Implement frontend hooks (useSessionStorage, useChatHistory)
5. Create ChatWidget components
6. Write tests for sync logic

