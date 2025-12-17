-- ============================================================================
-- Migration 002: Create Chat Messages Table
-- ============================================================================
-- Purpose: Store chat conversation history for RAG chatbot
-- Created: 2025-12-17
-- Feature: 009-ultimate-fix
-- Constitution: Principle V (Minimalism - raw SQL, no ORM)
-- ============================================================================

-- Drop table if exists (for idempotent migrations)
-- CAUTION: This will delete all existing chat messages
-- Comment out if you want to preserve data
-- DROP TABLE IF EXISTS chat_messages CASCADE;

-- Create chat_messages table
CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    user_id INTEGER,
    session_id UUID,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

-- Create indexes for efficient queries
CREATE INDEX IF NOT EXISTS idx_chat_messages_user_id
    ON chat_messages(user_id);

CREATE INDEX IF NOT EXISTS idx_chat_messages_session_id
    ON chat_messages(session_id);

CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at
    ON chat_messages(created_at DESC);

-- Add comments for documentation
COMMENT ON TABLE chat_messages IS
    'Stores chat conversation history between users and the RAG chatbot';

COMMENT ON COLUMN chat_messages.id IS
    'Auto-incrementing primary key';

COMMENT ON COLUMN chat_messages.role IS
    'Message author: user (human), assistant (AI), or system (notifications)';

COMMENT ON COLUMN chat_messages.content IS
    'Message text content (unlimited length)';

COMMENT ON COLUMN chat_messages.user_id IS
    'Foreign key to users table (NULL for anonymous users)';

COMMENT ON COLUMN chat_messages.session_id IS
    'UUID grouping messages from the same conversation session';

COMMENT ON COLUMN chat_messages.created_at IS
    'Timestamp when message was created';

COMMENT ON COLUMN chat_messages.updated_at IS
    'Timestamp when message was last modified';

-- ============================================================================
-- Success Criteria: FR-013, FR-014, FR-016
-- FR-013: Create chat_messages table with required columns
-- FR-014: Provide migration file for chat_messages table
-- FR-016: Migration is idempotent (safe to run multiple times)
-- ============================================================================
