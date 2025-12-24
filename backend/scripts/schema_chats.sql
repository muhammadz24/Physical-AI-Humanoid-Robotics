-- ============================================================================
-- CHATS TABLE MIGRATION
-- ============================================================================
-- Purpose: Store chat history for logged-in users (RAG chatbot interactions)
-- Dependencies: Requires 'users' table to exist
-- Cascading: ON DELETE CASCADE ensures user deletion removes all their chats
-- Created: 2025-12-24
-- ============================================================================

-- Drop table if re-running migration (WARNING: Destroys data)
-- DROP TABLE IF EXISTS chats CASCADE;

-- Create chats table
CREATE TABLE IF NOT EXISTS chats (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    query TEXT NOT NULL,
    response TEXT NOT NULL,
    metadata JSONB DEFAULT '{}',
    created_at TIMESTAMP WITH TIME ZONE DEFAULT NOW()
);

-- Index for faster history retrieval by user
CREATE INDEX IF NOT EXISTS idx_chats_user_id ON chats(user_id);

-- Index for sorting by creation date
CREATE INDEX IF NOT EXISTS idx_chats_created_at ON chats(created_at DESC);

-- ============================================================================
-- VERIFICATION QUERIES
-- ============================================================================
-- Run these after migration to verify:
--
-- 1. Check table exists:
--    SELECT table_name FROM information_schema.tables WHERE table_name = 'chats';
--
-- 2. Verify foreign key constraint:
--    SELECT constraint_name, table_name, constraint_type
--    FROM information_schema.table_constraints
--    WHERE table_name = 'chats' AND constraint_type = 'FOREIGN KEY';
--
-- 3. Verify indexes:
--    SELECT indexname FROM pg_indexes WHERE tablename = 'chats';
-- ============================================================================
