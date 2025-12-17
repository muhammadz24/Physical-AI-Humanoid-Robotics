-- ============================================================================
-- Migration 003: Auto-Delete Old Chat Messages
-- ============================================================================
-- Purpose: Cleanup old messages to manage free-tier database limits
-- Created: 2025-12-17
-- Feature: 009-ultimate-fix
-- Constitution: Principle III (Free-Tier Architecture - Neon Postgres limits)
-- ============================================================================

-- Function 1: Delete messages older than 24 hours
-- This helps manage storage on Neon Postgres free tier (3GB transfer/month)
CREATE OR REPLACE FUNCTION delete_old_chat_messages()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER;
BEGIN
    -- Delete messages older than 24 hours
    DELETE FROM chat_messages
    WHERE created_at < NOW() - INTERVAL '24 hours';

    GET DIAGNOSTICS deleted_count = ROW_COUNT;

    RAISE NOTICE 'Deleted % old chat messages (>24 hours)', deleted_count;

    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Function 2: Keep only last 50 messages per session
-- This prevents any single session from consuming too much storage
CREATE OR REPLACE FUNCTION cleanup_session_messages()
RETURNS INTEGER AS $$
DECLARE
    deleted_count INTEGER := 0;
    session_record RECORD;
BEGIN
    -- For each session, delete messages beyond the last 50
    FOR session_record IN
        SELECT DISTINCT session_id
        FROM chat_messages
        WHERE session_id IS NOT NULL
    LOOP
        -- Delete all but the 50 most recent messages for this session
        DELETE FROM chat_messages
        WHERE id IN (
            SELECT id
            FROM chat_messages
            WHERE session_id = session_record.session_id
            ORDER BY created_at DESC
            OFFSET 50
        );

        GET DIAGNOSTICS deleted_count = deleted_count + ROW_COUNT;
    END LOOP;

    RAISE NOTICE 'Deleted % messages (kept last 50 per session)', deleted_count;

    RETURN deleted_count;
END;
$$ LANGUAGE plpgsql;

-- Function 3: Comprehensive cleanup (runs both cleanups)
CREATE OR REPLACE FUNCTION cleanup_chat_messages()
RETURNS TEXT AS $$
DECLARE
    old_deleted INTEGER;
    session_deleted INTEGER;
BEGIN
    -- Run both cleanup functions
    old_deleted := delete_old_chat_messages();
    session_deleted := cleanup_session_messages();

    RETURN format(
        'Cleanup complete: %s old messages deleted, %s messages deleted (session limit)',
        old_deleted, session_deleted
    );
END;
$$ LANGUAGE plpgsql;

-- Add comments for documentation
COMMENT ON FUNCTION delete_old_chat_messages() IS
    'Deletes chat messages older than 24 hours to manage database size';

COMMENT ON FUNCTION cleanup_session_messages() IS
    'Keeps only the last 50 messages per session to prevent storage bloat';

COMMENT ON FUNCTION cleanup_chat_messages() IS
    'Comprehensive cleanup: deletes old messages and enforces session limits';

-- ============================================================================
-- USAGE INSTRUCTIONS
-- ============================================================================
--
-- Manual cleanup (run as needed):
--   SELECT cleanup_chat_messages();
--
-- Delete only old messages:
--   SELECT delete_old_chat_messages();
--
-- Enforce session limits only:
--   SELECT cleanup_session_messages();
--
-- Automated cleanup (requires pg_cron extension - not available on Neon free tier):
--   -- Install pg_cron extension (requires superuser)
--   -- CREATE EXTENSION IF NOT EXISTS pg_cron;
--   --
--   -- Schedule daily cleanup at 3 AM
--   -- SELECT cron.schedule('cleanup-chat-messages', '0 3 * * *', 'SELECT cleanup_chat_messages()');
--
-- Alternative: Use external cron job or Vercel cron to call API endpoint
-- that triggers: await db_manager.execute("SELECT cleanup_chat_messages()")
--
-- ============================================================================
-- Success Criteria: FR-015, FR-016
-- FR-015: Provide migration file for automatic message cleanup
-- FR-016: Migration is idempotent (CREATE OR REPLACE functions)
-- ============================================================================
