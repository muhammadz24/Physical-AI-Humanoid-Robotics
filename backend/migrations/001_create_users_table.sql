-- Migration: Create users table for authentication
-- Feature: 006-implement-auth
-- Created: 2025-12-15
--
-- This migration creates the users table to store user authentication data
-- including email/password credentials and experience levels for bonus points.

-- Create users table (idempotent - safe to re-run)
CREATE TABLE IF NOT EXISTS users (
    -- Primary key: UUID for security (prevents enumeration attacks)
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),

    -- User identity and credentials
    name VARCHAR(100) NOT NULL,
    email VARCHAR(255) NOT NULL UNIQUE,
    hashed_password VARCHAR(255) NOT NULL,

    -- Experience levels (PDF compliance - bonus points requirement)
    -- Software experience: beginner, intermediate, pro
    software_experience VARCHAR(20) NOT NULL
        CHECK (software_experience IN ('beginner', 'intermediate', 'pro')),

    -- Hardware experience: none, arduino, ros, professional
    hardware_experience VARCHAR(20) NOT NULL
        CHECK (hardware_experience IN ('none', 'arduino', 'ros', 'professional')),

    -- Timestamps
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Create unique index on email for fast login lookups (idempotent)
CREATE UNIQUE INDEX IF NOT EXISTS idx_users_email ON users(email);

-- Create index on created_at for analytics queries (optional, idempotent)
CREATE INDEX IF NOT EXISTS idx_users_created_at ON users(created_at);

-- Verify table creation
DO $$
BEGIN
    IF EXISTS (
        SELECT FROM information_schema.tables
        WHERE table_schema = 'public'
        AND table_name = 'users'
    ) THEN
        RAISE NOTICE 'Users table created successfully';
    ELSE
        RAISE EXCEPTION 'Users table creation failed';
    END IF;
END $$;
