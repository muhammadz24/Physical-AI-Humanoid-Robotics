# Database Migrations Guide

This directory contains database migration scripts for the Physical AI & Humanoid Robotics platform.

---

## 📋 Available Migrations

### 1. **Chats Table Migration** (`migrate_chats.py`)
**Purpose:** Create the `chats` table for storing chat history of logged-in users.

**Schema:**
- **Table:** `chats`
- **Columns:**
  - `id` (UUID, Primary Key)
  - `user_id` (UUID, Foreign Key → `users.id`)
  - `query` (TEXT, User's question)
  - `response` (TEXT, AI's answer)
  - `metadata` (JSONB, Additional context)
  - `created_at` (TIMESTAMP WITH TIME ZONE)
- **Constraints:**
  - Foreign Key: `user_id` references `users(id)` ON DELETE CASCADE
- **Indexes:**
  - `idx_chats_user_id` (for fast user history retrieval)
  - `idx_chats_created_at` (for chronological sorting)

---

## 🚀 Running Migrations

### Prerequisites

1. **Environment Variable:**
   Ensure `DATABASE_URL` is set (Neon Postgres connection string):
   ```bash
   export DATABASE_URL="postgresql://user:password@host.neon.tech/database?sslmode=require"
   ```

2. **Python Dependencies:**
   ```bash
   pip install asyncpg
   ```

3. **Users Table:**
   The `users` table must exist before running the chats migration.

---

### Execute Chats Table Migration

**From project root:**
```bash
python backend/scripts/migrate_chats.py
```

**Expected Output:**
```
🔄 Starting chats table migration...
📍 Database: host.neon.tech/database
📄 Loaded SQL schema from schema_chats.sql
🔌 Connecting to Neon Postgres...
⚙️  Executing migration...
✅ Verifying migration...
✅ MIGRATION SUCCESSFUL!
   - Table 'chats' created
   - Indexes created: 2
   - Foreign key constraint: ✅ YES
   - Cascading delete enabled: ✅ YES
🔌 Database connection closed
```

---

## 🔄 Rollback (DANGEROUS)

**WARNING:** This will delete the `chats` table and ALL data permanently!

```bash
python backend/scripts/migrate_chats.py --rollback
```

You will be prompted to type `DELETE CHATS TABLE` to confirm.

---

## ✅ Verification

After running the migration, verify it worked:

### Method 1: Using the Migration Script
The script automatically verifies after execution. Look for:
- ✅ Table 'chats' created
- ✅ Foreign key constraint: YES
- ✅ Cascading delete enabled: YES

### Method 2: Manual SQL Verification

**Check table exists:**
```sql
SELECT table_name FROM information_schema.tables WHERE table_name = 'chats';
```

**Verify foreign key constraint:**
```sql
SELECT constraint_name, table_name, constraint_type
FROM information_schema.table_constraints
WHERE table_name = 'chats' AND constraint_type = 'FOREIGN KEY';
```

**Verify indexes:**
```sql
SELECT indexname FROM pg_indexes WHERE tablename = 'chats';
```

**Test cascading delete:**
```sql
-- Create test user
INSERT INTO users (name, email, hashed_password, software_experience, hardware_experience)
VALUES ('Test User', 'test@example.com', 'hashed', 'beginner', 'none')
RETURNING id;

-- Create test chat (replace <user_id> with the UUID from above)
INSERT INTO chats (user_id, query, response)
VALUES ('<user_id>', 'Test query', 'Test response');

-- Delete user (should cascade and delete chat)
DELETE FROM users WHERE email = 'test@example.com';

-- Verify chat was deleted
SELECT COUNT(*) FROM chats WHERE user_id = '<user_id>';
-- Should return 0
```

---

## 🛠️ Troubleshooting

### Error: "DATABASE_URL environment variable not set"
**Solution:**
```bash
# Add to .env file
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require

# Or export in terminal
export DATABASE_URL="postgresql://user:pass@host.neon.tech/db?sslmode=require"
```

### Error: "'users' table does not exist"
**Solution:** Run the users table migration first. The chats table has a foreign key dependency on the users table.

### Error: "Table 'chats' already exists"
**Solution:** The migration uses `IF NOT EXISTS`, so this shouldn't happen. If you see this error, the table is already created. To recreate it:
```bash
python backend/scripts/migrate_chats.py --rollback
python backend/scripts/migrate_chats.py
```

### Error: "Connection refused" or "SSL error"
**Solution:**
- Check your Neon Postgres cluster is active (free tier may sleep)
- Verify `?sslmode=require` is in your DATABASE_URL
- Test connection: `psql $DATABASE_URL`

---

## 📂 Migration Files

```
backend/scripts/
├── schema_chats.sql         # SQL schema definition
├── migrate_chats.py         # Python migration runner
└── README_MIGRATIONS.md     # This file
```

---

## 🔐 Security Notes

- **Never commit `DATABASE_URL`** to version control
- Use `.env` file for local development
- Use Vercel environment variables for production
- The `chats` table stores user queries and AI responses (consider privacy implications)
- Cascading delete is intentional: deleting a user account removes all their chat history

---

## 📅 Migration History

| Date | Migration | Description | Status |
|------|-----------|-------------|--------|
| 2025-12-24 | `migrate_chats.py` | Create chats table with foreign key constraints | ✅ Active |

---

## 🔗 Related Documentation

- **Backend API:** `backend/app/api/routes.py` (Chat endpoints)
- **Database Config:** `backend/app/core/database.py`
- **User Service:** `backend/app/services/user_service.py`
- **Vercel Config:** `vercel.json`
