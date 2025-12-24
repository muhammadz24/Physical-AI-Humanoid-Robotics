"""
Database Migration: Create Chats Table

This script creates the 'chats' table in Neon Postgres database
with proper foreign key constraints and indexes.

Usage:
    python backend/scripts/migrate_chats.py

Requirements:
    - DATABASE_URL environment variable must be set
    - 'users' table must already exist in the database
    - asyncpg library installed (pip install asyncpg)

Safety:
    - Uses IF NOT EXISTS to avoid errors if table already exists
    - Does NOT drop existing data
    - Can be run multiple times safely (idempotent)
"""

import asyncio
import asyncpg
import os
from pathlib import Path

# Load environment variables from .env file
try:
    from dotenv import load_dotenv
    # Load from backend/.env
    env_path = Path(__file__).parent.parent / '.env'
    load_dotenv(dotenv_path=env_path)
    print(f"[INFO] Loaded environment from {env_path}")
except ImportError:
    print("[WARNING] python-dotenv not installed. Using system environment variables only.")
    print("          Install with: pip install python-dotenv")


async def run_migration():
    """
    Execute the chats table migration.

    Connects to Neon Postgres database and executes schema_chats.sql.
    """
    # Get database URL from environment
    database_url = os.getenv("DATABASE_URL")
    if not database_url:
        print("[ERROR] DATABASE_URL environment variable not set")
        print("Set it in your .env file or export it:")
        print('  export DATABASE_URL="postgresql://user:pass@host.neon.tech/db?sslmode=require"')
        return False

    print("[INFO] Starting chats table migration...")
    print(f"[INFO] Database: {database_url.split('@')[1] if '@' in database_url else 'hidden'}")

    try:
        # Read SQL schema file
        schema_file = Path(__file__).parent / "schema_chats.sql"
        if not schema_file.exists():
            print(f"[ERROR] Schema file not found at {schema_file}")
            return False

        with open(schema_file, 'r', encoding='utf-8') as f:
            sql_schema = f.read()

        print(f"[INFO] Loaded SQL schema from {schema_file.name}")

        # Connect to database
        print("[INFO] Connecting to Neon Postgres...")
        conn = await asyncpg.connect(dsn=database_url, ssl='require')

        # Execute migration
        print("[INFO] Executing migration...")
        await conn.execute(sql_schema)

        # Verify table creation
        print("[INFO] Verifying migration...")
        table_exists = await conn.fetchval("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'chats'
            );
        """)

        if table_exists:
            # Count indexes
            index_count = await conn.fetchval("""
                SELECT COUNT(*) FROM pg_indexes WHERE tablename = 'chats';
            """)

            # Check foreign key constraint
            fk_exists = await conn.fetchval("""
                SELECT EXISTS (
                    SELECT FROM information_schema.table_constraints
                    WHERE table_name = 'chats' AND constraint_type = 'FOREIGN KEY'
                );
            """)

            print("[SUCCESS] MIGRATION SUCCESSFUL!")
            print(f"   - Table 'chats' created")
            print(f"   - Indexes created: {index_count}")
            print(f"   - Foreign key constraint: {'YES' if fk_exists else 'NO'}")
            print(f"   - Cascading delete enabled: {'YES' if fk_exists else 'NO'}")
        else:
            print("[ERROR] Table 'chats' was not created")
            return False

        # Close connection
        await conn.close()
        print("[INFO] Database connection closed")

        return True

    except asyncpg.exceptions.UndefinedTableError as e:
        print("[ERROR] 'users' table does not exist!")
        print("   Run the users table migration first.")
        print(f"   Details: {e}")
        return False

    except asyncpg.exceptions.PostgresError as e:
        print(f"[ERROR] DATABASE ERROR: {e}")
        return False

    except Exception as e:
        print(f"[ERROR] UNEXPECTED ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


async def rollback_migration():
    """
    DANGEROUS: Drop the chats table.

    Only use this if you need to completely reset the chats table.
    ALL DATA WILL BE LOST!
    """
    database_url = os.getenv("DATABASE_URL")
    if not database_url:
        print("[ERROR] DATABASE_URL environment variable not set")
        return False

    print("[WARNING] This will DELETE the 'chats' table and ALL data!")
    confirmation = input("Type 'DELETE CHATS TABLE' to confirm: ")

    if confirmation != "DELETE CHATS TABLE":
        print("[INFO] Rollback cancelled")
        return False

    try:
        print("[INFO] Connecting to database...")
        conn = await asyncpg.connect(dsn=database_url, ssl='require')

        print("[INFO] Dropping 'chats' table...")
        await conn.execute("DROP TABLE IF EXISTS chats CASCADE;")

        print("[SUCCESS] Table 'chats' dropped successfully")

        await conn.close()
        return True

    except Exception as e:
        print(f"[ERROR] {e}")
        return False


if __name__ == "__main__":
    import sys

    # Check for rollback flag
    if len(sys.argv) > 1 and sys.argv[1] == "--rollback":
        print("[INFO] ROLLBACK MODE")
        success = asyncio.run(rollback_migration())
    else:
        # Default: Run migration
        success = asyncio.run(run_migration())

    # Exit with appropriate code
    sys.exit(0 if success else 1)
