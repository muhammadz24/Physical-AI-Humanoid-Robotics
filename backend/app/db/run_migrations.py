"""
Database Migration Runner

Executes SQL migration files in order to set up or update the database schema.
Uses asyncpg (existing dependency) to connect to Neon Postgres.

Constitution Compliance:
- Principle V: Minimalism (raw SQL, no heavy ORM like Alembic)
- Principle IX: Zero-Edit Deployment (works with DATABASE_URL env var)

Usage:
    python -m app.db.run_migrations

Environment Variables Required:
    DATABASE_URL - PostgreSQL connection string
"""

import asyncio
import asyncpg
from pathlib import Path
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from app.core.config import settings


async def run_migrations():
    """
    Execute all SQL migration files in the migrations directory.

    Returns:
        int: Number of migrations successfully executed
    """
    migrations_dir = Path(__file__).parent / "migrations"

    # Check if migrations directory exists
    if not migrations_dir.exists():
        print("[ERROR] Migrations directory not found:", migrations_dir)
        return 0

    # Get all .sql files sorted by name (001, 002, 003, etc.)
    migration_files = sorted(migrations_dir.glob("*.sql"))

    if not migration_files:
        print("[WARN] No migration files found in:", migrations_dir)
        return 0

    print(f"[START] Running {len(migration_files)} migration(s)...")
    print(f"[INFO] Database: {settings.database_url.split('@')[-1] if '@' in settings.database_url else 'localhost'}")
    print("-" * 60)

    # Connect to database
    try:
        conn = await asyncpg.connect(settings.database_url)
        print("[OK] Database connection established")
    except Exception as e:
        print(f"[ERROR] Failed to connect to database: {e}")
        print("[INFO] Check DATABASE_URL environment variable")
        return 0

    successful_migrations = 0
    failed_migrations = 0

    try:
        for migration_file in migration_files:
            migration_name = migration_file.name
            print(f"\n[RUNNING] {migration_name}...")

            try:
                # Read SQL file
                sql_content = migration_file.read_text(encoding='utf-8')

                # Execute SQL
                await conn.execute(sql_content)

                print(f"[OK] {migration_name} completed successfully")
                successful_migrations += 1

            except asyncpg.PostgresError as e:
                print(f"[ERROR] {migration_name} failed:")
                print(f"  PostgreSQL Error: {e}")
                print(f"  SQL State: {e.sqlstate if hasattr(e, 'sqlstate') else 'unknown'}")
                failed_migrations += 1

                # Continue with next migration (non-fatal)
                # Comment out the line below to stop on first error
                continue

            except Exception as e:
                print(f"[ERROR] {migration_name} failed with unexpected error:")
                print(f"  {type(e).__name__}: {e}")
                failed_migrations += 1
                continue

    finally:
        # Close database connection
        await conn.close()
        print("\n" + "-" * 60)
        print(f"[DISCONNECTED] Database connection closed")

    # Print summary
    print("\n" + "=" * 60)
    print(f"[SUMMARY] Migration Results:")
    print(f"  ✓ Successful: {successful_migrations}")
    print(f"  ✗ Failed:     {failed_migrations}")
    print(f"  Total:        {len(migration_files)}")
    print("=" * 60)

    return successful_migrations


async def check_tables():
    """
    Verify that migrations created the expected tables.

    Returns:
        bool: True if all expected tables exist
    """
    print("\n[CHECK] Verifying database tables...")

    try:
        conn = await asyncpg.connect(settings.database_url)

        # Check if chat_messages table exists
        result = await conn.fetchval("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_name = 'chat_messages'
            )
        """)

        if result:
            print("[OK] chat_messages table exists")

            # Get table info
            columns = await conn.fetch("""
                SELECT column_name, data_type
                FROM information_schema.columns
                WHERE table_name = 'chat_messages'
                ORDER BY ordinal_position
            """)

            print(f"[INFO] Table structure: {len(columns)} columns")
            for col in columns:
                print(f"  - {col['column_name']}: {col['data_type']}")
        else:
            print("[WARN] chat_messages table does not exist")
            print("[INFO] Run migrations to create the table")

        await conn.close()
        return result

    except Exception as e:
        print(f"[ERROR] Failed to verify tables: {e}")
        return False


async def main():
    """Main entry point for migration runner."""
    print("=" * 60)
    print("Database Migration Runner")
    print("Feature: 009-ultimate-fix")
    print("=" * 60)

    # Check DATABASE_URL
    if not settings.database_url:
        print("\n[ERROR] DATABASE_URL environment variable not set")
        print("[INFO] Set DATABASE_URL in .env file or environment")
        print("[INFO] Example: DATABASE_URL=postgresql://user:pass@host:5432/db")
        sys.exit(1)

    # Run migrations
    successful = await run_migrations()

    if successful > 0:
        # Verify tables were created
        await check_tables()
        print("\n[OK] Migrations completed successfully")
        sys.exit(0)
    else:
        print("\n[ERROR] No migrations were executed")
        sys.exit(1)


if __name__ == "__main__":
    # Run migrations
    asyncio.run(main())
