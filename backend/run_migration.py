"""
Temporary script to run database migrations.

This script reads and executes the SQL migration file to create the users table
in the Neon Postgres database.

Usage:
    python run_migration.py
"""

import asyncio
import asyncpg
import os
from pathlib import Path


async def run_migration():
    """Execute the users table creation migration."""
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Get database URL from environment
    database_url = os.getenv("DATABASE_URL")
    if not database_url:
        print("ERROR: DATABASE_URL not found in environment variables")
        print("   Make sure backend/.env file exists with DATABASE_URL set")
        return False

    # Read migration SQL file
    migration_file = Path(__file__).parent / "migrations" / "001_create_users_table.sql"
    if not migration_file.exists():
        print(f"ERROR: Migration file not found: {migration_file}")
        return False

    with open(migration_file, "r", encoding="utf-8") as f:
        migration_sql = f.read()

    print(f"Reading migration: {migration_file.name}")
    print(f"Connecting to database: {database_url[:50]}...")

    try:
        # Connect to database
        conn = await asyncpg.connect(dsn=database_url)
        print("SUCCESS: Connected to database")

        # Execute migration SQL
        print("Executing migration...")
        await conn.execute(migration_sql)

        # Verify table was created
        table_exists = await conn.fetchval("""
            SELECT EXISTS (
                SELECT FROM information_schema.tables
                WHERE table_schema = 'public'
                AND table_name = 'users'
            );
        """)

        if table_exists:
            print("SUCCESS: Users table created successfully")

            # Get table info
            columns = await conn.fetch("""
                SELECT column_name, data_type, is_nullable
                FROM information_schema.columns
                WHERE table_name = 'users'
                ORDER BY ordinal_position;
            """)

            print("\nUsers table schema:")
            for col in columns:
                nullable = "NULL" if col['is_nullable'] == 'YES' else "NOT NULL"
                print(f"   - {col['column_name']}: {col['data_type']} {nullable}")

            # Get constraints
            constraints = await conn.fetch("""
                SELECT constraint_name, constraint_type
                FROM information_schema.table_constraints
                WHERE table_name = 'users';
            """)

            print("\nConstraints:")
            for const in constraints:
                print(f"   - {const['constraint_name']}: {const['constraint_type']}")

            result = True
        else:
            print("ERROR: Users table was not created")
            result = False

        # Close connection
        await conn.close()
        print("\nDatabase connection closed")

        return result

    except Exception as e:
        print(f"ERROR: Migration failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    print("=" * 70)
    print("DATABASE MIGRATION: Create Users Table")
    print("=" * 70)
    print()

    success = asyncio.run(run_migration())

    print()
    print("=" * 70)
    if success:
        print("SUCCESS: Migration completed successfully")
    else:
        print("ERROR: Migration failed")
    print("=" * 70)
