"""
Neon Postgres Database Connection Management

Provides async connection pooling for Neon Serverless Postgres using asyncpg.
Handles chat session logging and analytics.
"""

import asyncpg
from typing import Optional
from contextlib import asynccontextmanager
from pathlib import Path

from backend.app.core.config import settings


class DatabaseManager:
    """
    Manages async connection pool to Neon Postgres database.

    Uses asyncpg for high-performance async PostgreSQL operations.
    Connection pool is initialized on startup and closed on shutdown.
    """

    def __init__(self):
        self.pool: Optional[asyncpg.Pool] = None
        self._is_connected: bool = False

    async def connect(self) -> None:
        """
        Initialize connection pool to Neon Postgres.

        Pool configuration:
        - min_size: 2 connections (always available)
        - max_size: 10 connections (free tier limit)
        - timeout: 30 seconds
        """
        if self.pool is not None:
            return

        try:
            self._pool = await asyncpg.create_pool(
                dsn=settings.database_url,
                min_size=1,
                max_size=5,
                command_timeout=60,
                ssl='require',
                server_settings={
                    'application_name': 'fastapi_vercel',
                    'statement_timeout': '60000'
                }
            )
            self.pool = self._pool
            self._is_connected = True
            print("[OK] Database connection pool established")
        except Exception as e:
            self._is_connected = False
            print(f"[ERROR] Database connection failed: {e}")
            raise

    async def disconnect(self) -> None:
        """Close all connections in the pool."""
        if self.pool is not None:
            await self.pool.close()
            self.pool = None
            self._is_connected = False
            print("[DISCONNECTED] Database connection pool closed")

    @asynccontextmanager
    async def get_connection(self):
        """
        Context manager to acquire a connection from the pool.

        Usage:
            async with db_manager.get_connection() as conn:
                result = await conn.fetchrow("SELECT 1")
        """
        if self.pool is None:
            raise RuntimeError("Database pool not initialized. Call connect() first.")

        async with self.pool.acquire() as connection:
            yield connection

    async def execute(self, query: str, *args) -> str:
        """Execute a query without returning results (INSERT, UPDATE, DELETE)."""
        async with self.get_connection() as conn:
            return await conn.execute(query, *args)

    async def fetchrow(self, query: str, *args):
        """Fetch a single row from a query."""
        async with self.get_connection() as conn:
            return await conn.fetchrow(query, *args)

    async def fetch(self, query: str, *args):
        """Fetch all rows from a query."""
        async with self.get_connection() as conn:
            return await conn.fetch(query, *args)

    async def health_check(self) -> bool:
        """
        Check if database connection is healthy.

        Returns:
            bool: True if database is reachable, False otherwise
        """
        if not self._is_connected or self.pool is None:
            return False

        try:
            async with self.get_connection() as conn:
                result = await conn.fetchval("SELECT 1")
                return result == 1
        except Exception as e:
            print(f"Database health check failed: {e}")
            return False

    async def init_db(self) -> None:
        """
        Initialize database schema by running migrations.

        CRITICAL: This must be called on application startup to ensure
        all required tables exist in Neon Postgres.

        Runs migrations in order:
        1. 001_create_users_table.sql - Creates users table with bonus fields
        """
        if self.pool is None:
            print("[ERROR] Cannot initialize database: No connection pool")
            return

        print("[DB_INIT] Running database migrations...")

        # Get migrations directory
        migrations_dir = Path(__file__).parent.parent.parent / "migrations"

        # Migration files in order
        migration_files = [
            "001_create_users_table.sql"
        ]

        try:
            async with self.get_connection() as conn:
                for migration_file in migration_files:
                    migration_path = migrations_dir / migration_file

                    if not migration_path.exists():
                        print(f"[WARN] Migration file not found: {migration_file}")
                        continue

                    print(f"[DB_INIT] Executing migration: {migration_file}")

                    # Read migration SQL
                    with open(migration_path, "r", encoding="utf-8") as f:
                        migration_sql = f.read()

                    # Execute migration (idempotent - uses CREATE TABLE IF NOT EXISTS)
                    await conn.execute(migration_sql)
                    print(f"[OK] Migration completed: {migration_file}")

                # Verify users table exists
                table_exists = await conn.fetchval("""
                    SELECT EXISTS (
                        SELECT FROM information_schema.tables
                        WHERE table_schema = 'public'
                        AND table_name = 'users'
                    );
                """)

                if table_exists:
                    print("[OK] Database schema initialized successfully")
                else:
                    print("[ERROR] Users table not found after migration")

        except Exception as e:
            print(f"[ERROR] Database initialization failed: {e}")
            import traceback
            traceback.print_exc()
            # Don't raise - allow app to start even if migrations fail
            # This prevents cascading failures in Vercel


# Global database manager instance
db_manager = DatabaseManager()


async def init_db():
    """
    Public function to initialize database.

    Called from FastAPI lifespan context manager.
    """
    await db_manager.connect()
    await db_manager.init_db()
