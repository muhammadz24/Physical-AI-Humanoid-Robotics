"""
Neon Postgres Database Connection Management

Provides async connection pooling for Neon Serverless Postgres using asyncpg.
Handles chat session logging and analytics.
"""

import asyncpg
from typing import Optional
from contextlib import asynccontextmanager

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


# Global database manager instance
db_manager = DatabaseManager()
