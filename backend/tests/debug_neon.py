import os
import asyncio
import asyncpg
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

async def test_neon_connection():
    """Test connection to Neon Postgres database with raw error output for diagnostics."""

    database_url = os.getenv('DATABASE_URL')

    if not database_url:
        print("[ERROR] DATABASE_URL environment variable not set")
        return False

    print("Attempting connection to Neon Postgres with SSL='require'...")

    try:
        # Attempt connection with SSL settings for Neon
        conn = await asyncpg.connect(
            dsn=database_url,
            timeout=10,
            ssl='require',  # Required for Neon Postgres
            command_timeout=10
        )

        print("[SUCCESS] Successfully connected to Neon Postgres")

        # Test basic query to verify connection is functional
        result = await conn.fetchval("SELECT version();")
        print(f"Database version: {result.split()[0:3]}...")

        await conn.close()
        print("[SUCCESS] Connection test completed successfully")
        return True

    except Exception as e:
        # Print the raw error message for diagnostic purposes
        print(f"[ERROR] Raw Error Type: {type(e).__name__}")
        print(f"[ERROR] Raw Error Message: {str(e)}")
        print("[ERROR] Connection failed - check NETWORK (Port 5432 blocked) or CONFIGURATION (SSL mode)")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_neon_connection())
    exit(0 if success else 1)