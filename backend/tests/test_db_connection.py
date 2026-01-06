import asyncio
import asyncpg
import os
from dotenv import load_dotenv

load_dotenv()

async def test_connection():
    try:
        print("Attempting to connect to Neon database...")
        conn = await asyncpg.connect(os.getenv('DATABASE_URL'), timeout=15)
        version = await conn.fetchval('SELECT version()')
        print(f"✅ CONNECTED: {version[:70]}")
        await conn.close()
        return True
    except Exception as e:
        print(f"❌ CONNECTION FAILED: {e}")
        return False

if __name__ == "__main__":
    success = asyncio.run(test_connection())
    exit(0 if success else 1)
