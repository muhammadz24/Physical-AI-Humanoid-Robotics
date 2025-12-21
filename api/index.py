import sys
import os
import traceback
from fastapi import FastAPI, Response

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    # Import FastAPI app from backend.main
    from backend.main import app
    print("[OK] Successfully imported FastAPI app from backend.main")
except Exception as e:
    # Fallback error display for debugging
    error_msg = traceback.format_exc()
    print(f"[ERROR] Failed to import app: {error_msg}")

    # Create dummy app that shows the error
    app = FastAPI()

    @app.api_route("/{path_name:path}", methods=["GET", "POST", "PUT", "DELETE"])
    async def catch_all(path_name: str):
        return Response(
            content=f"🐍 PYTHON IMPORT ERROR 🐍\n\n{error_msg}",
            media_type="text/plain",
            status_code=500
        )

# Vercel will use this 'app' instance as the serverless function
