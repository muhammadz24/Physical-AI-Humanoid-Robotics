"""
Vercel Serverless Entry Point for FastAPI Backend

This file serves as the entry point for Vercel's Python serverless functions.
It imports the FastAPI app from backend/main.py and exposes it for Vercel.

CRITICAL ROUTING NOTES:
- vercel.json rewrites /api/* to /api/index.py
- This file imports the FastAPI app which has routes prefixed with /api/
- Final URLs: /api/health, /api/chat, /api/auth/*, /api/personalize
- No duplicate /api prefix needed here - handled in backend/main.py
"""

import sys
import os
from pathlib import Path

# Add project root to Python path for imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

# Import the FastAPI app with error handling
try:
    from backend.main import app
    print(f"✅ [Vercel] Successfully imported FastAPI app from backend.main")
    print(f"✅ [Vercel] Project root: {project_root}")
    print(f"✅ [Vercel] App routes: {[route.path for route in app.routes]}")
except Exception as e:
    import traceback
    error_details = traceback.format_exc()
    print(f"❌ [Vercel] Import Error:\n{error_details}")

    # Create fallback error app
    from fastapi import FastAPI
    from fastapi.responses import JSONResponse

    app = FastAPI()

    @app.get("/{full_path:path}")
    async def error_handler(full_path: str):
        return JSONResponse(
            status_code=500,
            content={
                "error": "Module Import Failed",
                "details": str(e),
                "traceback": error_details,
                "path_attempted": full_path
            }
        )

# Export app for Vercel serverless function handler
