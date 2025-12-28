import sys
import os
from fastapi import FastAPI
from fastapi.responses import JSONResponse

# 1. Add root to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    # 2. Try to import the real app
    from backend.main import app
except Exception as e:
    # 3. IF IT CRASHES: Create a fallback app to show the error
    import traceback
    error_trace = traceback.format_exc()

    app = FastAPI()

    @app.get("/api/health")
    @app.get("/api/{path:path}")
    def crash_report(path: str = ""):
        return JSONResponse(
            status_code=500,
            content={
                "status": "startup_error",
                "message": "The Backend crashed while starting.",
                "error": str(e),
                "traceback": error_trace.split("\n"),
                "hint": "Check Vercel Environment Variables (GEMINI_API_KEY, DATABASE_URL, etc)."
            }
        )
