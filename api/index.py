import sys
import os
import traceback
from fastapi import FastAPI
from fastapi.responses import JSONResponse

# 1. ATTEMPT TO LOAD THE REAL APP
try:
    print("[BOOT] Attempting to import backend.main...")
    # Add project root to sys.path to mimic Vercel environment
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from backend.main import app
    print("[BOOT] backend.main imported successfully.")

except Exception as e:
    # 2. EMERGENCY FALLBACK IF APP CRASHES
    error_trace = traceback.format_exc()
    print(f"[CRITICAL STARTUP ERROR]\n{error_trace}")

    app = FastAPI()

    @app.api_route("/{path_name:path}", methods=["GET", "POST", "PUT", "DELETE", "OPTIONS", "HEAD"])
    async def catch_all_errors(path_name: str):
        return JSONResponse(
            status_code=200, # Return 200 so we can read the error in browser/terminal
            content={
                "status": "CRITICAL_STARTUP_ERROR",
                "message": str(e),
                "error_type": type(e).__name__,
                "traceback": error_trace.split("\n"),
                "working_dir": os.getcwd()
            }
        )
