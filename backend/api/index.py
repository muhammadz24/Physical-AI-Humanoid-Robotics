import sys
import os
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

# Vercel environment path setup
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)

if project_root not in sys.path:
    sys.path.insert(0, project_root)

try:
    # Asli App import karne ki koshish
    from backend.main import app
except Exception as e:
    # --- DEBUG MODE (Agar asli app fail ho jaye) ---
    app = FastAPI()

    # Ab yeh GET aur POST dono ko handle karega
    @app.api_route("/{path_name:path}", methods=["GET", "POST", "PUT", "DELETE"])
    async def catch_all(path_name: str, request: Request):
        return JSONResponse(
            status_code=500,
            content={
                "error": "Backend Loading Failed",
                "reason": str(e), # Yeh batayega ke asli wajah kya hai
                "type": type(e).__name__,
                "hint": "Check Vercel Logs. Likely missing Env Vars (DB/API Key) or missing requirements."
            }
        )