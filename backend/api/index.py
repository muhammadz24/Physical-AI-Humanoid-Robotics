import sys
import os
from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse

# --- PATH SETUP ---
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)

if project_root not in sys.path:
    sys.path.insert(0, project_root)

# --- MAIN IMPORT LOGIC ---
try:
    # Asli backend chalane ki koshish
    from backend.main import app
    
except Exception as e:
    # --- DEBUG MODE (Agar crash ho jaye) ---
    app = FastAPI()

    # YAHAN CHANGE HAI: "OPTIONS" add kiya hai
    @app.api_route("/{path_name:path}", methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"])
    async def catch_all(request: Request, path_name: str):
        return JSONResponse(
            status_code=500,
            content={
                "status": "CRASHED",
                "location": "api/index.py",
                "reason": str(e),  # <-- Yeh humein batayega ke asli masla kya hai
                "hint": "Check Vercel Logs or Env Vars"
            }
        )