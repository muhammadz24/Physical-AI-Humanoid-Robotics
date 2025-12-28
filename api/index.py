from fastapi import FastAPI
import os
import sys

app = FastAPI()

@app.get("/api/health")
def health():
    # Return debug info to verify environment
    return {
        "status": "probe_active",
        "message": "Routing is working!",
        "current_directory": os.getcwd(),
        "directory_contents": os.listdir("."),
        "backend_exists": os.path.exists("backend"),
        "sys_path": sys.path
    }

# Vercel needs this
from fastapi import Request
@app.get("/{full_path:path}")
async def catch_all(full_path: str):
    return {"message": f"Caught path: {full_path}"}
