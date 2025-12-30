print("🚀 STARTUP: Loading backend/main.py...")
import os
import sys
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from backend.app.api.routes import router as chat_router
from backend.app.core.config import settings

try:
    print("🔹 Initializing FastAPI App...")
    app = FastAPI(
        title="Physical AI Chatbot",
        version="1.0.0",
        docs_url="/api/docs",
        openapi_url="/api/openapi.json"
    )

    # CORS Setup (Clean URLs)
    origins = [
        "http://localhost:3000",
        "https://physical-ai-humanoid-robotics.vercel.app",
        "https://physical-ai-humanoid-robotics-mz24.vercel.app",
        "*" # Allow all for debugging if needed
    ]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include Routes
    app.include_router(chat_router, prefix="/api")

    @app.get("/api/health")
    async def health_check():
        return {"status": "ok", "model": settings.gemini_model}

    print("✅ STARTUP: FastAPI App initialized successfully.")

except Exception as e:
    print(f"🔥 STARTUP ERROR in main.py: {str(e)}")
    import traceback
    traceback.print_exc()
    # Re-raise so Vercel logs show the crash
    raise e
