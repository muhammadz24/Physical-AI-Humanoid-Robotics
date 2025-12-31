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
        openapi_url="/api/openapi.json",
        redirect_slashes=False  # CRITICAL for Vercel
    )

    # CORS Setup (Clean URLs)
    origins = [
        "http://localhost:3000",
        "https://physical-ai-humanoid-robotics.vercel.app",
        "https://physical-ai-humanoid-robotics-mz24.vercel.app",
        "*"  # Allow all for debugging
    ]

    app.add_middleware(
        CORSMiddleware,
        allow_origins=origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # CORRECT ROUTING: Resource-level prefix ONLY
    # Vercel rewrite adds /api context → Final URL: /api/chat
    app.include_router(chat_router, prefix="/chat", tags=["chat"])
    print("✅ Router registered: /chat (Final URL: /api/chat)")

    @app.get("/health")
    async def health_check():
        return {
            "status": "ok",
            "model": settings.gemini_model,
            "qdrant_url": settings.qdrant_url[:20] + "..." if settings.qdrant_url else "NOT SET",
            "database_url": "SET" if settings.database_url else "NOT SET"
        }

    print("✅ STARTUP: FastAPI App initialized successfully.")

except Exception as e:
    print(f"🔥 STARTUP ERROR in main.py: {str(e)}")
    import traceback
    traceback.print_exc()
    # Re-raise so Vercel logs show the crash
    raise e
