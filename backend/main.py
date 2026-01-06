print("[STARTUP] Loading backend/main.py...")
import os
import sys
from contextlib import asynccontextmanager

# --- NEW ADDITION: FORCE LOAD .ENV ---
# Ye 2 lines zaroori hain taake local PC par .env file read ho sakay
from dotenv import load_dotenv
load_dotenv()
# -------------------------------------

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from backend.app.api.routes import router as chat_router
from backend.app.api.auth import router as auth_router
from backend.app.core.config import settings
from backend.app.core.database import init_db, db_manager


# ============================================================================
# LIFECYCLE MANAGEMENT: Database Connection & Migration
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    FastAPI lifespan context manager for startup and shutdown events.

    CRITICAL: This ensures database tables are created on Vercel deployment.

    Startup:
    1. Connect to Neon Postgres database pool
    2. Run migrations to create users table with bonus fields
    3. Verify database schema is ready

    Shutdown:
    1. Close database connection pool gracefully
    """
    # STARTUP: Initialize database
    print("[LIFESPAN] Application startup initiated...")

    try:
        # Connect to database and run migrations
        await init_db()
        print("[LIFESPAN] Database initialized successfully")

    except Exception as e:
        print(f"[LIFESPAN ERROR] Database initialization failed: {e}")
        import traceback
        traceback.print_exc()
        # Don't raise - allow app to start even if DB fails
        # Auth endpoints will return 503 Service Unavailable

    yield  # Application runs here

    # SHUTDOWN: Cleanup resources
    print("[LIFESPAN] Application shutdown initiated...")

    try:
        await db_manager.disconnect()
        print("[LIFESPAN] Database disconnected successfully")

    except Exception as e:
        print(f"[LIFESPAN ERROR] Shutdown cleanup failed: {e}")


# ============================================================================
# FASTAPI APPLICATION
# ============================================================================

try:
    print("[STARTUP] Initializing FastAPI App...")
    app = FastAPI(
        title="Physical AI Chatbot",
        version="1.0.0",
        docs_url="/api/docs",  # Explicit /api prefix
        openapi_url="/api/openapi.json",  # Explicit /api prefix
        redirect_slashes=False,  # CRITICAL for Vercel
        lifespan=lifespan  # Bind lifecycle manager
    )

    # CORS Setup (Production Ready - Uses Environment Variables)
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # ========================================================================
    # ROUTING ARCHITECTURE (Resource Prefix Pattern - Vercel Serverless)
    # ========================================================================
    # LOGIC:
    # - Routers define resource-level prefix ONLY (e.g., /chat, /auth)
    # - Vercel rewrite adds /api context at serverless function level
    # - Final URLs: /api/chat, /api/auth, /api/health (via vercel.json rewrite)
    # - This prevents double-prefix issues (no /api/api/chat)
    # ========================================================================

    app.include_router(chat_router, prefix="/chat", tags=["chat"])
    print("[STARTUP] Router registered: /chat")

    app.include_router(auth_router, prefix="/auth", tags=["auth"])
    print("[STARTUP] Router registered: /auth")

    @app.get("/health")
    async def health_check():
        """
        Health check endpoint for monitoring and debugging.

        Returns:
        - status: "ok" if app is running
        - model: Gemini model name
        - qdrant_url: Vector DB connection (truncated)
        - database_url: "SET" if Neon DB URL is configured
        - database_connected: True if connection pool is active
        """
        return {
            "status": "ok",
            "model": settings.gemini_model,
            "qdrant_url": settings.qdrant_url[:20] + "..." if settings.qdrant_url else "NOT SET",
            "database_url": "SET" if settings.database_url else "NOT SET",
            "database_connected": db_manager.pool is not None
        }

    @app.get("/")  # Root endpoint for serverless function health check
    async def root():
        """
        Root endpoint for Vercel serverless function.

        NOTE: This will be accessible at /api/ when routed through vercel.json
        """
        return {"message": "Physical AI & Humanoid Robotics Backend API", "status": "healthy"}

    print("[STARTUP] FastAPI App initialized successfully.")

except Exception as e:
    print(f"[STARTUP ERROR] in main.py: {str(e)}")
    import traceback
    traceback.print_exc()
    # Re-raise so Vercel logs show the crash
    raise e
