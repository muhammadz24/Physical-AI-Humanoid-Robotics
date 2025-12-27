import os
import traceback
from contextlib import asynccontextmanager
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from backend.app.api.routes import router as chat_router
from backend.app.api.auth import router as auth_router
from backend.app.api.personalize import router as personalize_router
from backend.app.core.database import db_manager


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan context manager for FastAPI startup/shutdown events.

    Startup: Initialize database connection pool
    Shutdown: Close database connections gracefully
    """
    # Startup: Connect to database
    print("[STARTUP] Initializing database connection pool...")
    try:
        await db_manager.connect()
        print("[STARTUP] Database connection pool ready ✅")
    except Exception as e:
        print(f"[STARTUP] ⚠️ Database connection failed: {e}")
        print("[STARTUP] Auth endpoints will be unavailable until DB is connected")

    yield  # App runs here

    # Shutdown: Close database connections
    print("[SHUTDOWN] Closing database connections...")
    await db_manager.disconnect()
    print("[SHUTDOWN] Database connections closed ✅")


app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    version="1.0.0",
    root_path="/api",  # Critical: Vercel mounts app at /api, this strips the prefix
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
    redirect_slashes=False,
    lifespan=lifespan
)

# CORS - Smart origin detection for local dev + production
# Read allowed origins from environment variable, or use safe defaults
allowed_origins_env = os.getenv("ALLOWED_ORIGINS", "")
if allowed_origins_env:
    # Production: Use environment variable (comma-separated list)
    allowed_origins = [origin.strip() for origin in allowed_origins_env.split(",")]
else:
    # Development/Fallback: Allow localhost + wildcard for Vercel preview deployments
    allowed_origins = [
        "http://localhost:3000",
        "http://127.0.0.1:3000",
        "*"  # Fallback for production (should set ALLOWED_ORIGINS env var)
    ]

print(f"[CORS] Allowed origins: {allowed_origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS", "PUT", "DELETE"],
    allow_headers=["*"],
)

# GLOBAL EXCEPTION HANDLER - Catch all unhandled exceptions
@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """
    Global exception handler to prevent raw 500 errors from reaching users.

    Developer Experience:
    - Full error traceback printed to terminal for debugging

    User Experience:
    - Clean JSON response with user-friendly message
    - 503 status (Service Unavailable) instead of 500 (Internal Server Error)
    """
    # 1. Log for Developer (Visible in Terminal)
    print("=" * 80)
    print("🔥 CRITICAL ERROR - Global Exception Handler")
    print("=" * 80)
    print(f"Request URL: {request.url}")
    print(f"Request Method: {request.method}")
    print(f"Exception Type: {type(exc).__name__}")
    print(f"Exception Message: {str(exc)}")
    print("\nFull Traceback:")
    print(traceback.format_exc())
    print("=" * 80)

    # 2. Return Safe Message to User
    return JSONResponse(
        status_code=503,
        content={
            "detail": "System is currently busy. Please try again later.",
            "error_type": type(exc).__name__  # For debugging (safe to expose)
        }
    )

# ROUTES - Resource-level prefix ONLY (Vercel rewrite adds /api context)
# Final URLs: /api/chat, /api/auth, /api/personalize
app.include_router(chat_router, prefix="/chat", tags=["chat"])
app.include_router(auth_router, prefix="/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize"])

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.get("/")
def root():
    return {"message": "API is running. POST to /chat to talk."}
