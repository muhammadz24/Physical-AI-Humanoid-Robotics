from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

# FIXED: Added redirect_slashes=False to prevent 307 redirects that cause 405
app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json",
    redirect_slashes=False  # CRITICAL: Prevents trailing slash redirects
)

# CORS - Allow all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API Routes - PRIMARY
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

# Fallback Routes (for direct access without /api prefix)
app.include_router(chat_router, prefix="/chat", tags=["chat-fallback"])
app.include_router(auth_router, prefix="/auth", tags=["auth-fallback"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize-fallback"])

@app.get("/api/health")
def health_check():
    """Health check endpoint for monitoring"""
    return {
        "status": "healthy",
        "service": "Physical AI Chatbot API",
        "version": "1.0.0"
    }

# DO NOT MOUNT STATIC FILES HERE - Vercel handles frontend
# StaticFiles mounting causes 405 errors on API routes
