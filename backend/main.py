"""
FastAPI Backend Entry Point

Main application file for RAG Chatbot Backend.
Initializes FastAPI app, registers routes, and manages lifecycle events.
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.core.config import settings
from app.core.database import db_manager
from app.core.vector_store import vector_store
from app.services.embedding import embedding_service
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager.

    Startup:
    - Initialize database connection pool
    - Connect to Qdrant vector store

    Shutdown:
    - Close database connections
    - Disconnect from Qdrant
    """
    # Startup
    print("[START] Starting FastAPI backend...")

    # Initialize database connection pool (non-fatal for local testing)
    try:
        await db_manager.connect()
        print("[OK] Database connection pool established")
    except Exception as e:
        print(f"[WARN] Database connection failed (continuing anyway): {e}")

    # Connect to Qdrant vector store (non-fatal for local testing)
    try:
        vector_store.connect()
        print("[OK] Qdrant vector store connected")
    except Exception as e:
        print(f"[WARN] Qdrant connection failed (continuing anyway): {e}")

    # Load embedding model (required for chat functionality)
    try:
        print(f"[LOAD] Loading embedding model: {settings.embedding_model}...")
        embedding_service.load_model()
        print(f"[OK] Embedding model loaded successfully (dimension: {settings.embedding_dimension})")
    except Exception as e:
        print(f"[ERROR] Failed to load embedding model: {e}")
        print("[WARN] Chat functionality will be limited")

    print("[OK] All services initialized successfully")

    yield

    # Shutdown
    print("[STOP] Shutting down FastAPI backend...")
    await db_manager.disconnect()
    vector_store.disconnect()
    print("[OK] All services shut down gracefully")


# Initialize FastAPI application
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG Chatbot",
    description="FastAPI backend for textbook RAG chatbot with Qdrant vector search",
    version=settings.api_version,
    lifespan=lifespan,
    debug=settings.debug
)

# Configure CORS with environment-driven origins (Principle IX)
# Validate and log CORS configuration (FR-004, FR-005)
validated_origins = [
    origin for origin in settings.allowed_origins_list
    if origin.startswith(('http://', 'https://'))
]

if not validated_origins:
    validated_origins = ["http://localhost:3000", "http://127.0.0.1:3000"]

print(f"[OK] Allowed CORS origins: {validated_origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=validated_origins,  # Environment-driven, explicit list
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Register API routes
app.include_router(chat_router)
app.include_router(auth_router)
app.include_router(personalize_router, prefix="/api", tags=["personalization"])


@app.get("/health")
async def health_check():
    """
    Health check endpoint to verify all services are operational.

    Returns:
        dict: Service status including database, vector store, and uptime

    Status Codes:
        200: All services healthy
        503: One or more services degraded
    """
    # Check database connection
    db_healthy = await db_manager.health_check()

    # Check Qdrant connection
    qdrant_healthy = vector_store.health_check()

    # Determine overall status
    all_healthy = db_healthy and qdrant_healthy
    status = "healthy" if all_healthy else "degraded"

    response = {
        "status": status,
        "version": settings.api_version,
        "environment": settings.environment,
        "services": {
            "neon_postgres": "connected" if db_healthy else "disconnected",
            "qdrant_vector_store": "connected" if qdrant_healthy else "disconnected"
        }
    }

    # Return 503 if any service is down
    status_code = 200 if all_healthy else 503

    return response


@app.get("/")
async def root():
    """
    Root endpoint with API information.

    Returns:
        dict: API metadata and available endpoints
    """
    return {
        "name": "Physical AI & Humanoid Robotics RAG Chatbot API",
        "version": settings.api_version,
        "environment": settings.environment,
        "endpoints": {
            "health": "/health",
            "chat": "/api/chat",
            "docs": "/docs",
            "openapi": "/openapi.json"
        },
        "message": "Visit /docs for interactive API documentation"
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=settings.debug,
        log_level="info"
    )
