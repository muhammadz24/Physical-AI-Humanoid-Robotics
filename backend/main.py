from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.core.config import settings
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

app = FastAPI(
    title=settings.PROJECT_NAME,
    openapi_url=f"{settings.API_V1_STR}/openapi.json"
)

# Set all CORS enabled origins
if settings.BACKEND_CORS_ORIGINS:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[str(origin) for origin in settings.BACKEND_CORS_ORIGINS],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

# Include routers
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

# Add generic routes for flexibility (optional, helps with some frontend configs)
app.include_router(chat_router, prefix="/chat", tags=["chat-no-prefix"])
app.include_router(auth_router, prefix="/auth", tags=["auth-no-prefix"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize-no-prefix"])

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "physical-ai-chatbot"}

@app.get("/")
def root():
    return {"message": "Physical AI & Humanoid Robotics RAG Chatbot API is running"}
