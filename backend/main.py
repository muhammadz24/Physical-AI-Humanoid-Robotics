from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
# removing settings import that was causing crash
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

# Hardcoded values to bypass Settings crash
app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    openapi_url="/api/v1/openapi.json"
)

# Allow ALL origins for Hackathon demo to prevent CORS issues
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

# Add generic routes for flexibility
app.include_router(chat_router, prefix="/chat", tags=["chat-no-prefix"])
app.include_router(auth_router, prefix="/auth", tags=["auth-no-prefix"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize-no-prefix"])

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "physical-ai-chatbot"}

@app.get("/")
def root():
    return {"message": "Physical AI & Humanoid Robotics RAG Chatbot API is running"}
