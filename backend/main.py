from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    openapi_url="/api/v1/openapi.json",
    docs_url="/docs",
    redoc_url="/redoc"
)

# CORS: Allow all origins
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Routers
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

# Fallback Routes
app.include_router(chat_router, prefix="/chat", tags=["chat-no-prefix"])
app.include_router(auth_router, prefix="/auth", tags=["auth-no-prefix"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize-no-prefix"])

@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "physical-ai-chatbot"}

# REMOVED ROOT "/" route to let Vercel serve the Frontend
