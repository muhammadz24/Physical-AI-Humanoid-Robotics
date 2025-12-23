from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from backend.app.api.routes import router as chat_router
from backend.app.api.auth import router as auth_router
from backend.app.api.personalize import router as personalize_router

app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    version="1.0.0",
    docs_url="/api/docs",
    redoc_url="/api/redoc",
    openapi_url="/api/openapi.json",
    redirect_slashes=False
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["GET", "POST", "OPTIONS", "PUT", "DELETE"],
    allow_headers=["*"],
)

# ROUTES - Explicit /api prefix (brute-force fix for Vercel)
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

@app.get("/api/health")
def health_check():
    return {"status": "healthy"}

@app.get("/api")
def root():
    return {"message": "API is running. POST to /api/chat to talk."}
