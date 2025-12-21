from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

app = FastAPI(
    title="Physical AI & Humanoid Robotics Chatbot",
    version="1.0.0",
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
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

# ROUTES - No /api prefix (Vercel adds it via rewrite)
app.include_router(chat_router, prefix="/chat", tags=["chat"])
app.include_router(auth_router, prefix="/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize"])

@app.get("/health")
def health_check():
    return {"status": "healthy"}

@app.get("/")
def root():
    return {"message": "API is running. POST to /api/chat to talk."}
