import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from app.api.routes import router as chat_router
from app.api.auth import router as auth_router
from app.api.personalize import router as personalize_router

app = FastAPI(title="Physical AI Chatbot")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# API Routers (Must come BEFORE static files)
app.include_router(chat_router, prefix="/api/chat", tags=["chat"])
app.include_router(auth_router, prefix="/api/auth", tags=["auth"])
app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])

# Fallback API Routes
app.include_router(chat_router, prefix="/chat", tags=["chat-no-prefix"])
app.include_router(auth_router, prefix="/auth", tags=["auth-no-prefix"])
app.include_router(personalize_router, prefix="/personalize", tags=["personalize-no-prefix"])

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# SERVE UI (Dynamic Logic)
# 1. Define possible frontend paths
frontend_paths = ["dist", "build", "public", "frontend/dist", "frontend/build", "static"]
static_dir = None

# 2. Find which one exists
for path in frontend_paths:
    if os.path.exists(path) and os.path.exists(os.path.join(path, "index.html")):
        static_dir = path
        break

# 3. Mount Static Files if found
if static_dir:
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="ui")
else:
    # Fallback if build is missing (Prevent 404 Crash)
    @app.get("/")
    def root():
        return {
            "message": "UI Build Not Found. Please run 'npm run build' and ensure the output folder exists.",
            "backend_status": "Running",
            "api_docs": "/docs"
        }
