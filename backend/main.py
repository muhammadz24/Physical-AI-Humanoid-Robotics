from fastapi import FastAPI

from fastapi.middleware.cors import CORSMiddleware

from app.api.routes import router as chat_router

from app.api.auth import router as auth_router

from app.api.personalize import router as personalize_router



app = FastAPI(

    title="Physical AI & Humanoid Robotics Chatbot",

    version="1.0.0",

    docs_url="/api/docs",

    redoc_url="/api/redoc",

    openapi_url="/api/openapi.json",

    redirect_slashes=False

)



# CORS - Allow all origins

app.add_middleware(

    CORSMiddleware,

    allow_origins=["*"],

    allow_credentials=True,

    allow_methods=["*"],

    allow_headers=["*"],

)



# --- ROUTES ---

# Sirf ye routes rakhein, 'Fallback' routes ki zaroorat nahi hai kyunke vercel.json handle kar raha hai.



app.include_router(chat_router, prefix="/api/chat", tags=["chat"])

app.include_router(auth_router, prefix="/api/auth", tags=["auth"])

app.include_router(personalize_router, prefix="/api/personalize", tags=["personalize"])



# --- HEALTH CHECK ---

# DHYAN DEIN: Humne '/api/chat' wala health check hata diya hai taake conflict na ho.

# Sirf /api/health rakhein.



@app.get("/api/health")

def health_check():

    return {

        "status": "healthy",

        "service": "Physical AI Chatbot API",

        "version": "1.0.0"

    }



@app.get("/")

def root():

    return {"message": "API is running. Use POST /api/chat to talk."}