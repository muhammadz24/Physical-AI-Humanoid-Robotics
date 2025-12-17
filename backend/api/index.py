"""
Vercel Serverless Function Entry Point

This file serves as the entry point for Vercel's Python serverless functions.
It exports the FastAPI app instance which Vercel will use to handle HTTP requests.

IMPORTANT:
- This file must be at backend/api/index.py to match vercel.json configuration
- Do NOT include uvicorn.run() or __main__ blocks here
- Vercel handles the ASGI lifecycle automatically
- All routes defined in app.main are automatically available via /api/*

Constitution Compliance:
- Principle IX: Zero-Edit Deployment (Works on localhost AND Vercel without changes)
- Principle III: Free-Tier Architecture (Vercel serverless functions)
"""

from app.main import app

# Vercel imports and runs this app instance
# All FastAPI routes are automatically available at /api/*
# Example: /api/health, /api/chat, /api/auth/signup, etc.
