"""
Vercel Serverless Function Entry Point

This file serves as the entry point for Vercel's Python serverless functions.
It exports the FastAPI app instance which Vercel uses to handle HTTP requests.

EXPLICIT ROUTING (Feature 012.2):
- All routes are prefixed with /api in main.py (explicit, not implicit)
- No root_path manipulation needed - routes match 1:1 with frontend requests
- Example: POST /api/chat → FastAPI route @router.post("/chat") with prefix="/api"

Constitution Compliance:
- Principle I: Simplicity-First (Explicit routing, no magic)
- Principle IX: Zero-Edit Deployment (Works on localhost AND Vercel)
- Principle III: Free-Tier Architecture (Vercel serverless functions)

Feature: 012.2-explicit-api-routing
Fixes: 405 Method Not Allowed, root_path conflicts
"""

import sys
import os

# Add parent directory to path so we can import from backend.main
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app

# Vercel imports and runs this app instance
# All FastAPI routes are explicitly prefixed with /api in main.py
# Example routes: /api/chat, /api/health, /api/auth/signup, /api/auth/signin

# For debugging: Log when this module is imported
print("[Vercel Entry Point] FastAPI app loaded")
print(f"[Vercel Entry Point] Available routes: {[route.path for route in app.routes]}")
