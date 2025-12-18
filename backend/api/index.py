"""
Vercel Serverless Function Entry Point

This file serves as the entry point for Vercel's Python serverless functions.
It properly configures the FastAPI app to handle the /api path prefix.

CRITICAL PATH HANDLING:
- Vercel rewrites /api/* to this function
- FastAPI routes are defined without /api prefix (e.g., /chat, /health)
- We configure root_path="/api" so FastAPI knows to strip it

Examples:
- Request: /api/chat → FastAPI sees: /chat → Matches route → SUCCESS
- Request: /api/health → FastAPI sees: /health → Matches route → SUCCESS

Constitution Compliance:
- Principle IX: Zero-Edit Deployment (Works on localhost AND Vercel without changes)
- Principle III: Free-Tier Architecture (Vercel serverless functions)

Feature: 012.1-api-method-fix
Fixes: 405 Method Not Allowed error on Vercel
"""

import sys
import os

# Add parent directory to path so we can import from backend.main
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app

# Configure root_path to handle /api prefix
# This tells FastAPI that all requests come with /api prefix,
# so it strips it before routing
app.root_path = "/api"

# Vercel imports and runs this app instance
# All FastAPI routes are automatically available at /api/*
# Example: /api/health, /api/chat, /api/auth/signup, etc.

# For debugging: Log when this module is imported
print(f"[Vercel Entry Point] FastAPI app loaded with root_path={app.root_path}")
print(f"[Vercel Entry Point] Available routes: {[route.path for route in app.routes]}")
