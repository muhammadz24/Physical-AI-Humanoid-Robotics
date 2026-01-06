"""
Local Development Server with root_path Configuration

This script starts the FastAPI backend with the same root_path configuration
used in Vercel deployment, ensuring local/production parity.

Usage:
    python backend/start_dev.py

Environment:
    - Sets root_path="/api" to match Vercel serverless function behavior
    - Enables hot reload for development
    - Listens on 0.0.0.0:8000 for local network access

Routes (served at):
    - GET  /api/health
    - POST /api/chat
    - POST /api/auth/signup
    - POST /api/auth/signin
    - GET  /api/docs (OpenAPI documentation)
"""

import uvicorn
import sys
import os

# Add project root to Python path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

if __name__ == "__main__":
    print("=" * 60)
    print("Starting Physical AI Backend (Development Mode)")
    print("=" * 60)
    print("Root Path: /api (configured in api/index.py)")
    print("Base URL: http://localhost:8000/api")
    print("Health Check: http://localhost:8000/api/health")
    print("API Docs: http://localhost:8000/api/docs")
    print("=" * 60)
    print()
    print("NOTE: Using uvicorn with --root-path flag")
    print("This matches Vercel's serverless function behavior")
    print("=" * 60)

    # Use import string for reload support
    # The --root-path flag tells FastAPI it's mounted at /api
    uvicorn.run(
        "backend.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        log_level="info",
        root_path="/api"  # CRITICAL: Match Vercel deployment
    )
