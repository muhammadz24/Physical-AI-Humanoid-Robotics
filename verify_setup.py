#!/usr/bin/env python3
"""
Setup verification script for Physical AI & Humanoid Robotics application.
"""

import os
import sys
import subprocess
import socket
from pathlib import Path

def check_port(port):
    """Check if a port is available."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        result = s.connect_ex(('localhost', port))
        return result != 0

def main():
    print("🔍 Physical AI & Humanoid Robotics - Setup Verification")
    print("=" * 60)

    # Check if we're in the correct directory
    current_dir = Path.cwd()
    if not (current_dir / "docusaurus.config.js").exists():
        print("❌ Not in the correct project directory!")
        return 1

    print(f"✅ Project directory: {current_dir}")

    # Check if backend/venv exists
    backend_venv_exists = (current_dir / "backend" / "venv").exists()
    print(f"✅ Backend virtual environment exists: {backend_venv_exists}")

    # Check if backend requirements exist
    backend_requirements_exist = (current_dir / "backend" / "requirements.txt").exists()
    print(f"✅ Backend requirements.txt exists: {backend_requirements_exist}")

    # Check port availability
    port_8000_free = check_port(8000)
    print(f"✅ Port 8000 available: {port_8000_free}")

    port_3000_free = check_port(3000)
    print(f"✅ Port 3000 available: {port_3000_free}")

    print("\n📋 Recommended startup sequence:")
    print("1. Terminal 1 - Start the backend server:")
    print("   cd backend && python -m uvicorn main:app --reload --port 8000")
    print("2. Terminal 2 - Start the frontend:")
    print("   npm start")
    print("")
    print("⚠️  Note: If you don't have the backend virtual environment set up:")
    print("   cd backend && python -m venv venv && source venv/bin/activate && pip install -r requirements.txt")
    print("   (On Windows: cd backend && python -m venv venv && venv\\Scripts\\activate && pip install -r requirements.txt)")

    print("\n💡 For development:")
    print("   - Frontend runs on http://localhost:3000")
    print("   - Backend runs on http://localhost:8000")
    print("   - Frontend automatically sends API requests to the backend")
    print("   - API calls from frontend: fetch('/api/chat') -> backend receives: POST /chat")

    print("\n🔧 API Proxy Configuration:")
    print("   The frontend (in src/utils/api.js) detects localhost and automatically")
    print("   sends API requests to http://localhost:8000 when on localhost.")

    return 0

if __name__ == "__main__":
    sys.exit(main())