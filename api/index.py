import sys
import os
from pathlib import Path

# Add backend to path
sys.path.append(str(Path(__file__).parent.parent))

from backend.main import app as fastapi_app

# Vercel expects 'app' variable
app = fastapi_app
