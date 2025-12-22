import sys
import os
from pathlib import Path

# Fix pathing once and for all
root_dir = Path(__file__).parent.parent
sys.path.append(str(root_dir))

from backend.main import app

# This is for Vercel
handler = app
