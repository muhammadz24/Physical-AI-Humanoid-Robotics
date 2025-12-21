import sys
import os

# Project root ko path mein shamil karein
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Seedha 'backend' folder se app import karein
from backend.main import app

# Vercel ko ye 'app' variable chahiye