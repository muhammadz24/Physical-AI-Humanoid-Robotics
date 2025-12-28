import sys
import os

# 1. FIX PATH: Add the project root to sys.path so 'backend' module is found
# This solves the ModuleNotFoundError
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 2. IMPORT REAL APP: Bring in the actual Chatbot backend
from backend.main import app
