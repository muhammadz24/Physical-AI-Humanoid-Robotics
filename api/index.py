import sys
import os

# Add the current directory to sys.path so backend imports work
sys.path.append(os.path.dirname(os.path.abspath(__file__)) + '/../')

try:
    from backend.main import app
except Exception as e:
    print(f"🔥 CRITICAL: Failed to import FastAPI app in api/index.py: {str(e)}")
    import traceback
    traceback.print_exc()
    raise e
