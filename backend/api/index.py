import sys
import os

# Vercel environment mein root path set karna zaroori hai
# Hum current file ki directory se 2 step peeche ja kar root add kar rahay hain
current_dir = os.path.dirname(os.path.abspath(__file__))
root_dir = os.path.dirname(current_dir)
sys.path.append(root_dir)

try:
    # Ab backend se app import karo
    from backend.main import app
except ImportError as e:
    # Agar import fail ho to error print karo (Logs mein dikhega)
    print(f"Error importing backend: {e}")
    # Temporary fallback taake server crash na ho (Debug only)
    from fastapi import FastAPI
    app = FastAPI()
    @app.get("/api/debug")
    def debug():
        return {"error": str(e), "path": sys.path}