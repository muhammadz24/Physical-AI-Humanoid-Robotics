import sys
import os
import traceback
from fastapi import FastAPI, Response

# Add the parent directory (backend) to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

try:
    # Try to import the main app
    from main import app
    print("Successfully imported main app")
except Exception as e:
    # If it crashes, catch the error and show it in the browser
    error_msg = traceback.format_exc()
    print(f"Failed to start app: {error_msg}")

    # Create a dummy app to display the error
    app = FastAPI()

    @app.api_route("/{path_name:path}", methods=["GET", "POST", "PUT", "DELETE"])
    async def catch_all(path_name: str):
        return Response(
            content=f"🐍 PYTHON CRASH REPORT 🐍\n\n{error_msg}",
            media_type="text/plain",
            status_code=500
        )
