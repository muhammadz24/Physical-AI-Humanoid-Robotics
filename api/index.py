import os
import sys

# Protocol: Absolute path injection for isolated runtime
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.join(current_dir, "..")
sys.path.append(project_root)

# Protocol: Force look into Vercel's standard deployment path
venv_path = os.path.join(project_root, ".vercel", "python", "lib", "python3.12", "site-packages")
sys.path.append(venv_path)

print(f"PATH_TRACE: {sys.path}")

try:
    from backend.main import app
except ImportError as e:
    print(f"IMPORT_FAIL: {str(e)}")
    raise
