import sys
import os
from pathlib import Path

# Project root ko path mein shamil karein
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from backend.main import app