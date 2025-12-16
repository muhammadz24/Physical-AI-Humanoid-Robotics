@echo off
echo [INFO] Initializing Zero-Touch Development Environment...
cd /d "%~dp0"

:: Launch Backend with Hot-Reload (Watches for file changes)
start "Backend API (Auto-Reload)" cmd /k "cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload"

:: Launch Frontend
start "Frontend App" cmd /k "npm start"

echo [SUCCESS] Environment Launched. Backend: http://localhost:8000 - Frontend: http://localhost:3000
echo [INFO] Both servers will auto-reload on file changes. Keep windows minimized.
pause
