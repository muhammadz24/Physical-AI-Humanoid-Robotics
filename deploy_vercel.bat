@echo off
REM Automated Vercel Deployment Script
REM Reads .env and deploys to Vercel with environment variables

echo [INFO] Starting Vercel Deployment with Environment Variables...

REM Check if Vercel CLI is installed
where vercel >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Vercel CLI not found. Installing...
    call npm install -g vercel
)

REM Link to Vercel project (if not already linked)
echo [INFO] Linking to Vercel project...
call vercel link --yes

REM Extract and set environment variables from backend/.env
echo [INFO] Setting environment variables from backend/.env...

REM Read DATABASE_URL
for /f "tokens=2 delims==" %%a in ('findstr /B "DATABASE_URL=" backend\.env') do set DATABASE_URL=%%a
call vercel env add DATABASE_URL production < <(echo %DATABASE_URL%)

REM Read GEMINI_API_KEY
for /f "tokens=2 delims==" %%a in ('findstr /B "GEMINI_API_KEY=" backend\.env') do set GEMINI_API_KEY=%%a
call vercel env add GEMINI_API_KEY production < <(echo %GEMINI_API_KEY%)

REM Read JWT_SECRET_KEY
for /f "tokens=2 delims==" %%a in ('findstr /B "JWT_SECRET_KEY=" backend\.env') do set JWT_SECRET_KEY=%%a
call vercel env add JWT_SECRET_KEY production < <(echo %JWT_SECRET_KEY%)

REM Read QDRANT_URL
for /f "tokens=2 delims==" %%a in ('findstr /B "QDRANT_URL=" backend\.env') do set QDRANT_URL=%%a
call vercel env add QDRANT_URL production < <(echo %QDRANT_URL%)

REM Read QDRANT_API_KEY
for /f "tokens=2 delims==" %%a in ('findstr /B "QDRANT_API_KEY=" backend\.env') do set QDRANT_API_KEY=%%a
call vercel env add QDRANT_API_KEY production < <(echo %QDRANT_API_KEY%)

REM Deploy to production
echo [INFO] Deploying to Vercel Production...
call vercel --prod

echo [SUCCESS] Deployment complete!
pause
