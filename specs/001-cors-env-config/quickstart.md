# Quickstart: Environment-Driven CORS and API Configuration

**Feature**: 001-cors-env-config
**Date**: 2025-12-12
**Target Audience**: Developers and DevOps

## Overview

This guide shows how to configure CORS origins and API URLs using environment variables for local development and production deployments.

## Prerequisites

- Backend: Python 3.11+, FastAPI, pydantic-settings (already installed)
- Frontend: Node.js 18+, Docusaurus 3.x (already installed)
- Code changes from this feature applied (backend/main.py, src/components/ChatWidget/index.js)

## Local Development (No Configuration Needed)

### Step 1: Start Backend

```bash
cd backend
python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

**Expected Output**:
```
✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

### Step 2: Start Frontend

```bash
# From repository root
npx docusaurus start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

### Step 3: Test Chat Widget

1. Open browser: http://localhost:3000
2. Click floating chat button (bottom right)
3. Type a test message
4. Verify connection works (no CORS errors)

**Success Criteria**:
- ✅ Chat widget opens
- ✅ No CORS errors in browser console (F12 → Console)
- ✅ Backend logs show request received

## Production Deployment

### Vercel Deployment (Frontend + Backend)

#### Backend (Vercel/Railway/Render)

1. **Set Environment Variables** on hosting platform:

   **Vercel**:
   ```
   Project Settings → Environment Variables → Add
   ```

   **Railway**:
   ```
   Project → Variables → New Variable
   ```

   **Render**:
   ```
   Dashboard → Environment → Add Environment Variable
   ```

   **Required Variable**:
   ```bash
   ALLOWED_ORIGINS=https://yourusername.github.io,https://www.yourdomain.com
   ```

   **Example**:
   ```bash
   ALLOWED_ORIGINS=https://myapp.vercel.app
   ```

2. **Deploy Backend**

   Vercel will automatically redeploy when you push to main. For other platforms, follow their deployment guides.

3. **Verify**

   Check deployment logs for:
   ```
   ✅ Allowed CORS origins: ['https://myapp.vercel.app']
   ```

#### Frontend (Vercel/GitHub Pages)

1. **Set Build Environment Variable** on Vercel:

   ```
   Project Settings → Environment Variables → Add
   ```

   **Required Variable**:
   ```bash
   REACT_APP_API_URL=https://your-backend-api.vercel.app
   ```

   **Example**:
   ```bash
   REACT_APP_API_URL=https://api-myapp.vercel.app
   ```

2. **Rebuild and Deploy**

   Vercel will rebuild with the new environment variable.

3. **Verify**

   - Open production URL: https://myapp.vercel.app
   - Open browser DevTools (F12) → Network tab
   - Send a chat message
   - Verify fetch request goes to `https://api-myapp.vercel.app/api/chat`

### GitHub Pages + External Backend

#### Backend (Railway/Render)

Same as above (set `ALLOWED_ORIGINS=https://yourusername.github.io`).

#### Frontend (GitHub Pages)

**Option 1: Manual Build Locally**

```bash
# Set environment variable before build
export REACT_APP_API_URL=https://your-backend-api.com
npm run build

# Deploy to GitHub Pages
git add docs/
git commit -m "Deploy to GitHub Pages"
git push origin gh-pages
```

**Option 2: GitHub Actions (Recommended)**

Add to `.github/workflows/deploy.yml`:

```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18

      - name: Install dependencies
        run: npm ci

      - name: Build
        env:
          REACT_APP_API_URL: ${{ secrets.API_URL }}
        run: npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

Add secret in GitHub repo settings:
```
Settings → Secrets and variables → Actions → New repository secret
Name: API_URL
Value: https://your-backend-api.com
```

## Environment Variable Reference

### Backend Environment Variables

**File**: backend/.env (local) or platform environment variables (production)

```bash
# -----------------------------------------------------------------------------
# CORS Configuration
# -----------------------------------------------------------------------------
# Comma-separated list of allowed origins (no spaces)
# Local: http://localhost:3000,http://127.0.0.1:3000
# Production: https://yourdomain.com,https://www.yourdomain.com
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

**Default**: `http://localhost:3000` (if env var not set)
**Validation**: Each origin must start with `http://` or `https://`
**Fallback**: If all origins invalid, falls back to `["http://localhost:3000", "http://127.0.0.1:3000"]`

### Frontend Environment Variables

**File**: .env.local (local) or platform build environment (production)

```bash
# -----------------------------------------------------------------------------
# API Configuration
# -----------------------------------------------------------------------------
# Backend API base URL (without /api/chat path)
# Local: http://localhost:8000
# Production: https://api.yourdomain.com
REACT_APP_API_URL=http://localhost:8000
```

**Default**: `http://localhost:8000` (if env var not set)
**Usage**: Appends `/api/chat` automatically

## Testing Checklist

### Local Development Test

- [ ] Clone repository (or delete .env files)
- [ ] Start backend without .env file
- [ ] Verify backend logs show: `✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']`
- [ ] Start frontend without .env.local
- [ ] Open http://localhost:3000
- [ ] Send test message in chat widget
- [ ] Verify no CORS errors in browser console

### Production Simulation Test (Local)

- [ ] Create backend/.env:
  ```bash
  ALLOWED_ORIGINS=https://example.com
  ```
- [ ] Restart backend
- [ ] Verify backend logs show: `✅ Allowed CORS origins: ['https://example.com']`
- [ ] Try accessing from localhost:3000
- [ ] Expected: CORS error in browser console (correct behavior)

### Production Deployment Test

- [ ] Set backend `ALLOWED_ORIGINS` env var on hosting platform
- [ ] Deploy backend
- [ ] Verify deployment logs show correct origins
- [ ] Set frontend `REACT_APP_API_URL` env var
- [ ] Deploy frontend
- [ ] Open production URL
- [ ] Send test message in chat widget
- [ ] Verify fetch goes to production API URL (check Network tab)
- [ ] Verify no CORS errors

## Troubleshooting

### CORS Error in Browser Console

**Symptom**:
```
Access to fetch at 'http://localhost:8000/api/chat' from origin 'http://localhost:3000'
has been blocked by CORS policy: No 'Access-Control-Allow-Origin' header is present.
```

**Causes**:
1. Backend not running
2. Backend `ALLOWED_ORIGINS` doesn't include frontend origin
3. Backend crashed during startup

**Fix**:
1. Check backend logs for `✅ Allowed CORS origins: [...]`
2. Verify frontend origin is in the list
3. Restart backend with correct `ALLOWED_ORIGINS`

### Backend Logs Show Wrong Origins

**Symptom**:
```
✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']
```
But you set `ALLOWED_ORIGINS=https://myapp.com` in .env

**Cause**: .env file not loaded or backend not restarted

**Fix**:
1. Verify .env file is in backend/ directory
2. Check .env file has no typos: `ALLOWED_ORIGINS=...` (no quotes, no spaces around =)
3. Restart backend completely (kill process, start fresh)

### Frontend Still Uses Localhost in Production

**Symptom**: Network tab shows `http://localhost:8000/api/chat` instead of production URL

**Cause**: `REACT_APP_API_URL` not set during build

**Fix**:
1. Verify env var is set on hosting platform (Vercel settings)
2. Rebuild frontend (Vercel: redeploy; local: `npm run build`)
3. Clear browser cache and hard reload (Ctrl+Shift+R)

### Wildcard Origin Error

**Symptom**: Backend crashes with:
```
RuntimeError: Wildcard origin '*' not allowed with credentials
```

**Cause**: Someone manually set `ALLOWED_ORIGINS=*`

**Fix**:
1. Remove wildcard from `ALLOWED_ORIGINS`
2. Set explicit origins: `ALLOWED_ORIGINS=https://myapp.com`
3. Restart backend

## Next Steps

After verifying configuration works:
1. Run `/sp.tasks` to generate implementation tasks
2. Execute tasks to apply code changes
3. Test locally with this quickstart guide
4. Deploy to staging with environment variables
5. Deploy to production

## References

- **Backend .env.example**: backend/.env.example
- **Frontend .env.example**: .env.example (repository root)
- **Plan**: specs/001-cors-env-config/plan.md
- **Spec**: specs/001-cors-env-config/spec.md
- **Docusaurus Deployment**: https://docusaurus.io/docs/deployment
- **Vercel Env Vars**: https://vercel.com/docs/concepts/projects/environment-variables
