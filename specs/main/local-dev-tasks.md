# Tasks: Local Development Environment Setup & Testing

**Purpose**: Start development servers, verify environment, and provide testing URLs for UI fixes validation

**Context**: Post-implementation testing for UI Critical Fixes (Mobile Menu + Z-Index)

**Scope**: Start Docusaurus frontend, FastAPI backend (if needed), verify environment variables, confirm ports accessible

---

## Phase 1: Environment Verification

**Purpose**: Verify all dependencies and environment variables before starting servers

- [ ] T001 Check if `.env` file exists in project root
- [ ] T002 Verify `.env` has required backend variables (QDRANT_URL, QDRANT_API_KEY, DATABASE_URL, GEMINI_API_KEY, JWT_SECRET)
- [ ] T003 Check if `backend/.env` exists for backend-specific config
- [ ] T004 Verify Node.js and npm versions (should show Node 18+, npm 9+)
- [ ] T005 Verify Python version for backend (should show Python 3.12+)
- [ ] T006 Check if backend dependencies installed (`pip list | grep -E "(fastapi|qdrant|uvicorn)"`)

**Checkpoint**: Environment variables and dependencies verified

---

## Phase 2: Start Frontend (Docusaurus)

**Purpose**: Launch Docusaurus development server on port 3000

- [ ] T007 Clear Docusaurus cache (optional but recommended): `npm run clear`
- [ ] T008 Start Docusaurus dev server in background: `npm start` (runs on http://localhost:3000)
- [ ] T009 Wait for server startup (look for "Docusaurus website is running at" message)
- [ ] T010 Verify frontend accessible: Open browser and navigate to `http://localhost:3000`
- [ ] T011 Check browser console for errors (F12 → Console tab → Verify no red errors)
- [ ] T012 Verify hot-reload working: Make a small change to any file and verify auto-refresh

**Checkpoint**: Frontend running at http://localhost:3000

---

## Phase 3: Start Backend (FastAPI) - If Applicable

**Purpose**: Launch FastAPI backend server for RAG chatbot functionality

### Check if Backend is Required

- [ ] T013 Check if `backend/` directory exists
- [ ] T014 Check if `backend/main.py` exists (FastAPI entry point)
- [ ] T015 Verify backend is needed: Check if ChatWidget component calls `/api/*` endpoints

### Start Backend Server (if required)

- [ ] T016 Navigate to backend directory: `cd backend`
- [ ] T017 Activate Python virtual environment (if exists): `source venv/bin/activate` or `.\venv\Scripts\activate` (Windows)
- [ ] T018 Install backend dependencies: `pip install -r requirements.txt` (if not already installed)
- [ ] T019 Start FastAPI server in background: `uvicorn main:app --reload --host 0.0.0.0 --port 8000`
- [ ] T020 Wait for server startup (look for "Uvicorn running on http://0.0.0.0:8000" message)
- [ ] T021 Verify backend accessible: `curl http://localhost:8000/api/health` or open in browser
- [ ] T022 Check backend logs for startup errors

**Checkpoint**: Backend running at http://localhost:8000 (if applicable)

---

## Phase 4: Port Verification & URL Reporting

**Purpose**: Confirm all services are accessible and provide testing URLs

- [ ] T023 Verify frontend port 3000 is active: `netstat -an | grep :3000` (Linux/Mac) or `netstat -an | findstr :3000` (Windows)
- [ ] T024 Verify backend port 8000 is active (if backend running): `netstat -an | grep :8000` or `netstat -an | findstr :8000`
- [ ] T025 Test frontend homepage loads: `curl -I http://localhost:3000` (should return 200 OK)
- [ ] T026 Test backend health endpoint (if backend running): `curl http://localhost:8000/api/health`
- [ ] T027 Report accessible URLs to user:
  - Frontend: http://localhost:3000
  - Backend API: http://localhost:8000 (if running)
  - API Docs: http://localhost:8000/docs (FastAPI Swagger, if running)

**Checkpoint**: All servers running and accessible

---

## Phase 5: UI Fixes Testing URLs

**Purpose**: Provide specific URLs and test scenarios for validating Mobile Menu and Z-Index fixes

### Mobile Menu Fix Testing

**Test URL**: http://localhost:3000

**Test Procedure**:
1. Open http://localhost:3000 in browser
2. Open DevTools (F12)
3. Toggle device toolbar (Ctrl+Shift+M or Cmd+Option+M on Mac)
4. Test viewports:
   - iPhone SE (375x667 or 320x568)
   - iPad (768x1024)
   - iPad Pro (1024x1366)
   - Desktop (1920x1080)
5. At each viewport:
   - ✅ Verify hamburger menu (3 lines icon) is NOT visible
   - ✅ Verify desktop navbar items ARE visible (Introduction, Docs, etc.)

**Test with F12 Console**:
1. Full browser window (1920x1080)
2. Open F12 (docks bottom, viewport shrinks to ~1920x600)
3. ✅ Verify desktop navbar still visible, no hamburger menu appears

### Z-Index Fix Testing

**Test URL**: http://localhost:3000

**Test Procedure**:
1. Open http://localhost:3000 in browser
2. Locate ChatWidget floating button (bottom-right, cyan robot icon)
3. Click to open chatbot
4. Verify:
   - ✅ Navbar at top of page is fully visible (not covered by chatbot)
   - ✅ Chatbot modal appears below navbar
5. Click navbar links (Introduction, Docs) while chatbot is open
6. Verify:
   - ✅ Navbar links are clickable (not blocked by chatbot)
   - ✅ Page navigates successfully
7. Scroll page up/down with chatbot open
8. Verify:
   - ✅ Navbar stays on top (stacking order maintained)

**DevTools Inspection**:
1. Open DevTools (F12) → Elements tab
2. Inspect `.navbar` element
3. Check Computed styles → Verify `z-index: 1000` and `position: sticky`
4. Inspect `.chatModal` element (inside ChatWidget)
5. Check Computed styles → Verify `z-index: 98` and `position: fixed`
6. Confirm: 1000 > 98 (navbar above chatbot) ✅

### Additional Test Pages

**Homepage**: http://localhost:3000
**Introduction Page**: http://localhost:3000/docs/intro
**Any Docs Page**: http://localhost:3000/docs/chapter1 (or any existing chapter)

**Test Navigation**:
- Navigate between 5+ pages
- Verify hamburger menu never appears during navigation
- Verify chatbot (if opened) maintains correct z-index on all pages

---

## Phase 6: Production Build Testing (Optional)

**Purpose**: Verify fixes work in production build (static site)

- [ ] T028 Stop dev server (Ctrl+C in terminal)
- [ ] T029 Build production site: `npm run build`
- [ ] T030 Verify build succeeds with no errors
- [ ] T031 Start production preview server: `npm run serve`
- [ ] T032 Open production build: http://localhost:3000
- [ ] T033 Repeat Mobile Menu tests on production build
- [ ] T034 Repeat Z-Index tests on production build
- [ ] T035 Compare bundle size: `du -sh build/` (expect +50 bytes for CSS rule)

**Checkpoint**: Production build validated

---

## Execution Notes

### Running Servers in Parallel

**Option 1: Separate Terminals**
```bash
# Terminal 1: Frontend
npm start

# Terminal 2: Backend (if needed)
cd backend
uvicorn main:app --reload
```

**Option 2: Background Processes (Linux/Mac)**
```bash
# Start frontend in background
npm start &

# Start backend in background
cd backend && uvicorn main:app --reload &

# View running processes
jobs
```

**Option 3: tmux/screen (Advanced)**
```bash
# Create tmux session with 2 panes
tmux new-session \; split-window -h \; send-keys 'npm start' C-m \; select-pane -t 0 \; send-keys 'cd backend && uvicorn main:app --reload' C-m
```

### Stopping Servers

```bash
# Stop dev server (in terminal where running)
Ctrl+C

# Kill by port (if process stuck)
# Windows:
netstat -ano | findstr :3000
taskkill /PID <PID> /F

# Linux/Mac:
lsof -ti:3000 | xargs kill -9
lsof -ti:8000 | xargs kill -9
```

---

## Success Criteria

### Environment Setup ✅
- [ ] `.env` file exists with required variables
- [ ] Node.js 18+ and npm 9+ installed
- [ ] Python 3.12+ installed (if backend needed)
- [ ] Dependencies installed (npm, pip)

### Frontend Running ✅
- [ ] Docusaurus dev server running on http://localhost:3000
- [ ] Homepage loads without errors
- [ ] Browser console shows no errors
- [ ] Hot-reload working

### Backend Running (if applicable) ✅
- [ ] FastAPI server running on http://localhost:8000
- [ ] Health endpoint responds: http://localhost:8000/api/health
- [ ] API docs accessible: http://localhost:8000/docs

### UI Fixes Validated ✅
- [ ] Mobile menu hidden at all viewports (320px, 768px, 1024px, 1920px)
- [ ] Desktop navbar items visible at all viewports
- [ ] Navbar visible above chatbot (z-index correct)
- [ ] Navbar links clickable when chatbot open

---

## Quick Start Commands

```bash
# Frontend only (if backend not needed for testing)
npm start
# Open http://localhost:3000

# Frontend + Backend (if RAG chatbot testing needed)
# Terminal 1:
npm start

# Terminal 2:
cd backend
uvicorn main:app --reload
# Frontend: http://localhost:3000
# Backend API: http://localhost:8000
# API Docs: http://localhost:8000/docs
```

---

**Tasks Generated**: 35 tasks for complete local development setup and testing
**Estimated Time**: 10 minutes (setup) + 15 minutes (testing) = 25 minutes total
**Ready for Execution**: Yes
