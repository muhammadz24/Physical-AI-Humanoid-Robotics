# Hotfix Specification: Smart Routing & Dynamic API Detection

**Feature ID**: `009.1-hotfix-smart-routing`
**Status**: URGENT HOTFIX
**Created**: 2025-12-18
**Priority**: P0 (Critical Production Issue)
**Branch**: `009-ultimate-fix` (hotfix on same branch)

---

## 🚨 Critical Production Issues

### Issue 1: SPA Routing 404 Errors
- **Symptom**: Refreshing `/signin`, `/signup`, or any React route returns 404
- **Root Cause**: `vercel.json` missing catch-all rule for client-side routing
- **Impact**: Users cannot bookmark or refresh any page except homepage
- **Severity**: CRITICAL

### Issue 2: API Connection Failures
- **Symptom**: Chatbot shows "Failed to Fetch" error
- **Root Cause**: Frontend hardcoded to `http://localhost:8000` in production
- **Impact**: No backend connectivity on Vercel deployment
- **Severity**: CRITICAL (feature completely broken)

### Issue 3: Ghost Deployment Logs
- **Symptom**: Console logs may expose environment variables
- **Root Cause**: Debug logs showing env var access
- **Impact**: Potential security exposure
- **Severity**: MEDIUM

---

## ✅ Functional Requirements

### FR-009.1-001: SPA Routing Fallback
- **Priority**: P0 (Must Have)
- **Description**: Vercel must serve `index.html` for all non-API routes
- **Implementation**: Add catch-all rewrite rule in `vercel.json`
- **Acceptance Criteria**:
  - [ ] Refreshing `/signin` returns React app (not 404)
  - [ ] Refreshing `/signup` returns React app (not 404)
  - [ ] Refreshing `/docs/chapter-1` returns Docusaurus page (not 404)
  - [ ] API routes `/api/*` still route to backend
  - [ ] Static assets (CSS, JS, images) still serve correctly

### FR-009.1-002: Dynamic API URL Detection
- **Priority**: P0 (Must Have)
- **Description**: Automatically detect localhost vs. production and use appropriate API URL
- **Logic**:
  ```javascript
  const isLocalhost = typeof window !== 'undefined' &&
    (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');
  export const API_BASE_URL = isLocalhost ? 'http://localhost:8000' : '';
  ```
- **Acceptance Criteria**:
  - [ ] Localhost: Uses `http://localhost:8000/api/chat`
  - [ ] Vercel: Uses `/api/chat` (relative URL, same domain)
  - [ ] No environment variable required (auto-detection)
  - [ ] Console logs clearly show which API URL is active
  - [ ] No CORS errors on Vercel (same-origin requests)

### FR-009.1-003: Clean Console Logs
- **Priority**: P1 (Should Have)
- **Description**: Remove or sanitize console logs that may expose sensitive data
- **Acceptance Criteria**:
  - [ ] No environment variable values logged
  - [ ] API URL logged for debugging (safe, no secrets)
  - [ ] Environment type logged (development/production)
  - [ ] Clear, helpful debugging messages

---

## 🔧 Technical Requirements

### TR-009.1-001: Update vercel.json
- **File**: `vercel.json`
- **Changes**:
  ```json
  {
    "version": 2,
    "builds": [
      {
        "src": "backend/api/index.py",
        "use": "@vercel/python"
      }
    ],
    "rewrites": [
      {
        "source": "/api/(.*)",
        "destination": "/backend/api/index.py"
      },
      {
        "source": "/(.*)",
        "destination": "/index.html"
      }
    ]
  }
  ```
- **Explanation**:
  - Rule 1: `/api/*` routes to Python backend (existing)
  - Rule 2: Everything else routes to `index.html` (NEW - enables SPA routing)
  - Order matters: API rule first, catch-all second

### TR-009.1-002: Update src/config/api.js
- **File**: `src/config/api.js`
- **Changes**:
  ```javascript
  /**
   * API Configuration
   *
   * Automatically detects localhost vs. production and uses appropriate API URL.
   *
   * Constitution Compliance:
   * - Principle IX: Zero-Edit Deployment (auto-detection, no env vars needed)
   */

  /**
   * Detect if running on localhost
   * - window.location.hostname === 'localhost' (dev server)
   * - window.location.hostname === '127.0.0.1' (alternative localhost)
   */
  const isLocalhost = typeof window !== 'undefined' &&
    (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');

  /**
   * Base URL for backend API
   * - Localhost: 'http://localhost:8000' (separate backend server)
   * - Production: '' (relative URLs, same domain as frontend)
   */
  export const API_BASE_URL = isLocalhost ? 'http://localhost:8000' : '';

  /**
   * Log API configuration for debugging
   * Safe to log: no secrets, just configuration
   */
  if (typeof console !== 'undefined') {
    console.log(`[API Config] Mode: ${isLocalhost ? 'LOCALHOST' : 'PRODUCTION'}`);
    console.log(`[API Config] API Base: ${API_BASE_URL || '(relative URLs)'}`);
    console.log(`[API Config] Example: ${API_BASE_URL}/api/chat`);
  }
  ```
- **Explanation**:
  - Auto-detects environment using `window.location.hostname`
  - Localhost: Absolute URL to separate backend (`http://localhost:8000`)
  - Production/Vercel: Empty string (relative URLs like `/api/chat`)
  - No environment variables needed (Constitution Principle IX)
  - Clear console logs for debugging (no secrets exposed)

---

## 🎯 Success Criteria

### Must Pass (P0)
- [ ] Refreshing `/signin` loads React app (no 404)
- [ ] Refreshing `/signup` loads React app (no 404)
- [ ] Chatbot works on Vercel (no "Failed to Fetch")
- [ ] API requests use relative URLs (`/api/chat`) on Vercel
- [ ] API requests use absolute URLs (`http://localhost:8000/api/chat`) on localhost
- [ ] No CORS errors on Vercel
- [ ] No console errors or warnings
- [ ] Console logs show correct API mode (LOCALHOST or PRODUCTION)

### Should Pass (P1)
- [ ] No environment variable values exposed in logs
- [ ] Clear debugging information in console
- [ ] GitHub Pages deployment ignored (Vercel is primary)

---

## 📋 Implementation Plan

### Phase 1: Fix vercel.json (SPA Routing)
1. Open `vercel.json`
2. Add catch-all rewrite rule: `/(.*) -> /index.html`
3. Ensure API rule comes first

### Phase 2: Fix src/config/api.js (Dynamic API)
1. Open `src/config/api.js`
2. Add `isLocalhost` detection using `window.location.hostname`
3. Set `API_BASE_URL = isLocalhost ? 'http://localhost:8000' : ''`
4. Update console logs (safe, no secrets)

### Phase 3: Test Locally
1. `npm start` - Verify localhost mode works
2. Check console logs show "Mode: LOCALHOST"
3. Test chatbot works on localhost

### Phase 4: Deploy to Vercel
1. `git add .`
2. `git commit -m "hotfix(009.1): fix SPA routing and dynamic API detection"`
3. `git push origin 009-ultimate-fix`
4. Verify Vercel auto-deploys
5. Test production: Refresh `/signin`, test chatbot

---

## 🛡️ Constitution Compliance

### Principle IX: Zero-Edit Deployment
- ✅ Auto-detects localhost vs. production (no manual config)
- ✅ No environment variables required for API URL
- ✅ Same code works in both environments

---

## ✅ Definition of Done

Hotfix is complete when:
1. [ ] `vercel.json` updated with SPA catch-all rule
2. [ ] `src/config/api.js` uses dynamic API detection
3. [ ] Tested locally (chatbot works on localhost)
4. [ ] Deployed to Vercel (auto-deployment)
5. [ ] Tested on Vercel (refreshing `/signin` works, chatbot works)
6. [ ] No console errors or warnings
7. [ ] Git commit and push completed

---

**Status**: ✅ **READY TO IMPLEMENT**
