# Quickstart: Fix Frontend Environment Variable Crash

**Feature**: 002-fix-frontend-env-crash
**Date**: 2025-12-14
**Target Audience**: Developers

## Overview

This guide shows how to verify the frontend environment variable crash fix. The fix prevents `ReferenceError: process is not defined` errors in the browser by using a safe typeof check pattern.

## Prerequisites

- Backend: Feature 001-cors-env-config implemented (environment-driven CORS)
- Frontend: Docusaurus 3.x with ChatWidget component
- Browser: Any modern browser (Chrome, Firefox, Safari, Edge)

## Quick Verification (30 seconds)

### Step 1: Start Backend

```bash
cd backend
python -m uvicorn main:app --host 0.0.0.0 --port 8000
```

**Expected Output**:
```
✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Step 2: Start Frontend

```bash
# From repository root
npx docusaurus start
```

**Expected Output**:
```
[SUCCESS] Docusaurus website is running at http://localhost:3000/
```

### Step 3: Verify No Crash

1. Open browser: http://localhost:3000
2. **OLD BEHAVIOR (BEFORE FIX)**: White screen, console error:
   ```
   Uncaught ReferenceError: process is not defined
   ```
3. **NEW BEHAVIOR (AFTER FIX)**: Page loads successfully, chat button visible in bottom-right corner

4. Open DevTools Console (F12)
5. **VERIFY**: NO "ReferenceError: process is not defined" errors ✅

**Success Criteria**: Page loads, chat widget visible, no console errors

## Detailed Testing

### Test 1: Browser Console Verification (SC-004)

**Purpose**: Verify no JavaScript runtime errors

**Steps**:
1. Open http://localhost:3000
2. Press F12 to open DevTools
3. Go to Console tab
4. Refresh page (Ctrl+R or Cmd+R)
5. Check for errors

**Expected**:
- ✅ NO "ReferenceError: process is not defined"
- ✅ NO "Cannot read property 'env' of undefined"
- ✅ Console is clean (only normal Docusaurus logs)

**Failure Signs**:
- ❌ Red errors in console
- ❌ White screen / blank page
- ❌ Chat button missing

### Test 2: Page Load Test (SC-001, SC-002)

**Purpose**: Verify page renders correctly and quickly

**Steps**:
1. Clear browser cache (Ctrl+Shift+Delete)
2. Navigate to http://localhost:3000
3. Time how long it takes to see chat button

**Expected**:
- ✅ Page loads in <1 second
- ✅ Chat button appears in bottom-right corner
- ✅ Full page renders (no white screen)

**Metrics**:
- Load time: <1 second (SC-001)
- Render success rate: 100% (SC-002)

### Test 3: API Connection Test (SC-003)

**Purpose**: Verify chat widget connects to correct API URL

**Steps**:
1. Open http://localhost:3000
2. Press F12 → Network tab
3. Click chat button (bottom-right)
4. Type test message: "Hello"
5. Send message
6. Check Network tab for fetch request

**Expected**:
- ✅ Request URL: http://localhost:8000/api/chat (NOT http://undefined/api/chat)
- ✅ Request method: POST
- ✅ Request completes (may fail with 500 if no OpenAI key, but URL is correct)

**Failure Signs**:
- ❌ URL contains "undefined": http://undefined/api/chat
- ❌ No network request appears
- ❌ Request to wrong port (not 8000)

### Test 4: Environment Variable Test (Production Simulation)

**Purpose**: Verify typeof check doesn't break webpack replacement

**Steps**:
1. Create `.env.local` in repository root:
   ```bash
   REACT_APP_API_URL=http://test-backend:9000
   ```

2. Rebuild frontend:
   ```bash
   npm run build
   ```

3. Inspect build output:
   ```bash
   # Search for API URL in bundled JavaScript
   grep -r "test-backend:9000" build/
   ```

**Expected**:
- ✅ Build succeeds without errors
- ✅ Bundled code contains "test-backend:9000" (webpack replaced env var)
- ✅ Bundled code does NOT contain "localhost:8000" (env var took precedence)

**Note**: This test confirms webpack DefinePlugin still works with typeof check

## Troubleshooting

### Issue: Still seeing "process is not defined" error

**Symptom**: Console shows `ReferenceError: process is not defined`

**Causes**:
1. Fix not applied yet (ChatWidget/index.js line 5 still has unsafe code)
2. Frontend not restarted after applying fix
3. Browser cache serving old JavaScript bundle

**Fix**:
1. Verify ChatWidget/index.js line 5 has typeof check:
   ```javascript
   const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && process.env.REACT_APP_API_URL)
   ```
2. Kill frontend process (Ctrl+C)
3. Clear browser cache (Ctrl+Shift+Delete)
4. Restart frontend: `npx docusaurus start`
5. Hard refresh browser (Ctrl+Shift+R)

### Issue: Chat widget not connecting to backend

**Symptom**: Network tab shows no requests to /api/chat, or requests to wrong URL

**Causes**:
1. Backend not running
2. Backend CORS not configured (feature 001 not implemented)
3. Frontend using wrong API URL

**Fix**:
1. Verify backend running on localhost:8000
2. Check backend logs for CORS origins: `✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']`
3. Open Network tab, send message, verify URL is http://localhost:8000/api/chat

### Issue: White screen persists

**Symptom**: Page doesn't load, completely blank

**Causes**:
1. Different JavaScript error (not process.env related)
2. React component crash
3. Build error

**Fix**:
1. Open Console tab (F12)
2. Look for ANY red errors
3. Fix errors from top to bottom
4. If error is NOT "process is not defined", may be unrelated issue

## Verification Checklist

Before considering the fix complete, verify ALL items:

- [ ] Browser console shows NO "process is not defined" errors
- [ ] Page loads in <1 second
- [ ] Chat button appears in bottom-right corner
- [ ] Clicking chat button opens chat window
- [ ] Network tab shows fetch to http://localhost:8000/api/chat
- [ ] No white screen or blank page
- [ ] Hard refresh (Ctrl+Shift+R) still works without errors

**All items checked**: ✅ Fix is working correctly

## Next Steps

After verification passes:
1. Test on different browsers (Chrome, Firefox, Safari)
2. Test with production env var (REACT_APP_API_URL set)
3. Deploy to staging environment
4. Monitor browser error logs for any remaining issues

## References

- **Spec**: specs/002-fix-frontend-env-crash/spec.md
- **Plan**: specs/002-fix-frontend-env-crash/plan.md
- **Modified File**: src/components/ChatWidget/index.js:5
- **Parent Feature**: 001-cors-env-config
