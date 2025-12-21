# Tasks: Hotfix 009.1 - Smart Routing & Dynamic API

**Feature**: 009.1-hotfix-smart-routing
**Branch**: 009-ultimate-fix
**Status**: In Progress
**Priority**: P0 (CRITICAL HOTFIX)

---

## Task Status Legend
- `[ ]` Pending
- `[x]` Completed
- `[~]` In Progress

---

## Phase 1: Fix SPA Routing

### Task 1.1: Update vercel.json
**Status**: [x] Completed
**File**: `vercel.json`

**Changes**:
Add catch-all rewrite rule for SPA routing:
```json
{
  "source": "/(.*)",
  "destination": "/index.html"
}
```

**Success Criteria**:
- [FR-009.1-001] Refreshing `/signin` returns React app (not 404)
- [FR-009.1-001] Refreshing `/signup` returns React app (not 404)

---

## Phase 2: Fix Dynamic API Detection

### Task 2.1: Update src/config/api.js
**Status**: [x] Completed
**File**: `src/config/api.js`

**Changes**:
Replace environment variable logic with hostname detection:
```javascript
const isLocalhost = typeof window !== 'undefined' &&
  (window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1');
export const API_BASE_URL = isLocalhost ? 'http://localhost:8000' : '';
```

**Success Criteria**:
- [FR-009.1-002] Localhost uses `http://localhost:8000`
- [FR-009.1-002] Vercel uses relative URLs (empty base)
- [FR-009.1-003] Console logs show mode and API URL

---

## Phase 3: Testing

### Task 3.1: Local Testing
**Status**: [ ] Pending

**Commands**:
```bash
npm start
# Check console: Mode: LOCALHOST
# Test chatbot works
```

### Task 3.2: Production Testing
**Status**: [ ] Pending

**Commands**:
```bash
# After deploy to Vercel:
# 1. Refresh /signin - should work
# 2. Test chatbot - should connect
# 3. Check console: Mode: PRODUCTION
```

---

## Phase 4: Deployment

### Task 4.1: Git Commit & Push
**Status**: [ ] Pending

**Commands**:
```bash
git add .
git commit -m "hotfix(009.1): fix SPA routing and dynamic API detection"
git push origin 009-ultimate-fix
```

---

## Summary

**Total Tasks**: 5
**Completed**: 2
**Pending**: 3

**Next Steps**:
1. Update `vercel.json`
2. Update `src/config/api.js`
3. Test locally
4. Push to Vercel
5. Test production

---

**Estimated Time**: 10-15 minutes
**Critical**: YES (production is broken)
