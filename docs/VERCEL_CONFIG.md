# Vercel Deployment Configuration Rules

**Purpose**: This document outlines the critical configuration rules established to prevent 405 Method Not Allowed errors when deploying FastAPI on Vercel Serverless Functions.

**Last Updated**: 2025-12-18
**Constitution**: Principle IX (Zero-Edit Deployment Configuration)

---

## 🚨 The 3 Critical Rules

These rules **MUST** be followed to ensure the FastAPI backend works correctly on Vercel:

### Rule 1: Explicit API Prefixes in main.py

**❌ WRONG** (Implicit prefixing in router definitions):
```python
# In backend/app/api/routes.py
router = APIRouter(prefix="/api", tags=["chat"])  # ❌ Don't do this

# In backend/main.py
app.include_router(chat_router)  # ❌ Prefix is hidden
```

**✅ CORRECT** (Explicit prefixing in main.py):
```python
# In backend/app/api/routes.py
router = APIRouter(tags=["chat"])  # ✅ No prefix here

# In backend/main.py
app.include_router(chat_router, prefix="/api")  # ✅ Explicit prefix
```

**Why**: All routing logic must be centralized in `main.py` for clarity. When routes are defined implicitly in multiple files, it's easy to lose track of the final URL structure, causing mismatches with frontend requests.

---

### Rule 2: Disable Trailing Slash Redirects

**❌ WRONG** (Default FastAPI behavior):
```python
app = FastAPI(
    title="API",
    version="1.0"
)
# redirect_slashes defaults to True ❌
```

**Problem**:
- Request: `POST /api/chat/` (with trailing slash)
- FastAPI returns: `307 Temporary Redirect` to `/api/chat`
- Result: POST + Redirect + CORS = **405 Method Not Allowed**

**✅ CORRECT** (Explicit configuration):
```python
app = FastAPI(
    title="API",
    version="1.0",
    redirect_slashes=False  # ✅ Disable automatic redirects
)
```

**Why**: In serverless/CORS environments, POST requests cannot reliably follow 307 redirects. Disabling `redirect_slashes` forces exact path matching, preventing confusing 405 errors. Mismatched paths now return 404 (clear failure) instead of 405 (ambiguous failure).

---

### Rule 3: Explicitly Allow OPTIONS in CORS

**❌ WRONG** (Missing or restricted methods):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["GET", "POST"]  # ❌ OPTIONS missing
)
```

**Problem**:
- Browser sends: `OPTIONS /api/chat` (CORS preflight)
- Server blocks: `405 Method Not Allowed`
- Result: Actual POST request never sent

**✅ CORRECT** (Wildcard includes OPTIONS):
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=validated_origins,
    allow_credentials=True,
    allow_methods=["*"],  # ✅ Includes OPTIONS, GET, POST, PUT, DELETE, etc.
    allow_headers=["*"]
)
```

**Why**: CORS preflight requests use the OPTIONS method. If OPTIONS is not allowed, all POST/PUT/DELETE requests fail with 405 before the actual request is even attempted.

---

## 📁 File Checklist

Before deploying to Vercel, verify these files:

### 1. `backend/main.py`

```python
# ✅ FastAPI initialization
app = FastAPI(
    title="...",
    version="...",
    redirect_slashes=False  # Rule 2: No redirects
)

# ✅ CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=validated_origins,
    allow_credentials=True,
    allow_methods=["*"],  # Rule 3: OPTIONS allowed
    allow_headers=["*"]
)

# ✅ Router registration
app.include_router(chat_router, prefix="/api")  # Rule 1: Explicit prefix
app.include_router(auth_router, prefix="/api/auth")  # Rule 1: Explicit prefix
```

### 2. `backend/api/index.py`

```python
# ✅ Clean import (no magic)
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from main import app  # Export app for Vercel

# ❌ DO NOT ADD:
# app.root_path = "/api"  # This causes 405 errors!
```

### 3. `backend/app/api/*.py` (Router files)

```python
# ✅ Router without prefix
router = APIRouter(tags=["..."])  # Rule 1: No prefix here

# ❌ DO NOT ADD:
# router = APIRouter(prefix="/api", tags=["..."])  # Keep prefixes in main.py
```

### 4. `vercel.json`

```json
{
  "rewrites": [
    { "source": "/api/(.*)", "destination": "/backend/api/index.py" },
    { "source": "/((?!api/|assets/|static/|.*\\..*).*)", "destination": "/index.html" }
  ]
}
```

**Explanation**:
- First rule: Route `/api/*` to FastAPI backend
- Second rule: Route clean URLs (no extensions) to SPA for Docusaurus routing

---

## 🐛 Common Mistakes & How to Avoid Them

### Mistake 1: Using `root_path` to Strip Prefixes

```python
# ❌ WRONG
app.root_path = "/api"  # Attempts to strip /api from incoming paths
```

**Why it fails**: In Vercel's serverless environment, `root_path` manipulation is unreliable and causes routes to mismatch. The path stripping may work locally but break in production.

**Solution**: Use explicit prefixes in `main.py` (Rule 1).

---

### Mistake 2: Defining Routes with Inconsistent Slashes

```python
# ❌ WRONG
@router.post("/chat")  # Route without slash
# Frontend calls: POST /api/chat/ (with slash)
# Result: 307 Redirect → 405
```

**Solution**:
- Always disable `redirect_slashes=False` (Rule 2)
- Ensure frontend and backend agree on path format (no trailing slashes)

---

### Mistake 3: Forgetting CORS Preflight

```python
# ❌ WRONG
allow_methods=["GET", "POST"]  # OPTIONS not explicitly listed
```

**Solution**: Use `allow_methods=["*"]` to ensure OPTIONS is included (Rule 3).

---

## 🧪 Testing Checklist

Before pushing to production:

### Local Testing (Development)

```bash
# 1. Start backend
cd backend
uvicorn main:app --reload --port 8000

# 2. Test POST endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "test", "session_id": "test123"}'

# Expected: 200 OK
```

### Vercel Testing (Production)

```bash
# 1. After deploying, test API directly
curl -X POST https://your-app.vercel.app/api/chat \
  -H "Content-Type: application/json" \
  -H "Origin: https://your-app.vercel.app" \
  -d '{"message": "test", "session_id": "test123"}'

# Expected: 200 OK (not 405)

# 2. Test CORS preflight
curl -X OPTIONS https://your-app.vercel.app/api/chat \
  -H "Origin: https://your-app.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -H "Access-Control-Request-Headers: Content-Type"

# Expected: 200 OK with CORS headers
```

### Browser Console Checks

1. Open DevTools → Network tab
2. Make a POST request from frontend
3. Verify:
   - ✅ OPTIONS request returns 200 OK
   - ✅ POST request returns 200 OK (not 405)
   - ✅ CORS headers present in response

---

## 🔄 Deployment Workflow

### Before Merging to Main

1. ✅ Verify all 3 rules are followed
2. ✅ Test locally with `uvicorn`
3. ✅ Check `backend/api/index.py` has no `root_path`
4. ✅ Confirm routers use explicit prefixes in `main.py`
5. ✅ Ensure `redirect_slashes=False` in FastAPI init

### After Deploying to Vercel

1. ✅ Wait 2-5 minutes for rebuild
2. ✅ Hard refresh browser (`Ctrl+Shift+R`)
3. ✅ Test POST requests from frontend
4. ✅ Check Vercel logs for startup errors
5. ✅ Verify CORS headers in response

---

## 📚 Related Features

- **Feature 009.1**: Hotfix - Dynamic API Detection
- **Feature 012.1**: API Method Fix (Initial root_path attempt)
- **Feature 012.2**: Explicit API Routing (Removed root_path, added prefixes)
- **Feature 012.3**: Disable Slash Redirects (Added redirect_slashes=False)
- **Feature 012.4**: Verification & Documentation (This document)

---

## 🚀 Summary

**The 3 Rules** (memorize these):
1. **Explicit Prefixes**: All `/api` prefixes in `main.py`, not router files
2. **No Redirects**: `redirect_slashes=False` to prevent 307 → 405
3. **CORS Wildcards**: `allow_methods=["*"]` to include OPTIONS

**Result**: FastAPI backend works reliably on Vercel without 405 errors.

---

**Constitution Compliance**: Principle IX - Zero-Edit Deployment Configuration

**Last Verified**: 2025-12-18
**Author**: Claude Sonnet 4.5 (SpecKit Plus Workflow)
