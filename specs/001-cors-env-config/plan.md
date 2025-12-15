# Implementation Plan: Environment-Driven CORS and API Configuration

**Branch**: `001-cors-env-config` | **Date**: 2025-12-12 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-cors-env-config/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Refactor CORS origin and API URL configuration to use environment variables, eliminating manual code edits between local and production deployments. Backend will load allowed origins from `ALLOWED_ORIGINS` env var (defaults to localhost), and frontend will load API URL from `REACT_APP_API_URL` (defaults to localhost:8000). This implements Constitution Principle IX: Zero-Edit Deployment Configuration.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React 18.x (frontend Docusaurus)
**Primary Dependencies**: FastAPI 0.109.0, pydantic-settings 2.1.0, Docusaurus 3.x
**Storage**: N/A (configuration only)
**Testing**: Manual verification with server restart
**Target Platform**: Web application (FastAPI backend + Docusaurus frontend)
**Project Type**: Web (backend + frontend)
**Performance Goals**: No performance impact, zero-latency config loading at startup
**Constraints**: Must not break CORS security (no wildcards with credentials=True), must work locally without .env configuration
**Scale/Scope**: 2 files modified (backend/main.py, src/components/ChatWidget/index.js), 1 file updated (backend/.env.example)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle IX: Zero-Edit Deployment Configuration ✅ PASS
- **Requirement**: All environment-specific configuration MUST use environment variables
- **Status**: This feature directly implements Principle IX
- **Evidence**: Backend CORS will load from ALLOWED_ORIGINS, frontend API URL from REACT_APP_API_URL

### Principle V: Minimalism in Technology Stack ✅ PASS
- **Requirement**: Use smallest viable set of technologies
- **Status**: No new dependencies required (pydantic-settings already installed)
- **Evidence**: Using existing FastAPI + Docusaurus patterns for env vars

### Free-Tier Architecture (Principle III) ✅ PASS
- **Requirement**: Platform operates within free-tier constraints
- **Status**: No impact on service usage
- **Evidence**: Configuration change only, no new external services

### Simplicity-First Design (Principle I) ✅ PASS
- **Requirement**: Prioritize clarity and ease of understanding
- **Status**: Standard environment variable patterns, well-documented
- **Evidence**: .env.example will have clear examples and comments

## Project Structure

### Documentation (this feature)

```text
specs/001-cors-env-config/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── app/
│   └── core/
│       └── config.py          # Already has allowed_origins and allowed_origins_list
├── main.py                    # MODIFY: Replace hardcoded ["*"] with settings.allowed_origins_list
├── .env                       # Exists (user's local config, not committed)
└── .env.example               # UPDATE: Ensure ALLOWED_ORIGINS is documented with examples

frontend (Docusaurus root)/
├── src/
│   └── components/
│       └── ChatWidget/
│           └── index.js       # MODIFY: Replace hardcoded URL with process.env.REACT_APP_API_URL
├── docusaurus.config.js       # No changes needed (Docusaurus handles env vars natively)
└── .env.example               # CREATE: Document REACT_APP_API_URL for production deployments
```

**Structure Decision**: Web application structure (Option 2) with backend/ and frontend (root) directories. Backend uses Python/FastAPI with pydantic-settings for configuration. Frontend uses Docusaurus (React-based) with native `process.env` support for environment variables.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations detected. All constitution principles are satisfied by this implementation.*

## Phase 0: Research

### Research Questions

1. **Q**: Does pydantic-settings already support ALLOWED_ORIGINS env var?
   **A**: YES - `backend/app/core/config.py:36` already has `allowed_origins: str` field with default `"http://localhost:3000"`. The `allowed_origins_list` property (lines 56-59) splits comma-separated values.

2. **Q**: Does Docusaurus support REACT_APP_* environment variables pattern?
   **A**: YES - Docusaurus uses standard React environment variable pattern. Variables prefixed with `REACT_APP_` are available at build time via `process.env.REACT_APP_VARIABLE_NAME`. Source: https://docusaurus.io/docs/deployment#using-environment-variables

3. **Q**: What happens if CORS origins include wildcard ["*"] with credentials=True?
   **A**: FastAPI/Starlette CORSMiddleware will raise a RuntimeError. Spec FR-003 explicitly forbids this. Current code (backend/main.py:89-93) violates this by using `allow_origins=["*"]` with `allow_credentials=True`.

4. **Q**: What is the fallback behavior if env vars are missing?
   **A**: Backend defaults to `"http://localhost:3000"` (config.py:36). Frontend should default to `http://localhost:8000` using JavaScript fallback pattern: `process.env.REACT_APP_API_URL || 'http://localhost:8000'`.

### Best Practices

**Backend (FastAPI + Pydantic)**:
- Use pydantic-settings BaseSettings for type-safe env var loading ✅ (already implemented)
- Provide sensible localhost defaults for local development ✅ (already implemented)
- Validate origins have http/https protocol (Spec FR-004)
- Log allowed origins at startup for debugging (Spec FR-005)

**Frontend (Docusaurus/React)**:
- Use `REACT_APP_` prefix for custom environment variables
- Access via `process.env.REACT_APP_VARIABLE_NAME`
- Provide fallback value for local development: `const value = process.env.REACT_APP_VAR || 'default'`
- Document in .env.example for production deployments

**CORS Security**:
- NEVER use wildcard ["*"] with credentials=True
- Explicitly list allowed origins
- Validate origin format (protocol + domain)
- Log configuration at startup for audit trail

## Phase 1: Design Artifacts

### Data Model

*No database schema changes. Configuration-only feature.*

**Environment Variables Schema**:

Backend (.env):
```bash
# CORS Configuration
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000  # Comma-separated, no spaces
```

Frontend (build-time .env):
```bash
# API Configuration (Docusaurus/React pattern)
REACT_APP_API_URL=http://localhost:8000
```

### API Contracts

*No API endpoint changes. Configuration affects middleware behavior only.*

**CORS Middleware Behavior Contract**:

Input: `ALLOWED_ORIGINS` environment variable (comma-separated string)
Processing:
1. Load from env via pydantic-settings (backend/app/core/config.py)
2. Split by comma, trim whitespace (already implemented in `allowed_origins_list` property)
3. Validate each origin starts with http:// or https:// (FR-004)
4. Pass to CORSMiddleware `allow_origins` parameter

Output: Requests from listed origins receive CORS headers, others blocked

**Frontend API URL Resolution Contract**:

Input: `REACT_APP_API_URL` environment variable (optional)
Processing:
1. Read `process.env.REACT_APP_API_URL` at build time
2. Fallback to `'http://localhost:8000'` if undefined
3. Append `/api/chat` to base URL

Output: Fetch requests sent to correct API endpoint per environment

### Implementation Steps

#### Step 1: Backend CORS Configuration (backend/main.py)

**File**: backend/main.py
**Lines**: 87-93
**Current**:
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],           # ⚠️ HARDCODED WILDCARD (violates CORS spec)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**After**:
```python
# Validate and log CORS configuration (FR-004, FR-005)
validated_origins = [
    origin for origin in settings.allowed_origins_list
    if origin.startswith(('http://', 'https://'))
]

if not validated_origins:
    validated_origins = ["http://localhost:3000", "http://127.0.0.1:3000"]

print(f"✅ Allowed CORS origins: {validated_origins}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=validated_origins,  # Environment-driven, explicit list
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Rationale**: Use existing `settings.allowed_origins_list` property from config.py. Add validation (FR-004) and logging (FR-005). Provide localhost fallback if validation removes all origins.

#### Step 2: Frontend API URL Configuration (src/components/ChatWidget/index.js)

**File**: src/components/ChatWidget/index.js
**Lines**: ~1-10 (add constant at top)
**Current**: None (will add new constant)

**After**:
```javascript
// Environment-aware API URL (FR-006, FR-007)
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

**File**: src/components/ChatWidget/index.js
**Line**: 106
**Current**:
```javascript
const response = await fetch('http://localhost:8000/api/chat', {
```

**After**:
```javascript
const response = await fetch(`${API_BASE_URL}/api/chat`, {
```

**Rationale**: Extract hardcoded URL to configurable constant. Use Docusaurus/React env var pattern with localhost fallback.

#### Step 3: Update .env.example (backend/.env.example)

**File**: backend/.env.example
**Lines**: 31-35 (already exists, verify documentation)
**Current**:
```bash
# -----------------------------------------------------------------------------
# CORS Configuration
# -----------------------------------------------------------------------------
# Comma-separated list of allowed origins (no spaces)
# Example: https://yourusername.github.io,http://localhost:3000
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

**After**: Keep existing (already correct), ensure comments are clear

**Rationale**: .env.example already documents ALLOWED_ORIGINS correctly (lines 31-35). No changes needed, but verify during implementation.

#### Step 4: Create Frontend .env.example (Root Directory)

**File**: .env.example (create in repository root)
**Content**:
```bash
# =============================================================================
# Docusaurus Frontend Environment Variables
# =============================================================================
# Copy this file to .env.local for local overrides
# Build-time variables must be prefixed with REACT_APP_

# -----------------------------------------------------------------------------
# API Configuration
# -----------------------------------------------------------------------------
# Backend API base URL (without /api/chat path)
# Local development: http://localhost:8000
# Production: Set to your deployed backend URL (e.g., https://api.yourdomain.com)
REACT_APP_API_URL=http://localhost:8000
```

**Rationale**: Document frontend environment variable for production deployments (FR-011). Follows Docusaurus best practices.

### Verification Plan

1. **Local Development Test** (SC-001):
   - Fresh clone (or delete .env files)
   - Start backend: `cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
   - Start frontend: `npx docusaurus start`
   - Verify chat widget connects without CORS errors
   - Expected: Backend logs show `✅ Allowed CORS origins: ['http://localhost:3000', 'http://127.0.0.1:3000']`

2. **Production Simulation Test** (SC-002):
   - Set backend/.env: `ALLOWED_ORIGINS=https://myapp.vercel.app`
   - Set root/.env.local: `REACT_APP_API_URL=https://api.myapp.com`
   - Restart both servers
   - Verify backend logs show production origin
   - Verify frontend build includes production API URL

3. **Unauthorized Origin Test** (FR-002 validation):
   - Set `ALLOWED_ORIGINS=https://example.com`
   - Try accessing from localhost:3000
   - Expected: CORS error in browser console

4. **Wildcard Prevention Test** (FR-003 validation):
   - Set `ALLOWED_ORIGINS=*`
   - Start backend
   - Expected: Validation removes "*", falls back to localhost origins

### Dependencies and Assumptions

**Dependencies**:
- pydantic-settings 2.1.0+ (already installed ✅)
- FastAPI CORSMiddleware (already integrated ✅)
- Docusaurus 3.x with React env var support (already installed ✅)

**Assumptions**:
1. Users deploying to production can set environment variables on their platform (Vercel, Railway, etc.)
2. Frontend is built with environment variables at build time (standard Docusaurus/React pattern)
3. Backend restarts are acceptable to apply configuration changes
4. Localhost origins (3000, 127.0.0.1:3000) are safe to hardcode as defaults

### Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|-----------|
| User forgets to set ALLOWED_ORIGINS in production | High: CORS errors, frontend can't connect | .env.example documentation, README instructions, default fallback to localhost |
| Invalid origin format (missing protocol) | Medium: Origin ignored, potential connection failure | FR-004 validation removes invalid origins, logs warning |
| Wildcard with credentials | Critical: Security violation, FastAPI crash | FR-003 validation prevents wildcards, explicit list enforcement |
| Frontend .env.local not in .gitignore | Medium: Accidental secret exposure | Verify .gitignore includes .env.local (Docusaurus default) |

## Phase 2: Task Generation

**Deferred to `/sp.tasks` command.**

Tasks will be generated in dependency order covering:
1. Backend CORS configuration refactor (main.py)
2. Frontend API URL configuration (ChatWidget/index.js)
3. Documentation updates (.env.example files)
4. Verification tests (local dev, production simulation)

## Architectural Decision Records

### ADR Suggestion

📋 Architectural decision detected: **Environment-Variable Driven Configuration for Cross-Origin and API Endpoints**

**Context**: Previously used hardcoded CORS origins (["*"]) and hardcoded API URLs, requiring manual code changes for production deployment.

**Decision**: Use environment variables (ALLOWED_ORIGINS, REACT_APP_API_URL) with localhost defaults for local development.

**Alternatives Considered**:
1. **Configuration files** (.json, .yaml): Rejected because requires file edits and deployment of config artifacts
2. **Command-line arguments**: Rejected because hosting platforms typically use env vars, not CLI args
3. **Hardcoded multi-environment logic** (if/else for dev/staging/prod): Rejected because violates Zero-Edit Deployment principle

**Consequences**:
- ✅ Zero code changes between environments
- ✅ Aligns with twelve-factor app methodology
- ✅ Platform-agnostic (works on Vercel, Railway, etc.)
- ⚠️ Requires documentation for production deployment
- ⚠️ Frontend requires rebuild to change API URL (acceptable for Docusaurus static builds)

Document this reasoning? Run `/sp.adr environment-variable-configuration`

## Quality Gates

### Pre-Implementation
- [x] Constitution Principle IX compliance verified
- [x] No new dependencies required (using existing pydantic-settings)
- [x] Existing infrastructure supports env var pattern (config.py has allowed_origins)
- [x] Spec requirements are unambiguous and testable

### Post-Implementation
- [ ] Backend logs allowed origins at startup (FR-005)
- [ ] CORS wildcard with credentials prevented (FR-003)
- [ ] Local dev works without .env configuration (SC-001)
- [ ] .env.example files document all required variables (FR-010, FR-011)
- [ ] Frontend uses configurable API URL (FR-006, FR-007, FR-008)
- [ ] No hardcoded production URLs remain in source code (FR-008)

### Before Merge
- [ ] Manual testing: fresh clone works locally (SC-001)
- [ ] Manual testing: production env vars work correctly (SC-002)
- [ ] Manual testing: unauthorized origins blocked (FR-002)
- [ ] README updated with deployment instructions
- [ ] Git status clean (no .env files committed)

## References

- **Constitution**: `.specify/memory/constitution.md` (v1.2.0, Principle IX)
- **Spec**: `specs/001-cors-env-config/spec.md`
- **Existing Code**:
  - backend/app/core/config.py:36 (allowed_origins field)
  - backend/app/core/config.py:56-59 (allowed_origins_list property)
  - backend/main.py:87-93 (current CORS middleware)
  - src/components/ChatWidget/index.js:106 (current fetch call)
- **Docusaurus Environment Variables**: https://docusaurus.io/docs/deployment#using-environment-variables
- **FastAPI CORSMiddleware**: https://fastapi.tiangolo.com/tutorial/cors/
- **Twelve-Factor App**: https://12factor.net/config
