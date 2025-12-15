# Research: Environment-Driven CORS and API Configuration

**Feature**: 001-cors-env-config
**Date**: 2025-12-12
**Phase**: Phase 0 (Research & Exploration)

## Research Summary

All unknowns from Technical Context have been resolved through codebase analysis and documentation review. No external research required—existing infrastructure already supports environment-driven configuration patterns.

## Decision Log

### Decision 1: Use Existing pydantic-settings Infrastructure

**Context**: Backend needs to load ALLOWED_ORIGINS from environment variables.

**Investigation**:
- Checked `backend/app/core/config.py`
- Found existing `Settings` class using `pydantic-settings.BaseSettings`
- Line 36: `allowed_origins: str = "http://localhost:3000"` field already exists
- Lines 56-59: `allowed_origins_list` property already splits comma-separated values

**Decision**: USE existing `settings.allowed_origins_list` property. No new code needed in config.py.

**Rationale**:
- ✅ Infrastructure already exists
- ✅ Type-safe environment variable loading
- ✅ Localhost default already configured
- ❌ Adding new code would duplicate existing functionality

**Alternatives Considered**:
1. **Direct os.getenv() in main.py**: Rejected—no type safety, no validation, duplicates existing pattern
2. **New configuration class**: Rejected—existing Settings class already handles this
3. **JSON/YAML config files**: Rejected—violates Zero-Edit Deployment principle

### Decision 2: Validate CORS Origins in main.py

**Context**: Spec FR-003 forbids wildcard origins with credentials=True. Spec FR-004 requires protocol validation.

**Investigation**:
- FastAPI CORSMiddleware docs: https://fastapi.tiangolo.com/tutorial/cors/
- Wildcard ["*"] with credentials=True raises RuntimeError in Starlette
- Current code (main.py:89) uses ["*"]—VIOLATES CORS security spec

**Decision**: Add validation logic in main.py before CORSMiddleware initialization.

**Validation Logic**:
```python
validated_origins = [
    origin for origin in settings.allowed_origins_list
    if origin.startswith(('http://', 'https://'))
]

if not validated_origins:
    validated_origins = ["http://localhost:3000", "http://127.0.0.1:3000"]

print(f"✅ Allowed CORS origins: {validated_origins}")
```

**Rationale**:
- ✅ Prevents wildcard origins (FR-003)
- ✅ Validates protocol (FR-004)
- ✅ Provides safe fallback if all origins invalid
- ✅ Logs for debugging (FR-005)

**Alternatives Considered**:
1. **Validation in config.py**: Rejected—separation of concerns, config loads raw data, main.py applies business logic
2. **Pydantic validator**: Rejected—would crash app on invalid env var, prefer graceful fallback
3. **No validation**: Rejected—violates FR-003, FR-004 requirements

### Decision 3: Use REACT_APP_API_URL for Frontend

**Context**: Frontend needs configurable API URL for production deployments.

**Investigation**:
- Docusaurus docs: https://docusaurus.io/docs/deployment#using-environment-variables
- Docusaurus uses Create React App pattern for env vars
- Variables prefixed `REACT_APP_` are available via `process.env.REACT_APP_*`
- Build-time substitution (static site generation)

**Decision**: Use `process.env.REACT_APP_API_URL || 'http://localhost:8000'` pattern.

**Implementation**:
```javascript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
// Use: fetch(`${API_BASE_URL}/api/chat`, ...)
```

**Rationale**:
- ✅ Standard React/Docusaurus pattern (Principle IV: Docusaurus Best Practices)
- ✅ Localhost fallback for local dev (FR-007)
- ✅ No hardcoded production URLs (FR-008)
- ✅ Build-time substitution (no runtime overhead)

**Alternatives Considered**:
1. **Relative path /api/chat**: Rejected—requires proxy setup, adds complexity for separate backend deployment
2. **Custom config.js**: Rejected—Docusaurus env vars are standard pattern
3. **Runtime window.env injection**: Rejected—requires server-side rendering or custom build step

### Decision 4: Document with .env.example Files

**Context**: Spec FR-010, FR-011 require environment variable documentation.

**Investigation**:
- `backend/.env.example` already exists with ALLOWED_ORIGINS (lines 31-35)
- Root directory has no .env.example for frontend variables
- .gitignore already excludes .env, .env.local (Docusaurus default)

**Decision**:
- Backend: Verify existing .env.example documentation (no changes needed)
- Frontend: Create root `.env.example` with REACT_APP_API_URL

**Rationale**:
- ✅ Standard practice for environment variable documentation
- ✅ Prevents accidental credential commits (.env in .gitignore)
- ✅ Clear examples for production deployment
- ✅ Follows twelve-factor app methodology

**Alternatives Considered**:
1. **README only**: Rejected—.env.example is conventional, easier to copy
2. **Inline comments in code**: Rejected—not discoverable during deployment
3. **Separate DEPLOYMENT.md**: Rejected—additional file, .env.example is standard

## Best Practices Applied

### FastAPI CORS Configuration
**Source**: https://fastapi.tiangolo.com/tutorial/cors/

- Explicit origin list (no wildcards with credentials)
- Environment variable driven (not hardcoded)
- Logged at startup for debugging
- Validated for protocol (http/https)

### Pydantic Settings
**Source**: https://docs.pydantic.dev/latest/concepts/pydantic_settings/

- BaseSettings for type-safe env var loading ✅ (already used)
- Default values for local development ✅ (already configured)
- .env file support ✅ (model_config already has env_file=".env")

### Docusaurus Environment Variables
**Source**: https://docusaurus.io/docs/deployment#using-environment-variables

- REACT_APP_ prefix for custom variables
- Build-time substitution via process.env
- .env.local for local overrides (not committed)
- .env.example for documentation

### Twelve-Factor App Configuration
**Source**: https://12factor.net/config

- Store config in environment variables ✅
- Strict separation of config from code ✅
- Never commit credentials ✅
- Same code across all environments ✅

## Technical Feasibility

### Backend Changes (Feasibility: ✅ HIGH)
- **Effort**: 10 lines of code change in main.py
- **Risk**: LOW—pydantic-settings already tested and working
- **Dependencies**: None new (pydantic-settings 2.1.0 already installed)
- **Testing**: Manual verification with server restart

### Frontend Changes (Feasibility: ✅ HIGH)
- **Effort**: 2 lines of code change in ChatWidget/index.js
- **Risk**: LOW—standard Docusaurus env var pattern
- **Dependencies**: None new (Docusaurus 3.x already installed)
- **Testing**: Manual verification with frontend rebuild

### Documentation (Feasibility: ✅ HIGH)
- **Effort**: 1 new file (.env.example in root), verify backend/.env.example
- **Risk**: NONE—documentation only
- **Dependencies**: None
- **Testing**: Visual inspection

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|-----------|
| User forgets ALLOWED_ORIGINS in production | Medium | High | Default to localhost (safe fallback), .env.example with examples, README deployment guide |
| Invalid origin format crashes backend | Low | Medium | Validation removes invalid origins, logs warning, provides localhost fallback |
| Wildcard origin with credentials | Low | Critical | Validation prevents wildcards, explicit FR-003 requirement |
| Frontend build-time vs runtime confusion | Low | Low | Documentation clarifies build-time substitution, .env.example comments |

## Open Questions

**NONE**—All research questions resolved.

## References

1. **FastAPI CORS**: https://fastapi.tiangolo.com/tutorial/cors/
2. **Pydantic Settings**: https://docs.pydantic.dev/latest/concepts/pydantic_settings/
3. **Docusaurus Env Vars**: https://docusaurus.io/docs/deployment#using-environment-variables
4. **Twelve-Factor Config**: https://12factor.net/config
5. **Existing Code**:
   - backend/app/core/config.py:36 (allowed_origins field)
   - backend/app/core/config.py:56-59 (allowed_origins_list property)
   - backend/main.py:87-93 (current CORS middleware—needs update)
   - backend/.env.example:31-35 (ALLOWED_ORIGINS documentation—already correct)
   - src/components/ChatWidget/index.js:106 (current fetch call—needs update)

## Next Steps

Proceed to **Phase 1: Design Artifacts** (data-model.md, quickstart.md, contracts/).
