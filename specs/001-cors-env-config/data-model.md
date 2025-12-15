# Data Model: Environment-Driven CORS and API Configuration

**Feature**: 001-cors-env-config
**Date**: 2025-12-12
**Phase**: Phase 1 (Design Artifacts)

## Overview

This feature does NOT introduce new database entities or persistent data models. It modifies **runtime configuration loading** behavior only.

## Configuration Models

### Backend Environment Variables Schema

**Model**: Environment Variables (loaded via pydantic-settings)

```python
# backend/app/core/config.py (existing)
class Settings(BaseSettings):
    # ... other fields ...

    # CORS Configuration
    allowed_origins: str = "http://localhost:3000"

    @property
    def allowed_origins_list(self) -> List[str]:
        """Parse comma-separated ALLOWED_ORIGINS into a list."""
        return [origin.strip() for origin in self.allowed_origins.split(",")]
```

**Field Specification**:

| Field | Type | Default | Source | Validation |
|-------|------|---------|--------|------------|
| `allowed_origins` | str | `"http://localhost:3000"` | `ALLOWED_ORIGINS` env var | None (raw string) |
| `allowed_origins_list` | List[str] | `["http://localhost:3000"]` | Computed from `allowed_origins` | Split by comma, trim whitespace |

**Validation Rules** (applied in backend/main.py):
1. Each origin MUST start with `http://` or `https://` (FR-004)
2. Wildcard `*` is FORBIDDEN when credentials=True (FR-003)
3. If all origins invalid after validation, fallback to `["http://localhost:3000", "http://127.0.0.1:3000"]`

### Frontend Environment Variables Schema

**Model**: Build-time Environment Variables (Docusaurus/React pattern)

```javascript
// Loaded at build time via process.env
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

**Field Specification**:

| Field | Type | Default | Source | Validation |
|-------|------|---------|--------|------------|
| `REACT_APP_API_URL` | string | `'http://localhost:8000'` | Build-time env var | None (string fallback) |

**Usage Pattern**:
```javascript
// src/components/ChatWidget/index.js
const response = await fetch(`${API_BASE_URL}/api/chat`, {
  method: 'POST',
  // ... other fetch options
});
```

## State Transitions

**N/A** - Configuration is static per deployment. Changes require server/frontend restart.

### Backend CORS Configuration Lifecycle

```
1. App Startup
   ↓
2. Load Settings() class (pydantic-settings reads .env + env vars)
   ↓
3. Validate allowed_origins_list (main.py)
   ↓
4. Initialize CORSMiddleware with validated origins
   ↓
5. Log allowed origins
   ↓
6. App Ready (configuration immutable until restart)
```

### Frontend API URL Lifecycle

```
1. Docusaurus Build
   ↓
2. Read process.env.REACT_APP_API_URL (build-time substitution)
   ↓
3. Fallback to 'http://localhost:8000' if undefined
   ↓
4. Embed in static JavaScript bundle
   ↓
5. Client-side JavaScript uses embedded constant (immutable)
```

## Data Relationships

**N/A** - No entities, no relationships. Configuration is scalar values (strings).

## Storage Considerations

### Backend Configuration Storage

- **Source**: Environment variables set on hosting platform (Vercel, Railway, etc.) or local `.env` file
- **Format**: Plain text key-value pairs
- **Persistence**: Platform-managed (not in codebase, not in database)
- **Access**: Loaded once at app startup via pydantic-settings

### Frontend Configuration Storage

- **Source**: Environment variables set during build process
- **Format**: Build-time JavaScript constant substitution
- **Persistence**: Embedded in static JavaScript bundle (immutable until rebuild)
- **Access**: Direct access via `process.env` (no runtime overhead)

## Example Configurations

### Local Development

**Backend** (backend/.env):
```bash
# No .env file needed—defaults to localhost
# Optional: Explicitly set for clarity
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

**Frontend** (root/.env.local):
```bash
# No .env.local file needed—defaults to localhost:8000
# Optional: Explicitly set for proxy testing
REACT_APP_API_URL=http://localhost:8000
```

### Production Deployment

**Backend** (Platform env vars on Vercel/Railway):
```bash
ALLOWED_ORIGINS=https://myapp.vercel.app,https://www.myapp.com
```

**Frontend** (Vercel build environment):
```bash
REACT_APP_API_URL=https://api.myapp.com
```

### Multi-Environment Setup (Staging + Production)

**Staging Backend**:
```bash
ALLOWED_ORIGINS=https://staging.myapp.com
```

**Production Backend**:
```bash
ALLOWED_ORIGINS=https://myapp.com,https://www.myapp.com
```

**Staging Frontend**:
```bash
REACT_APP_API_URL=https://api-staging.myapp.com
```

**Production Frontend**:
```bash
REACT_APP_API_URL=https://api.myapp.com
```

## Validation and Constraints

### Backend Validation (main.py)

```python
# Pseudo-code for validation logic
validated_origins = [
    origin for origin in settings.allowed_origins_list
    if origin.startswith(('http://', 'https://'))
]

if not validated_origins:
    validated_origins = ["http://localhost:3000", "http://127.0.0.1:3000"]

# Log for audit trail (FR-005)
print(f"✅ Allowed CORS origins: {validated_origins}")
```

**Constraints**:
- Origins MUST NOT be wildcard `["*"]` when credentials=True (FR-003)
- Origins MUST start with `http://` or `https://` (FR-004)
- At least one valid origin MUST exist after validation (enforced by localhost fallback)

### Frontend Validation

**N/A** - No runtime validation. Build-time substitution with string fallback.

## Error Handling

### Backend Configuration Errors

| Error Scenario | Behavior | User Impact |
|----------------|----------|-------------|
| `ALLOWED_ORIGINS` not set | Use default `"http://localhost:3000"` | Local dev works, production needs manual env var |
| All origins invalid (no http/https) | Fallback to localhost origins | Safe default, logs warning |
| Wildcard `*` provided | Validation removes it, fallback to localhost | Prevents CORS security violation |
| Empty string | Split returns `[""]`, validation removes, fallback to localhost | Safe default |

### Frontend Configuration Errors

| Error Scenario | Behavior | User Impact |
|----------------|----------|-------------|
| `REACT_APP_API_URL` not set | Use default `'http://localhost:8000'` | Local dev works, production needs build-time env var |
| Invalid URL format | No validation—used as-is | Fetch will fail, user sees error in chat widget |

**Note**: Frontend does not validate URL format because:
1. Build-time validation would break builds
2. Runtime validation adds complexity
3. Invalid URL naturally fails with clear error message (FR-008 debug error handling already shows fetch errors)

## Migration and Backward Compatibility

### Migration Path

**From**: Hardcoded CORS origins `["*"]` and hardcoded API URL `'http://localhost:8000/api/chat'`
**To**: Environment-driven configuration

**Steps**:
1. Update backend/main.py CORS middleware (commit 1)
2. Update frontend ChatWidget API URL (commit 1)
3. Create root/.env.example (commit 1)
4. Deploy to staging with env vars set
5. Verify CORS and API connectivity
6. Deploy to production with production env vars

**Backward Compatibility**:
- ✅ Local development: Defaults to localhost (no breaking changes)
- ✅ Existing deployments: Need to add env vars (documented in .env.example)
- ❌ Current hardcoded `["*"]` is INSECURE—migration required

### Rollback Strategy

If issues occur:
1. Revert commit (git revert)
2. Redeploy previous version
3. Remove env vars from platform (optional)

**Data Loss Risk**: NONE—no persistent data changes

## Schema Versioning

**N/A** - Configuration schema is implicit (environment variable names). Changes require code updates.

**Future Schema Changes**:
- Adding new env vars: Backward compatible (existing deployments ignore new vars until code updated)
- Renaming env vars: Breaking change (requires documentation and migration guide)
- Removing env vars: Breaking change (requires fallback logic)

## References

- **Existing Code**: backend/app/core/config.py:36 (`allowed_origins` field)
- **Existing Code**: backend/app/core/config.py:56-59 (`allowed_origins_list` property)
- **Pydantic Settings Docs**: https://docs.pydantic.dev/latest/concepts/pydantic_settings/
- **Docusaurus Env Vars**: https://docusaurus.io/docs/deployment#using-environment-variables
