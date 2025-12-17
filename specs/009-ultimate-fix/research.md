# Research: Ultimate Fix Release

**Feature**: 009-ultimate-fix
**Date**: 2025-12-17
**Purpose**: Resolve all technical uncertainties before design phase

## Research Areas

### 1. Windows Unicode Handling in Python

**Decision**: Replace emoji characters with ASCII equivalents in log messages

**Rationale**:
- Windows console (Command Prompt, PowerShell) uses codepages (cp1252, cp437) that don't support UTF-8 emojis by default
- Python 3.x on Windows may default to `cp1252` encoding for stdout/stderr
- Emojis like ✅ (U+2705) and ❌ (U+274C) cause `UnicodeEncodeError` when printed
- ASCII-safe alternatives: `[OK]`, `[SUCCESS]`, `[FAIL]`, `[ERROR]`
- Alternative: Force UTF-8 with `PYTHONIOENCODING=utf-8`, but this requires user configuration

**Alternatives Considered**:
- **Force UTF-8 encoding**: `sys.stdout.reconfigure(encoding='utf-8')` - Rejected because it adds complexity and may not work in all Windows terminals
- **Suppress errors**: `errors='ignore'` or `errors='replace'` - Rejected because it hides the issue without fixing root cause
- **Keep emojis**: Use `try/except` blocks - Rejected because it clutters code and doesn't solve the problem

**Best Practice**: Use ASCII-only characters for console output; reserve Unicode/emojis for web UIs or logs that explicitly support UTF-8

---

### 2. Vercel FastAPI Deployment Architecture

**Decision**: Use Vercel Serverless Functions with rewrites to route `/api/*` to FastAPI ASGI app

**Rationale**:
- Vercel natively supports Python serverless functions via `vercel.json` configuration
- FastAPI must be exposed as an ASGI app (not running `uvicorn.run()` directly)
- Entry point pattern: `backend/api/index.py` exports the FastAPI `app` instance
- Vercel rewrites in `vercel.json` map HTTP requests from `/api/*` to the serverless function
- Cold start times: ~1-2s for Python functions (acceptable for this use case)

**Alternatives Considered**:
- **Deploy backend separately (Railway, Render)**: Rejected because it adds deployment complexity and separate service management
- **Use Vercel Edge Functions**: Rejected because they don't support Python (only JavaScript/TypeScript)
- **Monorepo with Next.js API routes**: Rejected because existing FastAPI backend would need complete rewrite

**Best Practice**:
- `vercel.json` rewrites: `{"source": "/api/(.*)", "destination": "/backend/api/index.py"}`
- Entry point exports app: `from app.main import app` (no `if __name__ == "__main__"` block)
- Environment variables set in Vercel dashboard

---

### 3. Environment Variable Security Patterns

**Decision**: Use `os.getenv()` with explicit validation and fail-fast on missing required variables

**Rationale**:
- Hardcoded secrets in source code are exposed in version control history (even if later removed)
- Environment variables keep secrets out of codebase and allow different values per environment
- Python pattern: `os.getenv("KEY_NAME")` returns `None` if not set; use `os.environ["KEY_NAME"]` to raise error
- Best practice: Validate all required env vars at startup, fail with clear error message if missing

**Alternatives Considered**:
- **Use `.env` files committed to repo**: Rejected due to security risk (secrets in version control)
- **Encrypt secrets in code**: Rejected because encryption keys still need to be managed
- **Use secrets management service (AWS Secrets Manager)**: Rejected because it's overkill for this project and not free-tier

**Best Practice**:
```python
import os

# Required variables (fail fast if missing)
DATABASE_URL = os.environ["DATABASE_URL"]  # Raises KeyError if missing
API_KEY = os.environ["OPENAI_API_KEY"]

# Optional variables (with defaults)
DEBUG = os.getenv("DEBUG", "false").lower() == "true"
PORT = int(os.getenv("PORT", "8000"))
```

**Frontend (Next.js/Docusaurus)**:
```javascript
// Build-time variables (NEXT_PUBLIC_* or REACT_APP_*)
const API_URL = process.env.NEXT_PUBLIC_API_URL || "http://localhost:8000";
```

---

### 4. FastAPI CORS Configuration for Production

**Decision**: Use dynamic CORS origins list with environment variable for production domain

**Rationale**:
- Local development needs `localhost:3000` and `127.0.0.1:3000` for frontend dev server
- Production needs deployed frontend URL (e.g., `https://yourapp.vercel.app`)
- Hardcoding production URL in code violates zero-edit deployment principle (Constitution IX)
- Solution: Default to localhost, append production origin from `ALLOWED_ORIGIN` env var

**Alternatives Considered**:
- **Allow all origins (`allow_origins=["*"]`)**: Acceptable for public APIs with no authentication, but less secure
- **Hardcode production URL**: Rejected because it requires code changes for deployment
- **Use regex pattern**: Overly complex for this use case

**Best Practice**:
```python
from fastapi.middleware.cors import CORSMiddleware
import os

origins = [
    "http://localhost:3000",
    "http://127.0.0.1:3000",
]

prod_origin = os.getenv("ALLOWED_ORIGIN")
if prod_origin:
    origins.append(prod_origin)

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

### 5. Database Migration Strategy for SQLite/PostgreSQL

**Decision**: Use plain SQL migration files with sequential numbering (001_, 002_, etc.)

**Rationale**:
- Current project uses asyncpg (PostgreSQL) and likely Neon Postgres free tier
- No ORM detected (SQLAlchemy, Django ORM), so no built-in migrations
- Plain SQL migrations are simple, explicit, and work with any database driver
- Sequential numbering ensures order: `001_initial.sql`, `002_create_chat_messages.sql`, `003_add_auto_delete.sql`
- Idempotency: Use `CREATE TABLE IF NOT EXISTS` and `ALTER TABLE IF EXISTS`

**Alternatives Considered**:
- **Alembic (SQLAlchemy migrations)**: Rejected because no SQLAlchemy ORM is in use
- **Django migrations**: Not applicable (not a Django project)
- **Custom Python migration runner**: Overkill for 2-3 simple migrations

**Best Practice**:
```sql
-- 002_create_chat_messages.sql
CREATE TABLE IF NOT EXISTS chat_messages (
    id SERIAL PRIMARY KEY,
    role VARCHAR(20) NOT NULL CHECK (role IN ('user', 'assistant', 'system')),
    content TEXT NOT NULL,
    user_id INTEGER REFERENCES users(id),
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX IF NOT EXISTS idx_chat_messages_user_id ON chat_messages(user_id);
CREATE INDEX IF NOT EXISTS idx_chat_messages_created_at ON chat_messages(created_at DESC);
```

```sql
-- 003_add_auto_delete.sql
-- Add retention policy: delete messages older than 30 days
CREATE OR REPLACE FUNCTION auto_delete_old_messages()
RETURNS void AS $$
BEGIN
    DELETE FROM chat_messages WHERE created_at < NOW() - INTERVAL '30 days';
END;
$$ LANGUAGE plpgsql;

-- Schedule periodic cleanup (requires pg_cron extension or external cron)
-- Note: This is a placeholder; actual scheduling depends on Neon Postgres capabilities
```

**Migration Runner** (simple Python script):
```python
import asyncpg
import os
from pathlib import Path

async def run_migrations(db_url: str, migrations_dir: Path):
    conn = await asyncpg.connect(db_url)
    try:
        for sql_file in sorted(migrations_dir.glob("*.sql")):
            print(f"Running migration: {sql_file.name}")
            sql = sql_file.read_text()
            await conn.execute(sql)
            print(f"✓ {sql_file.name} completed")
    finally:
        await conn.close()
```

---

### 6. CSS Z-Index Layer Management

**Decision**: Establish z-index scale with navbar at 100, chat widget at 99

**Rationale**:
- Z-index conflicts occur when components compete for same layer
- Standard practice: Define z-index scale (e.g., 10 increments: 10, 20, 30...100)
- Navbar is primary navigation → highest priority (z-index: 100)
- Chat widget is secondary → below navbar (z-index: 99)
- Modal overlays (if added later) would be 200+

**Alternatives Considered**:
- **Use CSS custom properties for z-index**: Good practice but overkill for 2 components
- **Use very high numbers (9999)**: Makes it harder to insert layers later
- **Rely on DOM order**: Unreliable; z-index is explicit and maintainable

**Best Practice**:
```css
/* Z-Index Scale */
:root {
  --z-base: 0;
  --z-dropdown: 10;
  --z-sticky: 50;
  --z-chat-widget: 99;
  --z-navbar: 100;
  --z-modal: 200;
  --z-tooltip: 300;
}

/* Apply to components */
.navbar {
  z-index: var(--z-navbar);
  position: fixed;
  top: 0;
}

.chat-widget {
  z-index: var(--z-chat-widget);
  position: fixed;
  bottom: 20px;
  right: 20px;
}

@media (max-width: 768px) {
  .chat-widget {
    z-index: var(--z-chat-widget); /* Ensure it's still below navbar */
  }
}
```

---

### 7. Python Dependency Pinning Strategy

**Decision**: Use `pip freeze > requirements.txt` for exact version pinning

**Rationale**:
- `pip freeze` outputs all installed packages with exact versions (e.g., `fastapi==0.104.1`)
- Ensures reproducible builds: same versions installed in dev, staging, and production
- Prevents breaking changes from automatic updates
- Alternative: `pip-tools` (pip-compile) - more advanced but adds complexity

**Alternatives Considered**:
- **Use version ranges** (`fastapi>=0.104`): Rejected because it allows breaking changes
- **Use Poetry or Pipenv**: Rejected because existing project uses pip
- **Manual version specification**: Error-prone and incomplete

**Best Practice**:
```bash
# Generate requirements.txt from current environment
pip freeze > requirements.txt

# Install from requirements.txt
pip install -r requirements.txt

# Optional: separate dev dependencies
pip freeze | grep -E "pytest|black|flake8" > requirements-dev.txt
```

**Version Pinning Policy**:
- Pin all direct dependencies with `==` (exact version)
- Review and update dependencies quarterly or when security patches are released
- Use `pip list --outdated` to check for updates
- Test updates in staging before production

---

## Summary of Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Windows Unicode | Replace emojis with ASCII (`[OK]`, `[FAIL]`) | Avoids `UnicodeEncodeError` on Windows consoles |
| Vercel Deployment | Use `vercel.json` rewrites + `backend/api/index.py` entry point | Native Vercel serverless function support for Python |
| Environment Variables | Use `os.getenv()` with fail-fast validation | Keeps secrets out of code, enables zero-edit deployment |
| CORS Configuration | Dynamic origins list with `ALLOWED_ORIGIN` env var | Supports local dev + production without code changes |
| Database Migrations | Sequential SQL files with idempotent DDL | Simple, explicit, no ORM dependency |
| Z-Index Management | Navbar: 100, Chat Widget: 99 | Clear layering hierarchy, prevents overlap |
| Dependency Pinning | `pip freeze > requirements.txt` | Reproducible builds with exact versions |

---

## Open Questions (None Remaining)

All technical uncertainties have been resolved. Ready to proceed to Phase 1 (Design & Contracts).
