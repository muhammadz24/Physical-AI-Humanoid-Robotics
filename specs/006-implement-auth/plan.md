# Implementation Plan: Email/Password Authentication with User Experience Data Collection

**Branch**: `006-implement-auth` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/006-implement-auth/spec.md`

## Summary

Implement email/password authentication to collect user background data (software/hardware experience levels) for bonus points. The system will use the existing FastAPI backend to handle auth endpoints, store user data in the existing Neon Postgres database, and create React-based signup/signin pages in the Docusaurus frontend.

**Key Architectural Decision**: The spec references "Better-Auth" and "Next.js", but the actual project uses **Docusaurus v3** (SSG framework) for the frontend per Constitution Principle IV (NON-NEGOTIABLE). Better-Auth requires Next.js server-side rendering capabilities. Therefore, this plan adapts the requirements to use **FastAPI-based authentication** (backend) + **React pages in Docusaurus** (frontend), which aligns with the existing architecture and constitutional constraints.

## Technical Context

**Language/Version**: Python 3.13 (backend), React 18 / Docusaurus 3.0 (frontend)
**Primary Dependencies**: FastAPI 0.115.5, passlib (password hashing), python-jose (JWT tokens), pg8000 (Postgres driver), React 18, @docusaurus/core 3.0
**Storage**: Neon Postgres (existing DATABASE_URL configured in backend/.env)
**Testing**: pytest (backend), manual frontend testing
**Target Platform**: Web application (Docusaurus static site + FastAPI backend API)
**Project Type**: Web (Docusaurus frontend + FastAPI backend)
**Performance Goals**:
- Signup completion: <2 minutes (SC-001)
- Signin response: <10 seconds (SC-002)
- Session persistence: 5+ page refreshes (SC-003)
**Constraints**:
- Must use existing Neon Postgres database (free tier)
- Docusaurus SSG (no server-side rendering) - cannot use Better-Auth
- Must NOT add Google/GitHub social logins (FR-018)
- Must collect software_experience and hardware_experience (PDF compliance)
**Scale/Scope**: Low-scale (hundreds of users expected), single database table addition

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Principle I (Simplicity-First Design)**:
- ✅ PASS: Adding authentication is justified for bonus points requirement
- ✅ PASS: Using existing infrastructure (FastAPI + Postgres) rather than adding new services
- ⚠️  CONCERN: Spec mentions "Better-Auth" which requires Next.js, but project uses Docusaurus

**Principle III (Free-Tier Architecture)**:
- ✅ PASS: Uses existing Neon Postgres (free tier, no additional database cost)
- ✅ PASS: No paid auth providers (implementing custom email/password)
- ✅ PASS: Session storage in database or JWT tokens (no Redis/external cache)

**Principle IV (Docusaurus Best Practices Compliance - NON-NEGOTIABLE)**:
- ❌ **VIOLATION**: Spec assumes Next.js frontend, but project IS Docusaurus
- ✅ **RESOLUTION**: This plan adapts to use Docusaurus + FastAPI backend for auth
- ✅ PASS: Auth pages will be standard Docusaurus React pages (src/pages/signup.js, src/pages/signin.js)
- ✅ PASS: Following Docusaurus routing patterns (file-based routing)

**Principle V (Minimalism in Technology Stack)**:
- ✅ PASS: Frontend is Docusaurus (already in stack)
- ✅ PASS: Backend is FastAPI (already in stack)
- ✅ PASS: Database is Neon Postgres (already in stack)
- ❌ **WOULD VIOLATE**: Adding Better-Auth + Next.js would require new tech stack
- ✅ **RESOLUTION**: Using FastAPI auth (Python standard: passlib + JWT)

**Principle IX (Zero-Edit Deployment Configuration)**:
- ✅ PASS: DATABASE_URL already uses environment variable
- ✅ PASS: CORS already configured with environment variables per Principle IX
- ✅ PASS: JWT secret will use environment variable (JWT_SECRET_KEY)
- ✅ PASS: No hardcoded URLs or credentials

**Constitution Compliance Summary**:
- **Status**: PASS with architectural adaptation
- **Critical Change**: Cannot use Better-Auth (Next.js only); using FastAPI auth instead
- **Justification**: Maintains constitutional compliance (Docusaurus + FastAPI stack), uses existing infrastructure, achieves all functional requirements

## Complexity Tracking

> **Filled because Constitution Check requires architectural adaptation**

| Spec Assumption | Why Changed | Rationale |
|-----------------|-------------|-----------|
| Better-Auth library | Project is Docusaurus, not Next.js | Better-Auth requires Next.js server-side capabilities. Docusaurus is SSG-only. Using FastAPI auth preserves existing stack. |
| Next.js frontend | Constitution Principle IV mandates Docusaurus | Project already uses Docusaurus v3. Adding Next.js would violate Principle V (Minimalism). |
| Frontend-side auth | Docusaurus has no server component | FastAPI backend will handle auth endpoints. Docusaurus pages will make fetch() calls to API. |

## Project Structure

### Documentation (this feature)

```text
specs/006-implement-auth/
├── plan.md              # This file
├── research.md          # Phase 0: Auth patterns, JWT vs sessions, password hashing
├── data-model.md        # Phase 1: User entity schema
├── contracts/           # Phase 1: API endpoint contracts
│   └── auth-api.yaml    # OpenAPI spec for auth endpoints
└── quickstart.md        # Phase 1: Local setup guide
```

### Source Code (repository root)

```text
backend/
├── app/
│   ├── core/
│   │   ├── config.py            # Add JWT_SECRET_KEY env var
│   │   ├── security.py          # NEW: Password hashing, JWT utils
│   │   └── database.py          # Existing DB connection
│   ├── models/
│   │   └── user.py              # NEW: User model (Pydantic)
│   ├── api/
│   │   ├── routes.py            # Existing chat route
│   │   └── auth.py              # NEW: /api/auth/signup, /signin, /logout, /me
│   ├── services/
│   │   └── user_service.py      # NEW: User CRUD operations
│   └── main.py                  # Add auth router
├── migrations/
│   └── 001_create_users_table.sql   # NEW: SQL migration
└── .env                         # Add JWT_SECRET_KEY

src/
├── pages/
│   ├── signup.js                # NEW: Signup form (React)
│   ├── signin.js                # NEW: Signin form (React)
│   └── index.js                 # Existing homepage
├── components/
│   ├── AuthProvider.js          # NEW: React context for auth state
│   ├── ProtectedRoute.js        # NEW: Client-side route protection
│   └── Navbar.js                # UPDATE: Add Login/Logout buttons
└── services/
    └── authService.js           # NEW: API calls to backend auth endpoints

.env.example                     # UPDATE: Add JWT_SECRET_KEY template
```

**Structure Decision**: Web application with separate backend (FastAPI) and frontend (Docusaurus). Auth logic in backend, UI in Docusaurus React pages. Follows Constitution Principle IV (Docusaurus) and Principle V (existing stack).

## Phase 0: Research & Technology Decisions

### Research Tasks

1. **Password Hashing in Python**
   - Research: passlib vs bcrypt vs argon2
   - Decision: Use passlib[bcrypt] (FastAPI standard, widely used)
   - Rationale: Industry-standard, well-documented, compatible with FastAPI

2. **Session Management Strategy**
   - Research: JWT tokens vs database sessions vs cookies
   - Decision: JWT tokens in httpOnly cookies
   - Rationale: Stateless (no session table), works with Docusaurus SSG, secure (httpOnly prevents XSS)

3. **Postgres Schema for Auth**
   - Research: UUID vs auto-increment IDs, enum vs text for experience levels
   - Decision: UUID primary key, TEXT type with CHECK constraints for enums
   - Rationale: UUIDs prevent enumeration attacks, TEXT with CHECK is Postgres-standard for small enums

4. **CORS Configuration for Auth**
   - Research: Credentials mode, allowed origins for cookies
   - Decision: CORS allow credentials=true, existing ALLOWED_ORIGINS
   - Rationale: Cookies require credentials mode, existing CORS setup already environment-aware (Principle IX)

5. **Frontend State Management**
   - Research: React Context vs Redux vs Zustand for auth state
   - Decision: React Context API
   - Rationale: Simplest solution (Principle I), no extra dependencies (Principle V)

**Output**: research.md documenting all decisions with rationale

## Phase 1: Data Model & API Contracts

### Data Model (data-model.md)

**Entity: User**

| Field | Type | Constraints | Notes |
|-------|------|-------------|-------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() | Unique identifier |
| name | VARCHAR(100) | NOT NULL | User's display name |
| email | VARCHAR(255) | NOT NULL, UNIQUE | User's email (login identifier) |
| hashed_password | VARCHAR(255) | NOT NULL | bcrypt hashed password |
| software_experience | VARCHAR(20) | NOT NULL, CHECK (software_experience IN ('beginner', 'intermediate', 'pro')) | PDF compliance field |
| hardware_experience | VARCHAR(20) | NOT NULL, CHECK (hardware_experience IN ('none', 'arduino', 'ros', 'professional')) | PDF compliance field |
| created_at | TIMESTAMP | DEFAULT NOW() | Registration timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW() | Last update timestamp |

**Indexes**:
- `CREATE UNIQUE INDEX idx_users_email ON users(email);` (for login lookups)

**Entity: Session (Not Stored)**

Using JWT tokens (stateless), no session table needed. JWT payload contains:
```json
{
  "user_id": "uuid",
  "email": "string",
  "exp": "timestamp"
}
```

### API Contracts (contracts/auth-api.yaml)

**POST /api/auth/signup**
- Request: `{ "name": "string", "email": "string", "password": "string", "software_experience": "beginner|intermediate|pro", "hardware_experience": "none|arduino|ros|professional" }`
- Response 201: `{ "user": { "id": "uuid", "name": "string", "email": "string", "software_experience": "string", "hardware_experience": "string" }, "message": "Account created successfully" }`
- Response 400: `{ "detail": "Email already registered" }` or validation errors
- Sets cookie: `auth_token` (httpOnly, secure, sameSite=lax)

**POST /api/auth/signin**
- Request: `{ "email": "string", "password": "string" }`
- Response 200: `{ "user": { "id": "uuid", "name": "string", "email": "string", "software_experience": "string", "hardware_experience": "string" }, "message": "Signed in successfully" }`
- Response 401: `{ "detail": "Invalid email or password" }`
- Sets cookie: `auth_token` (httpOnly, secure, sameSite=lax)

**POST /api/auth/logout**
- Request: None (reads cookie)
- Response 200: `{ "message": "Logged out successfully" }`
- Clears cookie: `auth_token`

**GET /api/auth/me**
- Request: None (reads cookie)
- Response 200: `{ "user": { "id": "uuid", "name": "string", "email": "string", "software_experience": "string", "hardware_experience": "string" } }`
- Response 401: `{ "detail": "Not authenticated" }`

### Frontend Flow

1. **Signup Flow**:
   - User navigates to `/signup`
   - Fills form (name, email, password, software dropdown, hardware dropdown)
   - JavaScript submits POST /api/auth/signup
   - Backend creates user, returns JWT in cookie
   - Frontend redirects to `/` (homepage)

2. **Signin Flow**:
   - User navigates to `/signin`
   - Fills form (email, password)
   - JavaScript submits POST /api/auth/signin
   - Backend validates, returns JWT in cookie
   - Frontend redirects to `/`

3. **Session Check**:
   - On page load, call GET /api/auth/me
   - If 200: user is authenticated, store in React Context
   - If 401: user is not authenticated

4. **Logout Flow**:
   - User clicks "Logout" button
   - JavaScript calls POST /api/auth/logout
   - Backend clears cookie
   - Frontend clears context, redirects to `/signin`

### Quickstart (quickstart.md)

**Backend Setup**:
1. Add to `backend/.env`: `JWT_SECRET_KEY=<generate-random-string>`
2. Run migration: `psql $DATABASE_URL < migrations/001_create_users_table.sql`
3. Install dependencies: `pip install "passlib[bcrypt]" python-jose`
4. Restart backend: `python -m uvicorn main:app --reload`

**Frontend Setup**:
1. No new dependencies needed (React + fetch built-in)
2. Create pages: `src/pages/signup.js`, `src/pages/signin.js`
3. Create components: `src/components/AuthProvider.js`
4. Update Navbar to show Login/Logout buttons
5. Test: Navigate to `http://localhost:3000/signup`

## File-by-File Implementation Details

### Backend Files

**File 1: `backend/app/core/security.py` (NEW)**
- Purpose: Password hashing and JWT token generation/verification
- Key functions:
  - `hash_password(password: str) -> str`: Uses passlib.context.CryptContext with bcrypt
  - `verify_password(plain_password: str, hashed_password: str) -> bool`
  - `create_access_token(data: dict, expires_delta: timedelta = None) -> str`: JWT encoding
  - `decode_access_token(token: str) -> dict`: JWT decoding with validation
- Dependencies: `from passlib.context import CryptContext`, `from jose import JWTError, jwt`

**File 2: `backend/app/models/user.py` (NEW)**
- Purpose: Pydantic models for User entity
- Models:
  - `UserCreate(BaseModel)`: For signup (name, email, password, software_experience, hardware_experience)
  - `UserLogin(BaseModel)`: For signin (email, password)
  - `UserResponse(BaseModel)`: For API responses (id, name, email, software_experience, hardware_experience, created_at)
- Validation: Email format, password min length (8 chars), enum values for experience fields

**File 3: `backend/app/services/user_service.py` (NEW)**
- Purpose: User database operations
- Functions:
  - `create_user(user: UserCreate, db_connection) -> UserResponse`: Insert user, return created record
  - `get_user_by_email(email: str, db_connection) -> dict | None`: Query by email
  - `get_user_by_id(user_id: str, db_connection) -> dict | None`: Query by ID
- Uses `backend/app/core/database.py` existing connection pool

**File 4: `backend/app/api/auth.py` (NEW)**
- Purpose: Auth API endpoints
- Routes:
  - `POST /signup`: Calls `create_user()`, sets JWT cookie, returns 201
  - `POST /signin`: Calls `get_user_by_email()`, verifies password, sets JWT cookie, returns 200
  - `POST /logout`: Clears JWT cookie, returns 200
  - `GET /me`: Reads JWT cookie, validates, returns user data or 401
- Dependencies: FastAPI Router, security.py functions, user_service.py functions

**File 5: `backend/main.py` (UPDATE)**
- Change: Add `app.include_router(auth.router, prefix="/api/auth", tags=["auth"])`
- Location: After existing chat router inclusion

**File 6: `backend/app/core/config.py` (UPDATE)**
- Change: Add `jwt_secret_key: str` field to Settings class
- Validation: Raise error if not set (required for security)

**File 7: `backend/migrations/001_create_users_table.sql` (NEW)**
- SQL to create users table with all fields and constraints
- Includes indexes for email lookup
- Idempotent (IF NOT EXISTS checks)

### Frontend Files

**File 8: `src/pages/signup.js` (NEW)**
- React component with form:
  - Text inputs: Name, Email, Password
  - Dropdowns: Software Experience (Beginner/Intermediate/Pro), Hardware Experience (None/Arduino/ROS/Professional)
  - Submit button
- On submit: `fetch('http://localhost:8000/api/auth/signup', { method: 'POST', credentials: 'include', body: JSON.stringify(formData) })`
- On success: Redirect to `/`
- On error: Display validation errors

**File 9: `src/pages/signin.js` (NEW)**
- React component with form:
  - Text inputs: Email, Password
  - Submit button
- On submit: `fetch('http://localhost:8000/api/auth/signin', { method: 'POST', credentials: 'include', body: JSON.stringify(formData) })`
- On success: Redirect to `/`
- On error: Display "Invalid email or password"

**File 10: `src/components/AuthProvider.js` (NEW)**
- React Context provider
- State: `{ user: UserResponse | null, loading: boolean }`
- useEffect: On mount, call `GET /api/auth/me` to check session
- Methods: `login(user)`, `logout()`
- Wraps app in Layout or root component

**File 11: `src/theme/Navbar/index.js` (UPDATE or CREATE)**
- Add conditional rendering:
  - If authenticated: Show "Logout" button (calls authContext.logout())
  - If not authenticated: Show "Login" link (to `/signin`)
- Uses `useContext(AuthContext)` to check auth state

**File 12: `.env.example` (UPDATE)**
- Add: `JWT_SECRET_KEY=your-secret-key-here-min-32-chars`
- Document: "Generate with: openssl rand -hex 32"

## Deployment Configuration (Principle IX Compliance)

**Backend Environment Variables**:
- `DATABASE_URL`: Already configured (existing)
- `ALLOWED_ORIGINS`: Already configured (existing, per Principle IX)
- `JWT_SECRET_KEY`: NEW, required for production (must be set on hosting platform)

**Frontend Environment Variables**:
- `REACT_APP_API_URL`: Optional (defaults to http://localhost:8000 for local dev)
- Production: Set to deployed backend URL (e.g., https://api.example.com)

**Zero-Edit Guarantee**:
- ✅ No hardcoded backend URLs (uses environment variable or localhost default)
- ✅ No hardcoded JWT secrets (environment variable required)
- ✅ No hardcoded CORS origins (already environment-aware)
- ✅ Database URL already uses environment variable

## Security Considerations

1. **Password Storage**: bcrypt hashing with salt (passlib default cost=12)
2. **JWT Secret**: 32+ character random string (environment variable)
3. **Cookie Security**: httpOnly (prevents XSS), secure flag in production (HTTPS only), sameSite=lax (CSRF protection)
4. **SQL Injection**: Using parameterized queries (pg8000 prepared statements)
5. **Email Uniqueness**: Database constraint prevents duplicates
6. **CORS**: Existing configuration already limits origins (Principle IX)

## Testing Strategy

**Backend Tests** (pytest):
1. Test password hashing: `test_hash_password()`, `test_verify_password()`
2. Test JWT creation: `test_create_access_token()`, `test_decode_access_token()`
3. Test signup endpoint: `test_signup_success()`, `test_signup_duplicate_email()`, `test_signup_validation()`
4. Test signin endpoint: `test_signin_success()`, `test_signin_invalid_credentials()`
5. Test /me endpoint: `test_me_authenticated()`, `test_me_unauthenticated()`

**Frontend Tests** (manual):
1. Signup form validation (missing fields, short password)
2. Signup success flow (form → API → redirect)
3. Signin success flow
4. Session persistence (refresh page, check /me call)
5. Logout flow (clear session, redirect)

**Integration Tests**:
1. Full user journey: Signup → Logout → Signin → Verify session
2. Duplicate email prevention
3. Invalid credentials handling

## Rollback Plan

1. **Database**: Keep migration reversible (DROP TABLE users IF EXISTS)
2. **Backend**: Git revert of auth.py, security.py, user_service.py, user.py
3. **Frontend**: Git revert of signup.js, signin.js, AuthProvider.js, Navbar changes
4. **Environment**: Remove JWT_SECRET_KEY from .env (optional, no impact if unused)

**Estimated Rollback Time**: <10 minutes (git revert + restart backend)

## Success Criteria Mapping

From spec.md Success Criteria:

- **SC-001** (Signup <2 min): Form is simple (5 fields), no email verification delay
- **SC-002** (Signin <10 sec): FastAPI response time <1s, redirect is instant
- **SC-003** (Session persistence 5+ refreshes): JWT expiry set to 7 days (default)
- **SC-004** (100% experience data capture): Database constraints enforce NOT NULL
- **SC-005** (Reject duplicate emails): UNIQUE constraint on email + validation
- **SC-006** (Reject invalid credentials): Password verification in signin endpoint
- **SC-007** (Logout clears session): Cookie cleared, /me returns 401 after logout
- **SC-008** (Zero plain-text passwords): passlib bcrypt hashing (verified in tests)
- **SC-009** (PDF compliance): Signup form includes both experience dropdowns (mandatory fields)

## Next Steps

1. **Phase 0**: Complete research.md with final decisions on passlib, JWT, and Postgres schema
2. **Phase 1**: Generate data-model.md, contracts/auth-api.yaml, quickstart.md
3. **Phase 2** (separate /sp.tasks command): Generate tasks.md with step-by-step implementation tasks
4. **Implementation**: Follow tasks.md to build backend auth endpoints, frontend pages, and database migration
5. **Testing**: Run pytest backend tests, manual frontend testing
6. **Deployment**: Set JWT_SECRET_KEY on hosting platform, run migration on production database

**Estimated Implementation Time**: 4-6 hours (backend auth logic + frontend forms + testing)
