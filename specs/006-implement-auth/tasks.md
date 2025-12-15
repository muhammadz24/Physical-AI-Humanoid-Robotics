# Implementation Tasks: Email/Password Authentication

**Feature Branch**: `006-implement-auth`
**Created**: 2025-12-15
**Input**: Design documents from `/specs/006-implement-auth/`
**Prerequisites**: plan.md, spec.md

**Architecture Note**: This feature uses **FastAPI backend + Docusaurus React frontend** (NOT Better-Auth/Next.js as originally specified). This adaptation maintains constitutional compliance with Principle IV (Docusaurus mandatory) and Principle V (Minimalism).

**Tests**: Tests are manual for this feature (no automated test tasks included per spec).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` for source code
- **Frontend**: `src/` for Docusaurus pages and components
- **Migrations**: `backend/migrations/` for SQL scripts
- **Config**: `.env` files in `backend/` and root

---

## Phase 1: Setup (Backend Environment)

**Purpose**: Prepare backend environment for authentication implementation

- [ ] T001 Install Python dependencies passlib[bcrypt] and python-jose[cryptography] in backend/requirements.txt
- [ ] T002 Generate JWT secret key using command: `openssl rand -hex 32`
- [ ] T003 Add JWT_SECRET_KEY to backend/.env file with generated value
- [ ] T004 Update .env.example with JWT_SECRET_KEY template and generation instructions

**Checkpoint**: Backend environment ready for auth implementation

---

## Phase 2: Foundational (Database Schema)

**Purpose**: Database schema that MUST be complete before ANY user story implementation

**⚠️ CRITICAL**: All user stories depend on the users table existing in the database

- [ ] T005 Create database migration file backend/migrations/001_create_users_table.sql with users table schema (UUID primary key, email unique index, CHECK constraints for experience enums)
- [ ] T006 Connect to Neon Postgres database using DATABASE_URL from backend/.env
- [ ] T007 Execute migration SQL to create users table with fields: id (UUID), name, email, hashed_password, software_experience, hardware_experience, created_at, updated_at
- [ ] T008 Verify users table exists in database with correct schema and constraints

**Checkpoint**: Database foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - User Registration with Experience Data (Priority: P1) 🎯 MVP

**Goal**: Users can create an account by providing name, email, password, and experience levels (software and hardware), with data stored in Postgres and session created via JWT cookie.

**Independent Test**: Navigate to http://localhost:3000/signup, fill form with valid data including both experience dropdowns, submit, verify user record created in database with all fields populated, and JWT cookie set in browser.

**PDF Compliance**: This story implements the REQUIRED Software Experience and Hardware Experience dropdown fields for bonus points.

### Backend Implementation for User Story 1

- [ ] T009 [P] [US1] Create backend/app/core/security.py with password hashing functions (hash_password using passlib.CryptContext with bcrypt, verify_password)
- [ ] T010 [P] [US1] Create backend/app/core/security.py with JWT functions (create_access_token using python-jose, decode_access_token with validation)
- [ ] T011 [P] [US1] Create backend/app/models/user.py with UserCreate Pydantic model (fields: name, email, password, software_experience enum validation for beginner/intermediate/pro, hardware_experience enum validation for none/arduino/ros/professional)
- [ ] T012 [P] [US1] Create backend/app/models/user.py with UserResponse Pydantic model (fields: id, name, email, software_experience, hardware_experience, created_at - excludes hashed_password)
- [ ] T013 [US1] Create backend/app/services/user_service.py with create_user function (takes UserCreate, hashes password using security.hash_password, inserts into users table via database connection, returns UserResponse)
- [ ] T014 [US1] Create backend/app/services/user_service.py with get_user_by_email function (queries users table by email, returns user dict or None)
- [ ] T015 [US1] Create backend/app/api/auth.py with FastAPI router and POST /signup endpoint (validates UserCreate, calls create_user, creates JWT token, sets httpOnly cookie auth_token with sameSite=lax, returns 201 with UserResponse)
- [ ] T016 [US1] Add duplicate email validation to POST /signup endpoint in backend/app/api/auth.py (catch database unique constraint error, return 400 with message "Email already registered")
- [ ] T017 [US1] Update backend/app/core/config.py Settings class to add jwt_secret_key field with validation (raise error if not set in environment)
- [ ] T018 [US1] Update backend/main.py to include auth router with prefix="/api/auth" and tags=["auth"] (add after existing chat router)

### Frontend Implementation for User Story 1

- [ ] T019 [US1] Create src/pages/signup.js React component with Docusaurus Layout wrapper
- [ ] T020 [US1] Add signup form to src/pages/signup.js with input fields: Name (text), Email (email type), Password (password type, min 8 chars validation)
- [ ] T021 [US1] Add Software Experience dropdown to src/pages/signup.js with options: Beginner, Intermediate, Pro (REQUIRED for PDF compliance - bonus points)
- [ ] T022 [US1] Add Hardware Experience dropdown to src/pages/signup.js with options: None, Arduino, ROS, Professional (REQUIRED for PDF compliance - bonus points)
- [ ] T023 [US1] Add form submission handler to src/pages/signup.js that POSTs to http://localhost:8000/api/auth/signup with credentials: 'include' (includes cookies in request)
- [ ] T024 [US1] Add success handler to src/pages/signup.js that redirects to / (homepage) using window.location.href or useHistory
- [ ] T025 [US1] Add error handler to src/pages/signup.js to display validation errors from API response (show "Email already registered" or field-specific errors)
- [ ] T026 [US1] Add form validation to src/pages/signup.js for required fields and password minimum length before API call

### Integration Testing for User Story 1

- [ ] T027 [US1] Start backend server on port 8000 and verify /api/auth/signup endpoint is accessible
- [ ] T028 [US1] Start Docusaurus dev server on port 3000 and navigate to http://localhost:3000/signup
- [ ] T029 [US1] Test successful signup flow: fill all fields including both experience dropdowns, submit, verify redirect to homepage, check users table for new record
- [ ] T030 [US1] Test duplicate email validation: attempt signup with existing email, verify 400 error and "Email already registered" message displayed
- [ ] T031 [US1] Test password validation: attempt signup with password <8 chars, verify client-side or server-side validation error
- [ ] T032 [US1] Test required fields validation: attempt signup with missing name/email/password/experience fields, verify error messages

**Checkpoint**: User Story 1 complete - users can successfully register with experience data, database stores all fields correctly, JWT cookies are set

---

## Phase 4: User Story 2 - User Sign In (Priority: P2)

**Goal**: Registered users can sign in with email and password to access their personalized experience and continue where they left off.

**Independent Test**: Navigate to http://localhost:3000/signin, enter valid credentials from User Story 1, submit, verify successful login with redirect to homepage and JWT cookie set.

### Backend Implementation for User Story 2

- [ ] T033 [P] [US2] Create backend/app/models/user.py with UserLogin Pydantic model (fields: email, password)
- [ ] T034 [US2] Create POST /signin endpoint in backend/app/api/auth.py (validates UserLogin, calls get_user_by_email, verifies password using security.verify_password, creates JWT token, sets httpOnly cookie, returns 200 with UserResponse)
- [ ] T035 [US2] Add authentication failure handling to POST /signin in backend/app/api/auth.py (return 401 with message "Invalid email or password" for wrong email or password)

### Frontend Implementation for User Story 2

- [ ] T036 [US2] Create src/pages/signin.js React component with Docusaurus Layout wrapper
- [ ] T037 [US2] Add signin form to src/pages/signin.js with input fields: Email (email type), Password (password type)
- [ ] T038 [US2] Add form submission handler to src/pages/signin.js that POSTs to http://localhost:8000/api/auth/signin with credentials: 'include'
- [ ] T039 [US2] Add success handler to src/pages/signin.js that redirects to / (homepage)
- [ ] T040 [US2] Add error handler to src/pages/signin.js to display "Invalid email or password" message on 401 response
- [ ] T041 [US2] Add redirect logic to src/pages/signin.js: if already authenticated (check session), redirect to homepage automatically

### Integration Testing for User Story 2

- [ ] T042 [US2] Test successful signin flow: navigate to /signin, enter valid credentials, verify redirect to homepage and JWT cookie set
- [ ] T043 [US2] Test invalid password: enter correct email but wrong password, verify 401 error and "Invalid email or password" displayed
- [ ] T044 [US2] Test non-existent email: enter email not in database, verify 401 error and "Invalid email or password" displayed
- [ ] T045 [US2] Test already authenticated redirect: sign in, then manually navigate to /signin again, verify automatic redirect to homepage

**Checkpoint**: User Story 2 complete - users can successfully sign in with credentials, sessions are created via JWT cookies

---

## Phase 5: User Story 3 - Session Persistence and Logout (Priority: P3)

**Goal**: Signed-in users' sessions persist across page refreshes and browser tabs, and users can securely log out to end their session.

**Independent Test**: Sign in via /signin, refresh page multiple times, open site in new tab, verify user remains logged in. Click logout, verify session cleared and redirected to /signin.

### Backend Implementation for User Story 3

- [ ] T046 [P] [US3] Create GET /me endpoint in backend/app/api/auth.py (reads auth_token cookie, decodes JWT using security.decode_access_token, queries user by ID, returns 200 with UserResponse or 401 if invalid/missing token)
- [ ] T047 [P] [US3] Create POST /logout endpoint in backend/app/api/auth.py (clears auth_token cookie by setting max_age=0, returns 200 with message "Logged out successfully")
- [ ] T048 [US3] Configure JWT token expiration in backend/app/core/security.py create_access_token to 7 days (timedelta(days=7))

### Frontend Implementation for User Story 3

- [ ] T049 [US3] Create src/components/AuthProvider.js React Context with state for user (UserResponse | null) and loading (boolean)
- [ ] T050 [US3] Add useEffect to src/components/AuthProvider.js to call GET /api/auth/me on component mount (check session on page load)
- [ ] T051 [US3] Add login method to AuthProvider context that updates user state
- [ ] T052 [US3] Add logout method to AuthProvider context that calls POST /api/auth/logout, clears user state, and redirects to /signin
- [ ] T053 [US3] Create or update src/theme/Navbar/index.js to swizzle Docusaurus Navbar component (if doesn't exist, run: npm run swizzle @docusaurus/theme-classic Navbar -- --eject)
- [ ] T054 [US3] Add conditional rendering to Navbar: if authenticated (useContext(AuthContext)), show Logout button; else show Login link to /signin
- [ ] T055 [US3] Connect Logout button in Navbar to authContext.logout() method
- [ ] T056 [US3] Wrap Docusaurus app with AuthProvider in src/theme/Root.js (create if doesn't exist, export default function Root({ children }) wrapping children with <AuthProvider>)

### Integration Testing for User Story 3

- [ ] T057 [US3] Test session persistence: sign in, refresh page 5+ times, verify user remains logged in and Navbar shows Logout button
- [ ] T058 [US3] Test cross-tab session: sign in on tab 1, open new tab 2, verify both tabs show authenticated state
- [ ] T059 [US3] Test logout flow: sign in, click Logout button in Navbar, verify redirect to /signin and Navbar shows Login link
- [ ] T060 [US3] Test /me endpoint directly: sign in, call GET /api/auth/me from browser console, verify 200 with user data returned
- [ ] T061 [US3] Test expired session: manually delete auth_token cookie, refresh page, verify Navbar shows Login link (unauthenticated state)

**Checkpoint**: User Story 3 complete - sessions persist across refreshes, users can log out, auth state is globally managed via React Context

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and documentation

- [ ] T062 [P] Update README.md with authentication setup instructions (JWT_SECRET_KEY generation, environment variables)
- [ ] T063 [P] Update .env.example with all required auth environment variables (JWT_SECRET_KEY with generation command comment)
- [ ] T064 Test full user journey end-to-end: Signup → Logout → Signin → Refresh page → Logout
- [ ] T065 Verify all success criteria from spec.md are met: SC-001 (signup <2 min), SC-002 (signin <10 sec), SC-003 (session persistence 5+ refreshes), SC-004 (experience data stored), SC-005 (duplicate email rejected), SC-006 (invalid credentials rejected), SC-007 (logout clears session), SC-008 (passwords hashed), SC-009 (PDF compliance dropdowns)
- [ ] T066 Check database for test users created during development, optionally clean up test data
- [ ] T067 Verify JWT_SECRET_KEY is NOT hardcoded anywhere in source code (only loaded from environment variable)
- [ ] T068 Verify CORS configuration allows credentials for auth cookies (ALLOWED_ORIGINS already configured per Principle IX)
- [ ] T069 Run backend FastAPI server and verify all /api/auth endpoints return correct responses and status codes
- [ ] T070 Run Docusaurus dev server and verify /signup, /signin pages render correctly with proper form validation

**Checkpoint**: Feature complete, all user stories tested, documentation updated, ready for deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup (Phase 1) completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2) - Can start after database is ready
- **User Story 2 (Phase 4)**: Depends on Foundational (Phase 2) AND User Story 1 backend models - Can start after US1 backend complete
- **User Story 3 (Phase 5)**: Depends on User Story 1 AND 2 - Needs signup/signin to exist for session management
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Independent - only depends on Foundational phase (database schema)
- **User Story 2 (P2)**: Depends on US1 backend models (UserResponse, security functions, user_service) - signup must exist for users to sign in
- **User Story 3 (P3)**: Depends on US1 and US2 - session management requires existing signup/signin functionality

### Task Dependencies Within User Stories

**User Story 1**:
- T009-T012 (security, models) can run in parallel [P]
- T013-T014 (services) depend on T009-T012 complete (need security and models)
- T015-T018 (API endpoints, config) depend on T013-T014 (need services)
- T019-T026 (frontend) can start in parallel with backend after T015 complete (API endpoint exists)
- T027-T032 (integration testing) require all above tasks complete

**User Story 2**:
- T033 (model) can run in parallel with T034-T035 if UserResponse from US1 is reused [P]
- T034-T035 (backend) depend on US1 T013-T014 (need get_user_by_email and verify_password)
- T036-T041 (frontend) can start after T034 complete (API endpoint exists)
- T042-T045 (testing) require all above complete

**User Story 3**:
- T046-T048 (backend endpoints) can run in parallel [P]
- T049-T052 (AuthProvider) can run in parallel with T046-T048 [P]
- T053-T056 (Navbar integration) depend on T049-T052 (need AuthProvider context)
- T057-T061 (testing) require all above complete

### Parallel Opportunities

**Phase 1 (Setup)**: All 4 tasks can run in parallel if team has capacity
- T001-T004 are independent file edits

**Phase 2 (Foundational)**: Sequential (database operations must be done in order)

**Phase 3 (User Story 1)**: High parallelization potential
- Backend parallel group: T009, T010, T011, T012 (different functions/classes)
- After backend foundation: T015-T018 and T019-T026 can overlap (backend API + frontend UI)

**Phase 4 (User Story 2)**: Moderate parallelization
- T033, T034, T035 can be done together if US1 models are available
- T036-T041 can be done as frontend-only work

**Phase 5 (User Story 3)**: High parallelization potential
- T046, T047, T048 (backend) can run in parallel with T049-T052 (frontend context)
- T053-T056 (Navbar) depend on context completion

**Phase 6 (Polish)**: Most tasks can run in parallel (documentation, verification)

---

## Parallel Execution Examples

### Executing User Story 1 Tasks in Parallel

If you have 3 developers available:

**Developer 1 (Backend Models & Security)**:
```bash
# Run in parallel
T009: Implement password hashing in security.py
T010: Implement JWT functions in security.py
T011: Create UserCreate model in user.py
T012: Create UserResponse model in user.py
```

**Developer 2 (Backend Services & API)**:
```bash
# After Dev 1 completes T009-T012
T013: Implement create_user in user_service.py
T014: Implement get_user_by_email in user_service.py
T015: Create POST /signup endpoint in auth.py
T016: Add duplicate email validation to /signup
T017: Update config.py with jwt_secret_key
T018: Update main.py to include auth router
```

**Developer 3 (Frontend Signup Page)**:
```bash
# Can start after T015 (API endpoint exists)
T019: Create signup.js component
T020: Add form inputs (name, email, password)
T021: Add Software Experience dropdown (PDF compliance)
T022: Add Hardware Experience dropdown (PDF compliance)
T023: Add form submission to /api/auth/signup
T024: Add success redirect handler
T025: Add error display handler
T026: Add client-side validation
```

**Integration (All developers)**:
```bash
# After all above complete
T027-T032: Integration testing (can be done collaboratively)
```

### Executing User Story 2 Tasks in Parallel

**Developer 1 (Backend)**:
```bash
T033: Create UserLogin model
T034: Create POST /signin endpoint
T035: Add authentication failure handling
```

**Developer 2 (Frontend)**:
```bash
# Can start after T034 complete
T036: Create signin.js component
T037: Add signin form
T038: Add form submission handler
T039: Add success redirect
T040: Add error display
T041: Add already-authenticated redirect
```

**Both developers**:
```bash
T042-T045: Integration testing
```

### Executing User Story 3 Tasks in Parallel

**Developer 1 (Backend Session Endpoints)**:
```bash
T046: Create GET /me endpoint
T047: Create POST /logout endpoint
T048: Configure JWT expiration
```

**Developer 2 (Frontend Auth Context)**:
```bash
# In parallel with Dev 1
T049: Create AuthProvider component
T050: Add session check on mount
T051: Add login method
T052: Add logout method
```

**Developer 2 (continues)**:
```bash
# After AuthProvider complete
T053: Swizzle Navbar component
T054: Add conditional rendering
T055: Connect logout button
T056: Wrap app with AuthProvider
```

**Both developers**:
```bash
T057-T061: Integration testing
```

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**MVP = User Story 1 ONLY** (Priority P1)

Delivering just User Story 1 provides:
- Users can create accounts with experience data (PDF compliance met)
- Database stores all required fields (software/hardware experience)
- JWT authentication foundation is established
- Bonus points requirement is satisfied

After MVP validation, incrementally add:
1. User Story 2 (Sign In) - enables returning users
2. User Story 3 (Session Persistence & Logout) - improves UX

### Incremental Delivery Milestones

**Milestone 1: Database Ready** (Phase 2 complete)
- Users table exists in Postgres with all constraints
- Database foundation enables all user stories
- Estimated: 30 minutes

**Milestone 2: MVP - Signup Working** (Phase 3 complete)
- Users can register with email/password and experience levels
- Database stores user data correctly
- JWT cookies are set (even if not yet used for persistence)
- PDF compliance achieved (both experience dropdowns functional)
- Estimated: 3-4 hours

**Milestone 3: Signin Working** (Phase 4 complete)
- Returning users can authenticate with credentials
- JWT cookies are validated on signin
- Estimated: 1-2 hours

**Milestone 4: Full Auth System** (Phase 5 complete)
- Sessions persist across page loads
- Users can log out securely
- Global auth state via React Context
- Navbar shows auth-aware UI
- Estimated: 2-3 hours

**Milestone 5: Production Ready** (Phase 6 complete)
- All edge cases tested
- Documentation complete
- Environment variables properly configured
- Estimated: 1 hour

**Total Estimated Implementation Time**: 4-6 hours (as per plan.md)

---

## Success Criteria Verification (from spec.md)

- **SC-001**: Users can complete signup (including both experience dropdowns) in <2 minutes → Verify with T029
- **SC-002**: Users can sign in and access homepage in <10 seconds → Verify with T042
- **SC-003**: Sessions persist across 5+ page refreshes → Verify with T057
- **SC-004**: 100% of registrations store experience data correctly → Verify with T029, check database
- **SC-005**: System rejects duplicate email registrations → Verify with T030
- **SC-006**: System rejects invalid credentials → Verify with T043, T044
- **SC-007**: Logout clears session in 100% of attempts → Verify with T059
- **SC-008**: Zero passwords stored in plain text (all bcrypt hashed) → Verify with T009, check database
- **SC-009**: Signup form displays and enforces both experience dropdowns → Verify with T021, T022 (PDF COMPLIANCE)

---

## File Summary

**Total Files to Create**: 12 new files
**Total Files to Update**: 4 existing files

### Backend Files (11 total)

**New Files (7)**:
1. `backend/app/core/security.py` - Password hashing and JWT token functions
2. `backend/app/models/user.py` - Pydantic models (UserCreate, UserLogin, UserResponse)
3. `backend/app/services/user_service.py` - User database operations (create, get by email, get by ID)
4. `backend/app/api/auth.py` - Auth API endpoints (signup, signin, logout, me)
5. `backend/migrations/001_create_users_table.sql` - Database schema migration
6. `backend/.env` - Environment configuration (already exists, UPDATE with JWT_SECRET_KEY)
7. `.env.example` - Environment template (already exists, UPDATE with JWT_SECRET_KEY)

**Updated Files (4)**:
1. `backend/app/core/config.py` - Add jwt_secret_key field to Settings class
2. `backend/main.py` - Include auth router
3. `backend/requirements.txt` - Add passlib[bcrypt] and python-jose[cryptography]
4. `.env.example` - Add JWT_SECRET_KEY template

### Frontend Files (5 new)

**New Files (5)**:
1. `src/pages/signup.js` - Signup page with form (name, email, password, software exp dropdown, hardware exp dropdown)
2. `src/pages/signin.js` - Signin page with email/password form
3. `src/components/AuthProvider.js` - React Context for global auth state
4. `src/theme/Navbar/index.js` - Swizzled Navbar with Login/Logout buttons (may need creation via swizzle)
5. `src/theme/Root.js` - Root wrapper to provide AuthProvider context (create if doesn't exist)

---

## Total Tasks: 70

**By Phase**:
- Phase 1 (Setup): 4 tasks
- Phase 2 (Foundational): 4 tasks
- Phase 3 (User Story 1): 24 tasks (8 backend, 8 frontend, 6 integration, 2 config)
- Phase 4 (User Story 2): 13 tasks (3 backend, 6 frontend, 4 integration)
- Phase 5 (User Story 3): 16 tasks (3 backend, 8 frontend, 5 integration)
- Phase 6 (Polish): 9 tasks

**By User Story**:
- User Story 1 (P1 - MVP): 24 tasks
- User Story 2 (P2): 13 tasks
- User Story 3 (P3): 16 tasks
- Setup & Foundational: 8 tasks
- Polish: 9 tasks

**Parallel Opportunities**: 25+ tasks marked [P] can run in parallel within their phase/story

**Format Validation**: ✅ All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description`
