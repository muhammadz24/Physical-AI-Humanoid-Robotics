# Tasks: Advanced Chatbot with History Management

**Input**: Design documents from `/specs/008-advanced-chatbot/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md

**Tests**: Not explicitly requested - manual testing approach used

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (api/, models/, services/, migrations/, jobs/)
- **Frontend**: `src/` (components/ChatWidget/, utils/)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify prerequisites and create database schema

- [ ] T001 Verify Feature 006 (Authentication) is complete and functional
- [ ] T002 Verify existing chat backend exists (POST /api/chat endpoint)
- [ ] T003 Verify Neon Postgres database is accessible
- [ ] T004 [P] Create feature branch 008-advanced-chatbot from main
- [ ] T005 Create database migration 002_create_chat_messages.sql in backend/migrations/
- [ ] T006 Create database migration 003_add_auto_delete_to_users.sql in backend/migrations/
- [ ] T007 Execute database migrations (create chat_messages table, add auto_delete columns to users)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core models and services that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T008 [P] Create ChatMessage Pydantic model in backend/app/models/chat.py (id, user_id, role, content, timestamp fields)
- [ ] T009 [P] Create ChatRequest model in backend/app/models/chat.py (message validation)
- [ ] T010 [P] Create ChatResponse model in backend/app/models/chat.py (answer, confidence, saved fields)
- [ ] T011 [P] Create ChatHistoryResponse model in backend/app/models/chat.py (messages list, pagination)
- [ ] T012 [P] Create SyncGuestHistoryRequest model in backend/app/models/chat.py (guest messages array)
- [ ] T013 Create chat_service.py in backend/app/services/ with save_message function
- [ ] T014 Add get_user_history function to backend/app/services/chat_service.py (query with pagination)
- [ ] T015 Add delete_user_history function to backend/app/services/chat_service.py (bulk delete)
- [ ] T016 Add sync_guest_messages function to backend/app/services/chat_service.py (atomic transaction)

### Frontend Foundation

- [ ] T017 [P] Create useSessionStorage hook in src/components/ChatWidget/useSessionStorage.js (save/load/clear guest messages)
- [ ] T018 [P] Create useChatHistory hook in src/components/ChatWidget/useChatHistory.js (unified interface for guest/user storage)
- [ ] T019 [P] Create syncGuestHistory utility in src/utils/syncGuestHistory.js (POST /api/chat/sync wrapper)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Guest Session Persistence (Priority: P1) 🎯 MVP

**Goal**: Guest users can send chat messages stored in sessionStorage, persisting across page refreshes but clearing on tab close

**Independent Test**: Use chatbot without logging in, send 3-5 messages, refresh page to verify history persists, close tab and reopen to verify history cleared

### Implementation for User Story 1

#### Frontend - Guest sessionStorage

- [ ] T020 [US1] Implement save logic in useSessionStorage.js (write to sessionStorage with key `chatbot_history_guest`)
- [ ] T021 [US1] Implement load logic in useSessionStorage.js (read from sessionStorage on component mount)
- [ ] T022 [US1] Implement clear logic in useSessionStorage.js (remove from sessionStorage)
- [ ] T023 [US1] Modify ChatWidget/index.js to use useSessionStorage for guest users (check isAuthenticated from AuthContext)
- [ ] T024 [US1] Add message limit enforcement in useSessionStorage.js (max 100 messages, drop oldest)

#### Testing

- [ ] T025 [US1] Manual test: Open chatbot as guest, send 5 messages
- [ ] T026 [US1] Manual test: Refresh page, verify messages restored
- [ ] T027 [US1] Manual test: Close tab, reopen site, verify messages cleared

**Checkpoint**: Guest users can chat with session persistence working

---

## Phase 4: User Story 2 - Authenticated User Persistent History (Priority: P1)

**Goal**: Authenticated users' chat messages are saved to Postgres database and persist across sessions

**Independent Test**: Log in, send messages, log out, log back in days later, verify all messages restored

### Implementation for User Story 2

#### Backend - Database Storage

- [ ] T028 [US2] Modify POST /api/chat endpoint in backend/app/api/chat.py to save authenticated user messages (call chat_service.save_message)
- [ ] T029 [US2] Create GET /api/chat/history endpoint in backend/app/api/chat.py (requires JWT auth, returns ChatHistoryResponse)
- [ ] T030 [US2] Add authentication check in POST /api/chat (if authenticated, save; if guest, skip save)

#### Frontend - Database Integration

- [ ] T031 [US2] Implement fetch_history function in useChatHistory.js (call GET /api/chat/history on mount for authenticated users)
- [ ] T032 [US2] Modify ChatWidget/index.js to load history on component mount for authenticated users
- [ ] T033 [US2] Add loading state for history fetch in ChatWidget/index.js

#### Testing

- [ ] T034 [US2] Manual test: Log in, send 5 messages
- [ ] T035 [US2] Manual test: Refresh page, verify messages loaded from database
- [ ] T036 [US2] Manual test: Log out, log back in, verify all history restored

**Checkpoint**: Authenticated users have permanent history in database

---

## Phase 5: User Story 3 - Guest→User Sync on Login (Priority: P2)

**Goal**: When guest logs in, their sessionStorage chat history automatically merges into their database history

**Independent Test**: Use chatbot as guest (5-10 messages), sign up/log in, verify all guest messages now in authenticated history

### Implementation for User Story 3

#### Backend - Sync Endpoint

- [ ] T037 [US3] Create POST /api/chat/sync endpoint in backend/app/api/chat.py (requires JWT auth, accepts SyncGuestHistoryRequest)
- [ ] T038 [US3] Implement transaction-safe sync in chat_service.sync_guest_messages (atomic INSERT all messages)
- [ ] T039 [US3] Add error handling and rollback in sync_guest_messages (keep sessionStorage on failure)

#### Frontend - Sync Trigger

- [ ] T040 [US3] Add sync listener to AuthProvider in src/components/AuthProvider.js (trigger on successful signin/signup)
- [ ] T041 [US3] Implement syncGuestHistory function in src/utils/syncGuestHistory.js (read sessionStorage, POST /api/chat/sync, clear on success)
- [ ] T042 [US3] Add error handling in syncGuestHistory (show retry option if sync fails, keep sessionStorage)

#### Testing

- [ ] T043 [US3] Manual test: Use chatbot as guest, accumulate 10 messages
- [ ] T044 [US3] Manual test: Sign up with new account
- [ ] T045 [US3] Manual test: Verify all guest messages now in authenticated history
- [ ] T046 [US3] Manual test: Verify sessionStorage cleared after successful sync

**Checkpoint**: Guest→User synchronization working seamlessly

---

## Phase 6: User Story 4 - Delete Chat History (Priority: P2)

**Goal**: Users can delete all their chat history with confirmation dialog

**Independent Test**: Accumulate chat history, click "Delete All History", confirm, verify database has no messages for that user

### Implementation for User Story 4

#### Backend - Delete Endpoint

- [ ] T047 [US4] Create DELETE /api/chat/history endpoint in backend/app/api/chat.py (requires JWT auth)
- [ ] T048 [US4] Implement delete_user_history in chat_service (DELETE FROM chat_messages WHERE user_id = X)

#### Frontend - Delete UI

- [ ] T049 [US4] Create ChatSettings component in src/components/ChatSettings/index.js (modal with delete button)
- [ ] T050 [US4] Add "Delete All History" button with confirmation dialog in ChatSettings
- [ ] T051 [US4] Implement delete handler (call DELETE /api/chat/history, clear local state on success)
- [ ] T052 [US4] Add settings button to ChatWidget header (opens ChatSettings modal)
- [ ] T053 [US4] For guest users: Add "Clear Chat" button that clears sessionStorage

#### Testing

- [ ] T054 [US4] Manual test: Accumulate 20 messages as authenticated user
- [ ] T055 [US4] Manual test: Click "Delete All History", confirm
- [ ] T056 [US4] Manual test: Verify chat interface empty, database has no messages
- [ ] T057 [US4] Manual test: As guest, click "Clear Chat", verify sessionStorage cleared

**Checkpoint**: Users can delete their history

---

## Phase 7: User Story 5 - Auto-Delete Old Conversations (Priority: P3)

**Goal**: Users can configure auto-delete (7, 30, 90 days) and background job deletes old messages

**Independent Test**: Create old test data, configure auto-delete to 7 days, run cleanup job manually, verify messages >7 days deleted

### Implementation for User Story 5

#### Backend - Auto-Delete Settings

- [ ] T058 [US5] Create UserSettingsUpdate model in backend/app/models/user.py (auto_delete_enabled, auto_delete_days)
- [ ] T059 [US5] Create PUT /api/user/settings endpoint in backend/app/api/auth.py (update auto_delete settings)
- [ ] T060 [US5] Create GET /api/user/settings endpoint in backend/app/api/auth.py (retrieve current settings)

#### Backend - Cleanup Job

- [ ] T061 [US5] Create cleanup_old_messages.py in backend/jobs/ (query users with auto_delete_enabled=true)
- [ ] T062 [US5] Implement deletion logic in cleanup_old_messages.py (DELETE messages older than auto_delete_days)
- [ ] T063 [US5] Add APScheduler integration to backend/main.py (schedule daily at 2 AM UTC)
- [ ] T064 [US5] Add logging and error handling to cleanup job (log deletions, handle failures gracefully)

#### Frontend - Settings UI

- [ ] T065 [US5] Add auto-delete toggle to ChatSettings component (enabled/disabled)
- [ ] T066 [US5] Add auto-delete days selector to ChatSettings (7, 30, 90 days dropdown)
- [ ] T067 [US5] Implement save settings handler (PUT /api/user/settings)
- [ ] T068 [US5] Load current settings on mount (GET /api/user/settings)

#### Testing

- [ ] T069 [US5] Manual test: Configure auto-delete to 7 days
- [ ] T070 [US5] Manual test: Create old test messages (mock timestamps)
- [ ] T071 [US5] Manual test: Run cleanup job manually (python backend/jobs/cleanup_old_messages.py)
- [ ] T072 [US5] Manual test: Verify messages >7 days deleted, recent messages preserved

**Checkpoint**: Auto-delete configured and working

---

## Phase 8: User Story 6 - Modern UI Overhaul (Priority: P2)

**Goal**: Redesigned chat interface with message bubbles, typing indicator, auto-scroll, timestamps

**Independent Test**: Visual inspection and interaction testing with new UI components

### Implementation for User Story 6

#### UI Components

- [ ] T073 [P] [US6] Create ChatMessage.js component in src/components/ChatWidget/ (message bubble with role-based styling)
- [ ] T074 [P] [US6] Create ChatHistory.js component in src/components/ChatWidget/ (scrollable message list)
- [ ] T075 [P] [US6] Create TypingIndicator.js component in src/components/ChatWidget/ (animated dots)
- [ ] T076 [P] [US6] Create ChatInput.js component in src/components/ChatWidget/ (textarea + send button)
- [ ] T077 [US6] Create ChatWidget.module.css with modern styling (message bubbles, spacing, colors)

#### UI Features

- [ ] T078 [US6] Implement auto-scroll to bottom in ChatHistory.js (scroll on new message)
- [ ] T079 [US6] Add scroll-to-bottom button in ChatHistory.js (appears when scrolled up)
- [ ] T080 [US6] Add timestamp display to ChatMessage.js (relative time: "2 minutes ago")
- [ ] T081 [US6] Integrate TypingIndicator in ChatWidget during LLM response
- [ ] T082 [US6] Refactor ChatWidget/index.js to use new components (ChatHistory, ChatMessage, ChatInput)

#### Testing

- [ ] T083 [US6] Manual test: Send messages, verify bubbles styled correctly (user right, assistant left)
- [ ] T084 [US6] Manual test: Verify auto-scroll to bottom on new messages
- [ ] T085 [US6] Manual test: Scroll up, verify scroll-to-bottom button appears
- [ ] T086 [US6] Manual test: Verify typing indicator shows during LLM response
- [ ] T087 [US6] Manual test: Verify timestamps display correctly

**Checkpoint**: All user stories complete, UI overhauled

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, production readiness

### Backend Enhancements

- [ ] T088 [P] Add rate limiting to POST /api/chat in backend/app/api/chat.py (10 messages/minute using SlowAPI)
- [ ] T089 [P] Add comprehensive error logging across all chat endpoints
- [ ] T090 [P] Add input sanitization for message content (prevent XSS)
- [ ] T091 Add response size limits (max 10KB per message)

### Frontend Enhancements

- [ ] T092 [P] Add error boundary to ChatWidget (catch and display React errors)
- [ ] T093 [P] Add accessibility attributes (ARIA labels, keyboard navigation)
- [ ] T094 Add loading skeletons for history fetch
- [ ] T095 Add empty state UI when no messages ("Start a conversation...")

### Documentation

- [ ] T096 [P] Create usage documentation in specs/008-advanced-chatbot/usage.md
- [ ] T097 [P] Add inline code comments for complex logic (sync, auto-delete)
- [ ] T098 Update DEPLOY_GUIDE.md with new backend dependencies (APScheduler, SlowAPI)

### Production Readiness

- [ ] T099 Test full flow end-to-end (guest → messages → login → sync → delete → auto-delete)
- [ ] T100 Verify all environment variables work in production build
- [ ] T101 Test database performance with 1000+ messages per user
- [ ] T102 Verify sessionStorage limits (show warning at 4MB)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel (different storage backends)
  - US3 depends on US1 and US2 completion (requires both guest and user storage)
  - US4, US5, US6 can proceed after US2 completion
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **US1 (P1)**: Can start after Foundational - No dependencies on other stories
- **US2 (P1)**: Can start after Foundational - Independent of US1
- **US3 (P2)**: Depends on US1 AND US2 completion (requires both storages working)
- **US4 (P2)**: Depends on US2 completion (requires database storage)
- **US5 (P3)**: Depends on US2 completion (requires database storage)
- **US6 (P2)**: Can start after Foundational - Independent (UI-only)

### Within Each User Story

- Backend models before services
- Services before endpoints
- Endpoints before frontend integration
- Core implementation before testing
- Manual tests after implementation complete

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
- T008-T012 (all Pydantic models) can run in parallel
- T017-T019 (frontend hooks) can run in parallel after models complete

**Within Phase 8 (US6 - UI Overhaul)**:
- T073-T076 (all UI components) can run in parallel

**Across User Stories** (if multiple developers):
- US1 and US2 can start simultaneously after Foundational
- US4, US5, US6 can start simultaneously after US2

---

## Parallel Example: User Story 1

```bash
# Launch all US1 frontend tasks together:
Task T020: "Implement save logic in useSessionStorage.js"
Task T021: "Implement load logic in useSessionStorage.js"
Task T022: "Implement clear logic in useSessionStorage.js"
```

---

## Parallel Example: User Story 6

```bash
# Launch all UI components together:
Task T073: "Create ChatMessage.js component"
Task T074: "Create ChatHistory.js component"
Task T075: "Create TypingIndicator.js component"
Task T076: "Create ChatInput.js component"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup (7 tasks)
2. Complete Phase 2: Foundational (12 tasks)
3. Complete Phase 3: User Story 1 (8 tasks) - **Guest persistence**
4. Complete Phase 4: User Story 2 (9 tasks) - **User database history**
5. **STOP and VALIDATE**: Test both guest and authenticated flows independently
6. Deploy/demo if ready - **MVP complete!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add US1 + US2 → Test independently → Deploy/Demo (MVP!)
3. Add US3 → Test sync → Deploy/Demo (Sync added!)
4. Add US4 → Test delete → Deploy/Demo (Delete added!)
5. Add US6 → Test UI → Deploy/Demo (Modern UI!)
6. Add US5 → Test auto-delete → Deploy/Demo (Complete!)
7. Complete Polish phase → Production ready

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup (Phase 1) + Foundational (Phase 2) together
2. Once Foundational is done:
   - Developer A: User Story 1 (T020-T027)
   - Developer B: User Story 2 (T028-T036)
   - Developer C: User Story 6 UI components (T073-T076)
3. After US1 & US2 complete:
   - Developer A: User Story 3 (T037-T046)
   - Developer B: User Story 4 (T047-T057)
4. After US2 complete:
   - Developer C: User Story 5 (T058-T072)
5. Final: All developers on Polish (T088-T102)

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story (US1-US6) for traceability
- Each user story should be independently completable and testable
- No automated tests requested in spec - using manual testing approach
- Commit after each logical task group (e.g., after completing all models, after completing endpoint)
- Stop at any checkpoint to validate story independently before proceeding
- Database migration required before coding (Phase 1)
- sessionStorage key: `chatbot_history_guest`
- Database table: `chat_messages`
- Background job: APScheduler (daily at 2 AM UTC)
- Rate limit: 10 messages/minute per user
- All file paths use absolute paths from repository root
- Backend follows existing FastAPI structure from Feature 006
- Frontend follows existing Docusaurus + React patterns

---

## Task Count Summary

- **Phase 1 (Setup)**: 7 tasks
- **Phase 2 (Foundational)**: 12 tasks
- **Phase 3 (User Story 1 - P1)**: 8 tasks
- **Phase 4 (User Story 2 - P1)**: 9 tasks
- **Phase 5 (User Story 3 - P2)**: 10 tasks
- **Phase 6 (User Story 4 - P2)**: 11 tasks
- **Phase 7 (User Story 5 - P3)**: 15 tasks
- **Phase 8 (User Story 6 - P2)**: 15 tasks
- **Phase 9 (Polish)**: 15 tasks

**Total**: 102 tasks

**Parallel Opportunities**: 14 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 + Phase 4 (36 tasks total) delivers working guest and authenticated user chat history

**Independent Test Criteria**:
- US1: Guest chats persist on refresh, clear on tab close
- US2: Authenticated user history persists across sessions
- US3: Guest history syncs to user database on login (no data loss)
- US4: User can delete all history with confirmation
- US5: Auto-delete removes old messages based on user settings
- US6: Modern UI with bubbles, typing indicator, auto-scroll
