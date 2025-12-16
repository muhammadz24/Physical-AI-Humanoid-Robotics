# Tasks: Content Personalization

**Input**: Design documents from `/specs/007-content-personalization/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/personalize-api.yaml, quickstart.md

**Tests**: Not explicitly requested in specification - manual testing approach used

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/` (api/, models/, services/, core/)
- **Frontend**: `src/` (components/, utils/, pages/)
- **Tests**: `backend/tests/` (backend), `src/__tests__/` (frontend)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify prerequisites and environment configuration

- [ ] T001 Verify Feature 006 (Authentication) is complete and functional
- [ ] T002 Verify GEMINI_API_KEY exists in backend/.env and is valid
- [ ] T003 Verify users table has software_experience and hardware_experience columns
- [ ] T004 Verify existing Gemini LLM integration in backend/app/services/llm.py is functional
- [ ] T005 [P] Create feature branch 007-content-personalization from main

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T006 [P] Create PersonalizationRequest model in backend/app/models/personalize.py (chapter_text, chapter_id fields with validation)
- [ ] T007 [P] Create PersonalizationResponse model in backend/app/models/personalize.py (personalized_text, metadata, timestamps)
- [ ] T008 [P] Create UserExperienceContext model in backend/app/models/personalize.py (user_id, software/hardware experience, prompt helpers)
- [ ] T009 Implement user experience retrieval function in backend/app/services/user_service.py (query software_experience, hardware_experience by user_id)
- [ ] T010 Create LLM prompt template builder in backend/app/services/llm.py (construct_personalization_prompt function with experience context)
- [ ] T011 Create POST /api/personalize endpoint skeleton in backend/app/api/personalize.py (route, request/response models, authentication dependency)
- [ ] T012 Implement JWT authentication extraction in backend/app/api/personalize.py (get user_id from access_token cookie)
- [ ] T013 Integrate endpoint with main router in backend/main.py (add personalize router)
- [ ] T014 Add input validation in backend/app/api/personalize.py (chapter_text length 100-50000 chars, chapter_id pattern)

### Frontend Foundation

- [ ] T015 [P] Add PERSONALIZE endpoint to API_ENDPOINTS in src/utils/api.js (POST /api/personalize)
- [ ] T016 [P] Create PersonalizeButton component directory structure src/components/PersonalizeButton/
- [ ] T017 [P] Create PersonalizeButton/index.js component skeleton (button render, authentication check)
- [ ] T018 [P] Create usePersonalization.js custom hook in src/components/PersonalizeButton/ (state management: idle, loading, personalized, error)
- [ ] T019 Create PersonalizeButton.module.css in src/components/PersonalizeButton/ (basic button styling)
- [ ] T020 Implement chapter content extraction function in usePersonalization.js (DOM query for article element, convert to text)
- [ ] T021 Implement API call function in usePersonalization.js (fetch POST /api/personalize with credentials, handle response)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Beginner Personalizes Technical Chapter (Priority: P1) 🎯 MVP

**Goal**: Beginner users can click "Personalize This Chapter" button to receive content rewritten in simpler language appropriate for their experience level

**Independent Test**: Authenticate as beginner user (software: beginner, hardware: none), navigate to Chapter 1, click "Personalize This Chapter" button, wait for loading state to complete, verify chapter content is rewritten with simpler language, basic analogies, and foundational explanations

### Implementation for User Story 1

#### Backend - Beginner Personalization Logic

- [ ] T022 [US1] Implement beginner-specific prompt guidelines in backend/app/models/personalize.py UserExperienceContext.get_adaptation_guidelines() (simple language, explain terms, real-world analogies)
- [ ] T023 [US1] Implement LLM personalization call in backend/app/api/personalize.py (call Gemini API with constructed prompt, 30s timeout)
- [ ] T024 [US1] Implement response construction in backend/app/api/personalize.py (build PersonalizationResponse with metadata: original_length, personalized_length, processing_time_ms)
- [ ] T025 [US1] Add error handling for LLM failures in backend/app/api/personalize.py (timeout 503, quota exceeded 503, invalid response 500)

#### Frontend - Beginner Personalization UI

- [ ] T026 [US1] Implement loading state UI in src/components/PersonalizeButton/index.js (spinner, "Personalizing content..." message, disable button)
- [ ] T027 [US1] Implement content replacement logic in usePersonalization.js (replace article innerHTML with personalized_text on success)
- [ ] T028 [US1] Add error state handling in src/components/PersonalizeButton/index.js (display error message, show retry button)
- [ ] T029 [US1] Add loading and error styling to PersonalizeButton.module.css (spinner animation, error message style)
- [ ] T030 [US1] Integrate PersonalizeButton into Chapter 1 MDX file in docs/chapters/chapter-1.mdx (import component, add at top of page)

#### Testing

- [ ] T031 [US1] Manual test: Create beginner user via /signup (software: beginner, hardware: none)
- [ ] T032 [US1] Manual test: Navigate to Chapter 1, click personalize button, verify loading state appears
- [ ] T033 [US1] Manual test: Verify personalized content uses simpler language, includes analogies
- [ ] T034 [US1] Manual test: Verify technical terms are explained in beginner-friendly way
- [ ] T035 [US1] Manual test: Verify markdown structure is preserved (headings, code blocks, lists)

**Checkpoint**: At this point, User Story 1 should be fully functional - beginner users can personalize Chapter 1

---

## Phase 4: User Story 2 - Expert Skips Basic Explanations (Priority: P2)

**Goal**: Expert users receive more concise, technical content that skips basic explanations and focuses on implementation details

**Independent Test**: Authenticate as expert user (software: pro, hardware: professional), navigate to Chapter 1, click personalize button, verify content becomes more technical, concise, and advanced with basic explanations removed

### Implementation for User Story 2

#### Backend - Expert Personalization Logic

- [ ] T036 [US2] Implement expert-specific prompt guidelines in backend/app/models/personalize.py UserExperienceContext.get_adaptation_guidelines() (concise, technical, skip basics, implementation focus)
- [ ] T037 [US2] Verify intermediate experience level handling in UserExperienceContext.get_adaptation_guidelines() (balanced technical depth)

#### Testing

- [ ] T038 [US2] Manual test: Create expert user via /signup (software: pro, hardware: professional)
- [ ] T039 [US2] Manual test: Navigate to Chapter 1, click personalize button
- [ ] T040 [US2] Manual test: Verify personalized content is concise and technical
- [ ] T041 [US2] Manual test: Verify basic explanations are removed, advanced concepts emphasized
- [ ] T042 [US2] Manual test: Compare beginner vs expert personalized content for same chapter

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - different experience levels get appropriate content

---

## Phase 5: User Story 3 - User Toggles Between Original and Personalized (Priority: P3)

**Goal**: Users can toggle between original and personalized content without re-requesting from API, enabling comparison and choice

**Independent Test**: Personalize Chapter 1, click "Show Original" to see static content, click "Show Personalized" to return to customized version - all without additional API calls, transitions should be instant (<200ms)

### Implementation for User Story 3

#### Frontend - Toggle Functionality

- [ ] T043 [US3] Add original content caching in usePersonalization.js (save original article HTML before replacement)
- [ ] T044 [US3] Add personalized content caching in usePersonalization.js (save API response in component state)
- [ ] T045 [US3] Add isShowingPersonalized state flag in usePersonalization.js
- [ ] T046 [US3] Create toggle UI in src/components/PersonalizeButton/index.js (Show Original / Show Personalized buttons after personalization)
- [ ] T047 [US3] Implement instant toggle function in usePersonalization.js (swap article innerHTML between original and personalized, no API call)
- [ ] T048 [US3] Add toggle button styling to PersonalizeButton.module.css (side-by-side buttons, active state indicator)
- [ ] T049 [US3] Add smooth transition effect to PersonalizeButton.module.css (fade in/out for content swap)

#### Testing

- [ ] T050 [US3] Manual test: Personalize Chapter 1, verify "Show Original" button appears
- [ ] T051 [US3] Manual test: Click "Show Original", verify content switches to static version instantly
- [ ] T052 [US3] Manual test: Click "Show Personalized", verify content switches back to personalized version instantly
- [ ] T053 [US3] Manual test: Verify no API calls are made during toggle (check browser DevTools Network tab)
- [ ] T054 [US3] Manual test: Toggle multiple times, verify performance is under 200ms

**Checkpoint**: All user stories should now be independently functional - users can personalize and toggle

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, production readiness

### Backend Enhancements

- [ ] T055 [P] Implement rate limiting in backend/app/api/personalize.py (10 requests/hour per user_id using SlowAPI or in-memory counter)
- [ ] T056 [P] Add comprehensive logging in backend/app/api/personalize.py (log user_id, chapter_id, success/failure, processing_time_ms)
- [ ] T057 [P] Add response validation in backend/app/api/personalize.py (verify personalized_text not empty, basic markdown structure check)
- [ ] T058 Add graceful degradation for missing user experience data in backend/app/api/personalize.py (return 400 with message to update profile)

### Frontend Enhancements

- [ ] T059 [P] Add responsive design to PersonalizeButton.module.css (mobile-friendly button, toggle layout)
- [ ] T060 [P] Add accessibility attributes to src/components/PersonalizeButton/index.js (ARIA labels, keyboard navigation, focus states)
- [ ] T061 Implement request cancellation in usePersonalization.js (abort in-flight API call if user navigates away)
- [ ] T062 Add loading state persistence in usePersonalization.js (prevent accidental duplicate clicks during processing)

### Documentation & Testing

- [ ] T063 [P] Update quickstart.md with actual test results and screenshots
- [ ] T064 [P] Create usage documentation in specs/007-content-personalization/usage.md (how to add PersonalizeButton to new chapters)
- [ ] T065 Validate quickstart.md test scenarios (follow all steps, verify accuracy)
- [ ] T066 [P] Add inline code comments in backend/app/api/personalize.py (explain LLM prompt construction, error handling)
- [ ] T067 [P] Add inline code comments in src/components/PersonalizeButton/index.js (explain state transitions, toggle logic)

### Production Readiness

- [ ] T068 Verify REACT_APP_API_URL environment variable works in production build
- [ ] T069 Test personalization with real Chapter 1 content (full length ~3000-5000 words)
- [ ] T070 Verify Gemini API quota monitoring and error messages
- [ ] T071 Test error scenarios: LLM timeout, quota exceeded, authentication failure, invalid input
- [ ] T072 Performance test: Verify personalization completes in under 10 seconds for typical chapter
- [ ] T073 Performance test: Verify toggle completes in under 200ms

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Builds on US1 prompt infrastructure but independently testable
- **User Story 3 (P3)**: Depends on US1 completion (needs personalization to work before toggle) - Can start after US1

### Within Each User Story

- Backend models before services
- Services before endpoints
- Endpoint before frontend integration
- Component structure before UI implementation
- Core implementation before testing
- Manual tests after implementation complete

### Parallel Opportunities

**Within Phase 2 (Foundational)**:
- T006, T007, T008 (all Pydantic models) can run in parallel
- T015, T016, T017, T018, T019 (frontend structure) can run in parallel after T006-T008

**Within Phase 3 (User Story 1)**:
- T029 (CSS) can run in parallel with T026-T028 (component logic)

**Within Phase 6 (Polish)**:
- T055, T056, T057 (backend enhancements) can run in parallel
- T059, T060 (frontend enhancements) can run in parallel
- T063, T064, T066, T067 (documentation) can run in parallel

**Across User Stories** (if multiple developers):
- US2 can start immediately after US1 implementation completes (T022-T030)
- US3 can start after US1 testing completes (T031-T035)

---

## Parallel Example: Foundational Phase

```bash
# Launch all Pydantic models together:
Task T006: "Create PersonalizationRequest model in backend/app/models/personalize.py"
Task T007: "Create PersonalizationResponse model in backend/app/models/personalize.py"
Task T008: "Create UserExperienceContext model in backend/app/models/personalize.py"

# Launch all frontend structure together:
Task T015: "Add PERSONALIZE endpoint to API_ENDPOINTS in src/utils/api.js"
Task T016: "Create PersonalizeButton component directory"
Task T017: "Create PersonalizeButton/index.js component skeleton"
Task T018: "Create usePersonalization.js custom hook"
Task T019: "Create PersonalizeButton.module.css"
```

---

## Parallel Example: User Story 1

```bash
# Launch CSS styling in parallel with component logic:
Task T026: "Implement loading state UI in PersonalizeButton/index.js"
Task T027: "Implement content replacement logic in usePersonalization.js"
Task T028: "Add error state handling in PersonalizeButton/index.js"
Task T029 [P]: "Add loading and error styling to PersonalizeButton.module.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005)
2. Complete Phase 2: Foundational (T006-T021) - CRITICAL: blocks all stories
3. Complete Phase 3: User Story 1 (T022-T035)
4. **STOP and VALIDATE**: Test User Story 1 independently as beginner user
5. Deploy/demo if ready - **MVP complete!**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP: Beginner personalization working!)
3. Add User Story 2 → Test independently → Deploy/Demo (Expert personalization added!)
4. Add User Story 3 → Test independently → Deploy/Demo (Toggle functionality added!)
5. Complete Polish phase → Production ready
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup (Phase 1) + Foundational (Phase 2) together
2. Once Foundational is done:
   - Developer A: User Story 1 (T022-T035) - **MVP priority**
   - Developer B: User Story 2 (T036-T042) - Can start after US1 backend foundation
   - Developer C: Polish tasks (T055-T067) - Can prep documentation in parallel
3. After US1 complete:
   - Developer A: User Story 3 (T043-T054)
4. Final: All developers on production readiness (T068-T073)

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story (US1, US2, US3) for traceability
- Each user story should be independently completable and testable
- No automated tests requested in spec - using manual testing approach per quickstart.md
- Commit after each logical task group (e.g., after completing all models, after completing endpoint)
- Stop at any checkpoint to validate story independently before proceeding
- Chapter 1 is POC - extend to other chapters after validation
- LLM prompt iteration expected based on quality feedback
- Monitor Gemini API quota usage in production
- Rate limiting protects quota (10 req/hr/user)
- Session-only caching means personalization regenerates on page reload (acceptable per research.md)
- Toggle requires no API call - instant client-side switch
- All file paths use absolute paths from repository root
- Backend follows existing FastAPI structure from Feature 006
- Frontend follows existing Docusaurus + React patterns from Feature 006

---

## Task Count Summary

- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundational)**: 16 tasks
- **Phase 3 (User Story 1 - P1)**: 14 tasks
- **Phase 4 (User Story 2 - P2)**: 7 tasks
- **Phase 5 (User Story 3 - P3)**: 12 tasks
- **Phase 6 (Polish)**: 19 tasks

**Total**: 73 tasks

**Parallel Opportunities**: 16 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (35 tasks total) delivers working beginner personalization on Chapter 1

**Independent Test Criteria**:
- US1: Beginner user personalizes Chapter 1, receives simpler content
- US2: Expert user personalizes Chapter 1, receives technical content
- US3: User toggles between original and personalized without API calls
