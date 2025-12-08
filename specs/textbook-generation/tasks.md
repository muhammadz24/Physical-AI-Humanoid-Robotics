# Tasks: Textbook Generation

**Input**: Design documents from `/specs/textbook-generation/`
**Prerequisites**: plan.md (complete), spec.md (complete), research.md (Phase 0), data-model.md (Phase 1), contracts/ (Phase 1)

**Tests**: Not explicitly requested in spec, so test tasks are EXCLUDED unless user requests TDD approach.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app structure**: `backend/src/`, `frontend/` (Docusaurus root), `docs/` (content)
- Frontend uses Docusaurus at repository root with `docs/` directory
- Backend uses FastAPI in `backend/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project directory structure per plan.md (docs/, backend/, src/, static/)
- [ ] T002 Initialize Node.js project with Docusaurus v3.x dependencies in package.json
- [ ] T003 [P] Initialize Python project with FastAPI dependencies in backend/requirements.txt
- [ ] T004 [P] Create Docusaurus configuration in docusaurus.config.js
- [x] T005 [P] Create sidebar configuration in sidebars.js (manual categories for chapters, Welcome first, no duplicates)
- [x] T006 [P] Setup .gitignore for Node and Python projects
- [ ] T007 [P] Create backend/.env.example with Qdrant, Neon, and API configuration templates
- [x] T008 [P] Setup custom CSS in src/css/custom.css (Cyber-Professional Dark Mode theme with glassmorphism and animations)
- [x] T009 [P] Add logo and favicon to static/img/
- [x] T009a [BONUS] Create GitHub Author component in src/components/GitHubAuthor.js with useEffect API integration
- [x] T009b [BONUS] Swizzle Navbar component in src/theme/Navbar/index.js for dynamic author display

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Backend Foundation

- [ ] T010 Create FastAPI app entry point in backend/src/main.py with CORS configuration
- [ ] T011 [P] Create config module in backend/src/config.py for environment variables
- [ ] T012 [P] Create Neon Postgres database schema in backend/scripts/migrate_db.py (chapters, sections, content_chunks tables)
- [ ] T013 [P] Create Qdrant collection setup script in backend/src/services/vector_db.py (textbook_chunks collection, 384 dimensions)
- [ ] T014 [P] Implement embeddings service in backend/src/services/embeddings.py using sentence-transformers all-MiniLM-L6-v2
- [ ] T015 Implement Postgres client in backend/src/services/postgres.py with connection pooling
- [ ] T016 [P] Implement rate limiting middleware in backend/src/utils/rate_limit.py (20 requests/minute)
- [ ] T017 [P] Create health check endpoint in backend/src/api/health.py (GET /health)
- [ ] T018 Create Pydantic models in backend/src/models/chat.py (ChatRequest, ChatResponse, Citation)
- [ ] T019 [P] Create Pydantic models in backend/src/models/metadata.py (ChapterMetadata, ContentChunk)

### Content Indexing Infrastructure

- [ ] T020 Implement text chunking utility in backend/src/utils/chunking.py (configurable 300-500 tokens per FR-018, default 400 tokens with 50 overlap)
- [ ] T021 Create content indexing script in backend/scripts/index_content.py (reads MDX, chunks, embeds, uploads to Qdrant/Neon)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Read Educational Content (Priority: P1) 🎯 MVP

**Goal**: Deliver 6 complete chapters with well-structured educational content, code examples, and self-assessment

**Independent Test**: Navigate to deployed site, open any chapter, verify content is readable, well-formatted, and complete

### Content Creation for User Story 1

- [ ] T022 [P] [US1] Create Chapter 1 structure in docs/chapter-01/index.md (Introduction to Physical AI)
- [ ] T023 [P] [US1] Write Chapter 1 sections in docs/chapter-01/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T024 [P] [US1] Create self-assessment questions in docs/chapter-01/self-assessment.md
- [ ] T025 [P] [US1] Create Chapter 2 structure in docs/chapter-02/index.md (Basics of Humanoid Robotics)
- [ ] T026 [P] [US1] Write Chapter 2 sections in docs/chapter-02/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T027 [P] [US1] Create self-assessment questions in docs/chapter-02/self-assessment.md
- [ ] T028 [P] [US1] Create Chapter 3 structure in docs/chapter-03/index.md (ROS 2 Fundamentals)
- [ ] T029 [P] [US1] Write Chapter 3 sections in docs/chapter-03/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T030 [P] [US1] Create self-assessment questions in docs/chapter-03/self-assessment.md
- [ ] T031 [P] [US1] Create Chapter 4 structure in docs/chapter-04/index.md (Digital Twin Simulation)
- [ ] T032 [P] [US1] Write Chapter 4 sections in docs/chapter-04/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T033 [P] [US1] Create self-assessment questions in docs/chapter-04/self-assessment.md
- [ ] T034 [P] [US1] Create Chapter 5 structure in docs/chapter-05/index.md (Vision-Language-Action Systems)
- [ ] T035 [P] [US1] Write Chapter 5 sections in docs/chapter-05/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T036 [P] [US1] Create self-assessment questions in docs/chapter-05/self-assessment.md
- [ ] T037 [P] [US1] Create Chapter 6 structure in docs/chapter-06/index.md (Capstone Project)
- [ ] T038 [P] [US1] Write Chapter 6 sections in docs/chapter-06/: introduction.md (200-300 words), conceptual-foundation.md (800-1200 words), technical-details.md (1000-1500 words), hands-on-examples.md (500-800 words), further-resources.md (per constitution)
- [ ] T039 [P] [US1] Create self-assessment questions in docs/chapter-06/self-assessment.md
- [ ] T040 [US1] Add chapter diagrams and images to static/img/chapter-01/ through chapter-06/
- [ ] T041 [US1] Verify all internal links and images render correctly in local dev server

**Checkpoint**: All 6 chapters created with content, ready for indexing and RAG system

---

## Phase 4: User Story 4 - Navigate with Auto-Generated Sidebar (Priority: P1) 🎯 MVP

**Goal**: Automatically generate sidebar showing all chapters and sections for easy navigation

**Independent Test**: Verify sidebar displays all 6 chapters with nested sections, clicking through verifies navigation

### Sidebar Implementation for User Story 4

- [ ] T042 [US4] Configure sidebar auto-generation in sidebars.js based on docs/ folder structure
- [ ] T043 [US4] Configure navbar in docusaurus.config.js with title and logo
- [ ] T044 [US4] Configure breadcrumb navigation in docusaurus.config.js (FR-010: show current location)
- [ ] T045 [US4] Test sidebar highlighting for current chapter/section in browser
- [ ] T046 [US4] Verify mobile sidebar drawer functionality (hamburger menu)

**Checkpoint**: Sidebar navigation fully functional on desktop and mobile

---

## Phase 5: User Story 7 - View Code Examples with Interactive Features (Priority: P1) 🎯 MVP

**Goal**: Code examples with syntax highlighting, line numbers, and copy buttons

**Independent Test**: View code blocks in any chapter, verify syntax highlighting, line numbers, and copy functionality work

### Code Examples Implementation for User Story 7

- [ ] T047 [P] [US7] Add Python code examples with syntax highlighting to Chapter 3 (ROS 2 nodes)
- [ ] T048 [P] [US7] Add YAML code examples to Chapter 3 (ROS 2 launch files)
- [ ] T049 [P] [US7] Add Python code examples to Chapter 6 (Capstone project implementation)
- [ ] T050 [US7] Configure Prism syntax highlighting in docusaurus.config.js for Python, YAML, Bash
- [ ] T051 [US7] Verify code copy button functionality and "Copied!" feedback in browser
- [ ] T052 [US7] Test line numbers display correctly for all code blocks

**Checkpoint**: Code examples functional with copy buttons and syntax highlighting across all chapters

---

## Phase 6: User Story 2 - Ask Questions via RAG Chatbot (Priority: P2)

**Goal**: AI chatbot that answers questions based exclusively on textbook content with citations

**Independent Test**: Open chatbot, ask sample questions from each chapter, verify answers are accurate and grounded in content

### Backend RAG Implementation for User Story 2

- [ ] T053 [P] [US2] Implement vector search service in backend/src/services/vector_db.py (top-k=3, threshold=0.7)
- [ ] T054 [P] [US2] Implement RAG orchestration service in backend/src/services/rag.py (query → embed → search → format)
- [ ] T055 [US2] Implement POST /v1/chat endpoint in backend/src/api/chat.py per contracts/chatbot-api.yaml
- [ ] T056 [US2] Add confidence threshold logic (>70%) and fallback messages in backend/src/services/rag.py
- [ ] T057 [US2] Add citation generation (chapter ID, title, URL) in backend/src/services/rag.py
- [ ] T058 [US2] Run content indexing script (backend/scripts/index_content.py) to populate Qdrant and Neon with all 6 chapters
- [ ] T059 [US2] Test /v1/chat endpoint with curl for sample queries and verify responses

### Frontend Chatbot Widget for User Story 2

- [ ] T060 [P] [US2] Create ChatbotWidget component in src/components/ChatbotWidget/index.tsx
- [ ] T061 [P] [US2] Create ChatMessage subcomponent in src/components/ChatbotWidget/ChatMessage.tsx
- [ ] T062 [P] [US2] Style chatbot modal and floating button in src/components/ChatbotWidget/ChatbotWidget.module.css
- [ ] T063 [US2] Integrate ChatbotWidget with FastAPI backend (POST /v1/chat) using fetch API
- [ ] T064 [US2] Implement session storage for chat history (sessionStorage) in ChatbotWidget
- [ ] T065 [US2] Add loading indicator for chatbot queries (3 second timeout warning)
- [ ] T066 [US2] Add citation links in chatbot responses that navigate to source chapters
- [ ] T067 [US2] Test chatbot end-to-end: ask questions, verify answers with citations, test fallback messages

**Checkpoint**: RAG chatbot fully functional with accurate answers and citations

---

## Phase 7: User Story 6 - Search Textbook Content (Priority: P2)

**Goal**: Search across all chapters for keywords with highlighted results

**Independent Test**: Type keywords in search box, verify relevant results appear

### Search Implementation for User Story 6

- [ ] T067 [US6] Configure Docusaurus local search plugin in docusaurus.config.js (@docusaurus/theme-search-algolia or local)
- [ ] T068 [US6] Test search functionality: type "ROS 2 nodes" and verify results from Chapter 3 appear
- [ ] T069 [US6] Verify search highlights matched text on result pages
- [ ] T070 [US6] Test "No results found" message for non-existent terms

**Checkpoint**: Search functional across all textbook content

---

## Phase 8: User Story 8 - Access Further Resources (Priority: P2)

**Goal**: Curated links to official docs, tutorials, and community resources at end of each chapter

**Independent Test**: Navigate to end of each chapter, verify links are present and functional

### Further Resources Implementation for User Story 8

- [ ] T071 [P] [US8] Add Further Resources section to docs/chapter-01/self-assessment.md (Physical AI resources)
- [ ] T072 [P] [US8] Add Further Resources section to docs/chapter-02/self-assessment.md (Humanoid Robotics resources)
- [ ] T073 [P] [US8] Add Further Resources section to docs/chapter-03/self-assessment.md (ROS 2 resources)
- [ ] T074 [P] [US8] Add Further Resources section to docs/chapter-04/self-assessment.md (Gazebo, Isaac Sim resources)
- [ ] T075 [P] [US8] Add Further Resources section to docs/chapter-05/self-assessment.md (VLA systems resources)
- [ ] T076 [P] [US8] Add Further Resources section to docs/chapter-06/self-assessment.md (Capstone resources)
- [ ] T077 [US8] Verify all external links open in new tabs and return 200 status (link checker)

**Checkpoint**: Further Resources available in all chapters with verified links

---

## Phase 9: User Story 5 - Toggle Dark Mode (Priority: P3)

**Goal**: Switch between light and dark themes with persistence

**Independent Test**: Click theme toggle, verify all pages switch themes

### Dark Mode Implementation for User Story 5

- [ ] T078 [US5] Configure dark mode in docusaurus.config.js (themeConfig.colorMode with defaultMode and respectPrefersColorScheme)
- [ ] T079 [US5] Test dark mode toggle in browser and verify persistence across page refreshes
- [ ] T080 [US5] Verify syntax highlighting adapts correctly to dark theme
- [ ] T081 [US5] Test chatbot widget styles in both light and dark modes

**Checkpoint**: Dark mode functional with theme persistence

---

## Phase 10: User Story 3 - Select Text and Ask AI (Priority: P3)

**Goal**: Highlight text in textbook and immediately ask AI to explain/clarify

**Independent Test**: Select text in any chapter, click "Ask AI", verify chatbot receives selected text as context

### Text Selection Implementation for User Story 3

- [ ] T082 [P] [US3] Create TextSelectionTooltip component in src/components/ChatbotWidget/TextSelectionTooltip.tsx
- [ ] T083 [US3] Implement text selection detection using Selection API in TextSelectionTooltip
- [ ] T084 [US3] Position tooltip near text selection using floating-ui or custom positioning
- [ ] T085 [US3] Pass selected text (max 300 words) to ChatbotWidget as context when "Ask AI" clicked
- [ ] T086 [US3] Test text selection on desktop: select text, click "Ask AI", verify chatbot opens with context
- [ ] T087 [US3] Test text selection on mobile: long-press text, verify "Ask AI" option accessible
- [ ] T088 [US3] Test text selection within code blocks and verify chatbot receives code context

**Checkpoint**: Text selection feature working on desktop and mobile

---

## Phase 11: User Story 9 - Switch to Urdu Translation (Priority: P4 - Optional)

**Goal**: Toggle between English and Urdu versions of textbook

**Independent Test**: Click language toggle, verify Urdu content appears

**NOTE**: This phase is OPTIONAL per spec. Only implement if user explicitly requests and resources allow.

### Urdu Translation Implementation for User Story 9 (OPTIONAL)

- [ ] T089 [P] [US9] Create Urdu content directory structure in docs/ur/chapter-01/ through chapter-06/
- [ ] T090 [P] [US9] Translate Chapter 1 content to Urdu in docs/ur/chapter-01/ files
- [ ] T091 [P] [US9] Translate Chapter 2 content to Urdu in docs/ur/chapter-02/ files
- [ ] T092 [P] [US9] Translate Chapter 3 content to Urdu in docs/ur/chapter-03/ files
- [ ] T093 [P] [US9] Translate Chapter 4 content to Urdu in docs/ur/chapter-04/ files
- [ ] T094 [P] [US9] Translate Chapter 5 content to Urdu in docs/ur/chapter-05/ files
- [ ] T095 [P] [US9] Translate Chapter 6 content to Urdu in docs/ur/chapter-06/ files
- [ ] T096 [US9] Configure Docusaurus i18n plugin in docusaurus.config.js for Urdu locale
- [ ] T097 [US9] Run indexing script for Urdu content to populate Qdrant with Urdu embeddings
- [ ] T098 [US9] Test language toggle and verify Urdu chatbot answers in Urdu
- [ ] T099 [US9] Verify code examples remain in original language but comments are in Urdu

**Checkpoint**: Urdu translation fully functional (if implemented)

---

## Phase 12: User Story 10 - Personalize Learning Path (Priority: P4 - Optional)

**Goal**: Mark chapters complete and track progress

**Independent Test**: Mark chapters complete, verify progress persists across sessions

**NOTE**: This phase is OPTIONAL per spec. Only implement if user explicitly requests.

### Progress Tracking Implementation for User Story 10 (OPTIONAL)

- [ ] T101 [P] [US10] Create ProgressTracker component in src/components/ProgressTracker/index.tsx
- [ ] T102 [US10] Implement localStorage-based progress tracking (chapter completion state)
- [ ] T103 [US10] Add "Mark as Complete" button to chapter pages
- [ ] T104 [US10] Display progress indicator on homepage showing % chapters completed
- [ ] T105 [US10] Add checkmark next to completed chapters in sidebar
- [ ] T106 [US10] Test progress persistence across browser sessions

**Checkpoint**: Progress tracking functional (if implemented)

---

## Phase 13: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Deployment & Performance

- [ ] T107 [P] Create GitHub Actions workflow in .github/workflows/frontend-deploy.yml for Docusaurus deployment to GitHub Pages
- [ ] T108 [P] Create GitHub Actions workflow in .github/workflows/backend-deploy.yml for FastAPI deployment to Railway (per ADR-003)
- [ ] T109 [P] Create Lighthouse CI workflow in .github/workflows/lighthouse-ci.yml with >90 score requirement on homepage and 2 chapter pages
- [ ] T110 [P] Configure image lazy-loading in docusaurus.config.js and implement React.lazy for ChatbotWidget component (FR-029)
- [ ] T111 [P] Verify Docusaurus code splitting configuration and analyze bundle size with webpack-bundle-analyzer (FR-030)
- [ ] T112 [P] Create test workflow in .github/workflows/test.yml for backend pytest execution

### Documentation & Quality

- [ ] T113 [P] Run accessibility audit with axe-core or Lighthouse accessibility module, fix violations to achieve WCAG 2.1 AA compliance (TC-014)
- [ ] T114 [P] Test all code examples from chapters 3-6 on fresh Ubuntu 22.04 + ROS 2 Humble environment, verify 100% execute successfully (FR-004, SC-007)
- [ ] T115 [P] Create comprehensive README.md with project overview, setup instructions, and architecture
- [ ] T116 [P] Create quickstart.md validation: follow all steps in fresh Ubuntu 22.04 environment and verify success
- [ ] T117 [P] Run npm audit and pip check, fix all high/critical vulnerabilities
- [ ] T118 [P] Verify all 6 chapters render correctly with no broken links or missing images
- [ ] T119 [P] Run Docusaurus build and verify completion in <60 seconds
- [ ] T120 Create LICENSE file (CC BY-SA 4.0 for content, MIT for code)
- [ ] T121 [P] Test mobile responsiveness on iOS Safari and Android Chrome (min-width 320px)
- [ ] T122 [P] Verify all P1 acceptance criteria from spec.md are met
- [ ] T123 Verify chatbot achieves >90% retrieval precision with 30-question test set
- [ ] T124 Deploy to production (GitHub Pages or Vercel) and verify site is live

**Checkpoint**: Project complete, deployed, and validated

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phases 3-12)**: All depend on Foundational phase completion
  - P1 stories (US1, US4, US7): MVP - deliver first in priority order
  - P2 stories (US2, US6, US8): Enhanced features - deliver after MVP
  - P3 stories (US3, US5): Nice-to-have - deliver after P2
  - P4 stories (US9, US10): Optional - only if requested
- **Polish (Phase 13)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 4 (P1)**: Can start after Foundational - No dependencies on other stories
- **User Story 7 (P1)**: Can start after Foundational - Integrates with US1 content
- **User Story 2 (P2)**: Depends on US1 (content must exist for RAG to index)
- **User Story 6 (P2)**: Depends on US1 (content must exist for search to index)
- **User Story 8 (P2)**: Depends on US1 (adds to existing chapters)
- **User Story 3 (P3)**: Depends on US2 (chatbot must exist to receive selected text)
- **User Story 5 (P3)**: Can start after Foundational - No dependencies on other stories
- **User Story 9 (P4)**: Depends on US1 and US2 (content and chatbot exist for translation)
- **User Story 10 (P4)**: Depends on US1 (chapters must exist to track progress)

### Within Each User Story

- Content creation before indexing (US1 → US2)
- Backend services before frontend components
- Models before services
- Services before API endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002-T009)
- All Foundational tasks marked [P] can run in parallel within Phase 2
- **Content creation for all 6 chapters can happen in parallel** (T022-T039)
- Backend models and services marked [P] can run in parallel
- Once Foundational phase completes, P1 stories (US1, US4, US7) can start in parallel
- After US1 completes, P2 stories (US2, US6, US8) can run in parallel
- Further Resources for all chapters can be added in parallel (T071-T076)
- Polish tasks marked [P] can run in parallel

---

## Parallel Example: User Story 1 (Content Creation)

```bash
# Launch all chapter content creation in parallel:
Task T022: "Create Chapter 1 structure in docs/chapter-01/index.md"
Task T025: "Create Chapter 2 structure in docs/chapter-02/index.md"
Task T028: "Create Chapter 3 structure in docs/chapter-03/index.md"
Task T031: "Create Chapter 4 structure in docs/chapter-04/index.md"
Task T034: "Create Chapter 5 structure in docs/chapter-05/index.md"
Task T037: "Create Chapter 6 structure in docs/chapter-06/index.md"

# All chapter sections can be written in parallel:
Task T023: "Write Chapter 1 sections in docs/chapter-01/"
Task T026: "Write Chapter 2 sections in docs/chapter-02/"
Task T029: "Write Chapter 3 sections in docs/chapter-03/"
Task T032: "Write Chapter 4 sections in docs/chapter-04/"
Task T035: "Write Chapter 5 sections in docs/chapter-05/"
Task T038: "Write Chapter 6 sections in docs/chapter-06/"
```

---

## Parallel Example: User Story 2 (Backend + Frontend)

```bash
# Backend RAG services can run in parallel:
Task T052: "Implement vector search service in backend/src/services/vector_db.py"
Task T053: "Implement RAG orchestration in backend/src/services/rag.py"

# Frontend components can run in parallel:
Task T059: "Create ChatbotWidget in src/components/ChatbotWidget/index.tsx"
Task T060: "Create ChatMessage in src/components/ChatbotWidget/ChatMessage.tsx"
Task T061: "Style chatbot in src/components/ChatbotWidget/ChatbotWidget.module.css"
```

---

## Implementation Strategy

### MVP First (P1 Stories: US1, US4, US7)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T021) - **CRITICAL BLOCKER**
3. Complete Phase 3: User Story 1 - Content (T022-T041)
4. Complete Phase 4: User Story 4 - Sidebar (T042-T045)
5. Complete Phase 5: User Story 7 - Code Examples (T046-T051)
6. **STOP and VALIDATE**: Test P1 stories independently
7. Deploy MVP to staging

### Incremental Delivery (Add P2 Stories)

1. Complete MVP (Phases 1-5) → Foundation + Core Content ready
2. Add Phase 6: User Story 2 - RAG Chatbot (T052-T066) → Test independently → Deploy
3. Add Phase 7: User Story 6 - Search (T067-T070) → Test independently → Deploy
4. Add Phase 8: User Story 8 - Resources (T071-T077) → Test independently → Deploy
5. Each story adds value without breaking previous stories

### Full Implementation (Add P3/P4 Stories if requested)

1. Complete MVP + P2 stories
2. Add Phase 9: User Story 5 - Dark Mode (T078-T081)
3. Add Phase 10: User Story 3 - Text Selection (T082-T088)
4. Add Phase 11: User Story 9 - Urdu (T089-T099) - **OPTIONAL**
5. Add Phase 12: User Story 10 - Progress (T101-T106) - **OPTIONAL**
6. Complete Phase 13: Polish & Deploy (T107-T120)

### Parallel Team Strategy

With multiple developers after Foundational phase (T021) completes:

- **Developer A**: User Story 1 (Content creation - T022-T041)
- **Developer B**: User Story 4 + 7 (Sidebar + Code - T042-T051)
- **Developer C**: Backend for User Story 2 (RAG - T052-T058)
- **Developer D**: Frontend for User Story 2 (Chatbot Widget - T059-T066)

Stories complete and integrate independently.

---

## Summary

- **Total Tasks**: 124 tasks
- **P1 User Stories (MVP)**: 3 stories (US1, US4, US7) - 32 tasks (T022-T052 + T001-T021)
- **P2 User Stories**: 3 stories (US2, US6, US8) - 36 tasks (T053-T078)
- **P3 User Stories**: 2 stories (US3, US5) - 11 tasks (T079-T089)
- **P4 User Stories (Optional)**: 2 stories (US9, US10) - 17 tasks (T090-T106)
- **Setup + Foundation**: 21 tasks (T001-T021)
- **Polish**: 18 tasks (T107-T124)

**Parallel Opportunities**: 47 tasks marked [P] can run in parallel within their phase

**Suggested MVP Scope**: Phases 1-5 (T001-T052) = Setup + Foundation + US1 + US4 + US7

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label (US1, US2, etc.) maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests are NOT included as spec did not explicitly request TDD approach
- P4 stories (US9, US10) are OPTIONAL per spec - only implement if user requests
- Backend uses FastAPI in `backend/`, frontend uses Docusaurus at repo root
- All content in `docs/`, all static assets in `static/`
