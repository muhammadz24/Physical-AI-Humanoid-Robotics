---
id: PHR-MAIN-002
title: UI Fixes Task Generation
stage: tasks
date: 2025-12-25T00:00:00Z
surface: claude-code-cli
model: claude-sonnet-4.5
feature: main
branch: main
user: developer
command: /sp.tasks
labels: [task-generation, mobile-menu, z-index, css-fixes]
links:
  spec: specs/main/spec.md
  plan: specs/main/plan.md
  tasks: specs/main/tasks.md
  ticket: N/A
  adr: N/A
  pr: N/A
files:
  - path: specs/main/tasks.md
    status: created
    lines: 354
tests:
  - name: Manual visual testing only
    status: defined-in-quickstart
---

## Prompt

User requested task generation for 2 out of 3 UI fixes only:
1. **Force Disable Mobile Menu** (User Story 1, Priority P1)
2. **Navbar Z-Index Verification** (User Story 2, Priority P1)

**SKIP**: User Story 3 (Ghost Popup) - User will provide screenshot later

**Context**: "Ghost popup ko abhi ke liye skip kardo (jab aega tab ss bhej dunga). Abhi sirf in 2 issues ke liye dependency-ordered tasks generate karo."

**Command**: `/sp.tasks` with instruction to immediately proceed to `/sp.implement` after task generation.

## Response snapshot

### Task Generation Process

**Prerequisites Verified**:
- ✅ plan.md exists (tech stack: JavaScript/React 18/Docusaurus 3.x)
- ✅ spec.md exists (3 user stories, P1/P1/P2 priorities)
- ✅ research.md exists (mobile menu CSS decision, z-index verification)
- ✅ data-model.md exists (component state documentation)
- ✅ contracts/ exists (CSS class contracts)

**User Stories Processed**:
1. **User Story 1** (P1): Desktop Navigation Always Visible
   - 12 tasks generated (T005-T016)
   - Focus: Add CSS override `.navbar__toggle { display: none !important }`
   - Testing: 4 viewport sizes (320px, 768px, 1024px, 1920px)

2. **User Story 2** (P1): Chatbot Positioned Below Navbar
   - 15 tasks generated (T017-T031)
   - Focus: Verify z-index hierarchy (navbar: 1000 > chatbot: 99)
   - Testing: Visual inspection, navbar clickability, zoom levels

3. **User Story 3** (P2): SKIPPED per user request

### Task Structure

**Phase 1: Setup** (T001-T004)
- Verify Node.js, npm, Docusaurus 3.x installed
- Start dev server and verify hot-reload
- Open DevTools and check for errors

**Phase 2: User Story 1** (T005-T016) - Mobile Menu
- Implementation: Add 3-line CSS rule to `src/css/custom.css`
- Testing: 11 tasks covering viewports, F12 console, zoom, navigation, DevTools inspection

**Phase 3: User Story 2** (T017-T031) - Z-Index
- Investigation: Read existing z-index values (T017-T019)
- Verification: 11 visual tests (chatbot interaction, navbar clickability, zoom)
- Optional: Comment fix in `styles.module.css` line 408

**Phase 4: Polish** (T032-T042)
- Production build test (`npm run build`, `npm run serve`)
- Lighthouse performance validation (score >90, CLS <0.1)
- Bundle size check (+50 bytes expected)
- Git diff review

### Key Decisions

**No Automated Tests**: Manual visual testing only (not requested in spec)
- Research.md specifies "Manual visual testing across breakpoints"
- Quickstart.md provides detailed test procedures
- No TDD approach requested

**Independent User Stories**:
- US1 and US2 are independent (no cross-dependencies)
- Can be implemented in parallel by 2 developers
- Can be completed sequentially by 1 developer

**Parallel Opportunities**:
- 8 tasks marked [P] (can run in parallel)
- US2 investigation (T017-T019) can overlap with US1 implementation (T005-T006)

### Task Format Validation

All tasks follow required checklist format:
```
- [ ] [TaskID] [P?] [Story] Description with file path
```

**Examples**:
- ✅ `- [ ] T006 [US1] Add CSS override for .navbar__toggle in src/css/custom.css`
- ✅ `- [ ] T018 [P] [US2] Read ChatWidget z-index values in src/components/ChatWidget/styles.module.css`
- ✅ `- [ ] T032 [P] Run production build test: npm run build`

## Outcome

- ✅ **Impact**: Complete dependency-ordered task list with 42 tasks covering both P1 user stories. Estimated total implementation time: 50 minutes (20 min US1 + 15 min US2 + 10 min polish + 5 min setup).

- 🧪 **Tests**: No automated tests generated (not requested). Manual visual testing procedures defined across 22 testing tasks (T008-T016 for US1, T020-T030 for US2, T034-T038 for production validation).

- 📁 **Files**: Generated `specs/main/tasks.md` (354 lines) with 4 phases, dependency graph, parallel execution examples, and success criteria checklist.

- 🔁 **Next prompts**: Immediately proceed to `/sp.implement` to execute tasks T001-T042 sequentially (per user instruction).

- 🧠 **Reflection**: User Story 3 (Ghost Popup) successfully excluded from task generation. Tasks organized by user story for independent testability. US1 has clear implementation path (CSS rule). US2 is verification-only (no code changes needed per research.md finding).

## Evaluation notes (flywheel)

- **Failure modes observed**: None. Task template correctly applied. All tasks have proper IDs, story labels, and file paths.

- **Graders run and results (PASS/FAIL)**:
  - Task Format Validation: ✅ PASS (all 42 tasks follow `- [ ] [ID] [P?] [Story] Description` format)
  - User Story Coverage: ✅ PASS (US1 and US2 fully covered, US3 intentionally skipped)
  - Dependency Order: ✅ PASS (Setup → US1/US2 → Polish)
  - File Path Specificity: ✅ PASS (all tasks include exact file paths)
  - Parallel Marking: ✅ PASS (8 tasks marked [P] appropriately)

- **Prompt variant (if applicable)**: Standard `/sp.tasks` with scope reduction (2 user stories instead of 3).

- **Next experiment (smallest change to try)**: For User Story 3 (Ghost Popup), when user provides screenshot, add investigation tasks (T043-T050) to identify popup source before implementation tasks.
