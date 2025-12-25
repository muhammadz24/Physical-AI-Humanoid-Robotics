# Tasks: UI Critical Fixes

**Input**: Design documents from `/specs/main/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Not requested - manual visual testing only (per quickstart.md)

**Organization**: Tasks grouped by user story for independent implementation and testing.

**Scope**: User Story 1 (Mobile Menu) and User Story 2 (Z-Index) only. User Story 3 (Ghost Popup) SKIPPED per user request (awaiting screenshot).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2)
- All file paths are absolute or relative to repository root

## Path Conventions

- **Project Type**: Web (Docusaurus static site + React components)
- **Frontend**: `src/` at repository root
- **Custom CSS**: `src/css/custom.css`
- **Component Styles**: `src/components/ChatWidget/styles.module.css`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify project environment and dependencies

- [X] T001 Verify Node.js 18+ and npm installed (`node --version`, `npm --version`)
- [X] T002 Verify project dependencies installed (`npm list` shows Docusaurus 3.x)
- [ ] T003 [P] Start Docusaurus dev server (`npm start`) and verify hot-reload working
- [ ] T004 [P] Open browser DevTools and verify no existing console errors

**Checkpoint**: Development environment ready for CSS modifications

---

## Phase 2: User Story 1 - Desktop Navigation Always Visible (Priority: P1) 🎯

**Goal**: Force disable mobile hamburger menu at ALL viewport sizes while maintaining desktop navbar items visibility

**Independent Test**: Resize browser to 320px, 768px, 1024px, 1920px and verify hamburger menu never appears while desktop navbar items remain visible

**Acceptance Criteria** (from spec.md):
1. Browser viewport at 320px → desktop navbar visible, hamburger hidden
2. Browser with F12 console open → desktop navbar visible, hamburger hidden
3. Any screen size 320px - 4K → only desktop navigation items rendered

### Implementation for User Story 1

- [X] T005 [US1] Read existing navbar styles in `src/css/custom.css` lines 87-101 to understand current implementation
- [X] T006 [US1] Add CSS override for `.navbar__toggle` after line 101 in `src/css/custom.css`:
  ```css
  /* ========================================
     Force Disable Mobile Menu (FR-001, FR-002)
     ======================================== */

  /* Hide hamburger toggle at ALL breakpoints to maintain desktop nav visibility.
     Overrides Docusaurus default responsive behavior (@media max-width: 996px).
     Related: User Story 1 - Desktop Navigation Always Visible */
  .navbar__toggle {
    display: none !important;
    visibility: hidden !important;
  }
  ```
- [ ] T007 [US1] Verify hot-reload applies CSS changes (check browser console for "Hot update applied" message)
- [ ] T008 [US1] Test viewport 320px: Open DevTools (F12) → Toggle device toolbar (Ctrl+Shift+M) → Set to "iPhone SE" (320x568) → Verify hamburger menu hidden, desktop items visible
- [ ] T009 [US1] Test viewport 768px: Set to "iPad" (768x1024) → Verify hamburger menu hidden, desktop items visible
- [ ] T010 [US1] Test viewport 1024px: Set to "iPad Pro" (1024x768) → Verify hamburger menu hidden, desktop items visible
- [ ] T011 [US1] Test viewport 1920px: Resize to desktop size → Verify hamburger menu hidden, desktop items visible
- [ ] T012 [US1] Test F12 console scenario: Full browser window (1920x1080) → Open F12 (docks bottom, viewport becomes ~1920x600) → Verify desktop navbar still visible, no hamburger
- [ ] T013 [US1] Test browser zoom levels: Zoom to 150% (Ctrl+) → 200% → Verify navbar items visible at all zoom levels
- [ ] T014 [US1] Inspect `.navbar__toggle` element in DevTools → Verify computed styles show `display: none` and `visibility: hidden` with `!important` flag
- [ ] T015 [US1] Navigate between 5+ pages (Introduction → Chapter 1 → Chapter 2 → Homepage → Docs) → Verify hamburger never appears during navigation
- [ ] T016 [US1] Check browser console for errors or warnings related to navbar → Verify no errors

**Checkpoint**: User Story 1 complete - Mobile menu permanently disabled at all breakpoints

---

## Phase 3: User Story 2 - Chatbot Positioned Below Navbar (Priority: P1) 🎯

**Goal**: Verify navbar (z-index: 1000) stays above ChatWidget components (z-index: 97-99) and optionally fix inaccurate comment

**Independent Test**: Open chatbot widget and verify navbar remains fully visible and interactive above chatbot container

**Acceptance Criteria** (from spec.md):
1. Chatbot closed → click to open → chatbot slides in below navbar without covering navbar elements
2. Chatbot open → scroll page → navbar and chatbot maintain proper stacking order
3. Chatbot open → click navbar links → navbar remains fully interactive and visible

### Investigation for User Story 2

- [X] T017 [US2] Read navbar z-index in `src/css/custom.css` line 94 → Verify value is 1000
- [X] T018 [P] [US2] Read ChatWidget z-index values in `src/components/ChatWidget/styles.module.css`:
  - Line 14: `.chatButton` (should be 99)
  - Line 38: `.modalBackdrop` (should be 97)
  - Line 56: `.chatModal` (should be 98)
  - Line 408: `.selectToAskTooltip` (should be 99)
- [X] T019 [US2] Verify z-index hierarchy: 97 < 98 < 99 < 1000 (ChatWidget below navbar) ✅

### Verification Testing for User Story 2

- [ ] T020 [US2] Open homepage (`http://localhost:3000`) in browser
- [ ] T021 [US2] Click ChatWidget floating button (bottom-right cyan robot icon) → Verify chatbot modal slides in from right
- [ ] T022 [US2] Visual inspection: Verify navbar at top of page is NOT covered by chatbot modal
- [ ] T023 [US2] Click "Introduction" link in navbar while chatbot open → Verify link is clickable (not blocked by chatbot)
- [ ] T024 [US2] Verify page navigates successfully → Chatbot should close or remain open without blocking navigation
- [ ] T025 [US2] Open chatbot again → Scroll page up and down → Verify navbar and chatbot maintain correct stacking order (navbar on top)
- [ ] T026 [US2] Test with browser zoom: Zoom to 150% (Ctrl+) → 200% → Verify navbar still above chatbot at all zoom levels
- [ ] T027 [US2] Inspect navbar element in DevTools → Check computed styles → Verify `z-index: 1000` and `position: sticky`
- [ ] T028 [US2] Inspect chatbot modal element (`.chatModal`) in DevTools → Check computed styles → Verify `z-index: 98` and `position: fixed`
- [ ] T029 [US2] Verify stacking context: Both navbar and chatbot should create independent stacking contexts (position != static, z-index != auto)
- [ ] T030 [US2] Test navbar interaction: With chatbot open, click 3+ different navbar links → All should be clickable and navigate successfully

### Optional Comment Fix for User Story 2

- [X] T031 [P] [US2] OPTIONAL: Update comment in `src/components/ChatWidget/styles.module.css` line 408:
  - Current: `/* Z-index 99: Below navbar (100) to prevent overlap with hamburger menu */`
  - Corrected: `/* Z-index 99: Below navbar (1000) to prevent overlap with navbar elements */`
  - Rationale: Navbar is actually z-index 1000, not 100 (accuracy fix)

**Checkpoint**: User Story 2 complete - Z-index hierarchy verified correct, navbar stays above chatbot

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and documentation

- [ ] T032 [P] Run production build test: `npm run build` → Verify build succeeds with no errors
- [ ] T033 [P] Run production preview: `npm run serve` → Verify built site works correctly at `http://localhost:3000`
- [ ] T034 [P] Test production build: Repeat User Story 1 tests (T008-T015) on production build → Verify mobile menu disabled
- [ ] T035 [P] Test production build: Repeat User Story 2 tests (T020-T030) on production build → Verify z-index hierarchy
- [ ] T036 Check bundle size impact: `du -sh build/` → Compare before/after (expected: +50 bytes for CSS rule)
- [ ] T037 [P] Verify Lighthouse performance score: Chrome DevTools → Lighthouse → Generate report → Performance should remain >90
- [ ] T038 [P] Verify no layout shift: Lighthouse → Cumulative Layout Shift (CLS) should remain <0.1
- [ ] T039 Review git diff: `git diff src/css/custom.css` → Verify only expected changes (`.navbar__toggle` rule)
- [ ] T040 Review git diff: `git diff src/components/ChatWidget/styles.module.css` → Verify only comment change (if T031 completed)
- [ ] T041 Verify no temporary debug code left in codebase (search for "DEBUG", "TEMP", "TODO" in modified files)
- [ ] T042 Update CLAUDE.md if needed: Document `.navbar__toggle` override pattern for future reference (optional)

**Checkpoint**: All fixes validated in development and production builds

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **User Story 1 (Phase 2)**: Depends on Setup (T001-T004) completion
- **User Story 2 (Phase 3)**: Can run in parallel with User Story 1 OR sequentially after User Story 1
  - Investigation tasks (T017-T019) can run in parallel with User Story 1 implementation (T005-T006)
  - Verification testing (T020-T030) should run after chatbot widget is confirmed working
- **Polish (Phase 4)**: Depends on both User Story 1 and User Story 2 completion

### User Story Dependencies

- **User Story 1**: INDEPENDENT - No dependencies on User Story 2
- **User Story 2**: INDEPENDENT - No dependencies on User Story 1 (only verification, no code changes needed)

### Within Each User Story

**User Story 1 (Sequential)**:
```
T005 (read styles) → T006 (add CSS) → T007 (verify hot-reload) → T008-T015 (testing) → T016 (verify no errors)
```

**User Story 2 (Sequential investigation, then testing)**:
```
T017-T019 (verify z-index values) → T020-T030 (visual testing) → T031 (optional comment fix)
```

### Parallel Opportunities

**Setup Phase (T001-T004)**:
- T003 and T004 can run in parallel (different tasks)

**User Story 1 and User Story 2**:
- User Story 2 investigation (T017-T019) can run in parallel with User Story 1 implementation (T005-T006)
- Both stories are independent and can be completed in parallel by different developers

**Polish Phase**:
- T032, T033, T034, T035, T037, T038 can all run in parallel (independent validation tasks)

---

## Parallel Example: Cross-Story Execution

### Option 1: Sequential (Single Developer)

```bash
# Complete Setup first
Tasks T001-T004: Setup and verify environment

# Complete User Story 1 (Mobile Menu)
Tasks T005-T016: Implement and test mobile menu disable

# Complete User Story 2 (Z-Index)
Tasks T017-T031: Verify and test z-index hierarchy

# Complete Polish
Tasks T032-T042: Final validation
```

### Option 2: Parallel (Two Developers)

```bash
# Both developers complete Setup together
Tasks T001-T004: Setup and verify environment

# Developer A: User Story 1
Tasks T005-T016: Mobile menu disable

# Developer B: User Story 2 (can start immediately after setup)
Tasks T017-T031: Z-index verification

# Both developers: Polish (after both stories complete)
Tasks T032-T042: Final validation
```

---

## Implementation Strategy

### MVP First (Both User Stories are P1 - High Priority)

Since both User Story 1 and User Story 2 are Priority P1, the recommended approach is:

1. **Complete Phase 1: Setup** (T001-T004)
2. **Complete Phase 2: User Story 1** (T005-T016) - Mobile Menu Disable
   - **STOP and VALIDATE**: Test at 4 viewports (320px, 768px, 1024px, 1920px)
   - Verify hamburger menu never appears
3. **Complete Phase 3: User Story 2** (T017-T031) - Z-Index Verification
   - **STOP and VALIDATE**: Test chatbot interaction, navbar clickability
   - Verify navbar always above chatbot
4. **Complete Phase 4: Polish** (T032-T042)
5. **Deploy/Commit**: Create pull request with both fixes

### Time Estimates

- **Setup**: 5 minutes (verify environment)
- **User Story 1**: 20 minutes (5 min implementation + 15 min testing across viewports)
- **User Story 2**: 15 minutes (5 min investigation + 10 min testing)
- **Polish**: 10 minutes (build test + validation)
- **Total**: ~50 minutes for complete implementation and testing

---

## Notes

- **[P] tasks**: Different files or independent operations, can run in parallel
- **[Story] label**: Maps task to specific user story (US1 = User Story 1, US2 = User Story 2)
- **No automated tests**: Manual visual testing per quickstart.md (no TDD requested in spec)
- **User Story 3 SKIPPED**: Ghost popup investigation deferred until user provides screenshot
- **Verification focused**: User Story 2 is primarily verification (z-index already correct per research.md)
- **Commit strategy**: Commit after each user story phase completion for granular rollback
- **Rollback available**: `git checkout main -- src/css/custom.css src/components/ChatWidget/` to undo changes

---

## Success Criteria Summary

### User Story 1: Desktop Navigation Always Visible ✅

- [ ] Hamburger menu hidden at 320px viewport
- [ ] Hamburger menu hidden at 768px viewport
- [ ] Hamburger menu hidden at 1024px viewport
- [ ] Hamburger menu hidden at 1920px viewport
- [ ] Hamburger menu hidden with F12 console open
- [ ] Desktop navbar items visible at all breakpoints
- [ ] No console errors or warnings
- [ ] Passes production build test

### User Story 2: Chatbot Positioned Below Navbar ✅

- [ ] Navbar visible above chatbot when chatbot open (visual inspection)
- [ ] Navbar links clickable when chatbot open (functional test)
- [ ] Z-index hierarchy verified: navbar (1000) > chatbot (99)
- [ ] Stacking order maintained during scroll
- [ ] Works at all zoom levels (100%, 150%, 200%)
- [ ] Passes production build test

### Overall Success ✅

- [ ] Both user stories independently functional
- [ ] Production build succeeds (`npm run build`)
- [ ] Lighthouse performance score >90
- [ ] Bundle size impact minimal (+50 bytes)
- [ ] No layout shift regression (CLS <0.1)
- [ ] Git diff shows only expected changes

---

**Task Generation Complete** ✅
**Total Tasks**: 42 tasks (4 setup + 12 US1 + 15 US2 + 11 polish)
**Parallel Opportunities**: 8 tasks marked [P]
**Estimated Total Time**: 50 minutes
**Ready for**: `/sp.implement` command
