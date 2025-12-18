# Tasks: Mobile UI Fix

**Input**: Design documents from `/specs/011-mobile-ui-fix/`
**Prerequisites**: plan.md, spec.md, research.md, quickstart.md

**Tests**: Manual testing per quickstart.md (no automated tests requested)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus web application with structure:
- `src/components/` - React components
- `docusaurus.config.js` - Docusaurus configuration
- Frontend-only changes (no backend)

---

## Phase 1: Setup (Preparation)

**Purpose**: Prepare environment and verify current state before making changes

- [ ] T001 [P] Create backup of src/components/ChatWidget/styles.module.css
- [ ] T002 [P] Create backup of docusaurus.config.js
- [ ] T003 Verify current z-index values in ChatWidget styles using browser DevTools

**Checkpoint**: Current state documented, ready for modifications

---

## Phase 2: Foundational (Pre-Flight Verification)

**Purpose**: Validate current issues and establish baseline before fixes

**⚠️ CRITICAL**: These tasks verify the problem exists before implementing fixes

- [ ] T004 Test hamburger menu functionality on Android device or Chrome DevTools mobile emulator (width < 768px)
- [ ] T005 Test text selection near navbar to reproduce selectTooltip overlap issue
- [ ] T006 Document current behavior with screenshots (before state)

**Checkpoint**: Issues confirmed, baseline documented - ready for user story implementation

---

## Phase 3: User Story 1 - Mobile Navigation Access (Priority: P1) 🎯 MVP

**Goal**: Enable navbar auto-hide on scroll for better mobile UX, ensuring hamburger menu remains accessible

**Independent Test**: Open site on mobile (width < 768px), scroll down to see navbar hide, scroll up to see it reappear. Hamburger menu should be accessible throughout.

### Implementation for User Story 1

- [x] T007 [US1] Add hideOnScroll: true to navbar configuration in docusaurus.config.js (line ~62, inside themeConfig.navbar object)
- [ ] T008 [US1] Start dev server and verify navbar auto-hide behavior on mobile viewport
- [ ] T009 [US1] Test hamburger menu accessibility with auto-hide enabled (per quickstart.md Test 3)
- [ ] T010 [US1] Verify navbar reappears when scrolling up on mobile

**Checkpoint**: Navbar hideOnScroll working. Hamburger menu accessible on scroll. User Story 1 functional and testable independently.

---

## Phase 4: User Story 2 - Chat Widget Accessibility (Priority: P1)

**Goal**: Fix selectTooltip z-index to prevent overlap with navbar, ensuring both ChatWidget and hamburger menu are accessible

**Independent Test**: Open site on mobile (width < 768px), select text near the top navigation area, verify "Ask AI" tooltip does not overlap hamburger menu.

### Implementation for User Story 2

- [x] T011 [US2] Update selectTooltip z-index from 100 to 99 in src/components/ChatWidget/styles.module.css (line 297)
- [x] T012 [US2] Add CSS comment explaining z-index layering: "/* Z-index 99: Below navbar (100) to prevent overlap with hamburger menu */"
- [ ] T013 [US2] Verify selectTooltip z-index change using browser DevTools (Inspect > Computed styles)
- [ ] T014 [US2] Test text selection near hamburger menu (per quickstart.md Test 2)
- [ ] T015 [US2] Verify ChatWidget button and modal display correctly on mobile (per quickstart.md Test 4)

**Checkpoint**: selectTooltip does not overlap navbar. ChatWidget and hamburger menu both accessible. User Story 2 functional and testable independently.

---

## Phase 5: User Story 3 - Proper Z-Index Layering (Priority: P2)

**Goal**: Document and verify z-index layering strategy is correctly implemented across all UI elements

**Independent Test**: Use browser DevTools to inspect z-index values of all UI elements and verify layering hierarchy matches design.

### Implementation for User Story 3

- [ ] T016 [P] [US3] Verify navbar z-index is 100 (Docusaurus default) using browser DevTools
- [ ] T017 [P] [US3] Verify floatingButton z-index is 99 in src/components/ChatWidget/styles.module.css (line 14)
- [ ] T018 [P] [US3] Verify chatModal z-index is 98 in src/components/ChatWidget/styles.module.css (line 44)
- [ ] T019 [US3] Document final z-index layering in plan.md (Navbar: 100, selectTooltip: 99, floatingButton: 99, chatModal: 98)
- [ ] T020 [US3] Test visual stacking order when navbar dropdown and ChatWidget are both visible

**Checkpoint**: All z-index values verified and documented. UI element stacking order correct. User Story 3 complete.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Comprehensive testing, validation, and documentation

### Manual Testing (per quickstart.md)

- [ ] T021 [P] Execute Test 1: Z-Index Layering Verification (DevTools inspection)
- [ ] T022 [P] Execute Test 2: Text Selection Tooltip Behavior (select text near navbar)
- [ ] T023 [P] Execute Test 3: Navbar hideOnScroll Behavior (scroll down/up)
- [ ] T024 [P] Execute Test 4: ChatWidget Mobile Responsiveness (320px, 375px, 414px, 768px)
- [ ] T025 Execute Test 5: Android Device Testing (if physical device available)
- [ ] T026 [P] Execute Test 6: Desktop Regression Testing (verify desktop UI unchanged)
- [ ] T027 [P] Execute Test 7: Accessibility Verification (touch target sizes ≥44px)

### Edge Case Testing

- [ ] T028 Test device rotation (portrait ↔ landscape around 768px breakpoint)
- [ ] T029 Test very small screens (<375px width)
- [ ] T030 Test simultaneous sidebar and chat modal opening
- [ ] T031 Test tablet viewport (768px-1024px width)

### Performance Validation

- [ ] T032 Measure touch response time for hamburger menu (<100ms target, per spec SC-006)
- [ ] T033 Verify visual separation between ChatWidget button and hamburger menu (≥8px, per spec SC-002)
- [ ] T034 Verify no horizontal scrollbar on mobile viewports (320px-768px, per spec SC-004)

### Documentation & Cleanup

- [ ] T035 Document before/after screenshots in specs/011-mobile-ui-fix/
- [ ] T036 Update CHANGELOG or commit message with fix summary
- [ ] T037 Remove backup files created in Phase 1 (if changes are verified successful)

**Checkpoint**: All tests pass. All success criteria met. Ready for deployment.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - Verifies baseline before fixes
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - US1 (Phase 3) and US2 (Phase 4) can proceed in parallel (different files)
  - US3 (Phase 5) should follow US1 and US2 for verification
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - Modifies docusaurus.config.js
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Modifies styles.module.css
- **User Story 3 (P2)**: Should follow US1 and US2 for verification - No file modifications

### Within Each User Story

- US1: Config change → restart server → test on mobile
- US2: CSS change → refresh browser → test text selection
- US3: DevTools inspection → documentation → visual testing

### Parallel Opportunities

- **Phase 1 (Setup)**: T001 and T002 (backups) can run in parallel
- **Phase 3 + Phase 4**: US1 and US2 can be worked on in parallel (different files, no conflicts)
- **Phase 5 (US3)**: T016, T017, T018 (DevTools inspections) can run in parallel
- **Phase 6 (Testing)**: Most tests marked [P] can run in parallel

---

## Parallel Example: User Story 1 and User Story 2

Since US1 and US2 modify different files, they can be implemented in parallel:

```bash
# Developer A (or first session): User Story 1
Task: "Add hideOnScroll: true to navbar configuration in docusaurus.config.js"
Task: "Test hamburger menu accessibility with auto-hide enabled"

# Developer B (or second session): User Story 2
Task: "Update selectTooltip z-index from 100 to 99 in styles.module.css"
Task: "Test text selection near hamburger menu"
```

Both can proceed simultaneously without file conflicts.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (backups)
2. Complete Phase 2: Foundational (verify issues exist)
3. Complete Phase 3: User Story 1 (hideOnScroll)
4. **STOP and VALIDATE**: Test navbar auto-hide on mobile
5. Deploy if US1 alone provides sufficient value

### Incremental Delivery (Recommended)

1. Complete Setup + Foundational → Baseline documented
2. Add User Story 1 → Test independently → Navbar auto-hide working (partial fix!)
3. Add User Story 2 → Test independently → No overlap issue (complete fix!)
4. Add User Story 3 → Verify z-index layering → Documentation complete
5. Polish phase → Comprehensive testing → Deploy

### Parallel Team Strategy

With 2 developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (docusaurus.config.js)
   - Developer B: User Story 2 (styles.module.css)
3. Merge both changes, then proceed to:
   - Either developer: User Story 3 (verification)
4. Both: Polish phase testing in parallel

---

## Success Criteria Checklist

All tasks complete when these success criteria from spec.md are met:

- [ ] **SC-001**: Mobile users on Android can successfully open and close sidebar navigation on first attempt
- [ ] **SC-002**: ChatWidget button and hamburger menu maintain ≥8px visual separation on screens 320px-768px wide
- [ ] **SC-003**: ChatWidget modal displays at full usable size (max width - 16px padding) on screens < 768px
- [ ] **SC-004**: Page loads and renders without horizontal scrollbar on mobile viewports (320px-768px width)
- [ ] **SC-005**: Z-index values correctly applied (Navbar: 100+, ChatWidget: 99) verified in DevTools
- [ ] **SC-006**: All interactive elements respond to touch events within 100ms on mobile devices

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label maps task to specific user story for traceability
- Manual testing per quickstart.md (no automated tests requested in spec)
- Changes are minimal (2 files, ~5 lines of code) per plan.md
- Desktop UI should remain unchanged (verify in Phase 6, Test 6)
- Commit after each user story phase for incremental progress
- Stop at any checkpoint to validate story independently
- Both US1 and US2 are P1 priority and can be done in parallel
