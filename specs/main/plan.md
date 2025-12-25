# Implementation Plan: UI Critical Fixes

**Branch**: `main` | **Date**: 2025-12-25 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/main/spec.md`

## Summary

This plan addresses 3 critical UI bugs in the Docusaurus-based Physical AI textbook platform:

1. **Force disable mobile hamburger menu** - Prevent Docusaurus mobile menu toggle from appearing at any viewport size
2. **Fix navbar/chatbot z-index hierarchy** - Ensure navbar (z-index: 1000) stays above ChatWidget components (currently z-index: 99)
3. **Eliminate ghost popup triggers** - Identify and remove unwanted automatic popup triggers

**Technical Approach**: CSS-only fixes for issues #1 and #2 via `src/css/custom.css`. Issue #3 requires investigation of ConfirmationModal trigger logic in ChatWidget component state management.

## Technical Context

**Language/Version**: JavaScript (ES6+) / React 18 / Docusaurus 3.x
**Primary Dependencies**:
- `@docusaurus/core` v3.x (static site generator)
- React 18 (component framework)
- CSS Modules (ChatWidget styling)

**Storage**: N/A (CSS-only fixes + state logic modification)
**Testing**: Manual visual testing across breakpoints (320px, 768px, 1024px, 1920px)
**Target Platform**: Web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
**Project Type**: Web (Docusaurus static site + React components)
**Performance Goals**: No performance impact (CSS-only changes)
**Constraints**:
- Must not break existing Docusaurus functionality
- Must preserve desktop navbar items visibility
- Chatbot floating button must remain accessible
**Scale/Scope**: 3 isolated CSS/component fixes affecting 2 files

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle IV: Docusaurus Best Practices Compliance (NON-NEGOTIABLE) ✅

**Status**: PASS

**Verification**:
- Using Docusaurus built-in navbar component (no custom replacement)
- Overriding mobile menu via CSS (`!important` on `.navbar__toggle`)
- Following Docusaurus CSS custom variables and theming system
- Not modifying Docusaurus core files or configuration

**Rationale**: CSS overrides via `src/css/custom.css` are the recommended Docusaurus approach for UI customization per official docs (https://docusaurus.io/docs/styling-layout#global-styles).

### Principle I: Simplicity-First Design ✅

**Status**: PASS

**Verification**:
- Minimal changes (CSS rules + state logic review)
- No new dependencies or frameworks
- No architectural changes

### Principle VI: Fast Build & Iteration Cycles ✅

**Status**: PASS

**Verification**:
- CSS changes hot-reload in <1 second
- No build time impact
- Component state changes trigger instant re-render

### Constitution Compliance Summary

| Principle | Status | Notes |
|-----------|--------|-------|
| I. Simplicity-First | ✅ PASS | CSS-only fixes, minimal code changes |
| II. Accuracy & Fidelity | N/A | No content changes |
| III. Free-Tier Architecture | ✅ PASS | No resource impact |
| IV. Docusaurus Compliance | ✅ PASS | Uses official CSS override approach |
| V. Minimalism in Stack | ✅ PASS | No new dependencies |
| VI. Fast Build Cycles | ✅ PASS | Instant hot-reload |
| VII. Content-First | N/A | UI fixes only |
| VIII. RAG Guardrails | N/A | Chatbot logic unchanged |
| IX. Zero-Edit Deployment | ✅ PASS | No environment-specific changes |

**GATE RESULT**: ✅ ALL CHECKS PASSED - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/main/
├── spec.md              # Feature specification (✅ complete)
├── plan.md              # This file (in progress)
├── research.md          # Phase 0 output (pending)
├── data-model.md        # Phase 1 output (pending - minimal for CSS fixes)
├── quickstart.md        # Phase 1 output (pending)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
src/
├── components/
│   ├── ChatWidget/
│   │   ├── index.js                    # [MODIFY] Remove ghost popup triggers
│   │   └── styles.module.css           # [MODIFY] Adjust z-index values
│   └── UI/
│       ├── ConfirmationModal.js        # [INVESTIGATE] Modal trigger logic
│       └── modal.module.css            # [READ ONLY] Verify z-index hierarchy
├── css/
│   └── custom.css                      # [MODIFY] Add mobile menu disable rules
└── pages/
    └── [unchanged]                     # No page-level changes required
```

**Structure Decision**: Existing Docusaurus web application structure. No new files or directories required. Changes confined to:
1. `src/css/custom.css` - Add `.navbar__toggle` override
2. `src/components/ChatWidget/styles.module.css` - Update z-index values
3. `src/components/ChatWidget/index.js` - Review/fix popup trigger logic

## Complexity Tracking

> **No constitution violations requiring justification.**

All fixes align with Simplicity-First and Docusaurus Compliance principles. Changes are minimal, scoped, and use standard CSS override patterns.

## Phase 0: Research & Investigation

### Research Tasks

#### R1: Docusaurus Mobile Menu Override Best Practices

**Question**: What is the recommended CSS approach to permanently hide `.navbar__toggle` without breaking responsive navbar functionality?

**Investigation Areas**:
- Official Docusaurus docs: https://docusaurus.io/docs/styling-layout#styling-your-site-with-infima
- CSS specificity requirements (`!important` usage)
- Alternative class selectors (`.navbar__toggle`, `button[aria-label="Toggle navigation bar"]`)
- Breakpoint considerations (does override need media queries?)

**Expected Outcome**: CSS rule(s) to add to `custom.css` that:
- Hide hamburger toggle at all breakpoints
- Maintain desktop navbar items visibility
- Do not break Docusaurus navbar functionality

---

#### R2: Z-Index Hierarchy Analysis

**Question**: What z-index values ensure navbar (1000) stays above ChatWidget (99) and ConfirmationModal (10000) maintains correct priority?

**Current State** (from codebase exploration):
```
Z-Index Stack:
├── 97  - ChatWidget modalBackdrop
├── 98  - ChatWidget chatModal
├── 99  - ChatWidget floating button & select-to-ask tooltip
├── 1000 - Docusaurus navbar
├── 9999 - ConfirmationModal backdrop
└── 10000 - ConfirmationModal container
```

**Investigation Areas**:
- Verify navbar at z-index 1000 is above ChatWidget (99) ✅
- Check if select-to-ask tooltip (99) conflicts with navbar on mobile
- Determine if ChatWidget modal (98) ever overlaps navbar due to positioning
- Review CSS `position` property (fixed vs absolute vs sticky) interaction with z-index

**Expected Outcome**:
- Confirm current navbar z-index (1000) is sufficient
- OR identify adjusted z-index values for ChatWidget components
- Document stacking context issues (if any)

---

#### R3: Ghost Popup Trigger Investigation

**Question**: What component is auto-triggering popups and what state logic causes unwanted invocations?

**Investigation Areas**:
1. **ConfirmationModal triggers** (from ChatWidget/index.js):
   - Line 192-245: `handleDelete(messageId)` - triggered by delete button click ✅
   - Line 248-294: `handleDeleteAllHistory()` - triggered by "Delete All" button ✅
   - Search for `setModalConfig({ isOpen: true })` calls without user event handlers

2. **Select-to-Ask tooltip** (lines 573-593):
   - `tooltipVisible` state triggers
   - `handleSelection()` event listener
   - Check if tooltip appears on page load or navigation

3. **Error messages** (inline in ChatWidget):
   - Lines 234-240, 282-290: Error state rendering
   - Check if error state persists across sessions (localStorage)

4. **Third-party scripts**:
   - Verify no analytics/tracking scripts injecting modals
   - Check Docusaurus plugins for popup features

**Expected Outcome**:
- Identify exact component and state variable causing ghost popup
- Document trigger conditions (page load, route change, chatbot open, etc.)
- Propose fix: remove trigger, add conditional guard, or reset state

---

### Research Deliverables

**Output File**: `specs/main/research.md`

**Required Sections**:
1. **Mobile Menu Override Decision**
   - Decision: [CSS rule(s) to use]
   - Rationale: [Why this approach works]
   - Alternatives considered: [Other CSS selectors tried]

2. **Z-Index Hierarchy Decision**
   - Decision: [Keep current values OR adjust to X, Y, Z]
   - Rationale: [Stacking context analysis]
   - Alternatives considered: [Other z-index schemes]

3. **Ghost Popup Root Cause**
   - Decision: [Component/state variable responsible]
   - Rationale: [Trigger condition analysis]
   - Alternatives considered: [Other potential causes ruled out]

**Completion Criteria**: All NEEDS CLARIFICATION items resolved with concrete decisions.

## Phase 1: Design & Contracts

### Data Model

**File**: `specs/main/data-model.md`

**Note**: Minimal data model required (CSS fixes don't involve data structures). Document component state variables for ghost popup fix.

**Required Sections**:

#### Component State Variables

**ChatWidget State** (`src/components/ChatWidget/index.js`):
```javascript
{
  isOpen: boolean,              // Modal visibility
  modalConfig: {                // ConfirmationModal control
    isOpen: boolean,
    title: string,
    message: string,
    onConfirm: function,
    confirmText: string,
    isDanger: boolean
  },
  tooltipVisible: boolean,      // Select-to-ask tooltip
  // ... other state variables
}
```

**State Transitions**:
- `modalConfig.isOpen`: `false` → `true` (on delete button click) → `false` (on confirm/cancel)
- `tooltipVisible`: `false` → `true` (on text selection) → `false` (on tooltip dismiss)

**Invariants**:
- `modalConfig.isOpen` MUST only be `true` after explicit user action (button click)
- `tooltipVisible` MUST only be `true` after text selection event

---

### API Contracts

**File**: `specs/main/contracts/`

**Note**: No API endpoints involved (frontend-only fixes). This section documents CSS interface contracts.

#### CSS Class Contracts

**File**: `specs/main/contracts/css-classes.md`

```markdown
# CSS Class Interface Contracts

## Navbar Mobile Menu Toggle

**Class**: `.navbar__toggle`
**Source**: Docusaurus core
**Override Location**: `src/css/custom.css`

**Contract**:
- MUST be hidden via `display: none !important`
- MUST apply at all breakpoints (no media queries)
- MUST not affect `.navbar__items` visibility

---

## ChatWidget Z-Index

**Classes**:
- `.modalBackdrop` (z-index: 97)
- `.chatModal` (z-index: 98)
- `.chatButton` (z-index: 99)
- `.selectToAskTooltip` (z-index: 99)

**Source**: `src/components/ChatWidget/styles.module.css`

**Contract**:
- All z-index values MUST be < 1000 (below navbar)
- `.chatModal` MUST have `position: fixed` to respect z-index
- Tooltip MUST not overlap navbar on mobile

---

## ConfirmationModal Z-Index

**Classes**:
- `.modalBackdrop` (z-index: 9999)
- `.modalContainer` (z-index: 10000)

**Source**: `src/components/UI/modal.module.css`

**Contract**:
- MUST stay above navbar (z-index > 1000)
- MUST be highest priority UI element
- MUST only render when `modalConfig.isOpen === true`
```

---

### Quickstart Guide

**File**: `specs/main/quickstart.md`

```markdown
# UI Critical Fixes - Developer Quickstart

## Prerequisites

- Node.js 18+ installed
- Project dependencies installed (`npm install`)
- Docusaurus dev server running (`npm start`)

## Testing the Fixes

### Fix 1: Mobile Menu Disabled

**Test Steps**:
1. Open browser dev tools (F12)
2. Toggle device toolbar (mobile emulation)
3. Resize viewport to 320px, 768px, 1024px
4. Verify hamburger menu never appears
5. Verify desktop navbar items remain visible

**Expected Result**: No hamburger toggle at any screen size.

---

### Fix 2: Navbar Above Chatbot

**Test Steps**:
1. Open chatbot by clicking floating button
2. Verify chatbot modal slides in below navbar
3. Click navbar links while chatbot is open
4. Verify navbar remains interactive (not covered)

**Expected Result**: Navbar always visible and clickable above chatbot.

---

### Fix 3: No Ghost Popups

**Test Steps**:
1. Load homepage (refresh browser)
2. Navigate between 5+ pages
3. Open/close chatbot multiple times
4. Monitor for unwanted popup appearances

**Expected Result**: Zero popups unless explicitly triggered by user click (delete button, etc.).

---

## Files Modified

1. `src/css/custom.css` - Added `.navbar__toggle { display: none !important; }`
2. `src/components/ChatWidget/styles.module.css` - Adjusted z-index values (if needed)
3. `src/components/ChatWidget/index.js` - Fixed ghost popup trigger logic

## Rollback

```bash
git checkout main -- src/css/custom.css src/components/ChatWidget/
npm start
```
```

---

### Agent Context Update

**Command**: `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude`

**New Technologies to Add** (if not already present):
- CSS Modules (component-scoped styling)
- Docusaurus v3 theming system
- React useState hooks (component state management)

**Context File**: `.claude/settings.local.json` or `CLAUDE.md`

**Additions**:
```markdown
## UI Customization Patterns

### Docusaurus CSS Overrides
- Global styles: `src/css/custom.css`
- Component-scoped: CSS Modules (`.module.css`)
- Override priority: `!important` for Docusaurus defaults
- Mobile menu class: `.navbar__toggle`

### Z-Index Hierarchy
- Navbar: 1000
- ChatWidget: 97-99
- ConfirmationModal: 9999-10000
- Page overlays: 10000+

### Component State Best Practices
- Modal visibility: Controlled via `isOpen` boolean state
- User action guards: Check event handlers before state updates
- Tooltip triggers: Event listener on text selection
```

## Phase 2: Task Generation

**Note**: Phase 2 is handled by `/sp.tasks` command (not part of `/sp.plan` output).

**Expected Tasks** (preview):
1. **Task 1.1**: Add CSS rule to hide `.navbar__toggle` in `custom.css`
2. **Task 1.2**: Test mobile menu disabled at 4 breakpoints
3. **Task 2.1**: Verify navbar z-index (1000) in `custom.css`
4. **Task 2.2**: Adjust ChatWidget z-index if needed in `styles.module.css`
5. **Task 2.3**: Test navbar visibility with chatbot open
6. **Task 3.1**: Investigate ghost popup trigger in `ChatWidget/index.js`
7. **Task 3.2**: Add conditional guard to prevent auto-trigger
8. **Task 3.3**: Test popup only appears on explicit user action

## Success Metrics

### Fix 1: Mobile Menu Disabled
- ✅ Hamburger toggle hidden at 320px, 768px, 1024px, 1920px
- ✅ Desktop navbar items visible at all breakpoints
- ✅ No console errors or warnings

### Fix 2: Navbar/Chatbot Hierarchy
- ✅ Chatbot never covers navbar (visual inspection)
- ✅ Navbar links clickable when chatbot open (functional test)
- ✅ Z-index hierarchy maintained (DevTools verification)

### Fix 3: Ghost Popup Eliminated
- ✅ Zero unwanted popups during 10+ page navigations
- ✅ Popup only appears on delete button click (manual trigger)
- ✅ No popup on page load, route change, or chatbot open

## Risks & Mitigations

### Risk 1: CSS Specificity Conflicts

**Risk**: `!important` on `.navbar__toggle` may conflict with future Docusaurus updates or theme changes.

**Mitigation**:
- Document CSS override in `CLAUDE.md` (codebase guide)
- Add comment in `custom.css` explaining override purpose
- Test after Docusaurus version upgrades

### Risk 2: Stacking Context Issues

**Risk**: Changing z-index on ChatWidget may break modal backdrop click-outside-to-close functionality.

**Mitigation**:
- Test modal backdrop interaction after z-index changes
- Verify `position: fixed` on modal elements
- Keep modalBackdrop z-index < chatModal z-index

### Risk 3: Ghost Popup Reappears

**Risk**: Fixing one trigger may reveal another auto-trigger source.

**Mitigation**:
- Comprehensive state flow analysis before fix
- Add unit tests for modal trigger conditions (if time permits)
- Document all `setModalConfig({ isOpen: true })` call sites

## Next Steps

1. **Run `/sp.tasks`** to generate dependency-ordered task list from this plan
2. **Execute Phase 0 Research** (create `research.md` with findings)
3. **Execute Phase 1 Design** (create `data-model.md`, `contracts/css-classes.md`, `quickstart.md`)
4. **Re-validate Constitution Check** after design artifacts complete
5. **Proceed to implementation** via `/sp.implement` command

---

**Plan Status**: ✅ COMPLETE - Ready for `/sp.tasks` command
**Constitution Compliance**: ✅ ALL GATES PASSED
**Estimated Implementation Time**: 2-3 hours (manual testing included)
