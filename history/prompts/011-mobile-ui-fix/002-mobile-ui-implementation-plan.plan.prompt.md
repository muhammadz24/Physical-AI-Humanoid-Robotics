---
id: 002
title: Mobile UI Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: 011-mobile-ui-fix
branch: 011-mobile-ui-fix
user: system
command: /sp.plan
labels: ["planning", "architecture", "mobile-ui", "research"]
links:
  spec: specs/011-mobile-ui-fix/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/011-mobile-ui-fix/plan.md
  - specs/011-mobile-ui-fix/research.md
  - specs/011-mobile-ui-fix/quickstart.md
tests:
  - Constitution Check validation (PASSED)
  - Phase 0 research completion (Docusaurus best practices verified)
---

## Prompt

Execute /sp.plan command for Feature 011 (Mobile UI Fix).

CONTEXT:
- Feature 011 specification completed and validated
- Spec defines mobile UI issues: broken hamburger menu on Android, ChatWidget overlap
- Requirements: Fix z-index layering, add mobile media queries, verify hideOnScroll

USER REQUEST:
Follow SpecKit Plus workflow to create implementation plan (Constitution -> Spec -> Plan -> Task).

WORKFLOW STEPS:
1. Run setup-plan.ps1 to initialize plan.md
2. Load feature spec and constitution
3. Execute plan workflow (Phase 0: Research, Phase 1: Design, Phase 2: Task prep)
4. Create research.md resolving all unknowns
5. Create quickstart.md for testing guidance
6. Create PHR documenting planning process

## Response snapshot

Created comprehensive implementation plan for Mobile UI Fix (Feature 011).

KEY DELIVERABLES:
1. **plan.md**: Full implementation plan with technical context, constitution check, and design decisions
2. **research.md**: Research findings resolving all "NEEDS CLARIFICATION" items from technical context
3. **quickstart.md**: Manual testing guide for mobile UI verification

TECHNICAL ANALYSIS:
- Analyzed current ChatWidget implementation (styles.module.css)
- Identified root cause: selectTooltip z-index: 100 conflicts with Docusaurus navbar z-index: 100
- Verified existing mobile media queries are comprehensive (lines 362-392)
- Confirmed hideOnScroll is absent from docusaurus.config.js

CONSTITUTION CHECK RESULTS:
- Principle I (Simplicity-First): ✅ PASS (minimal, focused changes)
- Principle IV (Docusaurus Best Practices): ⚠️ VERIFY (hideOnScroll validated in research)
- Principle VI (Fast Build Cycles): ✅ PASS (CSS changes, no build impact)
- Performance Standards: ✅ PASS (improves mobile UX)

PHASE 0 RESEARCH FINDINGS:
1. **hideOnScroll Verification**:
   - Confirmed as official Docusaurus navbar feature
   - Source: https://docusaurus.io/docs/api/themes/configuration#navbar
   - Type: boolean (default: false)
   - Behavior: Auto-hides navbar on scroll down, reappears on scroll up

2. **Z-Index Root Cause**:
   - Docusaurus navbar: z-index 100 (Infima CSS framework)
   - ChatWidget selectTooltip: z-index 100 (CONFLICT IDENTIFIED)
   - floatingButton: z-index 99 ✅
   - chatModal: z-index 98 ✅
   - **Fix**: Change selectTooltip to z-index 99

3. **Mobile UX Best Practices**:
   - Touch targets: 44px × 44px minimum (WCAG 2.1 AAA)
   - Current button: 56px × 56px ✅ (compliant)
   - Safe margins: 16px from edges ✅ (already implemented)

4. **Overlap Analysis**:
   - Physical positions: Hamburger (top-left), ChatWidget (bottom-right) → No spatial overlap
   - Z-index conflict: selectTooltip z-index 100 = navbar z-index 100 → Visual overlap possible
   - When text near navbar is selected, tooltip can obscure hamburger menu

DESIGN DECISIONS:
1. **Z-Index Strategy**:
   - Navbar: 100 (Docusaurus default, unchanged)
   - selectTooltip: 99 (CHANGE from 100)
   - floatingButton: 99 (keep)
   - chatModal: 98 (keep)

2. **Configuration Changes**:
   - Add hideOnScroll: true to navbar in docusaurus.config.js

3. **CSS Changes**:
   - Update selectTooltip z-index from 100 to 99
   - Add CSS comment explaining z-index rationale
   - No changes needed to mobile media queries (already comprehensive)

FILES TO MODIFY:
1. src/components/ChatWidget/styles.module.css (line 297: z-index)
2. docusaurus.config.js (navbar.hideOnScroll: true)

SCOPE:
- 2 files modified
- ~5 lines of code changed
- No new components or files
- Desktop UI unchanged
- Mobile-only improvements

## Outcome

- ✅ Impact: Complete implementation plan with research-backed design decisions; ready for task generation
- 🧪 Tests: Constitution check passed, Docusaurus best practices verified, mobile UX standards validated
- 📁 Files: 3 files created (plan.md, research.md, quickstart.md)
- 🔁 Next prompts: /sp.tasks (generate testable task list from plan)
- 🧠 Reflection: Root cause identified through code analysis and research. Minimal surgical changes required (z-index fix + hideOnScroll config). Existing mobile media queries are already comprehensive. Plan emphasizes Constitution compliance (Docusaurus best practices, simplicity-first).

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan workflow executed smoothly. Research resolved all unknowns from technical context.
- Graders run and results (PASS/FAIL): Constitution Check - PASS (all gates), Research Validation - PASS (hideOnScroll documented, z-index analysis correct)
- Prompt variant (if applicable): N/A (standard /sp.plan workflow)
- Next experiment (smallest change to try): Proceed with /sp.tasks to generate granular, testable tasks. Consider ADR suggestion for z-index layering strategy if it's architecturally significant.
