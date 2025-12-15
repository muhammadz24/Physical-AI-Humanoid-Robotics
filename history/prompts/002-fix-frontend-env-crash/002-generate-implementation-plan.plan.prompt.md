---
id: 002
title: Generate Implementation Plan for Frontend Crash Fix
stage: plan
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5
feature: 002-fix-frontend-env-crash
branch: 002-fix-frontend-env-crash
user: system
command: /sp.plan
labels: ["bug-fix", "frontend", "typeof-check", "browser-safety", "implementation-plan"]
links:
  spec: specs/002-fix-frontend-env-crash/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/002-fix-frontend-env-crash/plan.md
 - specs/002-fix-frontend-env-crash/research.md
 - specs/002-fix-frontend-env-crash/data-model.md
 - specs/002-fix-frontend-env-crash/quickstart.md
tests:
 - None (planning phase - manual browser tests defined in quickstart.md)
---

## Prompt

Execute `/sp.plan` workflow to generate implementation plan for feature `002-fix-frontend-env-crash`.

**Spec Input**: `specs/002-fix-frontend-env-crash/spec.md`

**User Requirements** (from @CLAUDE_STRICT_MODE context):
- Replace unsafe `process.env` access with `typeof process !== 'undefined'` check
- Fix must be explicitly documented in plan
- No vibe coding - plan before implementation
- No git push - localhost testing only

**Planning Requirements**:
1. Document the exact code change (line 5 in ChatWidget/index.js)
2. Show current unsafe code vs. safe typeof check pattern
3. Explain why typeof check is needed (browser vs Node.js environments)
4. Verification steps for browser testing

**Output Goal**: Create `specs/002-fix-frontend-env-crash/plan.md` with full implementation architecture.

## Response snapshot

Generated complete implementation plan with Phase 0 (Research), Phase 1 (Design Artifacts), and detailed implementation steps.

### Key Artifacts Created:

1. **plan.md** (265 lines):
   - Technical Context: JavaScript ES6+, Docusaurus 3.x, React 18.x
   - Constitution Check: ✅ PASS on Principles IX, V, I
   - Phase 0 Research: 4 questions resolved (webpack behavior, browser crash cause, safe pattern, replacement compatibility)
   - Phase 1 Design: Single implementation step (replace line 5 in ChatWidget/index.js)
   - Verification Plan: 4 tests (console, page load, API connection, env var)
   - Quality Gates: Pre/post-implementation checklists

2. **research.md** (145 lines):
   - 3 research decisions documented with rationale
   - Decision 1: Use typeof check for safe process.env access
   - Decision 2: Maintain Docusaurus build-time substitution
   - Decision 3: Fallback to localhost:8000
   - Best practices: typeof operator, Docusaurus env vars, webpack DefinePlugin
   - Technical feasibility: VERY HIGH (1 line change, standard pattern)

3. **data-model.md** (123 lines):
   - API_BASE_URL constant specification
   - Resolution logic: typeof check → env var OR fallback
   - State transitions: Constant resolution lifecycle
   - Example scenarios: Local dev, production build, Node.js dev server
   - Migration path from unsafe to safe access

4. **quickstart.md** (186 lines):
   - Quick verification (30 seconds): Start backend, frontend, verify no crash
   - Detailed testing: 4 test scenarios with expected outcomes
   - Test 1: Browser console verification (no ReferenceError)
   - Test 2: Page load test (<1 second, 100% render)
   - Test 3: API connection test (correct URL in Network tab)
   - Test 4: Environment variable test (webpack replacement still works)
   - Troubleshooting guide for common issues

### Implementation Summary:

**File**: src/components/ChatWidget/index.js
**Line**: 5

**Current (Unsafe)**:
```javascript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```
**Problem**: Throws `ReferenceError: process is not defined` in browser

**After (Safe)**:
```javascript
const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && process.env.REACT_APP_API_URL)
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000';
```
**Solution**: typeof check prevents ReferenceError, provides fallback

### Research Findings:

**Q1**: How does Docusaurus/webpack handle REACT_APP_* variables?
**A**: webpack DefinePlugin performs static replacement at build time, searching for exact pattern matches

**Q2**: Why does direct process.env crash in browser?
**A**: `process` is Node.js-only global, doesn't exist in browser runtime

**Q3**: What's the safe pattern?
**A**: `typeof process !== 'undefined'` check before accessing properties (typeof doesn't throw ReferenceError)

**Q4**: Does typeof check break webpack replacement?
**A**: No - DefinePlugin runs before minification, typeof check evaluated after replacement

### Verification Plan:

1. **Browser Console**: Open localhost:3000, check F12 console for NO "process is not defined" errors
2. **Page Load**: Verify page renders in <1 second, chat button appears
3. **API Connection**: Network tab shows fetch to localhost:8000/api/chat (not undefined)
4. **Env Var Test**: Build with REACT_APP_API_URL set, verify webpack replacement works

### Implementation Impact:

- **Scope**: 1 file (src/components/ChatWidget/index.js), 1 line changed
- **Risk**: VERY LOW - standard JavaScript pattern, well-tested
- **Dependencies**: None new (native typeof operator)
- **Testing**: Manual browser verification
- **Rollback**: Trivial (revert 1 line)

## Outcome

- ✅ Impact: Complete implementation plan with 1 implementation step, 4 verification tests, 3 design artifacts
- 🧪 Tests: Manual browser tests defined (console verification, page load, API connection, env var)
- 📁 Files: specs/002-fix-frontend-env-crash/{plan.md, research.md, data-model.md, quickstart.md}
- 🔁 Next prompts: `/sp.tasks` to generate task list, then implement the 1-line fix
- 🧠 Reflection: This is a critical but minimal bug fix. The plan explicitly documents the exact code change required (typeof check pattern) and provides comprehensive verification steps for browser testing. Phase 0 research confirmed the pattern is standard in React/Docusaurus ecosystem and won't break webpack's static replacement.

## Evaluation notes (flywheel)

- Failure modes observed: Direct process.env access assumes Node.js environment. Webpack DefinePlugin doesn't always succeed at static replacement (complex expressions). Browser crash prevents all chat functionality.
- Graders run and results (PASS/FAIL): PASS - All constitution checks passed (Principles IX, V, I). Implementation step clearly documents current vs. after code. Verification plan has 4 specific browser tests with expected outcomes.
- Prompt variant (if applicable): Standard `/sp.plan` workflow for bug fix with explicit code change documentation
- Next experiment (smallest change to try): Implement the single-line typeof check fix, restart frontend dev server, test in browser console for NO "process is not defined" errors
