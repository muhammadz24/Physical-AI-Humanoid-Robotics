# Feature Specification: Fix Frontend Environment Variable Crash

**Feature Branch**: `002-fix-frontend-env-crash`
**Created**: 2025-12-14
**Status**: Draft
**Input**: User description: "Fix frontend environment variable crash when accessing process.env in browser"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Browser Loads Chat Widget Without Crash (Priority: P1)

Users visiting the Docusaurus site can see and interact with the chat widget without encountering JavaScript runtime errors or white screen crashes.

**Why this priority**: Critical bug fix - without this, the entire chat widget feature is broken and users cannot access the RAG chatbot functionality. This is a blocker for all chat widget usage.

**Independent Test**: Open http://localhost:3000 in a browser, verify the page loads completely, chat button appears in bottom-right corner, and browser console (F12) shows no "ReferenceError: process is not defined" errors.

**Acceptance Scenarios**:

1. **Given** a fresh browser session, **When** user navigates to http://localhost:3000, **Then** the page loads successfully with no JavaScript errors in console
2. **Given** the page has loaded, **When** user opens browser DevTools console (F12), **Then** there are no "process is not defined" or "Cannot read property 'env' of undefined" errors
3. **Given** the page has loaded, **When** user clicks the floating chat button, **Then** the chat window opens without errors

---

### User Story 2 - Chat Widget Connects to Correct API (Priority: P2)

The chat widget automatically connects to the correct backend API URL based on the deployment environment (localhost for local development, production URL for deployed environments) without manual code changes.

**Why this priority**: Ensures the environment-driven configuration from feature 001-cors-env-config works correctly in the browser runtime without crashes.

**Independent Test**: Start backend and frontend locally, send a test message in chat widget, verify Network tab shows fetch request to http://localhost:8000/api/chat (not undefined or crash).

**Acceptance Scenarios**:

1. **Given** backend running on localhost:8000 and frontend on localhost:3000, **When** user sends a chat message, **Then** fetch request goes to http://localhost:8000/api/chat
2. **Given** REACT_APP_API_URL environment variable is set to a custom URL, **When** frontend is built, **Then** fetch requests use the custom URL instead of localhost
3. **Given** REACT_APP_API_URL is not set (undefined), **When** user sends a chat message, **Then** fetch falls back to http://localhost:8000/api/chat

---

### Edge Cases

- What happens when `process` is undefined in the browser? (ReferenceError crash - needs safe access check)
- How does the system handle missing `process.env.REACT_APP_API_URL`? (Should fall back to localhost:8000)
- What if `process.env` exists but is null or empty? (Should fall back gracefully)
- What if the developer forgets to rebuild the frontend after changing environment variables? (Build-time substitution, documented in spec 001)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: Chat widget component MUST check if `process` is defined before accessing `process.env`
- **FR-002**: API URL resolution MUST use safe property access to prevent runtime crashes when `process.env` is unavailable
- **FR-003**: API URL MUST fall back to `'http://localhost:8000'` when `REACT_APP_API_URL` environment variable is undefined or unavailable
- **FR-004**: Chat widget MUST render successfully in browser environments where `process` object does not exist
- **FR-005**: The fix MUST maintain compatibility with Docusaurus build-time environment variable substitution pattern
- **FR-006**: API URL constant MUST be defined before component initialization to prevent undefined errors in fetch calls

### Key Entities *(include if feature involves data)*

- **API_BASE_URL Constant**: String value representing the backend API base URL, resolved at build time from `REACT_APP_API_URL` environment variable or defaulting to localhost

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Frontend loads without JavaScript runtime errors in browser console (<1 second load time)
- **SC-002**: Chat widget renders successfully on first page load in 100% of browser sessions (no white screen)
- **SC-003**: API requests use correct URL (localhost in local development, production URL when REACT_APP_API_URL is set)
- **SC-004**: Zero "process is not defined" errors appear in browser console during normal usage

## Context & Background

### Problem Statement

Feature 001-cors-env-config introduced environment variable access via `process.env.REACT_APP_API_URL` in the ChatWidget component. However, `process` is a Node.js global object that does not exist in browser runtime environments. When Docusaurus builds the application, it uses webpack to perform static replacement of `process.env.REACT_APP_*` variables at build time. If the webpack replacement fails or the code structure prevents proper static analysis, the browser encounters a runtime `ReferenceError: process is not defined`, causing the chat widget to crash and display a white screen.

### Current Behavior

```javascript
// src/components/ChatWidget/index.js (current - causes crash)
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

When webpack cannot statically analyze and replace this pattern, the browser tries to evaluate `process.env.REACT_APP_API_URL` at runtime, resulting in:
```
Uncaught ReferenceError: process is not defined
```

### Desired Behavior

The code must use safe environment variable access that works both with webpack static replacement AND browser runtime:

```javascript
// Desired: Safe access with typeof check
const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && process.env.REACT_APP_API_URL)
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000';
```

This pattern:
1. Checks if `process` is defined using `typeof` (safe in all environments)
2. Checks if `process` itself is truthy
3. Checks if `process.env` exists
4. Only then accesses `process.env.REACT_APP_API_URL`
5. Falls back to localhost if any check fails

## Scope

### In Scope

- Add safe `typeof process !== 'undefined'` check before accessing `process.env` in ChatWidget component
- Ensure API_BASE_URL constant is properly defined with fallback logic
- Verify chat widget renders successfully in browser without runtime errors
- Test that environment variable substitution still works after fix (both with and without REACT_APP_API_URL set)

### Out of Scope

- Changes to backend CORS configuration (already completed in feature 001)
- Changes to .env.example files (already completed in feature 001)
- Build process modifications or webpack configuration changes
- Server-side rendering (Docusaurus uses static site generation)
- Alternative environment variable patterns or configuration libraries

## Assumptions

1. Docusaurus webpack configuration performs build-time environment variable substitution for `REACT_APP_*` variables
2. If static replacement fails, the safe typeof check will catch the error and use the fallback
3. Local development defaults to http://localhost:8000 (backend address)
4. The fix must be backward compatible with feature 001-cors-env-config's environment variable pattern

## Dependencies

- **Feature 001-cors-env-config**: This fix depends on the environment variable pattern introduced in feature 001. The CORS configuration and .env.example files must be in place.
- **Docusaurus 3.x**: Build-time environment variable substitution behavior
- **ChatWidget Component**: src/components/ChatWidget/index.js must exist and use the API_BASE_URL constant

## References

- **Parent Feature**: 001-cors-env-config (introduced REACT_APP_API_URL pattern)
- **Affected File**: src/components/ChatWidget/index.js (line 5 and line 109)
- **Related Principles**: Constitution Principle IX (Zero-Edit Deployment Configuration)
- **Docusaurus Env Vars**: https://docusaurus.io/docs/deployment#using-environment-variables
- **Webpack DefinePlugin**: Used by Docusaurus for env var substitution at build time
