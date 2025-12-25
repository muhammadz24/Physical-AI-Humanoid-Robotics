# Feature Specification: UI Critical Fixes

**Feature Branch**: `main`
**Created**: 2025-12-25
**Status**: Draft
**Input**: User description: "Force disable mobile menu, fix z-index hierarchy for navbar/chatbot, eliminate ghost popup state bug"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Desktop Navigation Always Visible (Priority: P1)

As a user viewing the textbook on any screen size, I expect to always see the desktop navigation menu, never the mobile hamburger menu, regardless of viewport width or browser dev tools being open.

**Why this priority**: The mobile menu creates UX friction and was not intended for this application. Desktop navigation should be the only interface.

**Independent Test**: Can be fully tested by resizing browser window to mobile breakpoints and verifying the hamburger menu never appears while desktop menu items remain visible.

**Acceptance Scenarios**:

1. **Given** browser viewport at 320px width, **When** page loads, **Then** desktop navbar items are visible and hamburger menu is hidden
2. **Given** browser with F12 console open, **When** viewport shrinks due to console panel, **Then** desktop navbar remains visible and hamburger menu stays hidden
3. **Given** any screen size from 320px to 4K, **When** user navigates the site, **Then** only desktop navigation items are rendered

---

### User Story 2 - Chatbot Positioned Below Navbar (Priority: P1)

As a user interacting with the RAG chatbot, I expect the chatbot widget to always appear below the top navigation bar without overlapping or covering navbar elements.

**Why this priority**: Navbar overlap breaks core navigation functionality and creates a poor user experience by hiding critical UI controls.

**Independent Test**: Can be fully tested by opening the chatbot widget and verifying the navbar remains fully visible and interactive above the chatbot container.

**Acceptance Scenarios**:

1. **Given** chatbot is closed, **When** user clicks to open chatbot, **Then** chatbot slides in below the navbar without covering any navbar elements
2. **Given** chatbot is open, **When** user scrolls the page, **Then** navbar and chatbot maintain proper stacking order
3. **Given** chatbot is open, **When** user clicks navbar links, **Then** navbar remains fully interactive and visible above chatbot

---

### User Story 3 - No Unwanted Popup Triggers (Priority: P2)

As a user browsing the textbook, I should not see any unwanted popup notifications repeatedly appearing without user action.

**Why this priority**: Ghost popups disrupt reading flow and create a frustrating user experience. This is a critical UX bug.

**Independent Test**: Can be fully tested by navigating through multiple pages and verifying no popup appears unless explicitly triggered by user action.

**Acceptance Scenarios**:

1. **Given** user loads any page, **When** page renders, **Then** no popup appears automatically
2. **Given** user navigates between pages, **When** route changes, **Then** no popup is triggered
3. **Given** user interacts with chatbot, **When** chatbot state changes, **Then** popup only appears if explicitly triggered by user action (e.g., explicit button click)

---

### Edge Cases

- What happens when user has custom browser zoom levels (150%, 200%)?
- How does the navbar behave on ultra-wide monitors (3440px+)?
- What if chatbot is open and user rapidly resizes window?
- How does the system handle popup state if user refreshes page while chatbot is open?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST permanently hide the mobile hamburger menu toggle button (`.navbar__toggle`) via CSS `display: none !important`
- **FR-002**: System MUST maintain desktop navbar items visibility at all viewport widths (320px - 4K+)
- **FR-003**: System MUST set chatbot container z-index lower than navbar z-index to prevent overlap
- **FR-004**: System MUST position navbar with z-index sufficient to stay above all page content including chatbot
- **FR-005**: System MUST identify and remove/disable the state logic triggering unwanted popup notifications
- **FR-006**: System MUST ensure popup notifications only trigger on explicit user actions (button clicks, form submissions)

### Key Entities

- **Navbar Component**: Top navigation bar rendered by Docusaurus, contains site title and navigation links
- **ChatWidget Component**: RAG chatbot interface component, toggleable overlay/panel
- **Popup/Notification Component**: Modal or toast notification component being triggered unexpectedly

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Mobile hamburger menu is never visible at any screen size (verified via manual testing on 320px, 768px, 1024px, 1920px viewports)
- **SC-002**: Desktop navbar items remain visible and functional at all tested breakpoints
- **SC-003**: Chatbot widget never overlaps navbar elements (verified via visual inspection and z-index hierarchy)
- **SC-004**: Navbar remains interactive when chatbot is open (all navbar links remain clickable)
- **SC-005**: Zero unwanted popup appearances during normal browsing flow (verified by navigating 10+ pages without seeing popup)
- **SC-006**: Popup only appears when triggered by explicit user action (100% of popup appearances can be traced to user click/interaction)

## Assumptions

1. **Docusaurus Version**: Assumes Docusaurus v3.x standard navbar structure with `.navbar__toggle` class for mobile menu
2. **CSS Override Authority**: Assumes custom CSS in `src/css/custom.css` can override Docusaurus default styles with `!important`
3. **ChatWidget Implementation**: Assumes ChatWidget is a custom React component with controllable z-index via inline styles or CSS class
4. **Popup Source**: Assumes popup is triggered by React component state logic, not third-party script or browser extension
5. **Browser Support**: Assumes modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+) with standard CSS z-index support

## Out of Scope

- Redesigning navbar layout or adding new navigation features
- Adding responsive design improvements beyond hiding mobile menu
- Implementing new chatbot features or functionality
- Adding popup/notification features or notification center
- Performance optimization of navbar or chatbot rendering
- Accessibility improvements (ARIA labels, keyboard navigation) unless directly related to the 3 fixes

## Dependencies

- Access to `src/css/custom.css` for CSS overrides
- Access to ChatWidget component source code for z-index modification
- Access to component that triggers popup (requires code investigation to identify)
- Docusaurus build system for CSS compilation and hot reload during development
