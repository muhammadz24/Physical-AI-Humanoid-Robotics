# CSS Class Interface Contracts

**Feature**: UI Critical Fixes
**Date**: 2025-12-25
**Type**: CSS Interface Specifications

## Overview

This document defines the CSS class contracts for the 3 UI fixes. Since no API endpoints are involved (frontend-only changes), this serves as the interface specification for CSS modifications.

---

## 1. Navbar Mobile Menu Toggle

### Contract Specification

**Class**: `.navbar__toggle`
**Source**: Docusaurus core (`@docusaurus/theme-classic`)
**Override Location**: `src/css/custom.css`
**Scope**: Global (affects all pages)

### Requirements

**FR-001 Implementation**:
```css
.navbar__toggle {
  display: none !important;
  visibility: hidden !important;
}
```

### Contract Rules

1. **MUST hide element**: `display: none` removes from layout flow
2. **MUST prevent visibility**: `visibility: hidden` as defense-in-depth
3. **MUST use !important**: Override Docusaurus default responsive styles
4. **MUST NOT use media queries**: Apply at all breakpoints (320px - 4K+)
5. **MUST NOT affect sibling elements**: `.navbar__items` remain visible

### Acceptance Criteria

- ✅ Hamburger toggle not visible at 320px viewport
- ✅ Hamburger toggle not visible at 768px viewport
- ✅ Hamburger toggle not visible at 1024px viewport
- ✅ Hamburger toggle not visible at 1920px viewport
- ✅ Desktop navbar items remain visible and functional
- ✅ No console errors or warnings
- ✅ Docusaurus build succeeds without CSS conflicts

### Specificity Analysis

**Docusaurus Default** (from `@docusaurus/theme-classic`):
```css
@media (max-width: 996px) {
  .navbar__toggle {
    display: flex;
  }
}
```

**Override Specificity**:
```
Default: .navbar__toggle (specificity: 0,1,0) + media query
Override: .navbar__toggle !important (specificity: 0,1,0 + !important flag)

Result: Override wins due to !important
```

### Rollback Procedure

```css
/* To re-enable mobile menu, comment out or remove: */
/*
.navbar__toggle {
  display: none !important;
  visibility: hidden !important;
}
*/
```

---

## 2. ChatWidget Z-Index Hierarchy

### Contract Specification

**Classes**:
- `.modalBackdrop` (ChatWidget backdrop)
- `.chatModal` (ChatWidget container)
- `.chatButton` (Floating button)
- `.selectToAskTooltip` (Text selection tooltip)

**Source**: `src/components/ChatWidget/styles.module.css`
**Scope**: Component-scoped (CSS Modules)

### Current Values (Verified Correct)

```css
/* styles.module.css:14 */
.chatButton {
  z-index: 99;
  position: fixed;
  /* ... other styles ... */
}

/* styles.module.css:38 */
.modalBackdrop {
  z-index: 97;
  position: fixed;
  /* ... other styles ... */
}

/* styles.module.css:56 */
.chatModal {
  z-index: 98;
  position: fixed;
  /* ... other styles ... */
}

/* styles.module.css:408 */
.selectToAskTooltip {
  z-index: 99; /* Below navbar (1000) to prevent overlap */
  position: absolute;
  /* ... other styles ... */
}
```

### Contract Rules

1. **MUST maintain hierarchy**:
   ```
   modalBackdrop (97) < chatModal (98) < chatButton (99) < navbar (1000)
   ```

2. **MUST use position: fixed or absolute**:
   - Required for z-index to take effect
   - Creates stacking context

3. **MUST NOT exceed navbar z-index**:
   - All ChatWidget z-index values < 1000
   - Ensures navbar always on top

4. **MUST NOT conflict with ConfirmationModal**:
   - ConfirmationModal at z-index 10000 (always top priority)

### Acceptance Criteria

- ✅ Navbar visible above ChatWidget when chatbot open
- ✅ Navbar links clickable when chatbot open
- ✅ Modal backdrop dims content below chatModal
- ✅ chatButton appears above backdrop when modal closed
- ✅ Tooltip does not cover navbar elements

### Documentation Fix Required

**Current Comment** (styles.module.css:408):
```css
/* Z-index 99: Below navbar (100) to prevent overlap with hamburger menu */
```

**Corrected Comment**:
```css
/* Z-index 99: Below navbar (1000) to prevent overlap with navbar elements */
```

**Rationale**: Navbar is actually z-index 1000 (custom.css:94), not 100. Update comment for accuracy.

---

## 3. Docusaurus Navbar Z-Index

### Contract Specification

**Class**: `.navbar`
**Source**: Docusaurus core with custom override
**Override Location**: `src/css/custom.css:94`
**Scope**: Global

### Current Implementation

```css
/* custom.css:87-101 */
.navbar {
  /* Glassmorphism effect */
  background: var(--ifm-navbar-background-color);
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px);
  border-bottom: 1px solid
    color-mix(
      in srgb,
      var(--ifm-color-primary) 30%,
      var(--ifm-background-color) 70%
    );
  z-index: 1000; /* Line 94 */
}
```

### Contract Rules

1. **MUST stay at z-index 1000**:
   - Standard navbar z-index convention
   - Above all page content (ChatWidget: 99)
   - Below critical modals (ConfirmationModal: 10000)

2. **MUST use position: sticky** (Docusaurus default):
   - Allows navbar to scroll with page until threshold
   - Creates stacking context for z-index

3. **MUST NOT be modified** for this feature:
   - Current value is correct
   - No changes required

### Acceptance Criteria

- ✅ Navbar at z-index 1000 (verified in custom.css:94)
- ✅ Navbar above ChatWidget (1000 > 99)
- ✅ Navbar below ConfirmationModal (1000 < 10000)
- ✅ Navbar sticky positioning functional

---

## 4. ConfirmationModal Z-Index

### Contract Specification

**Classes**:
- `.modalBackdrop` (ConfirmationModal backdrop)
- `.modalContainer` (ConfirmationModal container)

**Source**: `src/components/UI/modal.module.css`
**Scope**: Component-scoped (CSS Modules)

### Current Implementation

```css
/* modal.module.css:25 */
.modalBackdrop {
  z-index: 9999;
  position: fixed;
  /* ... full-screen backdrop styles ... */
}

/* modal.module.css:56 */
.modalContainer {
  z-index: 10000;
  position: fixed;
  /* ... centered modal styles ... */
}
```

### Contract Rules

1. **MUST be highest z-index in application**:
   - Critical user confirmations (delete actions)
   - Must appear above all UI including navbar

2. **MUST maintain backdrop < container**:
   ```
   modalBackdrop (9999) < modalContainer (10000)
   ```

3. **MUST use position: fixed**:
   - Full viewport coverage
   - Creates stacking context

4. **MUST NOT be modified** for this feature:
   - Current values correct
   - No changes required

### Acceptance Criteria

- ✅ ConfirmationModal appears above navbar (10000 > 1000)
- ✅ ConfirmationModal appears above ChatWidget (10000 > 99)
- ✅ Backdrop dims entire viewport including navbar
- ✅ Modal container centered and clickable

---

## Z-Index Hierarchy Summary

### Complete Stacking Order

```
Layer 10: ConfirmationModal Container (10000)
Layer 9:  ConfirmationModal Backdrop (9999)
         ┌─────────────────────────────────┐
         │  Critical User Confirmations    │
         └─────────────────────────────────┘

Layer 8:  [Reserved for future overlays]

Layer 7:  Docusaurus Navbar (1000)
         ┌─────────────────────────────────┐
         │  Global Navigation              │
         └─────────────────────────────────┘

Layer 6:  ChatWidget Button (99)
Layer 5:  Select-to-Ask Tooltip (99)
Layer 4:  ChatWidget Modal (98)
Layer 3:  ChatWidget Backdrop (97)
         ┌─────────────────────────────────┐
         │  Page Content & Chat Interface  │
         └─────────────────────────────────┘

Layer 2:  Page Content (default z-index: auto)
Layer 1:  Page Background (z-index: -1 or auto)
```

### Interaction Rules

| Interaction | Expected Behavior | Z-Index Enforcement |
|-------------|-------------------|---------------------|
| Click navbar while chatbot open | Navbar link fires | navbar (1000) > chatModal (98) |
| Delete chat message | ConfirmationModal appears | modalContainer (10000) > navbar (1000) |
| Select text with chatbot open | Tooltip hidden | `{tooltipVisible && !isOpen}` condition |
| Mobile resize to 320px | Navbar items visible, toggle hidden | `.navbar__toggle { display: none !important }` |

---

## Testing Contract

### Visual Testing Matrix

| Test Case | Viewport | Expected State | Validation Method |
|-----------|----------|----------------|-------------------|
| Mobile menu hidden | 320px | Hamburger toggle not visible | DevTools inspect `.navbar__toggle` |
| Mobile menu hidden | 768px | Hamburger toggle not visible | DevTools inspect `.navbar__toggle` |
| Mobile menu hidden | 1024px | Hamburger toggle not visible | DevTools inspect `.navbar__toggle` |
| Mobile menu hidden | 1920px | Hamburger toggle not visible | DevTools inspect `.navbar__toggle` |
| Navbar above chatbot | Any | Navbar links clickable | Click navbar link with chatbot open |
| ConfirmationModal top | Any | Modal above all elements | Trigger delete, inspect z-index stack |

### Automated Testing (Optional)

```javascript
// Example Playwright test for z-index hierarchy
test('navbar appears above chatbot', async ({ page }) => {
  await page.goto('/');
  await page.click('.chatButton'); // Open chatbot

  const navbarZIndex = await page.$eval('.navbar', el =>
    window.getComputedStyle(el).zIndex
  );
  const chatModalZIndex = await page.$eval('.chatModal', el =>
    window.getComputedStyle(el).zIndex
  );

  expect(parseInt(navbarZIndex)).toBeGreaterThan(parseInt(chatModalZIndex));
});
```

---

## Versioning & Compatibility

### Docusaurus Version Compatibility

**Current Version**: Docusaurus 3.x
**Tested Against**: `@docusaurus/core@^3.0.0`

**Upgrade Considerations**:
- Monitor `.navbar__toggle` class name changes in future Docusaurus releases
- Test mobile menu override after Docusaurus minor/major version upgrades
- Re-verify z-index hierarchy if Docusaurus updates navbar styles

### Browser Compatibility

**Supported Browsers**:
- Chrome 90+ ✅
- Firefox 88+ ✅
- Safari 14+ ✅
- Edge 90+ ✅

**CSS Features Used**:
- `z-index` (universal support)
- `!important` (universal support)
- `display: none` (universal support)
- `position: fixed/absolute/sticky` (IE11+)

---

## Change Log

| Date | Change | Rationale |
|------|--------|-----------|
| 2025-12-25 | Added `.navbar__toggle` override contract | FR-001: Force disable mobile menu |
| 2025-12-25 | Documented z-index hierarchy (no changes) | FR-003, FR-004: Verify navbar above chatbot |
| 2025-12-25 | Identified comment correction needed | styles.module.css:408 inaccurate comment |

---

**Contract Status**: ✅ COMPLETE
**Implementation Ready**: Yes
**Breaking Changes**: None
