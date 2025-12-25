# UI Critical Fixes - Developer Quickstart

**Feature**: UI Critical Fixes (Mobile Menu, Z-Index, Ghost Popup)
**Date**: 2025-12-25
**Estimated Time**: 30 minutes (implementation) + 15 minutes (testing)

## Overview

This guide walks through implementing and testing the 3 UI fixes:

1. Force disable mobile hamburger menu
2. Verify navbar z-index hierarchy (already correct)
3. Investigate ghost popup (clarification needed)

---

## Prerequisites

### Required Software

- **Node.js**: 18.0.0 or higher
- **npm**: 9.0.0 or higher
- **Git**: Any recent version
- **Modern Browser**: Chrome 90+, Firefox 88+, Safari 14+, or Edge 90+

### Verify Installation

```bash
node --version   # Should output v18.x.x or higher
npm --version    # Should output 9.x.x or higher
git --version    # Any version OK
```

### Install Dependencies

```bash
# From project root
npm install

# Expected output: 1000+ packages installed (Docusaurus + dependencies)
```

---

## Development Setup

### Start Docusaurus Dev Server

```bash
# From project root
npm start

# Expected output:
# [INFO] Starting the development server...
# [SUCCESS] Docusaurus website is running at: http://localhost:3000/
```

**Dev Server Features**:
- Hot reload: CSS changes appear instantly
- React Fast Refresh: Component changes reload automatically
- Error overlay: Build errors shown in browser

### Open Browser Dev Tools

1. Navigate to `http://localhost:3000`
2. Press `F12` (or `Cmd+Option+I` on Mac)
3. Open "Elements" tab for CSS inspection
4. Open "Console" tab for error monitoring

---

## Implementation Guide

### Fix 1: Disable Mobile Menu

#### Step 1: Locate Custom CSS File

**File**: `src/css/custom.css`

#### Step 2: Find Navbar Styling Section

```css
/* Existing navbar styles (around line 87-101) */
.navbar {
  background: var(--ifm-navbar-background-color);
  backdrop-filter: blur(10px);
  -webkit-backdrop-filter: blur(10px);
  border-bottom: 1px solid
    color-mix(
      in srgb,
      var(--ifm-color-primary) 30%,
      var(--ifm-background-color) 70%
    );
  z-index: 1000;
}
```

#### Step 3: Add Mobile Menu Override

**Insert after line 101** (after `.navbar` closing brace):

```css
/* ========================================
   Force Disable Mobile Menu (FR-001)
   ======================================== */

/* Hide hamburger toggle at ALL breakpoints to maintain desktop nav visibility.
   Overrides Docusaurus default responsive behavior (@media max-width: 996px).
   Related FR: FR-002 (Desktop navbar items remain visible) */
.navbar__toggle {
  display: none !important;
  visibility: hidden !important;
}
```

**Expected Result**: File should hot-reload; no build errors in console.

#### Step 4: Verify Change in Browser

1. **Without resizing browser**: Check console for hot-reload message:
   ```
   [Fast Refresh] Hot update applied ✓
   ```

2. **Refresh if needed**: Press `Ctrl+R` (or `Cmd+R` on Mac)

3. **Inspect navbar**: Right-click navbar → "Inspect Element"
   - Look for `.navbar__toggle` element
   - Verify `display: none` is applied
   - Element should show `0px` height/width

---

### Fix 2: Verify Z-Index Hierarchy

**Status**: ✅ No code changes needed (already correct)

#### Step 1: Verify Navbar Z-Index

**File**: `src/css/custom.css` (line 94)

```css
.navbar {
  /* ... other styles ... */
  z-index: 1000;  /* ✅ Correct value */
}
```

#### Step 2: Verify ChatWidget Z-Index

**File**: `src/components/ChatWidget/styles.module.css`

**Current Values**:
```css
.chatButton {
  z-index: 99;  /* ✅ Below navbar (1000) */
}

.chatModal {
  z-index: 98;  /* ✅ Below button (99) */
}

.modalBackdrop {
  z-index: 97;  /* ✅ Below modal (98) */
}
```

**Hierarchy**: `97 < 98 < 99 < 1000` ✅ CORRECT

#### Step 3: Optional Comment Fix

**File**: `src/components/ChatWidget/styles.module.css` (line 408)

**Current Comment**:
```css
/* Z-index 99: Below navbar (100) to prevent overlap with hamburger menu */
```

**Corrected Comment**:
```css
/* Z-index 99: Below navbar (1000) to prevent overlap with navbar elements */
```

**Rationale**: Navbar is actually z-index 1000, not 100. Update for accuracy.

---

### Fix 3: Ghost Popup Investigation

**Status**: ⚠️ Requires user clarification

#### Step 1: Check for Auto-Triggers (Already Completed)

**Research Finding**: No auto-trigger found in code analysis.

**Verified**:
- ✅ `modalConfig.isOpen` defaults to `false`
- ✅ All `setModalConfig({ isOpen: true })` calls are within `onClick` handlers
- ✅ No `useEffect` hooks trigger modal on mount or route change

#### Step 2: Request User Clarification

**Before implementing fix**, ask user:

1. **Popup Description**:
   - Is it the cyan "Ask AI" tooltip (appears on text selection)?
   - Is it the red "Delete" confirmation modal?
   - Is it a browser-native popup (password manager, notification permission)?

2. **Reproduction Steps**:
   - Which page URL does it appear on?
   - Does it appear on every page load or intermittently?
   - Does it appear when chatbot is closed or open?

3. **Visual Identification**:
   - Screenshot or description of popup content
   - Popup background color (red, cyan, white, etc.)
   - Button labels on popup

#### Step 3: Temporary Logging (For Debugging)

If user cannot provide clear description, add temporary logging:

**File**: `src/components/ChatWidget/index.js`

**Add after line 33**:
```javascript
// TEMPORARY DEBUG: Log modal state changes
useEffect(() => {
  if (modalConfig.isOpen) {
    console.log('[DEBUG] ConfirmationModal opened:', {
      title: modalConfig.title,
      message: modalConfig.message,
      timestamp: new Date().toISOString(),
      stack: new Error().stack, // Call stack trace
    });
  }
}, [modalConfig.isOpen]);
```

**Usage**:
1. Ask user to navigate site with console open
2. When popup appears, check console for `[DEBUG] ConfirmationModal opened:` log
3. Analyze call stack to identify trigger source

**Remove after debugging** (temporary code only).

---

## Testing Guide

### Test 1: Mobile Menu Disabled

#### Test Steps

1. **Open browser dev tools** (F12)

2. **Toggle device toolbar**:
   - Chrome: Click device icon (Ctrl+Shift+M)
   - Firefox: Click responsive design mode (Ctrl+Shift+M)

3. **Test 320px viewport**:
   ```
   Viewport: 320 x 568 (iPhone SE)
   Expected: Hamburger menu HIDDEN, desktop items VISIBLE
   ```
   - Look for `.navbar__toggle` button
   - Should NOT be visible
   - Desktop navbar links should be visible

4. **Test 768px viewport**:
   ```
   Viewport: 768 x 1024 (iPad)
   Expected: Hamburger menu HIDDEN, desktop items VISIBLE
   ```

5. **Test 1024px viewport**:
   ```
   Viewport: 1024 x 768 (Tablet landscape)
   Expected: Hamburger menu HIDDEN, desktop items VISIBLE
   ```

6. **Test 1920px viewport**:
   ```
   Viewport: 1920 x 1080 (Desktop)
   Expected: Hamburger menu HIDDEN, desktop items VISIBLE
   ```

7. **Test with F12 console**:
   - Open full browser window (1920x1080)
   - Press F12 to open console (docks bottom)
   - Viewport shrinks to ~1920x600 (console takes 480px)
   - Expected: Desktop navbar still visible, no hamburger menu

#### Pass Criteria

- ✅ Hamburger toggle not visible at any tested viewport
- ✅ Desktop navbar items remain visible and clickable
- ✅ No console errors or warnings
- ✅ Page layout does not break

#### Troubleshooting

**Issue**: Hamburger menu still visible

**Solution**:
1. Check CSS file saved (`Ctrl+S` in editor)
2. Verify dev server hot-reloaded (check console for "Hot update applied")
3. Hard refresh browser (`Ctrl+Shift+R`)
4. Inspect element and check computed styles for `.navbar__toggle`
   - Should show `display: none` with `!important` flag

---

### Test 2: Navbar Above Chatbot

#### Test Steps

1. **Navigate to homepage** (`http://localhost:3000`)

2. **Open chatbot**:
   - Click floating button (bottom-right, cyan robot icon)
   - Expected: Chatbot modal slides in from right

3. **Verify navbar visibility**:
   - Navbar should remain visible at top of page
   - Navbar should NOT be covered by chatbot modal
   - Visual inspection: No overlap

4. **Test navbar interaction**:
   - Click "Introduction" link in navbar
   - Expected: Page navigates, chatbot closes or remains open
   - Link should be clickable (not blocked by chatbot)

5. **Test with browser zoom**:
   - Zoom to 150% (`Ctrl++`)
   - Zoom to 200%
   - Expected: Navbar still above chatbot at all zoom levels

6. **Inspect z-index**:
   - Right-click navbar → Inspect
   - Check computed styles: `z-index: 1000`
   - Right-click chatbot modal → Inspect
   - Check computed styles: `z-index: 98`
   - Verify: 1000 > 98 ✅

#### Pass Criteria

- ✅ Chatbot never covers navbar elements
- ✅ Navbar links remain clickable when chatbot open
- ✅ Z-index hierarchy verified in DevTools (navbar: 1000, chatbot: 98)
- ✅ No visual overlap at any zoom level or viewport size

#### Troubleshooting

**Issue**: Chatbot covers navbar

**Solution**:
1. Verify navbar z-index in `custom.css` (should be 1000)
2. Verify chatbot z-index in `styles.module.css` (should be 97-99)
3. Check for stacking context issues:
   - Navbar should have `position: sticky`
   - Chatbot should have `position: fixed`
4. Inspect parent elements for `z-index` or `transform` that create new stacking context

---

### Test 3: No Ghost Popups

#### Test Steps

1. **Fresh browser session**:
   - Close all browser tabs
   - Clear sessionStorage: `sessionStorage.clear()` in console
   - Navigate to `http://localhost:3000`

2. **Page load test**:
   - Load homepage
   - Wait 10 seconds
   - Expected: NO popups appear automatically

3. **Navigation test**:
   - Navigate to "Introduction" → "Chapter 1" → "Chapter 2" → Homepage
   - Wait 5 seconds on each page
   - Expected: NO popups appear during navigation

4. **Chatbot interaction test**:
   - Open chatbot (click floating button)
   - Type message: "What is ROS?"
   - Send message
   - Wait for response
   - Close chatbot (click X button)
   - Expected: NO unwanted popups (confirmation modal should NOT appear)

5. **Text selection test**:
   - Select text on page (highlight paragraph)
   - Expected: "Ask AI" tooltip appears (THIS IS INTENTIONAL FEATURE)
   - Click elsewhere to dismiss tooltip
   - Expected: Tooltip disappears

6. **Delete action test**:
   - Open chatbot
   - Hover over message
   - Click delete icon (trash can)
   - Expected: RED confirmation modal appears (THIS IS CORRECT BEHAVIOR)
   - Click "Cancel"
   - Expected: Modal closes, message NOT deleted

#### Pass Criteria

- ✅ Zero unwanted popups during 10+ page navigations
- ✅ Popup only appears when triggered by explicit user action:
  - Text selection → "Ask AI" tooltip (feature, not bug)
  - Delete button click → Red confirmation modal (correct behavior)
- ✅ No popup on page load, route change, or chatbot open/close

#### Troubleshooting

**Issue**: Popup appears unexpectedly

**Solution**:
1. Open browser console (F12)
2. Check for `[DEBUG] ConfirmationModal opened:` log (if temporary logging added)
3. Analyze call stack to identify trigger source
4. Verify popup content:
   - Cyan tooltip → Select-to-ask feature (intentional)
   - Red modal → Delete confirmation (should only appear on button click)
   - Other → Investigate component rendering logic

---

## Verification Checklist

Before marking task complete:

### Code Changes

- [ ] `src/css/custom.css` updated with `.navbar__toggle` override
- [ ] Optional: `styles.module.css:408` comment corrected
- [ ] Git diff reviewed (only expected files modified)
- [ ] No temporary debug code left in codebase

### Functional Tests

- [ ] Mobile menu hidden at 320px
- [ ] Mobile menu hidden at 768px
- [ ] Mobile menu hidden at 1024px
- [ ] Mobile menu hidden at 1920px
- [ ] Mobile menu hidden with F12 console open
- [ ] Desktop navbar items visible at all breakpoints
- [ ] Navbar above chatbot (visual inspection)
- [ ] Navbar clickable when chatbot open (functional test)
- [ ] Z-index hierarchy verified (DevTools)
- [ ] Zero unwanted popups during 10+ page navigations
- [ ] Popup only on explicit user action (delete button)

### Build & Deploy

- [ ] No console errors in dev server
- [ ] No build warnings
- [ ] Production build succeeds: `npm run build`
- [ ] Production preview works: `npm run serve`

---

## Rollback Procedure

If issues arise after deployment:

### Quick Rollback (CSS Only)

**File**: `src/css/custom.css`

**Remove or comment out**:
```css
/* .navbar__toggle {
  display: none !important;
  visibility: hidden !important;
} */
```

**Save file** → Dev server hot-reloads → Mobile menu re-appears.

### Full Git Rollback

```bash
# Undo all changes to UI fix files
git checkout main -- src/css/custom.css src/components/ChatWidget/

# Restart dev server
npm start
```

---

## Performance Verification

### Expected Impact

- **Build Time**: No change (CSS-only)
- **Bundle Size**: +50 bytes (CSS rule)
- **Runtime Performance**: No change (CSS overhead negligible)
- **Lighthouse Score**: No change (layout shift avoided by hiding menu)

### Verify No Regressions

```bash
# Build production bundle
npm run build

# Check bundle size (should be ~same as before)
du -sh build/

# Run Lighthouse audit (optional)
# Chrome DevTools → Lighthouse tab → "Generate report"
# Performance score should remain >90
```

---

## Common Issues & Solutions

### Issue 1: CSS Not Applying

**Symptoms**: Hamburger menu still visible after adding CSS rule

**Solutions**:
1. Verify file saved: `Ctrl+S` in editor
2. Check dev server console for hot-reload confirmation
3. Hard refresh browser: `Ctrl+Shift+R`
4. Clear browser cache: `Ctrl+Shift+Delete`
5. Restart dev server: `Ctrl+C` then `npm start`

### Issue 2: Desktop Navbar Items Hidden

**Symptoms**: Both hamburger menu AND desktop links disappear

**Cause**: CSS selector too broad (e.g., `.navbar > *` instead of `.navbar__toggle`)

**Solution**:
1. Verify selector targets ONLY `.navbar__toggle`
2. Check for typos in class name
3. Inspect element to confirm correct class targeted

### Issue 3: Build Fails After Changes

**Symptoms**: `npm run build` exits with error

**Solutions**:
1. Check CSS syntax: Missing semicolons, unclosed braces
2. Verify no invalid CSS properties
3. Read error message for specific line number
4. Run `npm run clear` to clear Docusaurus cache
5. Retry build: `npm run build`

---

## Next Steps

After completing this quickstart:

1. **Run full test suite**: Complete all verification checklist items
2. **Document findings**: Update research.md with test results
3. **Create tasks.md**: Run `/sp.tasks` to generate implementation tasks
4. **Implement fixes**: Execute tasks in dependency order
5. **Create pull request**: Commit changes with descriptive message

**Commit Message Template**:
```
fix(ui): force disable mobile menu and verify z-index hierarchy

- Add CSS override to hide .navbar__toggle at all breakpoints (FR-001)
- Verify navbar z-index (1000) above ChatWidget (99) (FR-003, FR-004)
- Update styles.module.css comment for accuracy

Resolves: #[issue-number]
See: specs/main/plan.md
```

---

## Additional Resources

- **Docusaurus CSS Docs**: https://docusaurus.io/docs/styling-layout
- **CSS Z-Index Guide**: https://developer.mozilla.org/en-US/docs/Web/CSS/z-index
- **Infima CSS Framework**: https://infima.dev/ (Docusaurus default styles)
- **Project Constitution**: `.specify/memory/constitution.md` (Principle IV)

---

**Quickstart Status**: ✅ COMPLETE
**Estimated Completion Time**: 30 minutes (implementation) + 15 minutes (testing)
**Difficulty**: Easy (CSS-only changes, no complex logic)
