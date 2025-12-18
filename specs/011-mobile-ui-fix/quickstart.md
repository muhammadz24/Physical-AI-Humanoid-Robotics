# Testing Quickstart: Mobile UI Fix

**Feature**: 011-mobile-ui-fix
**Purpose**: Guide for manual testing of mobile UI fixes on Android and other mobile platforms

## Prerequisites

- Node.js and npm installed
- Repository cloned and dependencies installed (`npm install`)
- Access to Android device OR Chrome DevTools for mobile emulation
- Test on multiple screen sizes (320px, 375px, 414px, 768px)

---

## Local Development Setup

### 1. Start Development Server

```bash
npm run start
```

Wait for Docusaurus to build and open at `http://localhost:3000`.

### 2. Open Chrome DevTools Mobile Emulation

1. Press `F12` or `Ctrl+Shift+I` (Windows) / `Cmd+Option+I` (Mac)
2. Click the device toggle icon or press `Ctrl+Shift+M` / `Cmd+Shift+M`
3. Select device presets or set custom dimensions

---

## Test Cases

### Test 1: Z-Index Layering Verification

**Objective**: Ensure navbar (hamburger menu) is always above ChatWidget elements

**Steps**:
1. Open site in mobile view (width < 768px)
2. Verify ChatWidget floating button is visible at bottom-right
3. Open Chrome DevTools > Elements tab
4. Inspect the following elements and verify z-index values:
   - `.navbar` → should be 100 (Docusaurus default)
   - `.selectTooltip` → should be 99 (FIXED from 100)
   - `.floatingButton` → should be 99
   - `.chatModal` → should be 98

**Expected Result**:
- Navbar z-index (100) > selectTooltip z-index (99)
- No visual overlap between navbar and ChatWidget elements

---

### Test 2: Text Selection Tooltip Behavior

**Objective**: Verify selectTooltip does not overlap hamburger menu when text near top is selected

**Steps**:
1. Open site in mobile view (width < 768px)
2. Select text near the top of the page (close to hamburger menu)
3. Observe the "Ask AI" tooltip that appears
4. Verify tooltip does NOT cover the hamburger menu icon
5. Click hamburger menu to open sidebar
6. Verify sidebar opens without issues

**Expected Result**:
- Tooltip appears below navbar (does not obscure hamburger menu)
- Sidebar opens and closes smoothly

**Failure Scenario** (before fix):
- Tooltip with z-index: 100 would overlap navbar (z-index: 100)
- Hamburger menu might be unclickable

---

### Test 3: Navbar hideOnScroll Behavior

**Objective**: Verify navbar auto-hides when scrolling down on mobile

**Steps**:
1. Open site in mobile view (width < 768px)
2. Scroll down the page slowly
3. Observe navbar behavior
4. Scroll back up
5. Observe navbar reappearance

**Expected Result**:
- Navbar slides up/hides when scrolling down
- Navbar slides down/reappears when scrolling up
- Provides more screen real estate on mobile

**Configuration Verification**:
- Check `docusaurus.config.js`:
  ```javascript
  navbar: {
    hideOnScroll: true,  // Must be present
    // ...
  }
  ```

---

### Test 4: ChatWidget Mobile Responsiveness

**Objective**: Verify ChatWidget button and modal display correctly on various mobile screen sizes

**Test Screens**:
- **Small phone**: 320px × 568px (iPhone SE)
- **Standard phone**: 375px × 667px (iPhone 8)
- **Large phone**: 414px × 896px (iPhone 11 Pro Max)
- **Tablet**: 768px × 1024px (iPad)

**Steps for each screen size**:
1. Set DevTools to screen dimensions
2. Verify ChatWidget button is visible and not cut off
3. Click ChatWidget button to open modal
4. Verify modal displays correctly:
   - Full-screen on phones (<768px)
   - Sized modal on tablets (≥768px)
5. Type a test message and send
6. Close modal and verify button reappears

**Expected Results**:
- **<768px**: Button 56px × 56px, modal full-screen
- **≥768px**: Button 60px × 60px, modal 400px × 600px
- All touch targets ≥44px (WCAG 2.1 AAA compliance)
- No horizontal scrollbar on any screen size

---

### Test 5: Android Device Testing (Real Device)

**Objective**: Verify fixes work on actual Android hardware

**Devices to Test**:
- Android phone with Chrome browser (primary)
- Android tablet (if available)

**Steps**:
1. Connect Android device to same network as development machine
2. Find local IP: `ipconfig` (Windows) or `ifconfig` (Mac/Linux)
3. Open `http://<YOUR_IP>:3000` on Android Chrome
4. Repeat Test Cases 1-4 on physical device
5. Pay special attention to:
   - Touch target responsiveness
   - Hamburger menu functionality
   - Text selection and tooltip behavior
   - Scroll performance with hideOnScroll

**Expected Results**:
- All interactions respond within 100ms (from spec SC-006)
- No overlap between hamburger menu and ChatWidget
- Smooth animations and transitions

---

### Test 6: Desktop Regression Testing

**Objective**: Ensure changes do not break desktop UI

**Steps**:
1. Open site in desktop view (width > 768px)
2. Verify navbar displays normally (no hideOnScroll effect)
3. Verify ChatWidget button is 60px × 60px at bottom-right
4. Open ChatWidget modal and verify it's sized (not full-screen)
5. Select text and verify "Ask AI" tooltip appears
6. Verify tooltip and button z-index work correctly

**Expected Results**:
- No visual changes to desktop UI
- Z-index fixes do not affect desktop experience
- hideOnScroll only applies to mobile (check Docusaurus docs for breakpoint)

---

### Test 7: Accessibility Verification

**Objective**: Verify mobile UI meets WCAG 2.1 accessibility standards

**Steps**:
1. Verify touch target sizes:
   - Hamburger menu: ≥44px × 44px
   - ChatWidget button: 56px × 56px (mobile) ✅
   - Close button in modal: ≥44px × 44px
2. Test keyboard navigation (if applicable on mobile)
3. Verify color contrast ratios (no changes expected)
4. Test with screen reader (TalkBack on Android, VoiceOver on iOS)

**Expected Results**:
- All interactive elements meet WCAG 2.1 Level AA (minimum)
- Touch targets meet Level AAA where possible (44px × 44px)

---

## Visual Regression Checklist

Use this checklist to compare before/after states:

### Mobile (<768px)
- [ ] Hamburger menu is visible and clickable
- [ ] ChatWidget button is visible at bottom-right
- [ ] No overlap between navbar and ChatWidget elements
- [ ] Text selection tooltip appears below navbar
- [ ] Navbar hides on scroll down, reappears on scroll up
- [ ] Modal is full-screen when ChatWidget is opened
- [ ] All touch targets are ≥44px

### Desktop (>768px)
- [ ] No visual changes from previous version
- [ ] Navbar behavior unchanged (no hideOnScroll effect)
- [ ] ChatWidget button and modal display as before
- [ ] Z-index fixes do not affect desktop layout

---

## Known Issues & Workarounds

### Issue 1: hideOnScroll not working
**Symptom**: Navbar does not auto-hide when scrolling
**Possible Causes**:
- `hideOnScroll: true` not added to config
- Docusaurus version incompatibility
- Browser cache issue

**Workaround**:
1. Verify config syntax in `docusaurus.config.js`
2. Clear browser cache and hard refresh (`Ctrl+Shift+R`)
3. Restart dev server: `npm run start`

### Issue 2: Tooltip still overlaps navbar
**Symptom**: "Ask AI" tooltip covers hamburger menu
**Possible Causes**:
- Z-index change not applied
- CSS module not reloaded

**Workaround**:
1. Verify `selectTooltip` z-index is 99 in `styles.module.css`
2. Hard refresh browser
3. Check browser DevTools > Elements for actual computed z-index

---

## Performance Benchmarks

**From Spec Success Criteria**:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| Touch response time | <100ms | Use Chrome DevTools > Performance tab, record interaction |
| Visual separation | ≥8px | Use DevTools > Elements > Computed styles, measure margins |
| Modal width | 100vw - 32px on mobile | DevTools > Elements > Computed styles |
| No horizontal scroll | Pass | Visual inspection, scroll horizontally |
| Z-index correct | navbar=100, tooltip=99 | DevTools > Elements > Computed styles |

---

## Reporting Issues

If any test case fails:

1. **Document the failure**:
   - Test case number and name
   - Expected vs actual result
   - Screenshots/screen recordings
   - Device/browser information

2. **Check console errors**:
   - Open DevTools > Console
   - Look for JavaScript errors
   - Copy full error messages

3. **Verify configuration**:
   - `docusaurus.config.js` has `hideOnScroll: true`
   - `styles.module.css` has `z-index: 99` for selectTooltip

4. **Create bug report**:
   - Include all documentation from steps 1-3
   - Reference spec.md requirements
   - Suggest potential fixes if known

---

## Success Criteria Summary

All tests pass when:

✅ Hamburger menu is fully functional on Android
✅ No overlap between navbar and ChatWidget elements
✅ Text selection tooltip appears below navbar
✅ Navbar auto-hides on scroll (mobile only)
✅ ChatWidget displays correctly on all screen sizes
✅ Desktop UI is unchanged
✅ All touch targets ≥44px (WCAG 2.1 AAA)
✅ Performance metrics meet targets

**Ready for deployment** when all checkboxes are marked.
