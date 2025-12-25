# Data Model: UI Critical Fixes

**Feature**: UI Critical Fixes
**Date**: 2025-12-25
**Type**: Frontend Component State (No Database Schema)

## Overview

This feature involves CSS-only fixes and minimal component state modifications. No new data entities, API contracts, or database schemas are required. This document captures component state variables relevant to the ghost popup investigation.

---

## Component State Variables

### ChatWidget Component State

**File**: `src/components/ChatWidget/index.js`
**State Hook**: `useState`

```javascript
// Modal configuration state (lines 26-33)
const [modalConfig, setModalConfig] = useState({
  isOpen: boolean,        // Modal visibility flag
  title: string,          // Modal header text
  message: string,        // Modal body text
  onConfirm: function,    // Callback for confirm button
  confirmText: string,    // Confirm button label
  isDanger: boolean       // Style variant (red vs blue button)
});

// Other relevant state
const [isOpen, setIsOpen] = useState(false);           // ChatWidget modal visibility
const [tooltipVisible, setTooltipVisible] = useState(false);  // Select-to-ask tooltip
const [messages, setMessages] = useState([]);         // Chat message history
const [isLoading, setIsLoading] = useState(false);    // API request loading
const [isLoadingHistory, setIsLoadingHistory] = useState(true); // History load status
```

---

## State Transitions

### ConfirmationModal State Machine

```
Initial State: { isOpen: false, ... }
      │
      │ User clicks delete button
      ▼
State 1: { isOpen: true, title: "Delete Message", onConfirm: confirmDelete, isDanger: true }
      │
      │ User clicks "Delete" (confirm)
      ▼
Action: onConfirm() executes → API call to delete message
      │
      │ onConfirm callback completes
      ▼
State 2: { isOpen: false, ... } (modal closes)
      │
      │ OR User clicks "Cancel"
      ▼
State 2: { isOpen: false, ... } (modal closes without action)
```

**Trigger Locations**:
1. **Delete Message** (index.js:192-245):
   ```javascript
   const handleDelete = (messageId) => {
     setModalConfig({
       isOpen: true,
       title: "Delete Message",
       message: "Are you sure you want to delete this message?",
       onConfirm: () => confirmDelete(messageId),
       confirmText: "Delete",
       isDanger: true,
     });
   };
   ```

2. **Delete All History** (index.js:248-294):
   ```javascript
   const handleDeleteAllHistory = () => {
     setModalConfig({
       isOpen: true,
       title: "Delete All Chat History",
       message: "This will permanently delete all your chat messages...",
       onConfirm: confirmDeleteAll,
       confirmText: "Delete All",
       isDanger: true,
     });
   };
   ```

---

### Select-to-Ask Tooltip State

```
Initial State: { tooltipVisible: false }
      │
      │ User selects text on page
      ▼
Event: mouseup event listener triggers handleSelection()
      │
      │ if (selectedText.trim().length > 0 && !isOpen)
      ▼
State 1: { tooltipVisible: true, tooltipPosition: { x, y }, selectedText: "..." }
      │
      │ User clicks tooltip OR clicks elsewhere
      ▼
State 2: { tooltipVisible: false }
```

**Implementation** (index.js:573-593):
```javascript
{tooltipVisible && !isOpen && (
  <div
    className={styles.selectToAskTooltip}
    style={{
      position: "absolute",
      left: `${tooltipPosition.x}px`,
      top: `${tooltipPosition.y}px`,
      zIndex: 99, // Below navbar (1000)
    }}
  >
    {/* Tooltip content */}
  </div>
)}
```

---

## State Invariants

### Modal Visibility Rules

1. **User Action Required**:
   ```
   modalConfig.isOpen === true
   ⟹ User MUST have clicked delete button (explicit onClick event)
   ```

2. **Single Modal at a Time**:
   ```
   tooltipVisible === true ∧ isOpen === true
   ⟹ CONTRADICTION (tooltip hides when modal open)
   ```
   **Enforcement**: Line 573 condition `{tooltipVisible && !isOpen && ( ... )}`

3. **Default State Safety**:
   ```
   Component Mount ⟹ modalConfig.isOpen === false
   ```
   **Enforcement**: `useState({ isOpen: false, ... })` initialization

### State Persistence Rules

**SessionStorage** (index.js:57-77):
```javascript
useEffect(() => {
  const loadHistory = async () => {
    // Load messages from sessionStorage
    const storedHistory = sessionStorage.getItem('chatHistory');
    if (storedHistory) {
      setMessages(JSON.parse(storedHistory));  // ✅ Restore messages
    }
    // ❌ Modal state is NOT persisted
  };
  loadHistory();
}, [isAuthenticated]);
```

**Not Persisted**:
- `modalConfig.isOpen` (always resets to false on page load)
- `tooltipVisible` (always resets to false)
- `isOpen` (ChatWidget modal visibility)

**Persisted**:
- `messages` array (chat history via sessionStorage)

---

## CSS State (Z-Index Hierarchy)

### Z-Index Data Model

```
Entity: UI Layer
Attributes:
  - component: string (component name)
  - className: string (CSS class)
  - zIndex: number (stacking order)
  - position: enum(fixed|absolute|sticky|relative)
  - file: string (source file path)

Relationships:
  - ABOVE: zIndex > other.zIndex ∧ position ∈ {fixed, absolute, sticky}
  - STACKING_CONTEXT: Creates new context if position ≠ static ∧ zIndex ≠ auto
```

**Instances**:

| Component | Class | Z-Index | Position | File |
|-----------|-------|---------|----------|------|
| ChatWidget Backdrop | `.modalBackdrop` | 97 | fixed | styles.module.css:38 |
| ChatWidget Modal | `.chatModal` | 98 | fixed | styles.module.css:56 |
| ChatWidget Button | `.chatButton` | 99 | fixed | styles.module.css:14 |
| Select-to-Ask Tooltip | `.selectToAskTooltip` | 99 | absolute | styles.module.css:408 |
| Docusaurus Navbar | `.navbar` | 1000 | sticky | custom.css:94 |
| Confirmation Backdrop | `.modalBackdrop` | 9999 | fixed | modal.module.css:25 |
| Confirmation Modal | `.modalContainer` | 10000 | fixed | modal.module.css:56 |

**Ordering Constraint**:
```
ChatWidget (97-99) < Navbar (1000) < ConfirmationModal (9999-10000)
```

---

## Validation Rules

### Modal Trigger Validation

```javascript
// Rule: Modal MUST NOT auto-trigger on page load
function validateNoAutoTrigger(componentDidMount) {
  const modalOpenCalls = findSetModalConfigCalls(componentDidMount);
  const userEventGuards = modalOpenCalls.filter(call => hasOnClickHandler(call));

  assert(modalOpenCalls.length === userEventGuards.length,
    "All setModalConfig({ isOpen: true }) calls MUST be within onClick handlers");
}
```

**Current Implementation**: ✅ PASS
- Both `handleDelete` and `handleDeleteAllHistory` are only called via `onClick` events
- No `useEffect` hooks trigger modal state changes

### Tooltip Trigger Validation

```javascript
// Rule: Tooltip MUST only appear on text selection
function validateTooltipTrigger() {
  assert(tooltipVisible === true ⟹ userSelectedText === true,
    "tooltipVisible can only be true after mouseup event with selected text");
}
```

**Current Implementation**: ✅ PASS
- `setTooltipVisible(true)` only called in `handleSelection` function
- `handleSelection` registered as `mouseup` event listener

---

## Edge Cases

### State Reset Scenarios

1. **Page Navigation**:
   ```
   User navigates to new route
   → ChatWidget unmounts
   → All state resets to default
   → modalConfig.isOpen = false ✅
   ```

2. **Browser Refresh**:
   ```
   User refreshes page (F5)
   → Component re-mounts
   → State re-initializes from useState defaults
   → modalConfig.isOpen = false ✅
   → messages restored from sessionStorage ✅
   ```

3. **Modal Escape Key**:
   ```
   User presses Escape key while modal open
   → ConfirmationModal.js:36-50 handles keydown event
   → onCancel() called → setModalConfig({ isOpen: false })
   → Modal closes ✅
   ```

---

## Summary

**Data Structures Modified**: None (CSS-only fixes)
**State Variables Modified**: None (investigation only; no ghost popup trigger found)
**Storage Impact**: None (no new sessionStorage/localStorage keys)

**Key Finding**: All modal state transitions are properly guarded by user event handlers. No auto-trigger mechanism exists in current codebase.

**Recommendation**: Proceed with CSS fixes (mobile menu, z-index comment correction) and request user clarification on ghost popup before implementing state logic changes.
