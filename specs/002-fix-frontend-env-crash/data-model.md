# Data Model: Fix Frontend Environment Variable Crash

**Feature**: 002-fix-frontend-env-crash
**Date**: 2025-12-14
**Phase**: Phase 1 (Design Artifacts)

## Overview

This bug fix does NOT introduce new data models or persistent data. It modifies a single constant definition for safer runtime behavior.

## Configuration Models

### API_BASE_URL Constant

**Model**: JavaScript constant (string value)

```javascript
// Before (unsafe - crashes in browser)
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

// After (safe - works in browser and Node.js)
const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && process.env.REACT_APP_API_URL)
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000';
```

**Constant Specification**:

| Property | Type | Source | Default Value | Validation |
|----------|------|--------|---------------|------------|
| `API_BASE_URL` | string | `REACT_APP_API_URL` env var or fallback | `'http://localhost:8000'` | None (string literal) |

**Resolution Logic**:
1. Check if `process` is defined (safe typeof check)
2. If defined, check if `process.env` exists
3. If exists, check if `REACT_APP_API_URL` is set
4. If all checks pass, use env var value
5. Otherwise, use fallback `'http://localhost:8000'`

## State Transitions

**N/A** - Constant is defined once at module load time, immutable thereafter.

### Constant Resolution Lifecycle

```
1. Module Load (ChatWidget/index.js)
   ↓
2. Evaluate typeof process !== 'undefined'
   ├─ true → Check process.env.REACT_APP_API_URL
   │   ├─ defined → API_BASE_URL = env var value
   │   └─ undefined → API_BASE_URL = 'http://localhost:8000'
   └─ false → API_BASE_URL = 'http://localhost:8000' (browser environment)
   ↓
3. Constant defined (immutable)
   ↓
4. Used in fetch calls throughout component lifecycle
```

## Data Relationships

**N/A** - Single constant, no relationships.

## Storage Considerations

### Build-Time vs Runtime

**Build-Time** (Webpack DefinePlugin):
- Webpack attempts to replace `process.env.REACT_APP_API_URL` with string literal
- If replacement succeeds: Code becomes `const API_BASE_URL = 'literal' || 'http://localhost:8000'`
- If replacement fails: Runtime evaluation uses typeof check

**Runtime** (Browser):
- If webpack replacement failed, browser evaluates typeof check
- `process` is undefined in browser → typeof returns "undefined"
- Condition is false, fallback to localhost

## Example Scenarios

### Scenario 1: Local Development (No .env.local)

**Environment**: Browser, no REACT_APP_API_URL set
**Webpack Behavior**: No env var to replace, leaves code as-is
**Runtime Evaluation**:
```javascript
typeof process !== 'undefined' // false (browser has no process)
→ API_BASE_URL = 'http://localhost:8000' // ✅ FALLBACK
```
**Result**: Uses localhost:8000 (correct for local backend)

### Scenario 2: Production Build with REACT_APP_API_URL

**Environment**: CI/CD build with `REACT_APP_API_URL=https://api.production.com`
**Webpack Behavior**: Replaces `process.env.REACT_APP_API_URL` with `"https://api.production.com"`
**Bundled Code**:
```javascript
const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && "https://api.production.com")
  ? "https://api.production.com"
  : 'http://localhost:8000';
```
**Optimization**: Minifier may optimize to `const API_BASE_URL = "https://api.production.com";`
**Result**: Uses production URL (correct)

### Scenario 3: Node.js Dev Server (Docusaurus start)

**Environment**: Node.js, REACT_APP_API_URL may or may not be set
**Runtime Evaluation**:
```javascript
typeof process !== 'undefined' // true (Node.js has process)
process.env.REACT_APP_API_URL // undefined or set value
→ API_BASE_URL = env var OR 'http://localhost:8000' // ✅ WORKS
```
**Result**: Uses env var if set, otherwise localhost

## Validation and Constraints

### Runtime Safety Constraints

1. **typeof Check**: MUST use `typeof` operator to prevent ReferenceError
2. **Chained Checks**: MUST check process && process.env before accessing properties
3. **Fallback Value**: MUST provide non-empty fallback string
4. **No Mutations**: Constant MUST be defined with `const` (immutable)

### Error Handling

**No Explicit Error Handling Needed**:
- typeof check prevents ReferenceError
- Chained && operators prevent null/undefined access
- Fallback ensures constant always has valid value
- Browser fetch will naturally fail with clear error if URL is wrong (already handled in existing code)

## Migration and Backward Compatibility

### Migration Path

**From**: Unsafe direct process.env access (causes crash)
```javascript
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';
```

**To**: Safe typeof-checked access (works in all environments)
```javascript
const API_BASE_URL = (typeof process !== 'undefined' && process && process.env && process.env.REACT_APP_API_URL)
  ? process.env.REACT_APP_API_URL
  : 'http://localhost:8000';
```

**Backward Compatibility**:
- ✅ Environment variable pattern unchanged (REACT_APP_API_URL)
- ✅ Fallback value unchanged (localhost:8000)
- ✅ Webpack replacement still works (typeof check doesn't interfere)
- ✅ No breaking changes to feature 001-cors-env-config

### Rollback Strategy

If issues occur:
1. Revert single line change in ChatWidget/index.js
2. Redeploy (no database migrations, no data loss)
3. Root cause: Fix is minimal, rollback is trivial

**Data Loss Risk**: NONE - no persistent data changes

## References

- **Existing Code**: src/components/ChatWidget/index.js:5 (line to modify)
- **Feature 001**: 001-cors-env-config (introduced REACT_APP_API_URL pattern)
- **JavaScript typeof**: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/typeof
