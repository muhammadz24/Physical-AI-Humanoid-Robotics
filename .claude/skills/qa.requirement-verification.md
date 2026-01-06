---
name: qa.requirement-verification
description: Verify implementation matches spec acceptance criteria for Todo App Phase 1
owner: QA Agent
tags: [verification, requirements, acceptance-criteria, phase-1]
---

## Purpose

Verify that implemented features for the Todo App Phase 1 match their specifications by:
- Comparing implementation behavior to spec acceptance criteria
- Validating CLI commands match specified interface
- Ensuring CRUD operations behave as documented
- Checking error messages match spec requirements
- Confirming edge cases are handled per spec
- Identifying gaps between spec and implementation

## When to Use

Invoke this skill when:
- Feature implementation is complete and tests are passing
- Conducting formal acceptance review before merging
- Resolving disputes about "correct" behavior
- Preparing for demo or hackathon judging
- Validating bug fixes match spec requirements

**Trigger phrases:**
- "Verify implementation against spec"
- "Check if [feature] matches requirements"
- "Validate acceptance criteria for [feature]"
- "Does the implementation follow the spec?"
- "Review [feature] against specification"

## Inputs

**Required:**
- `feature_name` - The feature to verify (must have corresponding spec file)

**Optional:**
- `focus_area` - Specific aspect: "cli-interface", "crud-operations", "error-handling", "edge-cases", "all" (default: "all")
- `strict_mode` - Fail on ANY deviation from spec, even minor (default: false)

**Example invocations:**
```
Verify add-task implementation against spec
Check if list-tasks CLI matches requirements
Validate delete-task error handling per spec
Review update-task feature for acceptance criteria
```

## Step-by-Step Process

### 1. Load Specification

**Locate and read spec file:**
```bash
# Identify feature spec
SPEC_FILE="specs/${feature_name}/spec.md"

# Verify spec exists
if [ ! -f "$SPEC_FILE" ]; then
    echo "❌ ERROR: Spec not found at $SPEC_FILE"
    exit 1
fi
```

**Extract from spec:**
- Feature overview and purpose
- Acceptance criteria (AC-001, AC-002, etc.)
- CLI command syntax and arguments
- Expected outputs (success and error cases)
- Edge cases and error handling requirements
- Data validation rules

**Parse acceptance criteria:**
```
Example spec section:
## Acceptance Criteria

- AC-001: User can add a todo with a title
- AC-002: System assigns a unique numeric ID
- AC-003: System returns confirmation message
- AC-004: Empty title is rejected with error
```

Create checklist:
```markdown
## Acceptance Criteria Checklist
- [ ] AC-001: User can add a todo with a title
- [ ] AC-002: System assigns a unique numeric ID
- [ ] AC-003: System returns confirmation message
- [ ] AC-004: Empty title is rejected with error
```

### 2. Identify Implementation Files

**Locate implementation:**
```bash
# Find files modified for this feature (from tasks.md or git)
grep -l "TODO.*${feature_name}" src/*.py

# Or check recent commits
git log --name-only --grep="${feature_name}" --pretty=format: | sort -u
```

**Expected locations for Phase 1 Todo App:**
- `todo.py` or `src/todo.py` - Core todo operations
- `cli.py` or `src/cli.py` - CLI interface
- `storage.py` or `src/storage.py` - In-memory storage
- `main.py` or `src/main.py` - Entry point

### 3. Verify CLI Interface

**Check command syntax matches spec:**

For each CLI command in spec, verify:
- Command name is exact match
- Arguments are in correct order
- Optional vs required arguments match
- Argument types match (string, int, etc.)
- Help text is present and accurate

**Example verification for "add" command:**

Spec says:
```
Command: todo add <title>
Arguments:
  - title (required, string): The todo item title
Output: "Todo added: <title> (ID: <id>)"
```

Verification steps:
```bash
# Test exact command syntax
python main.py add "Buy milk"

# Expected output
"Todo added: Buy milk (ID: 1)"

# Test help text
python main.py add --help

# Should show: "Add a new todo item"
```

**Verification checklist:**
- [ ] Command name matches spec exactly
- [ ] Required arguments enforced
- [ ] Optional arguments work as specified
- [ ] Argument order matches spec
- [ ] Help text present and accurate
- [ ] Invalid usage shows error (e.g., missing required arg)

### 4. Verify CRUD Operations

**For each CRUD operation, verify behavior:**

#### Create (Add)
- [ ] Accepts valid input per spec
- [ ] Rejects invalid input per spec (empty, too long, etc.)
- [ ] Generates unique ID as specified
- [ ] Returns confirmation message matching spec format
- [ ] Stores in memory correctly

#### Read (Get/Show)
- [ ] Retrieves todo by ID
- [ ] Returns correct format per spec
- [ ] Handles non-existent ID per spec
- [ ] Shows all todo fields specified

#### Update (Edit/Modify)
- [ ] Accepts valid updates per spec
- [ ] Rejects invalid updates per spec
- [ ] Updates specified fields only
- [ ] Returns confirmation matching spec
- [ ] Preserves other fields unchanged

#### Delete (Remove)
- [ ] Removes todo by ID
- [ ] Returns confirmation matching spec
- [ ] Handles non-existent ID per spec
- [ ] Actually removes from storage

#### List (List All)
- [ ] Shows all todos in specified format
- [ ] Handles empty list per spec
- [ ] Includes all required fields per spec
- [ ] Sorting/ordering matches spec (if specified)

**Testing approach:**
```bash
# Create test sequence
python main.py add "Task 1"  # Should return ID: 1
python main.py add "Task 2"  # Should return ID: 2
python main.py list          # Should show both tasks
python main.py delete 1      # Should confirm deletion
python main.py list          # Should show only Task 2
```

### 5. Verify Error Handling

**Extract error scenarios from spec:**

Common Phase 1 error scenarios:
- Empty or invalid input
- Non-existent ID
- Invalid command
- Malformed arguments

**For each error scenario in spec:**
1. Identify specified error behavior
2. Trigger error condition
3. Verify error message matches spec
4. Verify exit code (if specified)
5. Verify no data corruption

**Example error verification:**

Spec says:
```
Error: Empty title
Message: "Error: Title cannot be empty"
Exit code: 1
```

Verification:
```bash
# Trigger error
python main.py add ""

# Verify output
Expected: "Error: Title cannot be empty"
Actual: [capture output]
Match: ✅ / ❌

# Verify exit code
echo $?
Expected: 1
Actual: [capture code]
Match: ✅ / ❌

# Verify no side effects
python main.py list
# Should not show empty todo
```

**Error handling checklist:**
- [ ] Error messages match spec exactly (if exact match required)
- [ ] Error messages convey meaning clearly (if not exact)
- [ ] Appropriate exit codes (0 = success, non-zero = error)
- [ ] No stack traces for expected errors
- [ ] Errors go to stderr (if specified in spec)
- [ ] No data corruption on error

### 6. Verify Edge Cases

**Identify edge cases from spec:**

Phase 1 Todo App common edge cases:
- Empty list (no todos)
- Maximum capacity (if specified)
- Special characters in title
- Very long titles
- ID wraparound (if many todos)
- Concurrent operations (if relevant)

**For each edge case in spec:**
1. Set up edge condition
2. Execute operation
3. Verify behavior matches spec
4. Check for undefined behavior not in spec

**Example edge case verification:**

Spec says:
```
Edge Case: Empty list
Command: todo list
Output: "No todos found"
```

Verification:
```bash
# Ensure empty state
# (fresh start or delete all)

# Execute command
python main.py list

# Verify output
Expected: "No todos found"
Actual: [capture output]
Match: ✅ / ❌
```

**Edge case checklist:**
- [ ] Empty list handled per spec
- [ ] Special characters handled (if specified)
- [ ] Boundary values handled (max/min)
- [ ] No crashes or undefined behavior
- [ ] Graceful degradation where appropriate

### 7. Verify Data Validation

**Extract validation rules from spec:**

Example rules:
- Title: required, non-empty, max 200 chars
- ID: numeric, positive, unique
- Status: enum (if present) - "pending", "complete"

**For each validation rule:**
1. Test valid input (should succeed)
2. Test invalid input (should fail with error)
3. Test boundary values
4. Verify error messages match spec

**Validation test matrix:**

| Rule | Valid Input | Invalid Input | Expected Behavior |
|------|-------------|---------------|-------------------|
| Title required | "Buy milk" | "" | Error: Title cannot be empty |
| Title max 200 | "A"*200 | "A"*201 | Error: Title too long |
| ID positive | 1 | -1 | Error: Invalid ID |
| ID exists | 1 (exists) | 999 (not) | Error: Todo not found |

### 8. Compare Output Formats

**Verify output matches spec format:**

For each command output in spec:
1. Execute command
2. Capture actual output
3. Compare to spec format
4. Check for extra/missing fields
5. Verify formatting (spacing, punctuation)

**Example output comparison:**

Spec says:
```
Output format:
ID: 1
Title: Buy milk
Status: pending
```

Actual output:
```
ID: 1
Title: Buy milk
Status: pending
```

Comparison:
```
✅ All fields present
✅ Field order matches
✅ Formatting matches
✅ No extra fields
```

**If spec allows flexibility:**
- Verify all required fields present
- Accept variations in formatting
- Flag only if missing critical information

### 9. Generate Verification Report

**Create detailed verification report:**

```markdown
# Requirement Verification Report: [Feature Name]

## Specification Reference
- **Spec File:** specs/add-task/spec.md
- **Verification Date:** 2025-12-24
- **Verifier:** QA Agent

## Acceptance Criteria Results

### AC-001: User can add a todo with a title
- **Status:** ✅ PASS
- **Evidence:** Command `todo add "Buy milk"` successfully creates todo
- **Output:** "Todo added: Buy milk (ID: 1)" (matches spec)

### AC-002: System assigns a unique numeric ID
- **Status:** ✅ PASS
- **Evidence:** Sequential IDs assigned (1, 2, 3...)
- **Test:** Added 3 todos, IDs were 1, 2, 3 (unique and numeric)

### AC-003: System returns confirmation message
- **Status:** ⚠️ PARTIAL
- **Evidence:** Confirmation present but format differs slightly
- **Expected:** "Todo added: {title} (ID: {id})"
- **Actual:** "Added todo: {title} with ID {id}"
- **Impact:** Low (message is clear, just different format)
- **Recommendation:** Update to match spec or update spec

### AC-004: Empty title is rejected with error
- **Status:** ❌ FAIL
- **Evidence:** Empty title causes crash, not graceful error
- **Expected:** "Error: Title cannot be empty"
- **Actual:** "IndexError: string index out of range"
- **Impact:** HIGH (critical error handling failure)
- **Required Fix:** Add validation before processing title

## CLI Interface Verification

### Command: todo add <title>
- ✅ Command name correct
- ✅ Argument order correct
- ✅ Required argument enforced
- ✅ Help text present

## CRUD Operations Verification

### Create (Add)
- ✅ Accepts valid input
- ❌ Does not reject empty input gracefully
- ✅ Generates unique ID
- ⚠️ Confirmation format differs from spec

### List
- ✅ Shows all todos
- ✅ Handles empty list ("No todos found")
- ✅ Format matches spec

### Delete
- ✅ Removes by ID
- ✅ Confirms deletion
- ✅ Handles non-existent ID

## Error Handling Verification

### Empty Title Error
- ❌ FAIL: Crashes instead of error message
- Required: "Error: Title cannot be empty"
- Actual: Stack trace

### Non-Existent ID Error
- ✅ PASS: "Error: Todo #999 not found"
- Matches spec requirement

## Edge Cases Verification

### Empty List
- ✅ PASS: Shows "No todos found"

### Special Characters
- ⚠️ NOT SPECIFIED in spec
- Tested: Works with basic punctuation
- Recommendation: Add to spec

## Summary

### Overall Status: ⚠️ CONDITIONAL PASS

**Passed:** 8/10 acceptance criteria
**Partial:** 1/10 (minor format difference)
**Failed:** 1/10 (critical: empty title crash)

### Blocking Issues
1. AC-004: Empty title causes crash (CRITICAL)
   - Fix: Add input validation
   - Priority: HIGH

### Non-Blocking Issues
2. AC-003: Confirmation message format differs
   - Fix: Update implementation or spec
   - Priority: LOW

### Recommendations
1. Fix empty title validation (REQUIRED before merge)
2. Decide on confirmation message format (update code or spec)
3. Add special character handling to spec
4. All fixes should reference AC IDs in commit messages

### Next Steps
- [ ] Fix AC-004 (empty title validation)
- [ ] Re-run verification after fix
- [ ] Decide on AC-003 resolution
- [ ] Update spec if needed
```

## Output

**Success case (all criteria pass):**
```
✅ REQUIREMENT VERIFICATION COMPLETE: [Feature Name]

📋 Acceptance Criteria: 10/10 passed (100%)
✅ CLI Interface: Matches spec
✅ CRUD Operations: All verified
✅ Error Handling: All scenarios handled
✅ Edge Cases: All covered

🎯 IMPLEMENTATION MATCHES SPECIFICATION
✅ APPROVED FOR MERGE

[Link to detailed verification report]
```

**Partial pass (minor deviations):**
```
⚠️ REQUIREMENT VERIFICATION: [Feature Name]

📋 Acceptance Criteria: 8/10 passed, 2/10 minor deviations
✅ CLI Interface: Matches spec
⚠️ Output Format: Minor formatting differences
✅ Error Handling: All scenarios handled

Non-blocking issues:
1. Confirmation message format differs (LOW priority)
2. Special characters not specified in spec

🎯 CONDITIONAL APPROVAL
✅ Can merge with documented deviations
📝 Recommend updating spec or implementation for consistency

[Link to detailed verification report]
```

**Failure case (blocking issues):**
```
❌ REQUIREMENT VERIFICATION FAILED: [Feature Name]

📋 Acceptance Criteria: 7/10 passed, 3/10 FAILED
❌ Critical Issues Found:
   1. AC-004: Empty title causes crash (HIGH)
   2. AC-007: Delete doesn't update storage (HIGH)
   3. AC-009: Invalid ID shows wrong error (MEDIUM)

🚫 BLOCKING ISSUES PREVENT MERGE

Required fixes:
1. Add empty title validation
2. Fix delete operation storage update
3. Correct error message for invalid ID

📋 Return to implementation
🔒 Re-verify after fixes

[Link to detailed verification report]
```

## Failure Handling

### Spec File Not Found
- Check if feature name is correct
- Verify spec exists in `specs/[feature-name]/spec.md`
- If spec missing: BLOCK and require spec creation
- Cannot verify implementation without specification

### Implementation Not Found
- Check if implementation is complete
- Verify files exist in expected locations
- If implementation missing: Report as incomplete feature
- Cannot verify non-existent implementation

### Acceptance Criteria Ambiguous
- Flag ambiguous criteria in report
- Request clarification from domain expert
- Document assumption if verification proceeds
- Recommend updating spec for clarity

### Output Format Mismatch
**If strict_mode = true:**
- FAIL any deviation from spec format
- Require exact match

**If strict_mode = false (default):**
- PASS if meaning is preserved
- FLAG if format differs significantly
- Recommend alignment for consistency

### Undocumented Behavior
- Flag any behavior not in spec
- Document actual behavior
- Recommend adding to spec if intentional
- Consider as potential scope creep

## Quality Gates

**PASS (approved for merge):**
- ✅ 100% of acceptance criteria pass
- ✅ CLI interface matches spec
- ✅ All CRUD operations verified
- ✅ Error handling per spec
- ✅ Edge cases handled
- ✅ No blocking issues

**CONDITIONAL PASS (can merge with documentation):**
- ⚠️ ≥80% acceptance criteria pass
- ⚠️ Minor format deviations (non-breaking)
- ⚠️ Extra features not in spec (documented)
- ✅ All critical paths verified
- ✅ No data integrity issues

**FAIL (blocking, cannot merge):**
- ❌ <80% acceptance criteria pass
- ❌ Critical AC failures (core CRUD broken)
- ❌ Data integrity violations
- ❌ Crashes or undefined behavior
- ❌ Security violations

## Integration with SDD Workflow

**After verification:**
1. If PASS: Approve for merge, create PHR (stage: green)
2. If CONDITIONAL: Document deviations, approve with notes
3. If FAIL: Block merge, create bug-fix tasks, create PHR (stage: red)

**Traceability:**
- Link verification report to spec file
- Reference AC IDs in verification results
- Create tasks for failed criteria
- Document deviations in ADR if significant

## Examples

### Example 1: Perfect Match
```
Input: "Verify add-task against spec"

Output:
✅ VERIFICATION COMPLETE: add-task
📋 All 5 acceptance criteria pass
✅ CLI matches spec exactly
✅ Output format matches spec
✅ Error handling per spec

Verified:
- AC-001: Add with title ✅
- AC-002: Unique ID assignment ✅
- AC-003: Confirmation message ✅
- AC-004: Empty title rejection ✅
- AC-005: Storage persistence ✅

🎯 APPROVED FOR MERGE
```

### Example 2: Minor Deviation
```
Input: "Check list-tasks implementation"

Output:
⚠️ VERIFICATION: list-tasks
📋 4/5 acceptance criteria pass

Passed:
- AC-001: Shows all todos ✅
- AC-002: Empty list message ✅
- AC-004: Correct format ✅
- AC-005: Includes all fields ✅

Deviation:
- AC-003: Sorting order differs
  Expected: "by ID ascending"
  Actual: "by creation time"
  Impact: LOW (still usable)

📝 Recommendation: Update spec to match implementation
   OR update implementation to sort by ID

✅ CONDITIONAL APPROVAL
```

### Example 3: Critical Failure
```
Input: "Verify delete-task feature"

Output:
❌ VERIFICATION FAILED: delete-task
📋 2/4 acceptance criteria pass

CRITICAL FAILURES:
- AC-002: Delete doesn't remove from storage
  Evidence: Deleted todo still appears in list
  Impact: CRITICAL (data integrity)

- AC-004: Non-existent ID causes crash
  Expected: "Error: Todo not found"
  Actual: KeyError exception
  Impact: HIGH (user-facing crash)

Passed:
- AC-001: Accepts valid ID ✅
- AC-003: Confirmation message ✅

🚫 BLOCKED: Cannot merge
📋 Required fixes:
   1. Fix storage.remove() call
   2. Add ID existence check
   3. Re-verify after fixes
```
