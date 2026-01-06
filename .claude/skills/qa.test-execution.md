---
name: qa.test-execution
description: Execute tests and validate coverage for Todo App Phase 1 features
owner: QA Agent
tags: [testing, quality, validation, phase-1]
---

## Purpose

Execute the test suite for the Todo App Phase 1 implementation and validate that:
- All tests pass with deterministic, reproducible results
- Test coverage meets minimum thresholds
- Tests correctly validate CLI behavior, CRUD operations, and in-memory storage
- Edge cases and error paths are tested
- Test output is clear and actionable for debugging

## When to Use

Invoke this skill when:
- A feature implementation is complete and ready for testing
- After fixing bugs to verify the fix and prevent regression
- During code review to validate test quality
- Before creating a commit or pull request
- When validating that spec acceptance criteria are covered by tests

**Trigger phrases:**
- "Run the tests"
- "Execute test suite"
- "Validate test coverage"
- "Check if tests pass"
- "Verify testing for [feature]"

## Inputs

**Required:**
- `feature_name` - The feature being tested (e.g., "add-task", "list-tasks", "delete-task")

**Optional:**
- `test_scope` - Specific test scope: "unit", "integration", "all" (default: "all")
- `coverage_threshold` - Minimum coverage percentage (default: 80)
- `verbose` - Include detailed test output (default: false)

**Example invocations:**
```
Run tests for add-task feature
Execute all tests with coverage report
Run unit tests for delete-task
Validate test coverage for list-tasks feature
```

## Step-by-Step Process

### 1. Pre-Flight Checks

**Verify test infrastructure exists:**
- [ ] Confirm `tests/` directory exists
- [ ] Check for `pytest` or Python unittest framework
- [ ] Verify test discovery patterns (test_*.py or *_test.py)
- [ ] Ensure test dependencies are installed

**Command:**
```bash
# Check test directory structure
ls -la tests/

# Verify pytest is available
python -m pytest --version || python -m unittest --version
```

**Expected outcome:**
- Test framework is available
- Test files are discoverable
- No import errors or missing dependencies

### 2. Execute Test Suite

**Run tests with coverage:**
```bash
# For pytest
python -m pytest tests/ -v --cov=. --cov-report=term-missing --cov-report=html

# For unittest
python -m coverage run -m unittest discover tests/
python -m coverage report -m
```

**Test execution validation:**
- [ ] All tests execute without crashes
- [ ] No skipped tests without justification
- [ ] Test output is clear and parseable
- [ ] Execution time is reasonable (<30s for Phase 1)

**Capture:**
- Total tests run
- Passed/failed/skipped counts
- Execution time
- Any warnings or deprecation notices

### 3. Analyze Test Results

**For each test failure:**
1. Extract test name and location
2. Capture failure message and stack trace
3. Identify if failure is:
   - Actual bug in implementation
   - Flaky test (non-deterministic)
   - Incorrect test expectations
   - Environment issue

**Failure categorization:**
```
IMPLEMENTATION BUG: Test correctly identifies code defect
FLAKY TEST: Test fails intermittently (UNACCEPTABLE for Phase 1)
WRONG EXPECTATION: Test assertions don't match spec
ENVIRONMENT: Missing dependency or configuration issue
```

**For Phase 1 Todo App - Critical test areas:**
- [ ] CLI argument parsing (valid/invalid inputs)
- [ ] CRUD operations (create, read, update, delete, list)
- [ ] In-memory storage integrity (add, retrieve, modify, remove)
- [ ] Error handling (invalid IDs, empty lists, malformed input)
- [ ] Edge cases (duplicate IDs, special characters, empty strings)
- [ ] CLI output formatting (human-readable, consistent)

### 4. Validate Test Coverage

**Coverage analysis:**
```bash
# Generate coverage report
python -m coverage report --show-missing

# Identify uncovered lines
python -m coverage html
```

**Coverage criteria for Phase 1:**
- **Minimum overall coverage:** 80%
- **Critical paths:** 100% (CRUD operations, CLI commands)
- **Error handlers:** 90% (all error paths tested)
- **Edge cases:** Explicitly tested (not just incidentally covered)

**Coverage red flags:**
- Uncovered error handling blocks
- Untested CLI command branches
- Missing edge case tests (empty input, invalid IDs, etc.)
- Unreachable code (indicates dead code or logic errors)

**Generate coverage gap report:**
```markdown
## Coverage Gaps

### Uncovered Critical Paths
- `todo.py:45-48` - Delete operation error handling
- `cli.py:102-105` - Invalid command error message

### Recommended Additional Tests
- Test delete with non-existent ID
- Test CLI with unrecognized command
```

### 5. Validate Test Quality

**Test quality checklist:**
- [ ] Tests follow Arrange-Act-Assert pattern
- [ ] Test names clearly describe what is being tested
- [ ] Each test tests ONE behavior
- [ ] Tests are independent (no shared state)
- [ ] Tests are deterministic (same input = same output)
- [ ] Mock/stub external dependencies (none expected in Phase 1)
- [ ] Assertions are specific and meaningful
- [ ] Error messages aid debugging

**Phase 1 specific quality checks:**
- [ ] CLI tests capture stdout/stderr correctly
- [ ] In-memory storage is reset between tests
- [ ] Tests validate both success and error outputs
- [ ] Tests verify exact CLI output format
- [ ] No time.sleep() or other timing dependencies

**Anti-patterns to flag:**
```python
# ❌ WRONG - Flaky test with timing
time.sleep(0.1)  # PROHIBITED in Phase 1

# ❌ WRONG - Vague assertion
assert result  # What exactly should result be?

# ❌ WRONG - Testing multiple behaviors
def test_todo_operations():  # Too broad
    add_todo()
    list_todos()
    delete_todo()

# ✅ CORRECT - Single behavior, clear assertion
def test_add_todo_returns_success_message():
    result = add_todo("Buy milk")
    assert result == "Todo added: Buy milk (ID: 1)"
```

### 6. Verify Spec Traceability

**Map tests to spec acceptance criteria:**
1. Read `specs/<feature>/spec.md`
2. Extract acceptance criteria
3. Verify each criterion has corresponding test(s)
4. Flag untested acceptance criteria

**Traceability report format:**
```markdown
## Spec Coverage

### Acceptance Criteria: Tested ✅
- AC-001: User can add todo with title → test_add_todo_with_title
- AC-002: System assigns unique ID → test_unique_id_assignment

### Acceptance Criteria: NOT Tested ❌
- AC-005: System rejects empty title → MISSING TEST
```

### 7. Generate Test Report

**Final test execution report:**
```markdown
# Test Execution Report: [Feature Name]

## Summary
- **Total Tests:** 42
- **Passed:** 40 ✅
- **Failed:** 2 ❌
- **Skipped:** 0
- **Coverage:** 87% (exceeds 80% threshold)
- **Execution Time:** 4.2s

## Failed Tests
1. `test_delete_nonexistent_todo` (tests/test_delete.py:34)
   - **Category:** IMPLEMENTATION BUG
   - **Issue:** Should return error, but raises exception
   - **Action Required:** Fix error handling in delete_todo()

2. `test_list_empty_todos` (tests/test_list.py:12)
   - **Category:** WRONG EXPECTATION
   - **Issue:** Test expects empty array, spec says "No todos found"
   - **Action Required:** Update test to match spec

## Coverage Gaps
- `cli.py:102-105` - Invalid command error handling (CRITICAL)
- `todo.py:78-80` - ID collision handling (EDGE CASE)

## Spec Traceability
- 8/10 acceptance criteria have tests (80%)
- Missing tests for AC-005, AC-009

## Recommendations
1. Fix implementation bug in delete_todo() error handling
2. Add test for invalid CLI commands
3. Add tests for AC-005 and AC-009
4. Update test_list_empty_todos to match spec

## Quality Assessment
✅ Tests are deterministic
✅ Tests are independent
✅ No flaky tests detected
⚠️ 2 tests need attention (see above)
```

## Output

**Success case:**
```
✅ TEST EXECUTION COMPLETE: [Feature Name]
📊 Results: 40/42 passed (95%)
📈 Coverage: 87% (exceeds threshold)
⏱️ Duration: 4.2s
⚠️ 2 issues require attention (see report below)

[Full test report]

🔗 Spec Traceability: 8/10 acceptance criteria tested
📋 Next Actions:
   1. Fix delete_todo() error handling
   2. Add test for AC-005, AC-009
   3. Ready for code review after fixes
```

**Failure case (blocking):**
```
❌ TEST EXECUTION FAILED: [Feature Name]
📊 Results: 35/42 passed (83%)
📉 Coverage: 68% (BELOW 80% threshold)
⏱️ Duration: 5.1s
🚫 BLOCKING ISSUES:

1. Coverage below threshold (68% < 80%)
2. 7 test failures (see report)
3. 2 acceptance criteria untested

📋 Required Actions Before Approval:
   1. Fix 7 failing tests
   2. Add tests to reach 80% coverage
   3. Cover AC-005, AC-009 from spec

🔒 IMPLEMENTATION NOT READY FOR MERGE
```

## Failure Handling

### Test Execution Fails
- Capture full error output
- Identify if issue is environmental or code-related
- Check for missing dependencies: `pip install -r requirements.txt`
- Verify Python version compatibility

### Coverage Below Threshold
- Generate detailed coverage report with missing lines
- Prioritize covering critical paths (CRUD, CLI commands)
- Write targeted tests for uncovered code
- Re-run coverage validation

### Flaky Tests Detected
- **IMMEDIATELY FLAG** - Flaky tests are PROHIBITED in Phase 1
- Identify source of non-determinism
- Either fix test to be deterministic or remove it
- Document why test was flaky and how it was fixed

### Tests Don't Match Spec
- Compare test expectations to spec acceptance criteria
- Determine if spec or test is correct
- If spec is correct: update test
- If test is correct: raise spec ambiguity with domain expert
- Never merge code where tests contradict spec

## Quality Gates

**PASS criteria:**
- ✅ All tests pass (0 failures)
- ✅ Coverage ≥ 80% overall
- ✅ Coverage = 100% on critical paths (CRUD, CLI)
- ✅ No flaky or non-deterministic tests
- ✅ All spec acceptance criteria have tests
- ✅ Test quality checks pass

**CONDITIONAL PASS (with documented plan):**
- ⚠️ Coverage 75-79% with plan to reach 80%
- ⚠️ 1-2 non-critical acceptance criteria deferred

**FAIL (blocking):**
- ❌ Any test failures
- ❌ Coverage < 75%
- ❌ Flaky tests present
- ❌ Critical acceptance criteria untested
- ❌ Test quality violations (shared state, non-deterministic)

## Integration with SDD Workflow

**After test execution:**
1. If PASS: Mark task as complete, ready for code review
2. If CONDITIONAL PASS: Create follow-up task for coverage gaps
3. If FAIL: Block implementation, create bug-fix tasks

**PHR Creation:**
- Stage: `red` (if tests fail) or `green` (if tests pass)
- Record: test results, coverage, gaps, actions taken

**Next Steps:**
- PASS → Proceed to code review or commit
- CONDITIONAL → Document plan, proceed with caution
- FAIL → Return to implementation, fix issues

## Examples

### Example 1: Successful Test Run
```
Input: "Run tests for add-task feature"

Output:
✅ TEST EXECUTION COMPLETE: add-task
📊 Results: 15/15 passed (100%)
📈 Coverage: 92%
⏱️ Duration: 2.1s

Critical Paths: 100% covered
Error Paths: 100% covered
Edge Cases: All tested

Spec Traceability: 5/5 acceptance criteria tested
✅ READY FOR CODE REVIEW
```

### Example 2: Tests Pass But Coverage Low
```
Input: "Execute all tests with coverage"

Output:
⚠️ TEST RESULTS: Mixed
📊 Results: 30/30 passed (100%)
📉 Coverage: 73% (BELOW THRESHOLD)

Uncovered Areas:
- cli.py:102-110 (invalid command handling)
- todo.py:45-48 (ID collision edge case)

📋 Required Actions:
1. Add test_invalid_cli_command
2. Add test_id_collision_handling
3. Re-run coverage validation

🔒 BLOCKED until coverage ≥ 80%
```

### Example 3: Test Failures
```
Input: "Run tests for delete-task"

Output:
❌ TEST EXECUTION FAILED: delete-task
📊 Results: 7/10 passed (70%)

Failed Tests:
1. test_delete_nonexistent_id - Expected error, got exception
2. test_delete_with_invalid_id - Wrong error message
3. test_delete_updates_list - List not updated

Root Cause Analysis:
- delete_todo() missing error handling
- In-memory storage not updated after delete
- Error messages don't match spec

📋 Required Fixes:
1. Add try-except for nonexistent ID
2. Update storage.remove() call
3. Match error messages to spec §3.2

🔒 BLOCKED - Return to implementation
```
