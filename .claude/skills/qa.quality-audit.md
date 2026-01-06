---
name: qa.quality-audit
description: Audit code quality, standards compliance, and Phase 1 constraints for Todo App
owner: QA Agent
tags: [quality, audit, standards, phase-1, python]
---

## Purpose

Perform comprehensive code quality audit for Todo App Phase 1 implementation:
- Verify adherence to Python best practices and PEP 8 standards
- Ensure Phase 1 constraints are respected (CLI, in-memory, Python only)
- Check code maintainability and readability
- Identify security vulnerabilities and anti-patterns
- Validate project structure and organization
- Confirm compliance with constitution principles (if defined)

## When to Use

Invoke this skill when:
- Feature implementation is complete and tests pass
- Before creating a pull request or merge
- During code review process
- After fixing bugs (ensure no quality regression)
- Preparing for demo or hackathon judging

**Trigger phrases:**
- "Audit code quality for [feature]"
- "Check code standards"
- "Review code for best practices"
- "Verify Phase 1 compliance"
- "Quality check the implementation"

## Inputs

**Required:**
- `feature_name` - The feature to audit (or "all" for entire codebase)

**Optional:**
- `focus_area` - Specific aspect: "standards", "security", "phase1-compliance", "structure", "all" (default: "all")
- `severity_threshold` - Report issues at level: "info", "warning", "error" (default: "warning")

**Example invocations:**
```
Audit code quality for add-task
Check Phase 1 compliance for entire codebase
Review security issues in delete-task
Quality audit all code with errors only
```

## Step-by-Step Process

### 1. Identify Audit Scope

**Determine files to audit:**
```bash
# For specific feature
git log --name-only --grep="${feature_name}" --pretty=format: | sort -u

# For all Python files
find . -name "*.py" -not -path "./tests/*" -not -path "./venv/*"
```

**Expected file structure for Phase 1:**
```
todo-app/
├── src/
│   ├── main.py          # CLI entry point
│   ├── cli.py           # Command-line interface
│   ├── todo.py          # Todo CRUD operations
│   └── storage.py       # In-memory storage
├── tests/
│   ├── test_cli.py
│   ├── test_todo.py
│   └── test_storage.py
├── requirements.txt     # Dependencies (minimal for Phase 1)
└── README.md
```

### 2. Verify Phase 1 Constraints

**Phase 1 boundaries (CRITICAL):**

✅ **ALLOWED:**
- Python standard library
- CLI argument parsing (argparse, click)
- In-memory data structures (list, dict, set)
- Basic testing frameworks (pytest, unittest)
- Code quality tools (pylint, flake8, black)

❌ **PROHIBITED:**
- Database libraries (SQLite, PostgreSQL, MongoDB, etc.)
- Web frameworks (Flask, Django, FastAPI, etc.)
- File persistence (pickle, JSON files, CSV, etc.)
- External APIs or network calls
- GUI libraries (tkinter, PyQt, etc.)
- Non-Python languages

**Audit steps:**

1. **Check imports for prohibited libraries:**
```python
# Scan all Python files for imports
grep -r "^import\|^from" src/ --include="*.py"
```

**Prohibited import patterns:**
```python
# ❌ DATABASE
import sqlite3
import psycopg2
from sqlalchemy import *
import pymongo

# ❌ WEB FRAMEWORKS
import flask
from django import *
import fastapi

# ❌ FILE PERSISTENCE
import pickle
import json  # OK to use, but NOT for persistence
import csv   # OK to use, but NOT for file I/O

# ❌ GUI
import tkinter
from PyQt5 import *

# ❌ EXTERNAL SERVICES
import requests  # Network calls prohibited
import boto3     # Cloud services prohibited
```

**Allowed import examples:**
```python
# ✅ STANDARD LIBRARY
import sys
import argparse
from dataclasses import dataclass
from typing import List, Dict, Optional

# ✅ CLI TOOLS
import click
import typer

# ✅ TESTING
import pytest
import unittest

# ✅ CODE QUALITY
import pylint
import black
```

2. **Check for file I/O operations:**
```bash
# Search for file operations
grep -r "open(\|with open\|\.write(\|\.read(" src/ --include="*.py"
```

If found, verify it's NOT for persistence:
- ❌ `open("todos.json", "w")` - Persistence prohibited
- ✅ `open("config.py", "r")` - Reading config OK (if needed)

3. **Verify storage is in-memory only:**
```python
# Audit storage.py or equivalent
# Should use ONLY in-memory structures

# ✅ CORRECT
class TodoStorage:
    def __init__(self):
        self.todos = {}  # In-memory dictionary
        self.next_id = 1

# ❌ WRONG
class TodoStorage:
    def __init__(self):
        self.db = sqlite3.connect("todos.db")  # Database prohibited

# ❌ WRONG
class TodoStorage:
    def save(self):
        with open("todos.json", "w") as f:  # File persistence prohibited
            json.dump(self.todos, f)
```

**Phase 1 compliance report:**
```markdown
## Phase 1 Compliance Audit

### Constraints Check
- ✅ Python only (no other languages)
- ✅ CLI interface (no web/GUI)
- ✅ In-memory storage (no databases)
- ✅ No file persistence
- ✅ No external network calls

### Prohibited Libraries: NONE FOUND

### Storage Implementation
- ✅ Uses in-memory dict/list only
- ✅ No database connections
- ✅ No file I/O for persistence
```

### 3. Python Standards Audit (PEP 8)

**Run automated linting:**
```bash
# Using flake8
flake8 src/ --max-line-length=100 --statistics

# Using pylint
pylint src/ --max-line-length=100

# Using black (formatting check)
black --check src/
```

**Manual code review for Python best practices:**

**Naming conventions:**
- [ ] Functions/variables: `snake_case`
- [ ] Classes: `PascalCase`
- [ ] Constants: `UPPER_SNAKE_CASE`
- [ ] Private members: `_leading_underscore`

**Code structure:**
- [ ] Functions ≤ 50 lines (guideline, not strict)
- [ ] Classes have clear single responsibility
- [ ] No deep nesting (max 3-4 levels)
- [ ] Proper spacing (2 blank lines between functions)

**Documentation:**
- [ ] Module-level docstrings present
- [ ] Public functions have docstrings
- [ ] Complex logic has inline comments
- [ ] Docstrings follow conventions (Google, NumPy, or PEP 257)

**Example standards check:**

```python
# ❌ WRONG - Poor naming, no docstring, too complex
def f(x,y):
    if x:
        if y:
            if x>y:
                return True
    return False

# ✅ CORRECT - Clear naming, docstring, simple logic
def is_valid_todo_id(todo_id: int, max_id: int) -> bool:
    """
    Check if todo ID is valid and within range.

    Args:
        todo_id: The ID to validate
        max_id: Maximum allowed ID value

    Returns:
        True if ID is valid, False otherwise
    """
    return todo_id is not None and 0 < todo_id <= max_id
```

**Standards audit report:**
```markdown
## Python Standards Audit

### Linting Results (flake8)
- Total issues: 12
- Errors: 0
- Warnings: 5
- Style: 7

### Top Issues
1. Line too long (3 occurrences) - cli.py:45, 67, 89
2. Missing docstring (2 occurrences) - todo.py:12, storage.py:34
3. Unused variable (1 occurrence) - main.py:23

### Naming Conventions
- ✅ Functions: All snake_case
- ✅ Classes: All PascalCase
- ⚠️ Constants: 2 not UPPER_CASE (todo.py:5, cli.py:8)

### Documentation Coverage
- Module docstrings: 3/4 files (75%)
- Function docstrings: 18/25 functions (72%)
- Recommendation: Add docstrings to undocumented functions
```

### 4. Security Audit

**Phase 1 security concerns:**

Even for simple CLI apps, check for:
- Command injection vulnerabilities
- Input validation and sanitization
- Error handling (no sensitive info leakage)
- No hardcoded secrets (even in comments)

**Security checklist:**

1. **Command Injection:**
```python
# ❌ DANGEROUS - Command injection risk
import os
def delete_todo(title):
    os.system(f"echo Deleted {title}")  # User input in shell command

# ✅ SAFE - No shell execution with user input
def delete_todo(todo_id: int):
    if todo_id in storage:
        del storage[todo_id]
```

2. **Input Validation:**
```python
# ❌ INSUFFICIENT - No validation
def add_todo(title):
    todos[next_id] = title  # What if title is 10MB string?

# ✅ PROPER - Validated input
def add_todo(title: str):
    if not title or not title.strip():
        raise ValueError("Title cannot be empty")
    if len(title) > 200:
        raise ValueError("Title too long (max 200 characters)")
    todos[next_id] = title.strip()
```

3. **Error Handling:**
```python
# ❌ WRONG - Leaks internal details
except Exception as e:
    print(f"Error: {e}")  # Might show stack trace to user

# ✅ CORRECT - User-friendly error
except KeyError:
    print("Error: Todo not found")
except ValueError as e:
    print(f"Error: {e}")  # Only expected errors
```

4. **No Hardcoded Secrets:**
```python
# ❌ WRONG (even if not used in Phase 1)
API_KEY = "sk-1234567890abcdef"  # NEVER hardcode secrets

# ✅ CORRECT
# No secrets needed for Phase 1 (in-memory CLI only)
```

**Security audit report:**
```markdown
## Security Audit

### Command Injection
- ✅ No os.system() or subprocess with user input
- ✅ No eval() or exec() with user input

### Input Validation
- ✅ Title length validated
- ✅ Empty input rejected
- ⚠️ Special character handling not explicit
- Recommendation: Document special char behavior

### Error Handling
- ✅ No stack traces exposed to users
- ✅ Generic error messages for expected failures
- ✅ Specific errors for user mistakes

### Secrets Management
- ✅ No hardcoded secrets found
- ✅ No credentials in code
- N/A for Phase 1 (no external services)

### Overall: ✅ NO CRITICAL SECURITY ISSUES
```

### 5. Code Quality Metrics

**Calculate quality metrics:**

1. **Cyclomatic Complexity:**
```bash
# Using radon
radon cc src/ -a -nb
```

Target: ≤ 10 per function (simple Phase 1 code)
Flag: > 15 (too complex, needs refactoring)

2. **Code Duplication:**
```bash
# Using pylint
pylint --disable=all --enable=duplicate-code src/
```

Target: < 5% duplication
Flag: Repeated code blocks > 10 lines

3. **Maintainability Index:**
```bash
# Using radon
radon mi src/ -nb
```

Target: > 20 (good maintainability)
Flag: < 10 (poor maintainability)

**Quality metrics report:**
```markdown
## Code Quality Metrics

### Complexity Analysis
- Average complexity: 4.2 (LOW - excellent)
- Max complexity: 8 (cli.py:parse_args)
- Functions > 10: 0

### Code Duplication
- Duplication: 3% (acceptable)
- Duplicated blocks: 1 (todo.py:45-52, todo.py:78-85)
- Recommendation: Extract common validation logic

### Maintainability Index
- Average MI: 68 (GOOD)
- Lowest MI: 54 (storage.py - still acceptable)
- All files > 50 (maintainable)

### Function Length
- Average: 12 lines
- Max: 34 lines (cli.py:handle_command)
- Functions > 50 lines: 0
```

### 6. Project Structure Audit

**Verify proper organization:**

```markdown
## Project Structure Audit

### Directory Structure
- ✅ Clear separation of concerns (src/, tests/)
- ✅ No mixed production/test code
- ✅ Logical module organization

### File Organization
- ✅ main.py - Entry point only
- ✅ cli.py - CLI interface logic
- ✅ todo.py - Business logic
- ✅ storage.py - Data management
- ✅ Separation of concerns maintained

### Dependencies
- ✅ requirements.txt present
- ✅ Minimal dependencies (2 packages)
- ✅ No unnecessary packages
- ✅ All Phase 1 compliant

### Documentation
- ✅ README.md present
- ⚠️ No CONTRIBUTING.md (optional for Phase 1)
- ⚠️ No architecture docs (could improve judging)
```

### 7. Anti-Pattern Detection

**Common anti-patterns to flag:**

```python
# ❌ ANTI-PATTERN: God Object
class TodoApp:
    def __init__(self):
        self.storage = []
    def add(self): pass
    def delete(self): pass
    def list(self): pass
    def parse_cli(self): pass
    def format_output(self): pass
    def validate_input(self): pass
    # Too many responsibilities!

# ✅ BETTER: Separation of Concerns
class TodoStorage: ...
class TodoCLI: ...
class TodoFormatter: ...

# ❌ ANTI-PATTERN: Magic Numbers
if len(title) > 200:  # Why 200?

# ✅ BETTER: Named Constants
MAX_TITLE_LENGTH = 200  # Clear, configurable
if len(title) > MAX_TITLE_LENGTH:

# ❌ ANTI-PATTERN: Mutable Default Arguments
def add_todo(title, tags=[]):  # Dangerous!
    tags.append("new")

# ✅ CORRECT: None with initialization
def add_todo(title, tags=None):
    if tags is None:
        tags = []

# ❌ ANTI-PATTERN: Bare Except
try:
    delete_todo(id)
except:  # Catches everything, even KeyboardInterrupt!
    pass

# ✅ CORRECT: Specific Exceptions
try:
    delete_todo(id)
except KeyError:
    print("Todo not found")
```

**Anti-pattern report:**
```markdown
## Anti-Pattern Detection

### Found Issues
1. Mutable default argument (todo.py:34)
   - Severity: ERROR
   - Fix: Use None and initialize inside function

2. Magic number (cli.py:67)
   - Severity: WARNING
   - Value: 200 (max title length)
   - Fix: Extract to named constant

3. Bare except (main.py:45)
   - Severity: ERROR
   - Fix: Catch specific exceptions

### Not Found (Good Practices)
- ✅ No god objects
- ✅ No circular imports
- ✅ No global state mutation
- ✅ No shadowing builtins
```

### 8. Constitution Compliance

**If .specify/memory/constitution.md exists:**

1. Read constitution principles
2. Verify code adheres to stated principles
3. Check for violations

**Example constitution checks:**
```markdown
## Constitution Compliance

### Principle: "Library-First"
- ⚠️ PARTIAL: No clear library structure
- Recommendation: Separate core logic from CLI

### Principle: "Test-First (NON-NEGOTIABLE)"
- ✅ PASS: All features have tests
- ✅ TDD followed (tests before implementation)

### Principle: "Simplicity"
- ✅ PASS: YAGNI respected
- ✅ No premature optimization
- ✅ Simple, readable code
```

### 9. Generate Comprehensive Audit Report

**Final consolidated report:**

```markdown
# Code Quality Audit Report: [Feature Name]

**Audit Date:** 2025-12-24
**Auditor:** QA Agent
**Scope:** [Feature/All Code]

## Executive Summary
- **Overall Quality:** GOOD ✅
- **Phase 1 Compliance:** FULL ✅
- **Security:** NO CRITICAL ISSUES ✅
- **Standards:** MINOR ISSUES ⚠️
- **Recommendation:** APPROVED with minor fixes

---

## Phase 1 Compliance ✅
- ✅ Python only
- ✅ CLI interface
- ✅ In-memory storage
- ✅ No prohibited dependencies
- ✅ No file persistence

## Python Standards ⚠️
**Grade: B+ (87/100)**

Issues:
- 3 lines exceed 100 chars
- 2 missing docstrings
- 7 style warnings

All issues are MINOR and non-blocking.

## Security ✅
**Grade: A (95/100)**

- ✅ No command injection risks
- ✅ Input validation present
- ✅ No hardcoded secrets
- ⚠️ One bare except clause (minor)

## Code Quality ✅
**Grade: A (92/100)**

Metrics:
- Complexity: 4.2 avg (excellent)
- Duplication: 3% (good)
- Maintainability: 68 (good)

## Anti-Patterns ⚠️
**Found:** 3 issues
1. Mutable default arg (ERROR)
2. Magic number (WARNING)
3. Bare except (ERROR)

**Impact:** LOW (easy fixes)

## Project Structure ✅
- ✅ Well-organized
- ✅ Clear separation
- ✅ Minimal dependencies

---

## Required Fixes (Before Merge)
1. Fix mutable default argument (todo.py:34)
2. Fix bare except clause (main.py:45)

## Recommended Improvements (Optional)
1. Add missing docstrings
2. Extract magic numbers to constants
3. Fix line length violations
4. Reduce code duplication in validation

---

## Overall Assessment

**APPROVED FOR MERGE** with required fixes

This implementation demonstrates:
- ✅ Strong Phase 1 compliance
- ✅ Good code quality practices
- ✅ Secure input handling
- ✅ Maintainable structure

After fixing 2 required issues, code will be production-ready for Phase 1 demo.

**Estimated fix time:** 15 minutes
**Re-audit required:** No (issues are straightforward)
```

## Output

**High quality code:**
```
✅ QUALITY AUDIT COMPLETE: [Feature Name]

📊 Overall Grade: A (92/100)

✅ Phase 1 Compliance: FULL
✅ Security: NO CRITICAL ISSUES
✅ Standards: MINOR ISSUES ONLY
✅ Quality Metrics: EXCELLENT

Required Fixes: 0
Recommended Improvements: 3 (optional)

🎯 APPROVED FOR MERGE
🏆 HIGH QUALITY IMPLEMENTATION

[Link to full audit report]
```

**Good quality with minor issues:**
```
⚠️ QUALITY AUDIT: [Feature Name]

📊 Overall Grade: B+ (87/100)

✅ Phase 1 Compliance: FULL
✅ Security: NO CRITICAL ISSUES
⚠️ Standards: 3 minor violations
✅ Quality Metrics: GOOD

Required Fixes: 2
- Mutable default argument
- Bare except clause

Recommended: 5 improvements (optional)

🎯 APPROVED with required fixes
⏱️ Est. fix time: 15 minutes

[Link to full audit report]
```

**Blocking quality issues:**
```
❌ QUALITY AUDIT FAILED: [Feature Name]

📊 Overall Grade: C (72/100)

❌ Phase 1 Compliance: VIOLATED (file persistence detected)
❌ Security: 2 CRITICAL ISSUES
⚠️ Standards: 12 violations
⚠️ Quality Metrics: POOR (complexity 18)

BLOCKING ISSUES:
1. File I/O detected (violates Phase 1)
2. Command injection vulnerability (CRITICAL)
3. SQL injection risk (CRITICAL)

🚫 CANNOT MERGE
📋 Required: Major refactoring
🔒 Re-audit required after fixes

[Link to full audit report]
```

## Failure Handling

### Phase 1 Violations Detected
- **BLOCK IMMEDIATELY**
- Document violation (what, where, why prohibited)
- Provide compliant alternative
- Require removal before proceeding

### Critical Security Issues
- **BLOCK IMMEDIATELY**
- Escalate to security review
- Provide secure alternative
- Require fix with verification

### Standards Violations
- Minor (style): Can merge with improvements recommended
- Major (anti-patterns): Should fix before merge
- Critical (mutable defaults, bare except): Must fix before merge

### Poor Quality Metrics
- High complexity (>15): Recommend refactoring
- High duplication (>10%): Recommend DRY refactoring
- Low maintainability (<20): Consider redesign

## Quality Gates

**PASS (approved for merge):**
- ✅ Full Phase 1 compliance
- ✅ No critical security issues
- ✅ No critical anti-patterns
- ✅ Standards violations ≤ minor
- ✅ Quality metrics in good range

**CONDITIONAL (approved with fixes):**
- ✅ Phase 1 compliant
- ⚠️ 1-2 non-critical security issues
- ⚠️ 1-3 fixable anti-patterns
- ⚠️ Some standards violations
- ⚠️ Acceptable quality metrics

**FAIL (blocking):**
- ❌ Phase 1 violations
- ❌ Critical security issues
- ❌ Multiple critical anti-patterns
- ❌ Poor quality metrics (complexity >20, MI <10)

## Integration with SDD Workflow

**After audit:**
1. If PASS: Approve for merge
2. If CONDITIONAL: Create fix tasks, re-audit after
3. If FAIL: Block merge, major refactoring needed

**PHR Creation:**
- Stage: `refactor` (if improvements needed) or `green` (if passed)
- Record: audit results, issues found, recommendations

**For hackathon judging:**
- High quality code demonstrates professionalism
- Clean audits show attention to detail
- Compliance shows understanding of requirements
