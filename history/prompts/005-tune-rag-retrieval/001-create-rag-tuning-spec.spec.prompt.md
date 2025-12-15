---
id: 001
title: Create RAG Retrieval Tuning Specification
stage: spec
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: 005-tune-rag-retrieval
branch: 005-tune-rag-retrieval
user: MUHAMMAD
command: /sp.specify
labels: ["spec", "rag", "retrieval", "tuning", "bug-fix"]
links:
  spec: specs/005-tune-rag-retrieval/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/005-tune-rag-retrieval/spec.md
 - specs/005-tune-rag-retrieval/checklists/requirements.md
tests:
 - (none - specification stage)
---

## Prompt

The RAG Chatbot returns "I couldn't find relevant information" even for simple queries like "What is Physical AI?". The data is fully ingested in Qdrant. This indicates that the score_threshold in the vector search is TOO HIGH, causing relevant chunks to be filtered out. Lower the score_threshold to 0.30 or 0.40 and add DEBUG logging to print actual similarity scores.

## Response snapshot

Created comprehensive specification for tuning RAG retrieval threshold and adding debug logging:

**Spec Overview**:
- Feature: Lower score_threshold from 0.5 to 0.35 to retrieve more relevant chunks
- Branch: 005-tune-rag-retrieval
- Primary User Story: Get Answer for Basic Questions (P1)
- Secondary User Story: Debug Retrieval Issues (P2)

**Key Requirements Defined**:
- FR-001: Lower vector search score_threshold from 0.5 to 0.35
- FR-002: Retrieve chunks with similarity scores >= 0.35
- FR-003: Log each retrieved chunk's similarity score to terminal
- FR-004: Debug logs include chunk ID and score format
- FR-005: Maintain "no results" behavior for scores < 0.35
- FR-006: Preserve all other RAG pipeline functionality

**Success Criteria**:
- SC-001: Queries with similarity 0.35-0.50 now return answers (previously failed)
- SC-002: "What is Physical AI?" returns answer with confidence > 0.6 and 3-5 citations
- SC-003: Debug logs show similarity scores for all retrieved chunks
- SC-004: Zero false negatives for topics in Chapters 1-13 (20 query test)
- SC-005: Response time stays under 5 seconds

**Problem Analysis**:
- Current threshold: 0.5 (too strict)
- New threshold: 0.35 (more inclusive)
- Impact: Retrieves chunks with moderate similarity (0.35-0.50) that were previously filtered out
- Debug logging enables ongoing monitoring and tuning

**Dependencies**:
- Qdrant collection with 382 chunks already ingested
- Backend running with Gemini 2.5 Flash
- Embedding service loaded with sentence-transformers

## Outcome

- ✅ Impact: Spec defines threshold tuning (0.5 → 0.35) and debug logging for retrieval transparency
- 🧪 Tests: Acceptance scenarios for basic queries, edge cases, and debug log verification
- 📁 Files: spec.md (153 lines), requirements.md checklist (passed all validation)
- 🔁 Next prompts: `/sp.plan` to locate score_threshold parameter and plan implementation
- 🧠 Reflection: Clear root cause (threshold too high) leads to focused fix with observability improvement

## Evaluation notes (flywheel)

- Failure modes observed: None - spec creation successful
- Graders run and results (PASS/FAIL): Checklist validation PASSED (all items checked)
- Prompt variant (if applicable): Standard /sp.specify workflow with clear problem statement
- Next experiment (smallest change to try): Proceed to planning with /sp.plan to locate threshold in routes.py or vector_store.py
