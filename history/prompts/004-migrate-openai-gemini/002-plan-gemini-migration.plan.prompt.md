---
id: 002
title: Plan Gemini Migration
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: 004-migrate-openai-gemini
branch: 004-migrate-openai-gemini
user: MUHAMMAD
command: /sp.plan
labels: ["plan", "gemini", "migration", "cost-optimization"]
links:
  spec: specs/004-migrate-openai-gemini/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/004-migrate-openai-gemini/plan.md
tests:
 - (none - planning stage)
---

## Prompt

$ARGUMENTS

## Response snapshot

Created implementation plan for OpenAI to Gemini migration:

**Key Findings**:
- OpenAI used in 5 files: requirements.txt, llm.py, config.py, .env.example, PHASE3 docs
- Main changes needed:   1. requirements.txt: Remove openai, add google-generativeai
  2. .env: Complete rewrite with real Qdrant + Gemini credentials
  3. config.py: Replace openai_api_key with gemini_api_key
  4. llm.py: Complete rewrite to use Gemini API

**API Mapping**:
- OpenAI: `client.chat.completions.create(messages=[...])`
- Gemini: `model.generate_content(single_prompt_string)`
- Key difference: Gemini uses concatenated prompts, not separate system/user messages

**Implementation Strategy**:
- 4 files to modify
- Complete rewrite of llm.py (~158 lines)
- Update config.py (3 lines removed, 3 added)
- Update requirements.txt (1 deletion, 1 addition)
- Rewrite .env with real credentials provided by user

**Constitution Check**: PASSED ✅
- Strengthens free-tier compliance (OpenAI paid → Gemini free)
- No new architectural complexity
- Fast iteration (4 file changes, <10 min testing)
- Environment variables for credentials

## Outcome

- ✅ Impact: Complete migration plan with file-by-file changes documented
- 🧪 Tests: Manual API test strategy defined
- 📁 Files: plan.md (180 lines)
- 🔁 Next prompts: `/sp.tasks` to generate implementation tasks
- 🧠 Reflection: Straightforward dependency swap - main complexity in llm.py rewrite for different API pattern

## Evaluation notes (flywheel)

- Failure modes observed: None - planning successful
- Graders run and results (PASS/FAIL): Constitution check PASSED
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Generate tasks and execute implementation
