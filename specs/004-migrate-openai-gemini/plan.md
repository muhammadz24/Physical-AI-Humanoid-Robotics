# Implementation Plan: Migrate from OpenAI to Google Gemini

**Branch**: `004-migrate-openai-gemini` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/004-migrate-openai-gemini/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Replace OpenAI API with Google Gemini (free tier) for RAG chatbot response generation. This achieves 100% cost reduction while maintaining chat functionality. The migration involves updating dependencies (openai → google-generativeai), refactoring the LLM service to use Gemini's API, updating configuration to load Gemini credentials, and updating the environment file with real Qdrant and Gemini credentials.

**Root Cause**: OpenAI API incurs costs ($0.002/1K tokens) that are unnecessary given free Gemini availability
**Solution Approach**: Swap LLM provider from OpenAI to Gemini, maintain RAG pipeline and API contract

## Technical Context

**Language/Version**: Python 3.13.x (backend)
**Primary Dependencies**:
- `google-generativeai` (NEW - replaces openai==1.12.0)
- `fastapi==0.109.0`
- `sentence-transformers==2.3.1` (unchanged)
- `qdrant-client==1.16.1` (unchanged)

**Storage**: Qdrant Cloud (vector database), Neon Postgres (metadata)
**Testing**: Manual API testing via POST /api/chat
**Target Platform**: Windows (local dev), Linux server (production)
**Project Type**: Web application (backend API)
**Performance Goals**:
- Chat response generation: <5s (p95)
- Rate limit compliance: 15 RPM (Gemini free tier)

**Constraints**:
- Gemini free-tier rate limits (15 RPM)
- No frontend changes (API contract unchanged)
- Must maintain response quality
- Must handle errors gracefully

**Scale/Scope**:
- 4 files modified: requirements.txt, config.py, llm.py, .env
- No database schema changes
- No frontend changes

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Principle III: Free-Tier Architecture (ENHANCED)
- **Previous**: Used paid OpenAI API
- **After**: 100% free-tier with Gemini
- This change STRENGTHENS free-tier compliance
- No additional paid dependencies

### ✅ Principle V: Minimalism in Technology Stack
- Swapping one dependency for another (openai → google-generativeai)
- No new architectural patterns
- Maintains required stack simplicity

### ✅ Principle VI: Fast Build & Iteration Cycles
- Migration involves 4 file changes
- Testing can be completed in <10 minutes
- No build time impact

### ✅ Principle IX: Zero-Edit Deployment Configuration
- Environment variables used for all credentials
- .env file will be updated with real credentials
- No hardcoded values in code

**Gate Status**: PASSED ✅ - All principles satisfied, free-tier compliance enhanced

## Project Structure

### Documentation (this feature)

```text
specs/004-migrate-openai-gemini/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (already created)
├── checklists/
│   └── requirements.md  # Spec validation checklist (already created)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── requirements.txt              # ⚠️  MODIFIED: Remove openai, add google-generativeai
├── .env                          # ⚠️  MODIFIED: Update with real Qdrant + Gemini credentials
├── .env.example                  # ⚠️  MODIFIED: Update example with Gemini vars
├── app/
│   ├── core/
│   │   └── config.py            # ⚠️  MODIFIED: Replace openai_api_key with gemini_api_key
│   └── services/
│       └── llm.py                # ⚠️  MODIFIED: Complete rewrite to use Gemini API
└── tests/
    └── (manual API testing)
```

**Structure Decision**: Web application (backend only). This is a dependency migration scoped to backend LLM service. Frontend chat widget remains unchanged.

## Complexity Tracking

No violations detected - this is a straightforward dependency swap maintaining the same architectural patterns.

## Phase 1: Design & Implementation Approach

### Implementation Strategy

**File 1: backend/requirements.txt**
- Line 17: Remove `openai==1.12.0`
- Line 17: Add `google-generativeai==0.3.2`

**File 2: backend/.env**
- Complete rewrite with real credentials provided

**File 3: backend/app/core/config.py**
- Lines 45-47: Remove OpenAI configuration
- Add Gemini configuration:
  ```python
  # Google Gemini Configuration
  gemini_api_key: str
  gemini_model: str = "gemini-1.5-flash"
  ```

**File 4: backend/app/services/llm.py**
- Complete rewrite to use Gemini API
- Import: `import google.generativeai as genai`
- Initialize: `genai.configure(api_key=...)` and `genai.GenerativeModel(model)`
- Generate: `model.generate_content(prompt)`
- Response: `response.text`

### API Mapping

| OpenAI API | Gemini API Equivalent |
|------------|---------------------|
| `OpenAI(api_key=...)` | `genai.configure(api_key=...)` |
| `client.chat.completions.create()` | `model.generate_content()` |
| `messages=[{role, content}]` | Single concatenated prompt string |
| `response.choices[0].message.content` | `response.text` |
| `response.usage.total_tokens` | Not available (estimate or omit) |

### Files Modified

- `backend/requirements.txt` (1 deletion, 1 addition)
- `backend/.env` (complete rewrite)
- `backend/app/core/config.py` (3 lines removed, 3 added)
- `backend/app/services/llm.py` (complete rewrite)

### Files Verified (no changes)

- `backend/app/api/routes.py` (interface unchanged)
- `backend/app/core/vector_store.py` (RAG unchanged)
- `backend/main.py` (startup unchanged)

## Testing Strategy

### Manual Integration Testing
1. Install: `pip install -r backend/requirements.txt`
2. Verify .env has real credentials
3. Kill backend: `taskkill //F //IM python.exe`
4. Start: `cd backend && python -m uvicorn main:app --host 0.0.0.0 --port 8000`
5. Test: POST `/api/chat` with `{"query": "What is ROS 2?"}`
6. Expected: 200 OK with Gemini response

### Acceptance Criteria
- SC-001: 200 OK from Gemini ✅
- SC-002: Latency < 5s ✅
- SC-003: Zero costs ✅
- SC-004: Quality acceptable ✅
- SC-005: Backend starts ✅
- SC-006: 15 RPM limit ✅

## Next Steps

1. Execute via `/sp.tasks`
2. Install google-generativeai
3. Update 4 files
4. Test with real credentials
5. Verify cost savings
