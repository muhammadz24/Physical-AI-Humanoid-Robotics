# Feature Specification: Migrate from OpenAI to Google Gemini

**Feature Branch**: `004-migrate-openai-gemini`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Migrate from OpenAI to Google Gemini (Free Tier) for cost optimization - Replace OpenAI dependency with google-generativeai, use gemini-1.5-flash model, and update environment configuration"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Chat Query with Gemini LLM (Priority: P1)

Users send chat queries to the RAG chatbot, which retrieves context from the vector store and generates responses using Google Gemini instead of OpenAI.

**Why this priority**: This is the core functionality replacement. The chatbot must continue to work after migrating from paid OpenAI API to free Gemini API, maintaining quality while reducing costs to zero.

**Independent Test**: Can be fully tested by sending a chat message (e.g., "What is ROS 2?") to the backend API and verifying a 200 OK response with an AI-generated answer using Gemini.

**Acceptance Scenarios**:

1. **Given** the backend is configured with Gemini API credentials, **When** a user sends a chat query, **Then** the system retrieves context and generates a response using Gemini
2. **Given** the vector store returns relevant chunks, **When** Gemini generates a response, **Then** the answer is grounded in the provided context
3. **Given** Gemini API is available, **When** multiple concurrent queries arrive, **Then** all responses are generated within acceptable latency (<5 seconds)
4. **Given** the backend restarts, **When** the system initializes, **Then** no OpenAI dependencies are loaded and Gemini is configured correctly

---

### Edge Cases

- What happens when Gemini API returns an error or times out?
- How does the system handle Gemini's free-tier rate limits?
- What happens when the generated response exceeds expected length?
- How does the system respond when Gemini returns low-confidence answers?
- What happens if API credentials are missing or invalid?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST use Google Gemini API instead of OpenAI for chat response generation
- **FR-002**: System MUST use the `gemini-1.5-flash` model for fast, free-tier inference
- **FR-003**: System MUST load Gemini credentials from environment variables (API key)
- **FR-004**: System MUST maintain the same chat API contract (no frontend changes required)
- **FR-005**: System MUST generate responses grounded in retrieved context chunks from RAG pipeline
- **FR-006**: System MUST handle Gemini API errors gracefully with informative error messages
- **FR-007**: System MUST remove all OpenAI dependencies from codebase and requirements
- **FR-008**: System MUST update environment configuration with Qdrant Cloud credentials

### Key Entities

- **Chat Request**: User query with optional parameters (top_k, chapter_filter)
- **Context Chunks**: Retrieved text segments from vector store
- **Gemini Prompt**: Formatted prompt combining user query and context
- **Chat Response**: AI-generated answer with citations and metadata

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive AI-generated responses from Gemini with 200 OK status
- **SC-002**: Chat response latency remains under 5 seconds for typical queries (p95)
- **SC-003**: Zero OpenAI API costs after migration (100% cost reduction)
- **SC-004**: Response quality matches or exceeds OpenAI baseline (subjective evaluation with sample queries)
- **SC-005**: Backend starts successfully with Gemini configured (no initialization errors)
- **SC-006**: System handles at least 15 requests per minute (Gemini free-tier limit)

## Assumptions

- Gemini free tier provides sufficient rate limits for expected traffic (15 RPM)
- Gemini 1.5 Flash model quality is acceptable for educational chatbot use case
- Existing prompt engineering patterns work with Gemini's API
- Environment has network access to Google AI services
- Qdrant Cloud credentials are valid and vector store is accessible

## Dependencies

- `google-generativeai` Python library (replaces `openai`)
- Google Gemini API credentials (API key)
- Qdrant Cloud credentials (URL and API key)
- Existing RAG pipeline (embedding service, vector store)

## Out of Scope

- Changing the embedding model (remains sentence-transformers)
- Modifying the vector store or re-embedding data
- Updating the frontend chat widget
- Implementing prompt caching or optimization
- Adding fallback to other LLM providers
- Implementing streaming responses
- Fine-tuning or customizing the Gemini model

## Cost Benefit Analysis

**Before (OpenAI)**:
- GPT-3.5-turbo: ~$0.002 per 1K tokens (input + output)
- Estimated cost for 1000 queries: ~$2-5

**After (Gemini)**:
- gemini-1.5-flash: Free tier (no cost)
- Estimated cost for 1000 queries: $0

**Savings**: 100% cost reduction while maintaining functionality
