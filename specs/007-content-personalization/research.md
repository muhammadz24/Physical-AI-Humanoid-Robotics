# Research: Content Personalization

**Feature**: 007-content-personalization
**Date**: 2025-12-16
**Phase**: 0 (Research & Discovery)

## Overview

This document captures technical research and decisions made during the planning phase for content personalization functionality.

## Research Areas

### 1. LLM Prompt Engineering for Educational Content

**Question**: How to instruct Gemini to adapt content for different experience levels while preserving accuracy?

**Research Findings**:
- **Best Practice**: Use explicit guidelines in system prompt with concrete examples
- **Structure**: Role (educational content adapter) + Context (user experience levels) + Guidelines (specific instructions per level) + Constraints (preserve structure, maintain accuracy)
- **Markdown Preservation**: Instruct LLM to maintain code blocks, headings, lists
- **Accuracy Safeguard**: Explicitly state "do not simplify to the point of incorrectness"

**Decision**: Use structured prompt template with experience-level specific guidelines

**Rationale**: Gemini performs better with explicit instructions rather than implicit adaptation. Clear guidelines reduce hallucination risk.

**Alternatives Considered**:
- Few-shot prompting with examples - Rejected: increases token usage significantly
- Separate prompts per level - Rejected: harder to maintain consistency
- Fine-tuning custom model - Rejected: requires significant data and resources

---

### 2. Content Chunking Strategy

**Question**: How to handle chapters that exceed Gemini's context window?

**Research Findings**:
- Gemini 2.5 Flash context limit: 32K tokens (~24K words)
- Average chapter length: 2000-5000 words (well within limits)
- Longest anticipated chapter: ~8000 words (still within limits)

**Decision**: No chunking required for POC. Send full chapter content in single request.

**Rationale**: All chapters fit comfortably within Gemini context limits. Chunking adds complexity without current need.

**Future Consideration**: If chapters exceed 20K words, implement sliding window chunking with context overlap.

---

### 3. Error Handling Strategies

**Question**: How to gracefully handle LLM failures, timeouts, and quota exhaustion?

**Research Findings**:
- Common failure modes: API timeout, quota exceeded, malformed response, network error
- User expectation: Clear error message + ability to retry
- Best practice: Preserve original content on any error

**Decision**: Implement comprehensive error handling with specific HTTP status codes

**Error Mapping**:
- `401 Unauthorized`: User not authenticated
- `400 Bad Request`: Invalid input (missing chapter text, empty content)
- `503 Service Unavailable`: LLM timeout, quota exceeded, Gemini API down
- `500 Internal Server Error`: Unexpected errors (logged for debugging)

**Rationale**: Users understand what went wrong and can take appropriate action. Original content always remains accessible.

---

### 4. Caching Strategy

**Question**: Should personalized content be cached? If so, where and for how long?

**Research Findings**:
- **Session-level caching**: Stored in component state, cleared on unmount
- **LocalStorage**: Persists across sessions but limited to 5-10MB
- **Database**: Permanent storage but adds complexity and storage costs

**Decision**: Session-level caching only (React component state)

**Rationale**:
- Aligns with Principle V (Minimalism) - no additional storage infrastructure
- Users can easily regenerate personalized content if they reload
- Avoids staleness issues (if chapter content is updated)
- No privacy concerns (data cleared on tab close)

**Out of Scope for POC**: Database persistence (future enhancement if user demand is high)

---

### 5. Rate Limiting

**Question**: How to prevent abuse of the personalization endpoint (Gemini quota protection)?

**Research Findings**:
- Gemini free tier: 15 requests/minute, 1500 requests/day
- Expected usage: 1-5 personalizations per user per session
- Attack vector: Malicious users could exhaust quota

**Decision**: Implement basic rate limiting (10 requests/hour per user)

**Implementation**: Use SlowAPI (already in dependencies) or simple in-memory counter with user_id key

**Rationale**: Protects Gemini quota without requiring complex infrastructure. 10/hour is generous for legitimate use (users won't personalize the same chapter 10 times).

---

### 6. Content Extraction from Docusaurus

**Question**: How to extract chapter content from Docusaurus MDX pages?

**Research Findings**:
- **Option A**: Server-side MDX parsing (read source files directly)
- **Option B**: Client-side DOM extraction (capture rendered HTML)
- **Option C**: Hybrid (use Docusaurus content plugin API)

**Decision**: Client-side DOM extraction for POC

**Rationale**:
- Simplest implementation (no server-side file I/O)
- Works with rendered content (users see what they're personalizing)
- Uses standard DOM APIs (document.querySelector, textContent)

**Implementation**: PersonalizeButton component extracts text from parent article element

**Alternative for Production**: Server-side MDX parsing for better performance and consistency

---

### 7. UI/UX Design Patterns

**Question**: What's the best UX for personalization toggle?

**Research Findings**:
- Users need to compare original vs personalized content
- Toggle should be instant (no API re-call)
- Clear visual indication of which version is displayed

**Decision**: Two-state toggle button ("Show Original" / "Show Personalized")

**UX Flow**:
1. Initial: "Personalize This Chapter" button (primary CTA)
2. Loading: Button disabled, spinner + "Personalizing content..."
3. Personalized: Two buttons side-by-side ("Show Original" | "Show Personalized")
4. Toggle: Swap content div innerHTML (instant, cached in state)

**Rationale**: Simple, intuitive, commonly used pattern (e.g., GitHub's "Rendered" / "Source" tabs)

---

## Technology Stack Decisions

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| LLM Service | Google Gemini | 2.5 Flash | Already integrated, free tier, fast |
| Backend Framework | FastAPI | 0.109.0 | Existing infrastructure |
| Frontend | React + Docusaurus | v3 | Existing infrastructure |
| State Management | React useState | Built-in | No redux needed for local state |
| API Client | Fetch API | Native | Already used in auth flows |
| Prompt Engineering | Template Strings | Native | No specialized library needed |

**New Dependencies**: None - all required technology already in place

---

## Open Questions (Resolved)

1. **Q**: Should we use streaming responses for long personalizations?
   **A**: No - adds complexity, and <10s response time is acceptable for POC

2. **Q**: Should we validate LLM response format?
   **A**: Yes - basic markdown structure validation (ensure headings intact)

3. **Q**: Should we log personalization requests for analytics?
   **A**: Yes - simple log (user_id, timestamp, success/failure) for monitoring

4. **Q**: Should we support undo/redo for personalization?
   **A**: Out of scope - toggle between original and personalized is sufficient

---

## Risk Mitigation

| Risk | Mitigation Strategy |
|------|---------------------|
| Gemini quota exhaustion | Rate limiting (10 req/hr/user) + monitoring |
| Poor personalization quality | Iterate on prompt template based on user feedback |
| Long response times | 30s timeout + clear loading indicator |
| Content staleness | Session-only cache (re-fetch on reload) |
| User confusion | Clear button labels + help text |

---

## Next Steps

1. Proceed to Phase 1: Design (data models, API contracts)
2. Create PersonalizationRequest/Response Pydantic models
3. Write OpenAPI spec for POST /api/personalize
4. Generate tasks.md for implementation

---

## References

- [Gemini API Documentation](https://ai.google.dev/docs)
- [Docusaurus MDX](https://docusaurus.io/docs/markdown-features)
- [FastAPI Best Practices](https://fastapi.tiangolo.com/tutorial/)
- [React Hooks Patterns](https://react.dev/reference/react/hooks)
