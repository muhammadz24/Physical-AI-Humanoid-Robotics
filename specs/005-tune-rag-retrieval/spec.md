# Feature Specification: Tune RAG Retrieval Threshold

**Feature Branch**: `005-tune-rag-retrieval`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "The RAG Chatbot returns 'I couldn't find relevant information' even for simple queries like 'What is Physical AI?'. The data is fully ingested in Qdrant. This indicates that the score_threshold in the vector search is TOO HIGH, causing relevant chunks to be filtered out. Lower the score_threshold to 0.30 or 0.40 and add DEBUG logging to print actual similarity scores."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Get Answer for Basic Questions (Priority: P1)

As a user of the RAG chatbot, I ask a basic question like "What is Physical AI?" that has clear matches in the ingested textbook content, and I receive a relevant answer with proper citations instead of "I couldn't find relevant information".

**Why this priority**: This is the core functionality of the chatbot. If users can't get answers to basic questions that clearly exist in the content, the entire system fails its primary purpose.

**Independent Test**: Can be fully tested by sending POST requests to `/api/chat` with queries known to exist in the textbook (e.g., "What is Physical AI?", "What are the three components?") and verifying that answers are returned with confidence > 0 and citations present.

**Acceptance Scenarios**:

1. **Given** the textbook content is ingested in Qdrant, **When** user asks "What is Physical AI?", **Then** the system returns a relevant answer with confidence score > 0 and 3-5 citations from Chapter 1
2. **Given** the textbook content is ingested, **When** user asks about any topic covered in Chapters 1-13, **Then** the system retrieves and returns matching chunks instead of "no results" message
3. **Given** a query with moderate relevance (similarity ~0.4-0.6), **When** the search is performed, **Then** those chunks are included in context for LLM generation

---

### User Story 2 - Debug Retrieval Issues (Priority: P2)

As a developer troubleshooting the RAG system, I can see the actual similarity scores of retrieved chunks in the terminal logs, allowing me to understand why certain queries succeed or fail.

**Why this priority**: Essential for system maintenance and tuning. Without visibility into similarity scores, it's impossible to diagnose retrieval problems or optimize the threshold further.

**Independent Test**: Can be tested by monitoring terminal output while sending queries. Logs should show each retrieved chunk with its similarity score (e.g., "DEBUG: Found chunk with score: 0.745").

**Acceptance Scenarios**:

1. **Given** a query is sent to the chatbot, **When** vector search retrieves chunks, **Then** each chunk's similarity score is logged to the terminal in the format "DEBUG: Found chunk 'ch01-002' with score: 0.745"
2. **Given** debug logging is enabled, **When** reviewing logs after a failed query, **Then** I can see which chunks were retrieved (if any) and their exact similarity scores
3. **Given** the threshold is adjusted, **When** testing queries, **Then** I can observe how score distribution changes and verify the threshold is appropriate

---

### Edge Cases

- What happens when a query has NO chunks above the new 0.35 threshold? (System should still return "I couldn't find relevant information" with retrieved_chunks: 0)
- What happens when a query matches many chunks with scores between 0.35-0.50? (System should retrieve up to top_k chunks and generate answer)
- How does the system handle queries with very high similarity (>0.9) vs moderate similarity (0.35-0.5)? (Both should succeed, but confidence scores will reflect the difference)

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST lower the vector search score_threshold from current value (0.5) to 0.35
- **FR-002**: System MUST retrieve chunks with similarity scores >= 0.35 for answer generation
- **FR-003**: System MUST log each retrieved chunk's similarity score to the terminal during vector search
- **FR-004**: Debug logs MUST include chunk ID and score in the format: "DEBUG: Found chunk 'chunk_id' with score: X.XXX"
- **FR-005**: System MUST maintain existing behavior for queries with no results (score < 0.35) - return "I couldn't find relevant information" message
- **FR-006**: System MUST preserve all other RAG pipeline functionality (embedding generation, LLM response, citations)

### Key Entities

- **Score Threshold**: The minimum similarity score (0.35) required for a chunk to be considered relevant and included in context
- **Similarity Score**: A float value (0.0-1.0) representing how closely a chunk matches the query vector, calculated by Qdrant's cosine similarity
- **Debug Log Entry**: Terminal output showing chunk ID and similarity score for each retrieved chunk

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users can successfully get answers for queries with similarity scores between 0.35-0.50 (previously failing with 0.5 threshold)
- **SC-002**: Query "What is Physical AI?" returns a relevant answer with confidence > 0.6 and 3-5 citations from Chapter 1
- **SC-003**: Debug logs show similarity scores for all retrieved chunks during every query
- **SC-004**: Zero false negatives for queries about topics clearly covered in Chapters 1-13 (measured by manual testing of 20 representative queries)
- **SC-005**: System performance remains unchanged - response time stays under 5 seconds for typical queries

## Dependencies & Assumptions *(optional)*

### Dependencies

- Qdrant collection `textbook_embeddings` must be fully populated with 382 chunks
- Backend must be running with Gemini 2.5 Flash configured
- Embedding service must be loaded with `sentence-transformers/all-MiniLM-L6-v2`

### Assumptions

- The score_threshold parameter exists in the codebase (likely in `backend/app/api/routes.py` or `backend/app/core/vector_store.py`)
- Cosine similarity scores from Qdrant range from 0.0 (no match) to 1.0 (perfect match)
- A threshold of 0.35 is appropriate for this embedding model and content type (may need further tuning based on observed results)
- Terminal logging is visible during uvicorn server execution
- The current "no results" behavior is triggered when search_results is empty or all scores are below threshold

## Out of Scope *(optional)*

- Changing the embedding model or dimension
- Modifying the chunking strategy or chunk size
- Adjusting the top_k parameter (number of chunks retrieved)
- Implementing dynamic threshold adjustment based on query type
- Adding database persistence for debug logs
- Creating a UI for threshold configuration
- Reingesting or reprocessing the textbook content
