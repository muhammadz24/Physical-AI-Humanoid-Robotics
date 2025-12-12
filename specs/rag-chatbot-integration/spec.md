# Feature Specification: RAG Chatbot Integration

**Feature Branch**: `rag-chatbot-integration`
**Created**: 2025-12-10
**Status**: Draft
**Dependencies**: `textbook-generation` (content must exist first)

## Overview

Integrate a Retrieval-Augmented Generation (RAG) chatbot into the existing Docusaurus textbook to provide students with instant, accurate answers grounded exclusively in textbook content. The system will support general Q&A and contextual "Select-to-Ask" functionality while adhering to free-tier constraints and constitutional principles.

## Tech Stack

### Backend Services
- **API Framework**: FastAPI (Python 3.10+)
  - Async request handling
  - OpenAPI documentation auto-generation
  - CORS middleware for cross-origin requests
  - Rate limiting (10 requests/minute per IP)

- **Embedding Generation**: Sentence Transformers
  - Model: `all-MiniLM-L6-v2` (384 dimensions)
  - Offline inference (no GPU required)
  - Average encoding time: <100ms per query

- **Vector Database**: Qdrant Cloud (Free Tier)
  - Storage: 1GB limit
  - Vectors: <1000 chunks (estimated 600-800 for 13 chapters)
  - Collection: `textbook_embeddings`
  - Distance metric: Cosine similarity

- **Relational Database**: Neon Serverless Postgres (Free Tier)
  - Storage: 512MB limit
  - Tables: `chat_sessions`, `user_feedback`, `query_analytics`
  - Connection pooling: 10 concurrent connections max

### Frontend Integration
- **UI Framework**: React 18.x (Docusaurus dependency)
  - Custom React component: `<ChatWidget />`
  - Docusaurus theme integration via `swizzling` (if needed)
  - State management: React useState + useContext

- **Styling**: CSS Modules (Docusaurus-native)
  - Dark mode support via Docusaurus theme variables
  - Mobile-responsive (breakpoints: 768px, 1024px)

### Deployment
- **Frontend**: GitHub Pages / Vercel (static hosting)
- **Backend**: Railway.app (free tier: 500 hrs/month) or Render.com
- **CI/CD**: GitHub Actions for automated deployment

## Functional Requirements

### FR-001: General Q&A Chatbot (Priority: P1)

**Description**: Users can ask questions about textbook content and receive accurate, citation-backed answers.

**User Story**: As a learner, I want to ask "What is ROS 2?" and receive an answer with references to specific chapters/sections.

**Acceptance Criteria**:
1. Chatbot interface accessible via floating button (bottom-right corner)
2. User types question in input field, presses Enter or clicks "Send"
3. System retrieves top-5 most relevant chunks from Qdrant (similarity > 0.7)
4. Response includes answer text + citations (chapter, section, chunk ID)
5. If no relevant content found (similarity < 0.7), display: "This topic is not covered in the current textbook."
6. Response time: <3 seconds (p95)
7. Chat history preserved in session (cleared on page reload)

**Edge Cases**:
- Empty query: Display validation message "Please enter a question"
- Query too long (>500 chars): Truncate with warning
- Backend unavailable: Display "Service temporarily unavailable. Please try again."

---

### FR-002: Select-to-Ask Functionality (Priority: P2)

**Description**: Users can highlight text on any page and ask contextual questions about the selection.

**User Story**: As a learner reading about "Isaac Sim GPU acceleration," I want to select that text and ask "How much faster is this than CPU simulation?"

**Acceptance Criteria**:
1. User selects text (mouse drag or touch-hold on mobile)
2. Tooltip appears with "Ask AI about this" button
3. Clicking tooltip opens chatbot with:
   - Pre-filled context: Selected text
   - Pre-filled prompt: "Explain: [selected text]" (editable)
4. Backend receives both query + context for enhanced retrieval
5. Context window: Selected text + 200 tokens before/after (for better understanding)
6. Mobile: Long-press selection shows tooltip (touch-friendly)

**Edge Cases**:
- Selection <5 chars: No tooltip (likely accidental)
- Selection >1000 chars: Truncate to first 1000 chars
- Selection from code block: Add metadata "This is code" to query
- Cross-paragraph selection: Include both paragraphs in context

---

### FR-003: Citation Rendering (Priority: P1)

**Description**: Every chatbot answer must include clickable citations linking back to source content.

**Acceptance Criteria**:
1. Citations displayed as: "Source: Chapter 3 - ROS 2 Fundamentals, Section 2"
2. Clicking citation scrolls page to exact section (deep link)
3. Multiple citations listed if answer combines multiple chunks
4. Citation UI: Small badge/chip with chapter icon

---

### FR-004: Chat History Persistence (Priority: P3)

**Description**: Store chat sessions in Neon Postgres for analytics (optional).

**Acceptance Criteria**:
1. Each query logged with: timestamp, query text, response, citations, response time
2. Anonymous data (no user identification)
3. Used for: Answer quality monitoring, popular topics analysis
4. User opt-out via UI toggle (localStorage flag)

---

## Data Models

### API Request/Response Schemas

#### POST `/api/query`

**Request:**
```json
{
  "query": "What are ROS 2 nodes?",
  "context": null,  // Optional: selected text for contextual queries
  "chapter_filter": null,  // Optional: e.g., "chapter-03" to search only Chapter 3
  "top_k": 5  // Optional: number of chunks to retrieve (default: 5)
}
```

**Response (Success):**
```json
{
  "status": "success",
  "answer": "ROS 2 nodes are individual processes that perform computation. Each node is responsible for a specific task, such as reading sensor data, processing images, or controlling motors. Nodes communicate with each other using topics, services, and actions.",
  "citations": [
    {
      "chapter": "3",
      "chapter_title": "ROS 2 Fundamentals",
      "section": "Nodes and Communication",
      "chunk_id": "ch3-s2-c01",
      "similarity_score": 0.92,
      "url": "/chapter-03/nodes-topics#nodes"
    },
    {
      "chapter": "3",
      "chapter_title": "ROS 2 Fundamentals",
      "section": "Creating Your First Node",
      "chunk_id": "ch3-s3-c02",
      "similarity_score": 0.87,
      "url": "/chapter-03/first-node"
    }
  ],
  "confidence": 0.92,
  "retrieved_chunks": 5,
  "response_time_ms": 1850
}
```

**Response (No Relevant Content):**
```json
{
  "status": "no_results",
  "answer": "This topic is not covered in the current textbook. Please refer to the Further Resources section in the relevant chapter.",
  "citations": [],
  "confidence": 0.45,
  "retrieved_chunks": 5,
  "response_time_ms": 1200
}
```

**Response (Error):**
```json
{
  "status": "error",
  "message": "Backend service temporarily unavailable",
  "error_code": "SERVICE_UNAVAILABLE"
}
```

#### GET `/api/health`

**Response:**
```json
{
  "status": "healthy",
  "services": {
    "qdrant": "connected",
    "neon_postgres": "connected",
    "embedding_model": "loaded"
  },
  "uptime_seconds": 3600,
  "version": "1.0.0"
}
```

---

### Database Schemas

#### Qdrant Collection: `textbook_embeddings`

**Payload Structure:**
```json
{
  "chunk_id": "ch3-s2-c01",
  "chapter": "3",
  "chapter_title": "ROS 2 Fundamentals",
  "section": "Nodes and Communication",
  "content": "ROS 2 nodes are individual processes that perform computation...",
  "token_count": 487,
  "url": "/chapter-03/nodes-topics#nodes",
  "metadata": {
    "sidebar_position": 3,
    "created_at": "2025-12-10T00:00:00Z",
    "source_file": "docs/chapter-03/nodes-topics.md"
  }
}
```

**Vector Configuration:**
- Dimension: 384 (all-MiniLM-L6-v2)
- Distance: Cosine
- Index: HNSW (Hierarchical Navigable Small World)

---

#### Neon Postgres Tables

**Table: `chat_sessions`**
```sql
CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(255) NOT NULL,
    query TEXT NOT NULL,
    answer TEXT NOT NULL,
    citations JSONB,
    confidence FLOAT,
    response_time_ms INT,
    created_at TIMESTAMP DEFAULT NOW(),
    user_opted_in BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_created_at ON chat_sessions(created_at DESC);
CREATE INDEX idx_session_id ON chat_sessions(session_id);
```

**Table: `user_feedback`**
```sql
CREATE TABLE user_feedback (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chat_session_id UUID REFERENCES chat_sessions(id),
    feedback_type VARCHAR(50),  -- 'helpful', 'not_helpful', 'inaccurate'
    comment TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

**Table: `query_analytics`**
```sql
CREATE TABLE query_analytics (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    date DATE NOT NULL,
    total_queries INT DEFAULT 0,
    avg_confidence FLOAT,
    avg_response_time_ms FLOAT,
    no_results_count INT DEFAULT 0,
    UNIQUE(date)
);
```

---

## Non-Functional Requirements

### Performance
- **Query Response Time**: <3s p95, <2s p50
- **Vector Search Latency**: <500ms p95
- **Embedding Generation**: <100ms per query
- **API Throughput**: 100 requests/minute (rate limited)

### Scalability
- **Concurrent Users**: 50-100 simultaneous chat sessions
- **Vector Storage**: <1000 embeddings (Qdrant free tier limit)
- **Database Storage**: <512MB (Neon free tier limit)

### Security
- **CORS Policy**: Whitelist only production domain (e.g., `https://yourusername.github.io`)
- **Rate Limiting**: 10 requests/minute per IP address
- **Input Sanitization**: Escape SQL injection, XSS attacks
- **No Authentication**: Public API (free-tier constraint)

### Reliability
- **Uptime**: 99% (Railway/Render free tier SLA)
- **Error Handling**: Graceful fallbacks for all service failures
- **Monitoring**: Basic logging (request count, errors, latency)

---

## Out of Scope (Phase 1)

The following features are **not** included in the initial release:
- ❌ Multi-language support (Urdu translation)
- ❌ User authentication / personalized chat history
- ❌ Real-time streaming responses (SSE/WebSocket)
- ❌ Voice input (speech-to-text)
- ❌ Feedback loop for model fine-tuning
- ❌ Admin dashboard for analytics

These may be added in future phases if free-tier budget allows.

---

## Success Metrics

### Quality Metrics
- **Answer Accuracy**: >90% of answers verifiable against source text
- **Citation Precision**: >95% of citations correctly link to source sections
- **Hallucination Rate**: <5% (answers containing info not in textbook)

### Engagement Metrics
- **Chatbot Usage**: >30% of users interact with chatbot per session
- **Average Queries**: 2-3 questions per user session
- **Select-to-Ask Adoption**: >10% of users try select-to-ask feature

### Technical Metrics
- **Response Time**: <3s for 95% of queries
- **Service Uptime**: >99% monthly
- **Error Rate**: <2% of API requests fail

---

## Dependencies

### Upstream
- **textbook-generation**: All 13 chapters must be complete before RAG indexing

### Downstream
- None (this is a leaf feature)

---

## Risks & Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Qdrant free tier limit exceeded | Medium | High | Monitor vector count, implement chunk deduplication |
| Backend cold starts (serverless) | High | Medium | Use Railway persistent container instead of serverless |
| Low answer quality (poor retrieval) | Medium | High | Implement confidence thresholding, tune top-k parameter |
| CORS issues in production | Low | High | Test CORS policy in staging environment first |
| Neon DB connection limit | Low | Medium | Implement connection pooling, async queries |

---

## Constitutional Compliance

✅ **Principle I (Simplicity)**: Minimal tech stack (FastAPI + React components)
✅ **Principle II (Accuracy)**: RAG grounded only in textbook chunks, confidence thresholds
✅ **Principle III (Free-Tier)**: Qdrant <1000 vectors, Neon <512MB, Railway 500hrs/month
✅ **Principle IV (Docusaurus Best Practices)**: React components follow Docusaurus theming
✅ **Principle V (Minimalism)**: No unnecessary libraries beyond FastAPI, Sentence Transformers
✅ **Principle VI (Fast Build)**: Backend deployment <5min, frontend unchanged
✅ **Principle VII (Content-First)**: Chatbot enhances learning, doesn't replace content
✅ **Principle VIII (RAG Guardrails)**: Confidence thresholds, fallback messages, citations required

---

**Version**: 1.0.0 | **Approved**: Pending | **Last Updated**: 2025-12-10
