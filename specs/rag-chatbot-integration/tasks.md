# Task Breakdown: RAG Chatbot Integration

**Feature Branch**: `rag-chatbot-integration` | **Created**: 2025-12-10 | **Status**: Ready for Implementation

## Task Organization

Tasks are organized by implementation phase and follow a sequential dependency structure. Each task includes:
- **ID**: Unique identifier (T001, T002, etc.)
- **Description**: Clear, actionable task statement
- **Acceptance Criteria**: Measurable success conditions
- **Dependencies**: Prerequisites that must be completed first
- **Effort**: Estimated time (S: 1-2hrs, M: 3-5hrs, L: 6-8hrs, XL: 8+hrs)
- **Phase**: Implementation phase (1, 2, or 3)

---

## Phase 1: Backend Infrastructure Setup (3-4 days)

**Goal**: Establish FastAPI backend with database connections and basic health checks.

### T001: Initialize FastAPI Project Structure
**Description**: Create backend project structure with proper directory organization and initial configuration files.

**Acceptance Criteria**:
- [ ] Directory structure created matching plan.md specification
- [ ] `backend/app/` directory with `__init__.py` created
- [ ] Subdirectories: `models/`, `services/`, `routes/`, `middleware/`
- [ ] Empty `__init__.py` files in all subdirectories
- [ ] `tests/` directory created with initial structure
- [ ] `requirements.txt` created with core dependencies
- [ ] `.env.example` created with all required environment variables
- [ ] `README.md` created with setup instructions

**Dependencies**: None

**Effort**: M (3-4 hours)

**Phase**: 1

---

### T002: Configure Environment Variables and Settings
**Description**: Implement configuration management using Pydantic BaseSettings for environment variables.

**Acceptance Criteria**:
- [ ] `app/config.py` created with Pydantic BaseSettings class
- [ ] Environment variables defined: API_HOST, API_PORT, ENVIRONMENT, DEBUG
- [ ] Qdrant config: QDRANT_URL, QDRANT_API_KEY, QDRANT_COLLECTION
- [ ] Neon config: DATABASE_URL with SSL mode
- [ ] CORS config: ALLOWED_ORIGINS (comma-separated list)
- [ ] Rate limiting config: RATE_LIMIT_PER_MINUTE
- [ ] Embedding model config: EMBEDDING_MODEL path
- [ ] `.env.example` updated with all variables and descriptions
- [ ] Config validation ensures required vars are present

**Dependencies**: T001

**Effort**: S (1-2 hours)

**Phase**: 1

---

### T003: Create Qdrant Cloud Collection
**Description**: Provision Qdrant Cloud account (free tier) and create vector collection for textbook embeddings.

**Acceptance Criteria**:
- [ ] Qdrant Cloud account created
- [ ] Collection `textbook_embeddings` created
- [ ] Vector configuration: 384 dimensions, Cosine distance
- [ ] HNSW index configured for fast search
- [ ] API key generated and stored securely
- [ ] Connection test script validates access
- [ ] Storage limit monitored (<1GB free tier)
- [ ] Documentation updated with Qdrant setup instructions

**Dependencies**: T002

**Effort**: M (2-3 hours)

**Phase**: 1

---

### T004: Provision Neon Postgres Database
**Description**: Create Neon Serverless Postgres database (free tier) and initialize schema with required tables.

**Acceptance Criteria**:
- [ ] Neon account created with free tier project
- [ ] Database created with connection string
- [ ] UUID extension enabled (`uuid-ossp`)
- [ ] `chat_sessions` table created with all columns
- [ ] Indexes created: `idx_chat_sessions_created_at`, `idx_chat_sessions_session_id`
- [ ] `user_feedback` table created with foreign key constraint
- [ ] `query_analytics` table created with UNIQUE date constraint
- [ ] Connection test validates SSL connectivity
- [ ] Migration script documented in README

**SQL Schema**:
```sql
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

CREATE TABLE chat_sessions (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    session_id VARCHAR(255) NOT NULL,
    query TEXT NOT NULL,
    answer TEXT NOT NULL,
    citations JSONB,
    confidence FLOAT,
    response_time_ms INT,
    created_at TIMESTAMP DEFAULT NOW(),
    user_opted_in BOOLEAN DEFAULT FALSE
);

CREATE INDEX idx_chat_sessions_created_at ON chat_sessions(created_at DESC);
CREATE INDEX idx_chat_sessions_session_id ON chat_sessions(session_id);

CREATE TABLE user_feedback (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    chat_session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    feedback_type VARCHAR(50) CHECK (feedback_type IN ('helpful', 'not_helpful', 'inaccurate')),
    comment TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE TABLE query_analytics (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    date DATE NOT NULL UNIQUE,
    total_queries INT DEFAULT 0,
    avg_confidence FLOAT,
    avg_response_time_ms FLOAT,
    no_results_count INT DEFAULT 0
);
```

**Dependencies**: T002

**Effort**: M (2-3 hours)

**Phase**: 1

---

### T005: Implement Database Models (Pydantic & SQLAlchemy)
**Description**: Create Pydantic models for API validation and SQLAlchemy models for database operations.

**Acceptance Criteria**:
- [ ] `app/models/query.py` created with Pydantic models:
  - [ ] `QueryRequest` (query, context, chapter_filter, top_k)
  - [ ] `Citation` (chapter, chapter_title, section, chunk_id, similarity_score, url)
  - [ ] `QueryResponse` (status, answer, citations, confidence, response_time_ms)
  - [ ] `ErrorResponse` (status, message, error_code)
- [ ] `app/models/database.py` created with SQLAlchemy models:
  - [ ] `ChatSession` (all columns from schema)
  - [ ] `UserFeedback` (all columns from schema)
  - [ ] `QueryAnalytics` (all columns from schema)
- [ ] All models have proper type hints and validation
- [ ] JSON serialization/deserialization working correctly

**Dependencies**: T004

**Effort**: M (3-4 hours)

**Phase**: 1

---

### T006: Implement Sentence Transformer Embedding Service
**Description**: Create service wrapper for Sentence Transformers model to generate query embeddings.

**Acceptance Criteria**:
- [ ] `app/services/embeddings.py` created
- [ ] `EmbeddingService` class implemented with:
  - [ ] `__init__()`: Load `all-MiniLM-L6-v2` model
  - [ ] `encode(text: str) -> np.ndarray`: Generate 384-dim embedding
  - [ ] `encode_batch(texts: List[str]) -> np.ndarray`: Batch encoding
  - [ ] Model caching to avoid reloading
- [ ] Average encoding time <100ms per query (tested)
- [ ] Memory usage <500MB for model
- [ ] Error handling for empty/invalid input
- [ ] Unit tests: `tests/test_embeddings.py` with 90%+ coverage

**Dependencies**: T002

**Effort**: M (3-4 hours)

**Phase**: 1

---

### T007: Implement Qdrant Client Service
**Description**: Create service for vector search operations against Qdrant collection.

**Acceptance Criteria**:
- [ ] `app/services/qdrant_client.py` created
- [ ] `QdrantService` class implemented with:
  - [ ] `__init__()`: Initialize Qdrant client with credentials
  - [ ] `search(query_vector, top_k, score_threshold)`: Vector similarity search
  - [ ] `upsert(points)`: Batch insert/update vectors
  - [ ] `get_collection_info()`: Return vector count and stats
  - [ ] Connection pooling for efficient reuse
- [ ] Search latency <500ms p95 (tested with sample data)
- [ ] Proper error handling for network failures
- [ ] Score threshold filtering (>0.7)
- [ ] Unit tests: `tests/test_qdrant.py` with 90%+ coverage

**Dependencies**: T003, T006

**Effort**: M (4-5 hours)

**Phase**: 1

---

### T008: Implement Postgres Client Service
**Description**: Create service for logging chat sessions and analytics to Neon Postgres.

**Acceptance Criteria**:
- [ ] `app/services/postgres_client.py` created
- [ ] `PostgresService` class implemented with:
  - [ ] `__init__()`: Initialize asyncpg connection pool (max 10 connections)
  - [ ] `log_chat_session(session_data)`: Insert chat session (async)
  - [ ] `log_user_feedback(feedback_data)`: Insert user feedback
  - [ ] `update_daily_analytics(date, metrics)`: Upsert analytics
  - [ ] Connection pool health check
- [ ] Async operations don't block API responses
- [ ] Transaction safety with rollback on errors
- [ ] Connection limit respected (<10 concurrent)
- [ ] Unit tests: `tests/test_postgres.py` with 90%+ coverage

**Dependencies**: T004, T005

**Effort**: M (4-5 hours)

**Phase**: 1

---

### T009: Implement Health Check Endpoint
**Description**: Create GET /api/health endpoint to verify all services are operational.

**Acceptance Criteria**:
- [ ] `app/routes/health.py` created
- [ ] GET `/api/health` endpoint implemented
- [ ] Response includes:
  - [ ] Overall status: "healthy" or "degraded"
  - [ ] Qdrant connection status
  - [ ] Neon Postgres connection status
  - [ ] Embedding model loaded status
  - [ ] Uptime in seconds
  - [ ] API version (1.0.0)
- [ ] Endpoint responds in <500ms
- [ ] Returns 200 if all services healthy, 503 if any degraded
- [ ] Integration test validates response structure

**Response Example**:
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

**Dependencies**: T006, T007, T008

**Effort**: S (2 hours)

**Phase**: 1

---

### T010: Implement CORS and Rate Limiting Middleware
**Description**: Configure CORS policy and rate limiting to protect API from abuse.

**Acceptance Criteria**:
- [ ] `app/middleware/cors.py` created
- [ ] CORS middleware configured with:
  - [ ] Allowed origins from ALLOWED_ORIGINS env var
  - [ ] Allowed methods: GET, POST, OPTIONS
  - [ ] Allowed headers: Content-Type, Authorization
  - [ ] Credentials allowed: False (no auth required)
- [ ] `app/middleware/rate_limit.py` created
- [ ] Rate limiting implemented:
  - [ ] 10 requests per minute per IP address
  - [ ] Returns 429 Too Many Requests when exceeded
  - [ ] X-RateLimit headers included in response
- [ ] Middleware registered in `app/main.py`
- [ ] Integration tests validate CORS and rate limit behavior

**Dependencies**: T009

**Effort**: M (3-4 hours)

**Phase**: 1

---

### T011: Initialize FastAPI Application and Router Registration
**Description**: Create main FastAPI app with all routes registered and middleware configured.

**Acceptance Criteria**:
- [ ] `app/main.py` created with FastAPI app initialization
- [ ] CORS middleware registered
- [ ] Rate limiting middleware registered
- [ ] Health check route registered
- [ ] Startup event handler loads embedding model
- [ ] Shutdown event handler closes database connections
- [ ] OpenAPI documentation auto-generated at `/docs`
- [ ] App runs on port 8000 with uvicorn
- [ ] Manual test: `curl http://localhost:8000/api/health` returns 200

**Dependencies**: T010

**Effort**: S (2 hours)

**Phase**: 1

---

### T012: Deploy Backend to Railway.app
**Description**: Deploy FastAPI backend to Railway.app free tier with environment variables configured.

**Acceptance Criteria**:
- [ ] Railway account created
- [ ] GitHub repo connected to Railway project
- [ ] `backend/` set as root directory in Railway config
- [ ] `Dockerfile` created for containerization
- [ ] Environment variables set in Railway dashboard:
  - [ ] QDRANT_URL, QDRANT_API_KEY
  - [ ] DATABASE_URL
  - [ ] ALLOWED_ORIGINS (production frontend URL)
  - [ ] RATE_LIMIT_PER_MINUTE
- [ ] Backend deployed and accessible at public URL
- [ ] Health check endpoint returns 200
- [ ] Railway auto-deploy on git push configured
- [ ] Free tier usage monitored (<500 hrs/month)

**Dependencies**: T011

**Effort**: M (3-4 hours)

**Phase**: 1

---

### T013: Write Backend README and Setup Documentation
**Description**: Document backend setup process, API endpoints, and deployment instructions.

**Acceptance Criteria**:
- [ ] `backend/README.md` created with sections:
  - [ ] Project overview and architecture
  - [ ] Local development setup (prerequisites, .env config)
  - [ ] Running locally with uvicorn
  - [ ] API endpoints documentation (health check)
  - [ ] Testing instructions (pytest)
  - [ ] Deployment guide (Railway)
  - [ ] Troubleshooting common issues
- [ ] `.env.example` fully documented with comments
- [ ] Setup steps tested by independent reviewer

**Dependencies**: T012

**Effort**: S (2 hours)

**Phase**: 1

---

## Phase 2: Content Ingestion Pipeline (2-3 days)

**Goal**: Process all textbook markdown files into vector embeddings and upload to Qdrant.

### T014: Create Markdown File Reader and Frontmatter Parser
**Description**: Implement utility to read all markdown files from docs/ directory and extract frontmatter metadata.

**Acceptance Criteria**:
- [ ] `backend/scripts/ingest.py` created
- [ ] Function `read_markdown_files(directory)`:
  - [ ] Uses glob to find all `docs/chapter-*/*.md` files
  - [ ] Returns list of file paths
- [ ] Function `parse_frontmatter(md_content)`:
  - [ ] Extracts YAML between `---` delimiters
  - [ ] Returns dict with title, sidebar_position, etc.
- [ ] Function `extract_metadata(file_path, frontmatter)`:
  - [ ] Extracts chapter number from path (regex: `chapter-(\d+)`)
  - [ ] Builds URL from path: `docs/chapter-03/nodes.md` → `/chapter-03/nodes`
  - [ ] Returns complete metadata dict
- [ ] Handles missing frontmatter gracefully
- [ ] Unit tests validate parsing accuracy

**Dependencies**: None (can start immediately)

**Effort**: M (3-4 hours)

**Phase**: 2

---

### T015: Implement Markdown Chunking Logic
**Description**: Split markdown content into semantically meaningful chunks of 300-500 tokens with 50-token overlap.

**Acceptance Criteria**:
- [ ] Function `chunk_markdown(content, max_tokens=500, overlap=50)` implemented
- [ ] Chunking strategy:
  - [ ] Split by H2 headings (`## Section Title`) first
  - [ ] If section >500 tokens, split by paragraphs
  - [ ] Add 50-token overlap from previous chunk
  - [ ] Preserve code blocks intact (don't split mid-code)
- [ ] Token counting function uses tiktoken (GPT tokenizer)
- [ ] Edge cases handled:
  - [ ] Very short sections (<50 tokens) merged with next
  - [ ] Very long code blocks (>500 tokens) kept intact
  - [ ] Empty sections skipped
- [ ] Output: List of chunk strings with consistent token counts
- [ ] Unit tests validate chunking quality on sample chapters

**Dependencies**: T014

**Effort**: L (6-8 hours)

**Phase**: 2

---

### T016: Implement Batch Embedding Generation
**Description**: Generate embeddings for all chunks using Sentence Transformers with batch processing.

**Acceptance Criteria**:
- [ ] Function `generate_embeddings(chunks, batch_size=32)` implemented
- [ ] Uses `EmbeddingService` from app/services/embeddings.py
- [ ] Processes chunks in batches of 32 for efficiency
- [ ] Returns list of 384-dim numpy arrays
- [ ] Progress bar displayed during processing (tqdm)
- [ ] Average throughput: >100 chunks/second
- [ ] Memory usage <2GB during processing
- [ ] Error handling for encoding failures

**Dependencies**: T006, T015

**Effort**: M (3-4 hours)

**Phase**: 2

---

### T017: Implement Qdrant Batch Upload
**Description**: Upload chunk embeddings with metadata to Qdrant collection in batches.

**Acceptance Criteria**:
- [ ] Function `upload_to_qdrant(embeddings, metadata, batch_size=100)` implemented
- [ ] Prepares Qdrant point objects:
  - [ ] ID: `chapter-03-nodes-topics-chunk-01`
  - [ ] Vector: 384-dim embedding array
  - [ ] Payload: chunk_id, chapter, chapter_title, section, content, token_count, url, metadata
- [ ] Uploads in batches of 100 points
- [ ] Progress bar displayed (tqdm)
- [ ] Validates vector count stays under 1000 (free tier limit)
- [ ] Error handling with retry logic (3 attempts)
- [ ] Logs upload statistics (total chunks, total time)

**Dependencies**: T007, T016

**Effort**: M (4-5 hours)

**Phase**: 2

---

### T018: Execute Full Ingestion for All Chapters
**Description**: Run complete ingestion pipeline on all 13 textbook chapters.

**Acceptance Criteria**:
- [ ] Ingestion script executed: `python backend/scripts/ingest.py`
- [ ] All 13 chapters processed:
  - [ ] Chapter 1: Introduction to Physical AI
  - [ ] Chapter 2: Python Fundamentals
  - [ ] Chapter 3: ROS 2 Fundamentals
  - [ ] Chapter 4: URDF & Robot Modeling
  - [ ] Chapter 5: Gazebo Simulation
  - [ ] Chapter 6: Computer Vision
  - [ ] Chapter 7: Advanced Simulation & Unity
  - [ ] Chapter 8: NVIDIA Isaac Platform
  - [ ] Chapter 9: Perception & Navigation
  - [ ] Chapter 10: Reinforcement Learning
  - [ ] Chapter 11: Humanoid Kinematics
  - [ ] Chapter 12: Manipulation
  - [ ] Chapter 13: Conversational Robotics
- [ ] Total vectors uploaded: 600-800 (estimated)
- [ ] Vector count verified in Qdrant dashboard
- [ ] Execution time: <5 minutes
- [ ] Logs captured with chunk counts per chapter

**Dependencies**: T017

**Effort**: S (1 hour execution + validation)

**Phase**: 2

---

### T019: Validate Vector Search Quality
**Description**: Test sample queries against Qdrant to ensure retrieval accuracy.

**Acceptance Criteria**:
- [ ] Test script `backend/scripts/test_search.py` created
- [ ] Test queries executed:
  - [ ] "What is ROS 2?" → Should return Chapter 3 chunks
  - [ ] "How to simulate sensors in Gazebo?" → Should return Chapter 7 chunks
  - [ ] "Isaac Sim GPU acceleration" → Should return Chapter 8 chunks
  - [ ] "Unitree G1 manipulation" → Should return Chapter 12 chunks
- [ ] Each query returns 5 results with similarity >0.7
- [ ] Citations include correct chapter, section, URL
- [ ] Response time <500ms per query
- [ ] Manual review confirms relevance of retrieved chunks

**Dependencies**: T018

**Effort**: M (2-3 hours)

**Phase**: 2

---

### T020: Document Ingestion Process and Re-indexing Strategy
**Description**: Create documentation for running ingestion pipeline and updating vectors when content changes.

**Acceptance Criteria**:
- [ ] `backend/scripts/README.md` created with:
  - [ ] Ingestion script usage instructions
  - [ ] Prerequisites (Python packages, environment variables)
  - [ ] Execution steps
  - [ ] Expected output and validation
  - [ ] Troubleshooting guide
- [ ] Re-indexing workflow documented:
  - [ ] Manual re-run command
  - [ ] GitHub Actions workflow example (optional)
  - [ ] When to re-index (content updates, model changes)
- [ ] Qdrant collection management documented (backup, restore)

**Dependencies**: T019

**Effort**: S (2 hours)

**Phase**: 2

---

## Phase 3: Frontend Integration & Deployment (5-7 days)

**Goal**: Build React UI components, integrate with backend API, deploy to production.

### T021: Implement Query API Endpoint (Backend)
**Description**: Create POST /api/query endpoint to handle user queries and return RAG responses.

**Acceptance Criteria**:
- [ ] `app/routes/query.py` created
- [ ] POST `/api/query` endpoint implemented
- [ ] Request validation using `QueryRequest` Pydantic model
- [ ] Query processing flow:
  1. [ ] Receive query + optional context
  2. [ ] Generate embedding using `EmbeddingService`
  3. [ ] Search Qdrant with top_k=5, score_threshold=0.7
  4. [ ] Format response with answer text + citations
  5. [ ] Log to Postgres (async, non-blocking)
  6. [ ] Return `QueryResponse` JSON
- [ ] Response time <3s p95
- [ ] Error handling:
  - [ ] Empty query → 400 Bad Request
  - [ ] Backend failure → 503 Service Unavailable
  - [ ] No results found (similarity <0.7) → "no_results" status
- [ ] Integration tests validate all response types

**Response Example**:
```json
{
  "status": "success",
  "answer": "ROS 2 nodes are individual processes that perform computation...",
  "citations": [
    {
      "chapter": "3",
      "chapter_title": "ROS 2 Fundamentals",
      "section": "Nodes and Communication",
      "chunk_id": "ch3-s2-c01",
      "similarity_score": 0.92,
      "url": "/chapter-03/nodes-topics#nodes"
    }
  ],
  "confidence": 0.92,
  "retrieved_chunks": 5,
  "response_time_ms": 1850
}
```

**Dependencies**: T006, T007, T008

**Effort**: L (6-8 hours)

**Phase**: 3

---

### T022: Create ChatWidget Component Structure
**Description**: Initialize React component structure for ChatWidget with floating button and modal.

**Acceptance Criteria**:
- [ ] `src/components/ChatWidget/` directory created
- [ ] `ChatWidget.jsx` main component created
- [ ] Component state initialized:
  - [ ] `isOpen`: Boolean for modal visibility
  - [ ] `messages`: Array of chat messages
  - [ ] `inputValue`: Current input text
  - [ ] `isLoading`: Boolean for API call state
- [ ] `FloatingButton` sub-component created:
  - [ ] Fixed position bottom-right (24px margin)
  - [ ] 60px diameter circular button
  - [ ] Chatbot icon displayed
  - [ ] onClick opens modal
- [ ] Basic component renders without errors
- [ ] TypeScript types defined (if using TS)

**Dependencies**: None (can start immediately)

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T023: Implement ChatModal Component
**Description**: Create modal dialog component to display chat interface.

**Acceptance Criteria**:
- [ ] `ChatModal.jsx` component created
- [ ] Modal structure:
  - [ ] Header with title "Ask AI Assistant" and close button
  - [ ] Message list container (scrollable)
  - [ ] Input box footer
- [ ] Modal positioning:
  - [ ] Desktop: Bottom-right, 400px width, 600px height
  - [ ] Mobile: Full-screen overlay
- [ ] Close button functionality:
  - [ ] Closes modal
  - [ ] Preserves chat history (doesn't reset messages)
- [ ] Modal visible only when `isOpen === true`
- [ ] CSS Modules styling applied
- [ ] Accessibility: ESC key closes modal, focus trap

**Dependencies**: T022

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T024: Implement MessageList Component
**Description**: Create scrollable message list component with user and bot messages.

**Acceptance Criteria**:
- [ ] `MessageList.jsx` component created
- [ ] Renders messages array with proper formatting
- [ ] `UserMessage` sub-component:
  - [ ] Right-aligned bubble
  - [ ] Light background color
  - [ ] User's query text displayed
- [ ] `BotMessage` sub-component:
  - [ ] Left-aligned bubble
  - [ ] Gray background color
  - [ ] Bot's answer text displayed
  - [ ] Citations rendered below answer (if present)
- [ ] Auto-scroll to bottom when new message added
- [ ] Scroll behavior smooth and performant
- [ ] Empty state message: "Ask me anything about the textbook!"

**Dependencies**: T023

**Effort**: M (4-5 hours)

**Phase**: 3

---

### T025: Implement CitationChip Component
**Description**: Create clickable citation badges that deep-link to source content.

**Acceptance Criteria**:
- [ ] `CitationChip.jsx` component created
- [ ] Citation display format: "📖 Chapter 3 - Nodes"
- [ ] Badge styling:
  - [ ] Inline-block with padding: 4px 8px
  - [ ] Border-radius: 4px
  - [ ] Light background color (theme-aware)
  - [ ] Hover effect: darker background
  - [ ] Cursor: pointer
- [ ] onClick handler:
  - [ ] Navigates to citation URL using `window.location.href`
  - [ ] Closes chat modal after click
  - [ ] Scrolls to exact section if URL has hash anchor
- [ ] Multiple citations rendered as flex row with wrapping
- [ ] Tooltip on hover shows full chapter title + section

**Dependencies**: T024

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T026: Implement InputBox Component
**Description**: Create text input area with send button for submitting queries.

**Acceptance Criteria**:
- [ ] `InputBox.jsx` component created
- [ ] Textarea element:
  - [ ] Placeholder: "Ask a question about the textbook..."
  - [ ] Rows: 3 (auto-expand up to 5 rows)
  - [ ] Enter key submits (Shift+Enter for newline)
  - [ ] Disabled during loading state
- [ ] Send button:
  - [ ] Icon: Paper plane or arrow
  - [ ] Disabled when input empty or loading
  - [ ] Loading spinner displayed during API call
- [ ] Input validation:
  - [ ] Empty query shows error: "Please enter a question"
  - [ ] Query >500 chars shows warning (still submits)
- [ ] Focus management: input focuses on modal open

**Dependencies**: T023

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T027: Implement API Integration Logic
**Description**: Connect ChatWidget to backend API with fetch calls and error handling.

**Acceptance Criteria**:
- [ ] `handleSendQuery` function implemented in ChatWidget.jsx
- [ ] API call configuration:
  - [ ] Endpoint: `POST https://your-backend.railway.app/api/query`
  - [ ] Headers: `Content-Type: application/json`
  - [ ] Body: `{query, context, top_k: 5}`
  - [ ] Timeout: 5 seconds
- [ ] Response handling:
  - [ ] Success (status: "success"): Display answer + citations
  - [ ] No results (status: "no_results"): Display fallback message
  - [ ] Error (status: "error"): Display error message
  - [ ] Network error: Display "Service unavailable" message
- [ ] Loading state management:
  - [ ] Set `isLoading = true` before API call
  - [ ] Set `isLoading = false` after response/error
  - [ ] Disable input during loading
- [ ] Message history updated with user query + bot response

**Dependencies**: T021, T026

**Effort**: M (4-5 hours)

**Phase**: 3

---

### T028: Implement Select-to-Ask Text Detection
**Description**: Detect text selection on page and display "Ask AI" tooltip.

**Acceptance Criteria**:
- [ ] `SelectToAsk.jsx` component created
- [ ] Text selection detection:
  - [ ] Listen to `mouseup` event on document
  - [ ] Listen to `touchend` event for mobile
  - [ ] Get selected text using `window.getSelection().toString()`
- [ ] Selection validation:
  - [ ] Ignore if <5 characters (likely accidental)
  - [ ] Ignore if >1000 characters (too long)
  - [ ] Only trigger on actual text content (not code blocks)
- [ ] Tooltip positioning:
  - [ ] Calculate selection bounding box
  - [ ] Position tooltip at bottom-right of selection
  - [ ] Adjust position if near viewport edge
- [ ] Tooltip hidden when:
  - [ ] User clicks elsewhere
  - [ ] User starts new selection
  - [ ] Chat modal opens

**Dependencies**: T022

**Effort**: L (6-7 hours)

**Phase**: 3

---

### T029: Implement SelectToAskTooltip Component
**Description**: Create tooltip with "Ask AI about this" button for selected text.

**Acceptance Criteria**:
- [ ] `SelectToAskTooltip.jsx` component created
- [ ] Tooltip appearance:
  - [ ] Small popup with rounded corners
  - [ ] Shadow for depth
  - [ ] Button: "🤖 Ask AI about this"
  - [ ] Positioned near selected text
- [ ] Button click behavior:
  - [ ] Opens chat modal
  - [ ] Pre-fills input with: "Explain: [selected text]"
  - [ ] Sets context parameter to selected text
  - [ ] Optionally auto-sends query (configurable)
- [ ] Tooltip animation:
  - [ ] Fade-in when selection detected (200ms)
  - [ ] Fade-out when hidden (200ms)
- [ ] Mobile-friendly touch interactions

**Dependencies**: T028

**Effort**: M (4-5 hours)

**Phase**: 3

---

### T030: Create ChatWidget CSS Modules
**Description**: Style ChatWidget components with CSS Modules and dark mode support.

**Acceptance Criteria**:
- [ ] `ChatWidget.module.css` created
- [ ] Floating button styles:
  - [ ] Fixed positioning (bottom: 24px, right: 24px)
  - [ ] Circular shape (60px diameter)
  - [ ] Primary color background (theme variable)
  - [ ] Box shadow for depth
  - [ ] Hover scale transform (1.05)
  - [ ] Z-index: 9999
- [ ] Chat modal styles:
  - [ ] Fixed positioning (bottom: 100px, right: 24px)
  - [ ] Width: 400px, Height: 600px
  - [ ] Rounded corners (12px)
  - [ ] Shadow: `0 8px 32px rgba(0,0,0,0.2)`
  - [ ] Background: theme background color
  - [ ] Flexbox column layout
- [ ] Message styles:
  - [ ] User messages: right-aligned, light background
  - [ ] Bot messages: left-aligned, gray background
  - [ ] Padding: 12px, border-radius: 12px
  - [ ] Margin-bottom: 12px
- [ ] Citation chip styles:
  - [ ] Inline-block, padding: 4px 8px
  - [ ] Light background, rounded corners
  - [ ] Hover: darker background
- [ ] Dark mode support:
  - [ ] Use Docusaurus theme variables
  - [ ] `[data-theme='dark']` selector overrides
  - [ ] Bot message background: gray-800 in dark mode
- [ ] Mobile responsive (@media max-width: 768px):
  - [ ] Chat modal full-screen
  - [ ] Floating button smaller (56px)

**Dependencies**: T022, T023, T024, T025

**Effort**: M (4-5 hours)

**Phase**: 3

---

### T031: Integrate ChatWidget into Docusaurus via Root Swizzle
**Description**: Add ChatWidget to all Docusaurus pages by swizzling Root theme component.

**Acceptance Criteria**:
- [ ] Command executed: `npm run swizzle @docusaurus/theme-classic Root -- --eject`
- [ ] `src/theme/Root.js` created/modified
- [ ] ChatWidget imported: `import ChatWidget from '@site/src/components/ChatWidget';`
- [ ] ChatWidget rendered after children: `<>{children}<ChatWidget /></>`
- [ ] ChatWidget appears on all pages:
  - [ ] Homepage
  - [ ] All chapter pages
  - [ ] API docs pages
- [ ] No conflicts with existing Docusaurus UI
- [ ] Performance: No noticeable page load impact
- [ ] Manual testing on desktop and mobile

**Root.js Example**:
```javascript
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
```

**Dependencies**: T030

**Effort**: S (1-2 hours)

**Phase**: 3

---

### T032: Test Mobile Responsiveness
**Description**: Validate ChatWidget UI on mobile devices (iOS Safari, Android Chrome).

**Acceptance Criteria**:
- [ ] Tested on iOS Safari (iPhone 12/13/14 simulators)
- [ ] Tested on Android Chrome (Pixel 5/6/7 emulators)
- [ ] Floating button:
  - [ ] Visible and tappable (56px size)
  - [ ] Not obstructed by browser UI
- [ ] Chat modal:
  - [ ] Full-screen on mobile
  - [ ] Scrollable message list
  - [ ] Input box visible (not hidden by keyboard)
  - [ ] Close button accessible
- [ ] Select-to-ask:
  - [ ] Long-press selection works
  - [ ] Tooltip appears near selection
  - [ ] Button tap opens chat with context
- [ ] No layout shifts or overflow issues
- [ ] Performance: smooth animations, no lag

**Dependencies**: T031

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T033: Write Frontend Tests (Unit + Integration)
**Description**: Create test suite for ChatWidget components using React Testing Library.

**Acceptance Criteria**:
- [ ] Test files created:
  - [ ] `tests/ChatWidget.test.jsx`
  - [ ] `tests/MessageList.test.jsx`
  - [ ] `tests/CitationChip.test.jsx`
  - [ ] `tests/SelectToAsk.test.jsx`
- [ ] Unit tests:
  - [ ] Component renders without errors
  - [ ] Props passed correctly to children
  - [ ] State updates on user interactions
  - [ ] Click handlers fire correctly
- [ ] Integration tests:
  - [ ] User types query and clicks send
  - [ ] Mock API response displayed correctly
  - [ ] Citations clickable and navigate to URL
  - [ ] Error handling displays error message
  - [ ] Loading state disables input
- [ ] Test coverage >80% for all components
- [ ] All tests pass with `npm test`

**Dependencies**: T031

**Effort**: L (6-8 hours)

**Phase**: 3

---

### T034: Deploy Frontend to GitHub Pages
**Description**: Configure GitHub Actions to auto-deploy Docusaurus site to GitHub Pages.

**Acceptance Criteria**:
- [ ] `.github/workflows/deploy.yml` created
- [ ] Workflow triggers on push to `main` branch
- [ ] Build steps:
  - [ ] Checkout repo
  - [ ] Setup Node.js 18
  - [ ] Install dependencies (`npm ci`)
  - [ ] Build Docusaurus (`npm run build`)
  - [ ] Deploy to `gh-pages` branch
- [ ] GitHub Pages enabled in repo settings
- [ ] Custom domain configured (if applicable)
- [ ] Production URL accessible: `https://yourusername.github.io/Physical-AI-Humanoid-Robotics/`
- [ ] ChatWidget functional on production site
- [ ] API calls to Railway backend successful (CORS configured)

**Dependencies**: T033

**Effort**: M (3-4 hours)

**Phase**: 3

---

### T035: End-to-End Testing and Production Validation
**Description**: Perform comprehensive testing of entire RAG chatbot system in production.

**Acceptance Criteria**:
- [ ] Production validation checklist:
  - [ ] Homepage loads correctly
  - [ ] ChatWidget floating button visible
  - [ ] Click floating button opens modal
  - [ ] Type query and send
  - [ ] Response received within 3 seconds
  - [ ] Citations displayed and clickable
  - [ ] Citation navigation to source chapter works
  - [ ] Select text on page shows tooltip
  - [ ] "Ask AI about this" button opens chat with context
  - [ ] Error handling: Backend unavailable shows error message
  - [ ] Mobile: All features functional on iOS and Android
- [ ] Performance metrics validated:
  - [ ] Query response time <3s p95
  - [ ] Vector search latency <500ms
  - [ ] API throughput >10 req/min sustained
- [ ] Monitoring setup:
  - [ ] Railway backend logs reviewed
  - [ ] Error rates monitored (<2%)
  - [ ] Qdrant vector count verified (<1000)
- [ ] Production documentation updated with deployment details

**Dependencies**: T034

**Effort**: M (4-5 hours)

**Phase**: 3

---

## Summary Statistics

### Task Distribution by Phase
- **Phase 1 (Backend)**: 13 tasks (T001-T013)
- **Phase 2 (Ingestion)**: 7 tasks (T014-T020)
- **Phase 3 (Frontend)**: 15 tasks (T021-T035)
- **Total**: 35 tasks

### Effort Distribution
- **Small (1-2 hours)**: 7 tasks
- **Medium (3-5 hours)**: 20 tasks
- **Large (6-8 hours)**: 7 tasks
- **Extra Large (8+ hours)**: 1 task

### Critical Path
T001 → T002 → T003 → T006 → T007 → T016 → T017 → T018 → T019 → T021 → T027 → T031 → T034 → T035

### Estimated Timeline
- **Phase 1**: 3-4 days (30-40 hours)
- **Phase 2**: 2-3 days (20-25 hours)
- **Phase 3**: 5-7 days (45-55 hours)
- **Total**: 10-14 days (95-120 hours)

---

## Task Execution Workflow

### Daily Standups (Recommended)
1. Review completed tasks from previous day
2. Identify blockers or dependencies
3. Assign next tasks based on critical path
4. Update task status in project board

### Task Status Tracking
- **Not Started**: Task not begun
- **In Progress**: Currently being worked on
- **Blocked**: Waiting on dependency or decision
- **In Review**: Code review or testing in progress
- **Completed**: All acceptance criteria met

### Completion Checklist
Before marking task as complete:
- [ ] All acceptance criteria met
- [ ] Code reviewed (if applicable)
- [ ] Tests passing (if applicable)
- [ ] Documentation updated (if applicable)
- [ ] Manual testing completed
- [ ] Dependencies unblocked for next tasks

---

**Version**: 1.0.0 | **Status**: Ready for Implementation | **Last Updated**: 2025-12-10
