# Implementation Plan: Textbook Generation

**Branch**: `textbook-generation` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/textbook-generation/spec.md`

## Summary

Build an AI-native textbook platform for Physical AI & Humanoid Robotics using Docusaurus v3.x for static site generation and FastAPI for RAG chatbot backend. The system will deliver 6 structured chapters with interactive learning through a vector-based question-answering system grounded exclusively in textbook content. All frontend decisions must strictly follow Docusaurus official documentation (https://docusaurus.io/docs).

**Primary Requirement**: Deliver educational content with AI-enhanced learning while maintaining free-tier constraints and fast build times.

**Technical Approach**:
- Frontend: Docusaurus v3.x with MDX content, leveraging built-in theming, navigation, and search
- Backend: FastAPI with Qdrant vector DB and Neon Postgres for RAG chatbot
- Deployment: GitHub Pages or Vercel free tier with automated CI/CD
- Embeddings: Sentence Transformers (all-MiniLM-L6-v2, 384 dimensions) running locally

## Technical Context

**Language/Version**:
- Frontend: Node.js 18.x or 20.x, React 18.x (Docusaurus dependency)
- Backend: Python 3.10+

**Primary Dependencies**:
- Frontend: Docusaurus v3.x (@docusaurus/core, @docusaurus/preset-classic)
- Backend: FastAPI, sentence-transformers, qdrant-client, psycopg2-binary
- Database Clients: qdrant-client (Python), psycopg2-binary (Neon Postgres)

**Storage**:
- Vector DB: Qdrant Cloud free tier (1GB, 1M vectors max)
- Relational DB: Neon Postgres free tier (512MB storage)
- Static Assets: GitHub repository + GitHub Pages/Vercel CDN

**Testing**:
- Frontend: Jest (Docusaurus built-in), Lighthouse CI
- Backend: pytest, pytest-asyncio
- E2E: Playwright (optional for critical user flows)

**Target Platform**:
- Frontend: Modern browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+)
- Backend: Linux server (Vercel serverless functions or Railway/Render containers)
- Mobile: Responsive web (iOS Safari 14+, Android Chrome 90+)

**Project Type**: Web application (frontend + backend microservice)

**Performance Goals**:
- Frontend: Lighthouse score >90, First Contentful Paint <1.5s, Time to Interactive <3.5s
- Backend: RAG query <2s p95, Vector search <500ms p95, API endpoints <1s p95
- Build: Docusaurus build <60s, Dev server start <10s

**Constraints**:
- Free-tier limits: Qdrant <1000 vectors, Neon <512MB, Vercel <100GB bandwidth/month
- No paid APIs (OpenAI, Cohere, etc.)
- No GPU in production
- Bundle size <2MB gzipped
- Mobile-first responsive design

**Scale/Scope**:
- Content: 6 chapters, ~15,000-20,000 words total
- Vectors: <1000 embeddings (300-500 token chunks)
- Users: Optimized for 100-1000 concurrent readers (free-tier constraint)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Simplicity-First Design ✅
- Docusaurus provides built-in navigation, dark mode, search → no custom UI framework needed
- MDX content is simple, version-controlled markdown
- Code examples are static with built-in syntax highlighting

### Principle II: Accuracy & Source Fidelity ✅
- RAG system retrieves from vector DB populated only with textbook chunks
- Confidence threshold (>70%) prevents low-quality answers
- Citations link back to source chapter/section

### Principle III: Free-Tier Architecture ✅
- Qdrant Cloud free tier: 1GB storage, sufficient for <1000 vectors
- Neon Postgres free tier: 512MB, sufficient for metadata
- GitHub Pages/Vercel: Free static hosting
- Sentence Transformers: Free, local embedding generation

### Principle IV: Docusaurus Best Practices Compliance ✅
- All frontend architecture decisions verified against https://docusaurus.io/docs
- Use Docusaurus plugins (@docusaurus/preset-classic)
- Follow recommended file structure (docs/, src/, static/)
- Leverage Docusaurus theming (no custom CSS frameworks)

### Principle V: Minimalism in Technology Stack ✅
- Frontend: Docusaurus v3.x only
- Backend: FastAPI only
- Databases: Qdrant + Neon only
- No additional frameworks or libraries beyond essentials

### Principle VI: Fast Build & Iteration Cycles ✅
- Docusaurus fast refresh during development (<300ms)
- Production build target <60s (monitored in CI)
- RAG re-indexing <5 minutes

### Principle VII: Content-First Development ✅
- 6 chapters authored before advanced features
- Content structured per specification (Introduction, Conceptual Foundation, etc.)

### Principle VIII: RAG Chatbot Guardrails ✅
- Answers only from textbook chunks
- Fallback message for out-of-scope queries
- Response time <3s with timeout handling

**Result**: ✅ All constitutional principles satisfied. No violations to justify.

## Project Structure

### Documentation (this feature)

```text
specs/textbook-generation/
├── spec.md              # Feature specification (/sp.specify output)
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Research findings
├── data-model.md        # Phase 1: Entity and data structure design
├── quickstart.md        # Phase 1: Developer setup guide
├── contracts/           # Phase 1: API contracts (OpenAPI specs)
│   ├── chatbot-api.yaml # RAG chatbot API contract
│   └── admin-api.yaml   # Content indexing API contract (internal)
└── tasks.md             # Phase 2: Generated by /sp.tasks (NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend: Docusaurus static site
docs/                    # Textbook content (MDX files)
├── intro.md             # Landing page
├── chapter-01/          # Chapter 1: Introduction to Physical AI
│   ├── index.md         # Chapter introduction
│   ├── what-is-physical-ai.md
│   ├── history.md
│   ├── applications.md
│   └── self-assessment.md
├── chapter-02/          # Chapter 2: Basics of Humanoid Robotics
│   ├── index.md
│   ├── components.md
│   ├── kinematics.md
│   └── self-assessment.md
├── chapter-03/          # Chapter 3: ROS 2 Fundamentals
│   ├── index.md
│   ├── architecture.md
│   ├── nodes-topics.md
│   ├── services-actions.md
│   └── self-assessment.md
├── chapter-04/          # Chapter 4: Digital Twin Simulation
│   ├── index.md
│   ├── gazebo-basics.md
│   ├── isaac-sim.md
│   └── self-assessment.md
├── chapter-05/          # Chapter 5: Vision-Language-Action Systems
│   ├── index.md
│   ├── vla-architecture.md
│   ├── integration.md
│   └── self-assessment.md
└── chapter-06/          # Chapter 6: Capstone Project
    ├── index.md
    ├── project-overview.md
    ├── implementation.md
    └── self-assessment.md

src/                     # Docusaurus custom components
├── components/          # React components for custom features
│   ├── ChatbotWidget/   # Floating chatbot button and modal
│   │   ├── index.tsx
│   │   ├── ChatbotWidget.module.css
│   │   ├── ChatMessage.tsx
│   │   └── TextSelectionTooltip.tsx
│   └── ProgressTracker/ # Optional: Chapter progress UI
│       ├── index.tsx
│       └── ProgressTracker.module.css
├── css/                 # Docusaurus custom CSS (minimal)
│   └── custom.css       # Theme customizations only
└── pages/               # Custom pages (optional)
    └── index.tsx        # Custom homepage (if not using docs homepage)

static/                  # Static assets
├── img/                 # Diagrams, illustrations
│   ├── logo.svg
│   ├── chapter-01/      # Chapter-specific images
│   ├── chapter-02/
│   └── ...
└── favicon.ico

docusaurus.config.js     # Docusaurus configuration
sidebars.js              # Sidebar configuration (auto-generated preferred)
package.json             # Node.js dependencies
tsconfig.json            # TypeScript configuration (optional)

# Backend: FastAPI RAG chatbot
backend/
├── src/
│   ├── main.py          # FastAPI app entry point
│   ├── config.py        # Environment variables, settings
│   ├── models/          # Pydantic models for request/response
│   │   ├── __init__.py
│   │   ├── chat.py      # ChatRequest, ChatResponse models
│   │   └── metadata.py  # ChapterMetadata, ContentChunk models
│   ├── services/        # Business logic
│   │   ├── __init__.py
│   │   ├── embeddings.py # Sentence Transformers embedding generation
│   │   ├── vector_db.py  # Qdrant client and search
│   │   ├── postgres.py   # Neon Postgres client for metadata
│   │   └── rag.py        # RAG orchestration (query → retrieve → respond)
│   ├── api/             # API routes
│   │   ├── __init__.py
│   │   ├── chat.py      # POST /chat endpoint
│   │   └── health.py    # GET /health endpoint
│   └── utils/           # Utilities
│       ├── __init__.py
│       ├── chunking.py  # Text chunking for embeddings
│       └── rate_limit.py # Rate limiting middleware
├── scripts/             # Operational scripts
│   ├── index_content.py # Generate embeddings and populate Qdrant
│   └── migrate_db.py    # Neon Postgres schema migration
├── tests/
│   ├── unit/            # Unit tests for services
│   │   ├── test_embeddings.py
│   │   ├── test_vector_db.py
│   │   └── test_rag.py
│   ├── integration/     # Integration tests (DB, API)
│   │   ├── test_chat_api.py
│   │   └── test_indexing.py
│   └── fixtures/        # Test data (sample chapters, chunks)
│       └── sample_content.json
├── requirements.txt     # Python dependencies
├── requirements-dev.txt # Development dependencies (pytest, etc.)
└── .env.example         # Environment variable template

# CI/CD and deployment
.github/
└── workflows/
    ├── frontend-deploy.yml  # Deploy Docusaurus to GitHub Pages/Vercel
    ├── backend-deploy.yml   # Deploy FastAPI to Vercel/Railway
    ├── lighthouse-ci.yml    # Lighthouse performance checks
    └── test.yml             # Run tests on PR

# Project metadata
README.md                # Project overview, setup instructions
LICENSE                  # License file (CC BY-SA 4.0 for content, MIT for code)
.gitignore               # Git ignore rules
```

**Structure Decision**: Web application architecture with clear frontend/backend separation. Docusaurus frontend follows official recommended structure (https://docusaurus.io/docs/installation#project-structure). Backend follows FastAPI best practices with service layer pattern for clean separation of concerns.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations to track.** All complexity is justified by functional requirements and stays within constitutional principles.

---

## Phase 0: Research & Technology Validation

### 0.1 Research Tasks

#### RT-001: Docusaurus v3.x Architecture Deep Dive
**Objective**: Understand Docusaurus architecture, plugin system, theming, and deployment options to ensure all frontend decisions align with best practices.

**Research Questions**:
1. What is the recommended file structure for docs-only sites? (https://docusaurus.io/docs/installation#project-structure)
2. How does Docusaurus handle MDX and custom React components? (https://docusaurus.io/docs/markdown-features/react)
3. What are the built-in search options (local vs Algolia)? (https://docusaurus.io/docs/search)
4. How to customize theme without breaking updates? (https://docusaurus.io/docs/styling-layout)
5. What deployment options are recommended? (https://docusaurus.io/docs/deployment)

**Artifacts**:
- `research.md` section: "Docusaurus Architecture"
- Decision matrix for search provider (local vs Algolia DocSearch)

**Sources**:
- Docusaurus. (2024). *Introduction*. https://docusaurus.io/docs
- Docusaurus. (2024). *Docs introduction*. https://docusaurus.io/docs/docs-introduction
- Docusaurus. (2024). *Deployment*. https://docusaurus.io/docs/deployment

---

#### RT-002: RAG Architecture Patterns for Educational Content
**Objective**: Research best practices for chunking, embedding, and retrieval in educational/textbook contexts to optimize answer quality.

**Research Questions**:
1. What chunk size (tokens) optimizes retrieval precision for educational content?
2. How to handle code blocks vs prose text in chunking strategy?
3. What similarity threshold (cosine similarity) indicates high-confidence retrieval?
4. How to structure metadata (chapter, section, page) for citation generation?
5. What prompt engineering techniques prevent hallucinations in RAG systems?

**Artifacts**:
- `research.md` section: "RAG Chunking and Retrieval Strategy"
- Decision: Chunk size (300-500 tokens recommended in spec, validate)
- Decision: Similarity threshold (tentative >0.7, validate)

**Sources**:
- Gao, Y., et al. (2023). *Retrieval-Augmented Generation for Knowledge-Intensive NLP Tasks*. arXiv. https://arxiv.org/abs/2005.11401
- Qdrant. (2024). *Best practices for search*. https://qdrant.tech/documentation/tutorials/optimize/
- OpenAI. (2024). *Prompt engineering guide*. https://platform.openai.com/docs/guides/prompt-engineering

---

#### RT-003: Sentence Transformers Model Selection
**Objective**: Validate all-MiniLM-L6-v2 for this use case or identify better alternatives within 384-dimension constraint.

**Research Questions**:
1. Is all-MiniLM-L6-v2 optimized for educational/technical content?
2. Are there newer models with similar dimensions and better performance?
3. What is the embedding generation speed (embeddings/second) on CPU?
4. How does model size impact deployment to Vercel serverless?

**Artifacts**:
- `research.md` section: "Embedding Model Evaluation"
- Decision: Confirm all-MiniLM-L6-v2 or propose alternative

**Sources**:
- Reimers, N., & Gurevych, I. (2019). *Sentence-BERT: Sentence Embeddings using Siamese BERT-Networks*. arXiv. https://arxiv.org/abs/1908.10084
- Hugging Face. (2024). *sentence-transformers/all-MiniLM-L6-v2*. https://huggingface.co/sentence-transformers/all-MiniLM-L6-v2
- MTEB Leaderboard. (2024). *Massive Text Embedding Benchmark*. https://huggingface.co/spaces/mteb/leaderboard

---

#### RT-004: Qdrant Cloud Free Tier Capacity Planning
**Objective**: Verify free-tier limits and calculate maximum content volume before hitting constraints.

**Research Questions**:
1. What are exact limits (storage, vectors, requests/month)?
2. How to estimate vector count from word count and chunk size?
3. What happens when limit is approached (warnings, hard cutoff)?
4. What is migration path if free tier is exceeded?

**Artifacts**:
- `research.md` section: "Qdrant Capacity Planning"
- Calculation: 20,000 words → ~X chunks → Y vectors (validate <1000)

**Sources**:
- Qdrant. (2024). *Cloud pricing*. https://qdrant.tech/pricing/
- Qdrant. (2024). *Collections*. https://qdrant.tech/documentation/concepts/collections/

---

#### RT-005: Neon Postgres Schema Design for Metadata
**Objective**: Design minimal schema for chapter/section metadata with free-tier constraints.

**Research Questions**:
1. What metadata is essential vs nice-to-have?
2. Should we use UUIDs or integer IDs for chunks?
3. How to index for fast lookups (chapter_id, section_id)?
4. What is Neon free-tier connection limit?

**Artifacts**:
- `research.md` section: "Neon Postgres Schema"
- Schema draft in `data-model.md`

**Sources**:
- Neon. (2024). *Free tier*. https://neon.tech/pricing
- Neon. (2024). *Branching*. https://neon.tech/docs/introduction/branching

---

#### RT-006: Text Selection UI Patterns
**Objective**: Research browser-compatible approaches for "select text → Ask AI" feature.

**Research Questions**:
1. How to detect text selection across browsers (Safari, Firefox, Chrome)?
2. What UI patterns work on mobile (touch selection)?
3. How to position tooltip/button near selection?
4. Are there existing libraries (e.g., tippy.js, floating-ui)?

**Artifacts**:
- `research.md` section: "Text Selection UI Implementation"
- Decision: Library vs custom implementation

**Sources**:
- MDN. (2024). *Selection API*. https://developer.mozilla.org/en-US/docs/Web/API/Selection
- Floating UI. (2024). *Introduction*. https://floating-ui.com/
- Tippy.js. (2024). *Documentation*. https://atomiks.github.io/tippyjs/

---

#### RT-007: Deployment Options Comparison
**Objective**: Compare GitHub Pages vs Vercel for frontend, and Vercel vs Railway vs Render for backend.

**Research Questions**:
1. GitHub Pages: Custom domain support, HTTPS, build limits?
2. Vercel: Serverless function limits (execution time, memory)?
3. Railway/Render: Free-tier limits for FastAPI backend?
4. Which platform has best integration with GitHub Actions?

**Artifacts**:
- `research.md` section: "Deployment Platform Selection"
- Decision matrix with tradeoffs

**Sources**:
- GitHub. (2024). *About GitHub Pages*. https://docs.github.com/en/pages/getting-started-with-github-pages/about-github-pages
- Vercel. (2024). *Limits*. https://vercel.com/docs/limits/overview
- Railway. (2024). *Pricing*. https://railway.app/pricing
- Render. (2024). *Free tier*. https://render.com/pricing

---

### 0.2 Research Output: `research.md` Structure

```markdown
# Research Findings: Textbook Generation

## 1. Docusaurus Architecture
[RT-001 findings]

**Decision**: Use docs-only mode with `docs` as root. No blog or custom pages initially.

**Rationale**: Simplifies structure, leverages auto-generated sidebar.

**Alternatives Considered**: Docs + custom homepage. Rejected: adds complexity without value for MVP.

**Reference**: Docusaurus. (2024). *Docs-only mode*. https://docusaurus.io/docs/docs-introduction#docs-only-mode

---

## 2. RAG Chunking Strategy
[RT-002 findings]

**Decision**: 400-token chunks with 50-token overlap. Separate chunking for code blocks.

**Rationale**: Balance between context and precision. Overlap prevents boundary issues.

**Alternatives Considered**: 300-token (rejected: too granular), 600-token (rejected: loses precision).

**Reference**: [Citations from research]

---

[Continue for RT-003 through RT-007...]
```

---

## Phase 1: Foundation & Contracts

### 1.1 Data Model Design

**Output File**: `data-model.md`

#### Entity: Chapter
**Purpose**: Represents a textbook chapter.

**Attributes**:
- `id` (integer, primary key): Chapter number (1-6)
- `title` (string, max 100 chars): Chapter title
- `slug` (string, max 50 chars): URL-friendly slug
- `description` (text): Brief chapter description for metadata
- `estimated_reading_minutes` (integer): Estimated reading time
- `order` (integer): Display order (same as id for this project)
- `created_at` (timestamp): Creation timestamp
- `updated_at` (timestamp): Last update timestamp

**Validation Rules**:
- `id` must be 1-6
- `slug` must match pattern `^[a-z0-9-]+$`
- `title` cannot be empty

**Storage**: Neon Postgres (`chapters` table)

---

#### Entity: Section
**Purpose**: Represents a section within a chapter.

**Attributes**:
- `id` (UUID, primary key): Unique section identifier
- `chapter_id` (integer, foreign key → chapters.id): Parent chapter
- `title` (string, max 200 chars): Section title
- `slug` (string, max 100 chars): URL-friendly slug
- `heading_level` (integer): Markdown heading level (2-4)
- `order` (integer): Display order within chapter
- `content_type` (enum): One of [text, code, diagram, assessment]

**Relationships**:
- Many sections belong to one chapter

**Storage**: Neon Postgres (`sections` table)

---

#### Entity: ContentChunk
**Purpose**: Represents a text chunk for RAG retrieval.

**Attributes**:
- `id` (UUID, primary key): Unique chunk identifier
- `chapter_id` (integer, foreign key → chapters.id): Source chapter
- `section_id` (UUID, nullable, foreign key → sections.id): Source section (if applicable)
- `text` (text): Chunk content (300-500 tokens)
- `token_count` (integer): Exact token count
- `embedding_vector` (vector[384]): Sentence Transformer embedding
- `metadata` (JSONB): Additional context (page reference, code language, etc.)
- `created_at` (timestamp): Creation timestamp

**Validation Rules**:
- `token_count` must be 300-500
- `embedding_vector` must have exactly 384 dimensions
- `text` cannot be empty

**Storage**:
- Metadata: Neon Postgres (`content_chunks` table)
- Embeddings: Qdrant Cloud (collection: `textbook_chunks`)

**Qdrant Collection Schema**:
```json
{
  "vector_size": 384,
  "distance": "Cosine",
  "payload": {
    "chunk_id": "uuid",
    "chapter_id": "integer",
    "section_id": "uuid or null",
    "text": "string",
    "token_count": "integer",
    "metadata": "object"
  }
}
```

---

#### Entity: ChatSession
**Purpose**: Tracks user chat sessions (optional, Phase 2).

**Attributes**:
- `session_id` (UUID, primary key): Unique session identifier
- `started_at` (timestamp): Session start time
- `last_activity_at` (timestamp): Last interaction time
- `message_count` (integer): Number of messages in session

**Storage**: Browser sessionStorage (not persisted to backend for MVP)

---

#### Entity: ChatMessage
**Purpose**: Represents a chat message (user query or bot response).

**Attributes**:
- `id` (UUID): Message identifier
- `session_id` (UUID): Parent session
- `role` (enum): One of [user, assistant]
- `content` (text): Message content
- `source_chunks` (array of UUIDs, optional): Retrieved chunk IDs (for assistant messages)
- `confidence_score` (float, optional): Retrieval confidence (0-1)
- `timestamp` (timestamp): Message creation time

**Storage**: Browser sessionStorage (not persisted to backend for MVP)

---

### 1.2 API Contracts

**Output Directory**: `contracts/`

#### Contract 1: Chatbot API

**File**: `contracts/chatbot-api.yaml`

```yaml
openapi: 3.0.3
info:
  title: Textbook RAG Chatbot API
  description: API for querying textbook content via RAG system
  version: 1.0.0
  contact:
    name: API Support
    email: support@example.com

servers:
  - url: https://api.textbook.example.com/v1
    description: Production server
  - url: http://localhost:8000/v1
    description: Local development server

paths:
  /chat:
    post:
      summary: Submit a question to the RAG chatbot
      description: |
        Retrieves relevant textbook content chunks and generates an answer.
        Returns citation information for source material.
      operationId: postChatQuery
      tags:
        - Chat
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/ChatRequest'
      responses:
        '200':
          description: Successful response with answer and citations
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ChatResponse'
        '400':
          description: Invalid request (empty query, query too long)
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '429':
          description: Rate limit exceeded
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'
        '500':
          description: Internal server error
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /health:
    get:
      summary: Health check endpoint
      description: Returns API health status and dependency availability
      operationId: getHealth
      tags:
        - System
      responses:
        '200':
          description: Service is healthy
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HealthResponse'
        '503':
          description: Service is unhealthy (database unavailable, etc.)
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/HealthResponse'

components:
  schemas:
    ChatRequest:
      type: object
      required:
        - query
      properties:
        query:
          type: string
          description: User's question or search query
          minLength: 1
          maxLength: 500
          example: "What are the key components of a humanoid robot?"
        session_id:
          type: string
          format: uuid
          description: Optional session ID for tracking conversation context
          example: "550e8400-e29b-41d4-a716-446655440000"
        context:
          type: string
          description: Optional additional context (e.g., selected text from page)
          maxLength: 1000
          example: "Humanoid robots typically have arms, legs, and a torso..."

    ChatResponse:
      type: object
      required:
        - answer
        - confidence
        - sources
      properties:
        answer:
          type: string
          description: Generated answer based on retrieved content
          example: "Humanoid robots have several key components: actuators for movement, sensors for perception, a control system for decision-making, and a power supply."
        confidence:
          type: number
          format: float
          minimum: 0
          maximum: 1
          description: Confidence score for the answer (based on retrieval similarity)
          example: 0.87
        sources:
          type: array
          description: Citations to source chapters/sections
          items:
            $ref: '#/components/schemas/Citation'
        fallback_message:
          type: string
          nullable: true
          description: Message shown when confidence is low or query is out of scope
          example: "I found some related information, but I'm not confident it fully answers your question. Please refer to Chapter 2: Basics of Humanoid Robotics."
        retrieved_chunks:
          type: integer
          description: Number of chunks retrieved for this query
          example: 3

    Citation:
      type: object
      required:
        - chapter_id
        - chapter_title
      properties:
        chapter_id:
          type: integer
          minimum: 1
          maximum: 6
          description: Chapter number
          example: 2
        chapter_title:
          type: string
          description: Chapter title
          example: "Basics of Humanoid Robotics"
        section_id:
          type: string
          format: uuid
          nullable: true
          description: Section ID (if applicable)
        section_title:
          type: string
          nullable: true
          description: Section title
          example: "Key Components"
        url:
          type: string
          format: uri
          description: Direct link to source content
          example: "https://textbook.example.com/chapter-02/components#key-components"

    ErrorResponse:
      type: object
      required:
        - error
        - message
      properties:
        error:
          type: string
          description: Error code
          example: "RATE_LIMIT_EXCEEDED"
        message:
          type: string
          description: Human-readable error message
          example: "You have exceeded the rate limit of 20 requests per minute. Please try again later."
        details:
          type: object
          nullable: true
          description: Additional error details (for debugging)

    HealthResponse:
      type: object
      required:
        - status
        - timestamp
      properties:
        status:
          type: string
          enum: [healthy, unhealthy]
          description: Overall service health
          example: "healthy"
        timestamp:
          type: string
          format: date-time
          description: Health check timestamp
          example: "2025-12-06T14:30:00Z"
        dependencies:
          type: object
          description: Health status of dependencies
          properties:
            qdrant:
              type: string
              enum: [up, down]
              example: "up"
            postgres:
              type: string
              enum: [up, down]
              example: "up"
        version:
          type: string
          description: API version
          example: "1.0.0"
```

---

#### Contract 2: Admin/Indexing API (Internal)

**File**: `contracts/admin-api.yaml`

```yaml
openapi: 3.0.3
info:
  title: Textbook Indexing API (Internal)
  description: Internal API for content indexing and RAG system management
  version: 1.0.0

servers:
  - url: http://localhost:8000/admin
    description: Local development only (not exposed to public)

paths:
  /index:
    post:
      summary: Re-index textbook content
      description: |
        Reads all MDX files, chunks content, generates embeddings, and updates
        Qdrant and Neon databases. This is a long-running operation (up to 5 minutes).
      operationId: postReindex
      tags:
        - Indexing
      requestBody:
        required: false
        content:
          application/json:
            schema:
              type: object
              properties:
                force:
                  type: boolean
                  description: Force re-indexing even if content hash matches
                  default: false
                chapters:
                  type: array
                  description: Specific chapters to re-index (default: all)
                  items:
                    type: integer
                    minimum: 1
                    maximum: 6
      responses:
        '202':
          description: Indexing job started
          content:
            application/json:
              schema:
                type: object
                properties:
                  job_id:
                    type: string
                    format: uuid
                  status:
                    type: string
                    enum: [processing]
                  estimated_duration_seconds:
                    type: integer
                    example: 300
        '500':
          description: Indexing failed
          content:
            application/json:
              schema:
                $ref: '#/components/schemas/ErrorResponse'

  /index/{job_id}:
    get:
      summary: Check indexing job status
      operationId: getIndexingStatus
      tags:
        - Indexing
      parameters:
        - name: job_id
          in: path
          required: true
          schema:
            type: string
            format: uuid
      responses:
        '200':
          description: Job status
          content:
            application/json:
              schema:
                type: object
                properties:
                  job_id:
                    type: string
                    format: uuid
                  status:
                    type: string
                    enum: [processing, completed, failed]
                  progress_percent:
                    type: integer
                    minimum: 0
                    maximum: 100
                  chunks_created:
                    type: integer
                  errors:
                    type: array
                    items:
                      type: string

components:
  schemas:
    ErrorResponse:
      type: object
      required:
        - error
        - message
      properties:
        error:
          type: string
        message:
          type: string
```

---

### 1.3 Quickstart Guide

**Output File**: `quickstart.md`

```markdown
# Quickstart: Textbook Generation Development

## Prerequisites

- Node.js 18.x or 20.x
- Python 3.10+
- Git
- Qdrant Cloud account (free tier)
- Neon Postgres account (free tier)

## Frontend Setup (Docusaurus)

### 1. Install Dependencies

\`\`\`bash
# Navigate to project root
cd "E:\\IT\\GIAIC\\GIAIC - QUARTER 4\\HACKATHONS\\Physical AI & Humanoid Robotics"

# Install Node.js dependencies
npm install
\`\`\`

### 2. Configure Docusaurus

Edit `docusaurus.config.js`:

\`\`\`javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Learn Physical AI and Humanoid Robotics',
  url: 'https://your-username.github.io',
  baseUrl: '/your-repo-name/',
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  favicon: 'img/favicon.ico',

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          routeBasePath: '/', // Docs-only mode
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Logo',
        src: 'img/logo.svg',
      },
    },
    footer: {
      copyright: \`Copyright © \${new Date().getFullYear()} Physical AI Project\`,
    },
    colorMode: {
      defaultMode: 'light',
      respectPrefersColorScheme: true,
    },
  },
};
\`\`\`

### 3. Start Development Server

\`\`\`bash
npm start
\`\`\`

Docusaurus will start at http://localhost:3000

### 4. Build for Production

\`\`\`bash
npm run build
\`\`\`

Static files generated in `build/` directory.

---

## Backend Setup (FastAPI)

### 1. Install Python Dependencies

\`\`\`bash
cd backend

# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\\Scripts\\activate

# Activate (Linux/macOS)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
\`\`\`

### 2. Configure Environment Variables

Create `backend/.env`:

\`\`\`env
# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key

# Neon Postgres
DATABASE_URL=postgresql://user:password@your-cluster.neon.tech/dbname

# API Settings
API_RATE_LIMIT=20
API_CORS_ORIGINS=http://localhost:3000,https://your-username.github.io

# Embedding Model
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2
EMBEDDING_DIMENSION=384
\`\`\`

### 3. Run Database Migrations

\`\`\`bash
python scripts/migrate_db.py
\`\`\`

### 4. Index Textbook Content

\`\`\`bash
python scripts/index_content.py --chapters all
\`\`\`

This will:
- Read all MDX files from `docs/`
- Chunk content (400 tokens, 50 overlap)
- Generate embeddings
- Populate Qdrant and Neon

Expected duration: 3-5 minutes for 6 chapters.

### 5. Start FastAPI Server

\`\`\`bash
uvicorn src.main:app --reload --port 8000
\`\`\`

API available at http://localhost:8000

Test health endpoint:
\`\`\`bash
curl http://localhost:8000/health
\`\`\`

---

## Testing the Integration

### 1. Test Chatbot API

\`\`\`bash
curl -X POST http://localhost:8000/v1/chat \\
  -H "Content-Type: application/json" \\
  -d '{
    "query": "What is Physical AI?"
  }'
\`\`\`

Expected response:
\`\`\`json
{
  "answer": "Physical AI refers to...",
  "confidence": 0.89,
  "sources": [
    {
      "chapter_id": 1,
      "chapter_title": "Introduction to Physical AI",
      "url": "http://localhost:3000/chapter-01/"
    }
  ]
}
\`\`\`

### 2. Test Frontend Chatbot Widget

1. Start both servers (frontend on :3000, backend on :8000)
2. Navigate to http://localhost:3000
3. Click chatbot widget (floating button)
4. Type a question
5. Verify answer appears with citations

---

## Deployment

### Frontend (GitHub Pages)

\`\`\`bash
# Configure GitHub Pages in repo settings
# Set source to: gh-pages branch

# Deploy
npm run deploy
\`\`\`

### Backend (Vercel)

\`\`\`bash
# Install Vercel CLI
npm install -g vercel

# Deploy
cd backend
vercel --prod
\`\`\`

---

## Troubleshooting

**Issue**: Docusaurus build fails with "Cannot find module"
**Solution**: Delete `node_modules` and `package-lock.json`, run `npm install` again

**Issue**: Backend 500 error on /chat
**Solution**: Check Qdrant and Neon credentials in `.env`, verify connectivity

**Issue**: Chatbot returns empty answers
**Solution**: Re-run indexing script: `python scripts/index_content.py --force`

---

## Next Steps

1. Write content for Chapter 1 in `docs/chapter-01/`
2. Run indexing script after adding content
3. Test chatbot with chapter-specific questions
4. Repeat for Chapters 2-6
\`\`\`

---

### 1.4 Update Agent Context

**Action**: Run agent context update script after completing Phase 1.

\`\`\`powershell
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
\`\`\`

This script will:
- Detect Claude agent context file (CLAUDE.md)
- Add Docusaurus, FastAPI, Qdrant, Neon to technology list
- Preserve manual additions between markers
- Update context for future planning/implementation phases

---

## Phase 2: Technical Sections & Tasks

**Note**: Phase 2 (task generation) is handled by `/sp.tasks` command, not `/sp.plan`.

This plan provides the foundation for task generation:
- Architecture decisions documented
- Data models defined
- API contracts specified
- Research questions resolved

### Pre-Task Generation Checklist

Before running `/sp.tasks`, ensure:
- [ ] `research.md` is complete with all decisions documented
- [ ] `data-model.md` defines all entities and relationships
- [ ] API contracts in `contracts/` are validated (OpenAPI linter)
- [ ] `quickstart.md` is tested with fresh environment
- [ ] Constitution Check re-evaluated (no new violations)

---

## Architecture Diagrams

### System Architecture

\`\`\`
┌─────────────────────────────────────────────────────────────┐
│                        User Browser                          │
│  ┌──────────────────────────────────────────────────────┐  │
│  │         Docusaurus Static Site (React)                │  │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────────┐  │  │
│  │  │ MDX Content │  │ ChatWidget   │  │ Navigation │  │  │
│  │  │ (Chapters)  │  │ (React)      │  │ (Sidebar)  │  │  │
│  │  └─────────────┘  └──────┬───────┘  └────────────┘  │  │
│  │                           │ POST /v1/chat             │  │
│  └───────────────────────────┼───────────────────────────┘  │
└────────────────────────────┼──────────────────────────────┘
                              │
                              ▼
              ┌───────────────────────────────┐
              │   FastAPI Backend (Python)    │
              │  ┌─────────────────────────┐  │
              │  │   /v1/chat Endpoint     │  │
              │  └──────────┬──────────────┘  │
              │             │                  │
              │  ┌──────────▼──────────────┐  │
              │  │   RAG Service           │  │
              │  │  - Query embedding      │  │
              │  │  - Vector search        │  │
              │  │  - Response generation  │  │
              │  └──┬───────────────┬──────┘  │
              └─────┼───────────────┼─────────┘
                    │               │
        ┌───────────▼──┐      ┌────▼────────────┐
        │   Qdrant     │      │  Neon Postgres  │
        │ Vector DB    │      │  (Metadata)     │
        │              │      │                 │
        │ - Embeddings │      │ - Chapters      │
        │ - Search     │      │ - Sections      │
        │ - Top-k      │      │ - Chunk refs    │
        └──────────────┘      └─────────────────┘
\`\`\`

### RAG Query Flow

\`\`\`
User Query: "What is Physical AI?"
    │
    ▼
┌─────────────────────────┐
│ 1. Embed Query          │
│ sentence-transformers   │
│ → 384-dim vector        │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│ 2. Vector Search        │
│ Qdrant.search()         │
│ top_k=3, threshold=0.7  │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│ 3. Retrieve Chunks      │
│ [Chunk1, Chunk2, Chunk3]│
│ with metadata (chapter) │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│ 4. Generate Answer      │
│ Prompt: "Based on..."   │
│ (No LLM for MVP,        │
│  return top chunk text) │
└───────────┬─────────────┘
            │
            ▼
┌─────────────────────────┐
│ 5. Format Response      │
│ {                       │
│   answer: "...",        │
│   confidence: 0.89,     │
│   sources: [...]        │
│ }                       │
└─────────────────────────┘
\`\`\`

**Note for MVP**: Answer generation can be simple concatenation of top chunks. Full LLM-based generation is optional (Phase 3) if using free models like Ollama locally.

---

## Quality Validation Mapping

### Frontend Quality Checks

| Acceptance Criterion | Technical Check | Tool/Method |
|---------------------|-----------------|-------------|
| Mobile responsive (320px+) | Viewport meta tag, breakpoints | Chrome DevTools, responsive mode |
| Dark mode functional | Theme toggle state persistence | localStorage inspection |
| Sidebar highlights current page | Active link class applied | DOM inspection |
| Code blocks have copy button | Docusaurus prism-react-renderer | Manual test, screenshot |
| Lighthouse Performance >90 | Core Web Vitals metrics | Lighthouse CI in GitHub Actions |
| Build time <60s | CI pipeline duration | GitHub Actions logs, timer |

### Backend Quality Checks

| Acceptance Criterion | Technical Check | Tool/Method |
|---------------------|-----------------|-------------|
| Chat response <3s (p95) | API response time histogram | Pytest with time assertions |
| Vector search <500ms | Qdrant query latency | Qdrant monitoring dashboard |
| Chatbot accuracy >90% | Manual test set (30 questions) | Test suite with expected answers |
| Confidence threshold prevents bad answers | Low-similarity queries return fallback | Unit test with synthetic queries |
| Rate limiting enforced | 21st request in 1 minute returns 429 | Integration test |

### Integration Quality Checks

| Acceptance Criterion | Technical Check | Tool/Method |
|---------------------|-----------------|-------------|
| Frontend can call backend | CORS headers allow origin | Browser network tab, Postman |
| Chatbot widget displays answers | React state updates on API response | React DevTools, console logs |
| Citations link to correct chapters | URL matches chapter slug | Automated link checker |
| Re-indexing updates search results | Query returns new content after index | Integration test |

---

## Architectural Decisions Requiring ADRs

Based on Phase 0 research, the following decisions may warrant ADRs:

### ADR-001: Search Provider Selection (Local vs Algolia DocSearch)
**Context**: Docusaurus supports local search (plugin) or Algolia DocSearch (free for open-source).
**Decision**: Use Docusaurus built-in local search for MVP
**Rationale**:
- 6-chapter textbook is small enough for local search performance
- No external dependencies maintains free-tier architecture (Principle III)
- Works offline and doesn't require API keys
- Algolia can be added later if search quality becomes insufficient
- Aligns with FR-011 requirement: "Docusaurus built-in search (local or Algolia DocSearch)"
**Tradeoffs**:
- Local: ✅ No external dependency, works offline, sufficient for 6 chapters
- Algolia: ❌ Adds external service dependency for minimal benefit at this scale

**Implementation**: Configure @docusaurus/theme-search-algolia with local indexing or use @easyops-cn/docusaurus-search-local plugin.

---

### ADR-002: RAG Answer Generation Strategy
**Context**: RAG systems can return raw chunks or use LLM to synthesize answers.
**Decision**: Use raw chunk concatenation for MVP, with optional Ollama synthesis for Phase 2+
**Rationale**:
- Raw chunks eliminate hallucination risk (Principle II: Accuracy & Source Fidelity)
- No paid API costs maintains free-tier constraint (TC-006, Principle III)
- Simple implementation reduces complexity (Principle V: Minimalism)
- Direct chunk display allows users to verify source accuracy
- Citations naturally map to retrieved chunks
- Ollama can be added later if answer coherence becomes critical
**Tradeoffs**:
- Raw chunks: ✅ Simple, fast, zero hallucination risk, maintains free tier
- LLM synthesis: ❌ Adds complexity, potential hallucinations, requires local Ollama or paid API

**Implementation**: backend/src/services/rag.py returns top-k chunks (k=3) concatenated with section headers. Each chunk includes citation (chapter ID, section, URL).

---

### ADR-003: Backend Deployment Platform
**Context**: Multiple free-tier options (Vercel, Railway, Render, Fly.io).
**Decision**: Deploy to Railway free tier for backend, Vercel for frontend
**Rationale**:
- Railway supports longer execution times needed for re-indexing (up to 5 minutes per FR-035)
- Vercel's 10-second serverless limit incompatible with indexing script
- Railway free tier: 500 hours/month, sufficient for low-traffic educational site
- Separates frontend (static) and backend (dynamic) deployment concerns
- Railway provides persistent container for background tasks
- Easy GitHub integration for automated deployments
**Tradeoffs**:
- Railway: ✅ Long execution time, persistent containers, 500h free tier, good for indexing
- Vercel: ❌ 10s execution limit blocks re-indexing operation
- Render: Similar to Railway, but Railway has better DX and GitHub integration

**Implementation**:
- Frontend: Vercel deployment via GitHub Actions (.github/workflows/frontend-deploy.yml)
- Backend: Railway deployment via GitHub Actions (.github/workflows/backend-deploy.yml)
- Backend URL configured in frontend via environment variable

---

## Edge Cases & Fallback Scenarios

### Edge Case 1: Qdrant Free Tier Exceeded
**Scenario**: Content grows beyond 1000 vectors.
**Detection**: Monitor Qdrant dashboard, set alert at 800 vectors.
**Fallback**:
1. Increase chunk size to 600 tokens (reduces count)
2. Remove low-value chunks (e.g., self-assessment questions)
3. If still exceeded, upgrade to paid tier ($10/month minimum)

### Edge Case 2: Neon Postgres Connection Limit
**Scenario**: Too many concurrent connections (free tier: 10).
**Detection**: Database connection errors in logs.
**Fallback**:
1. Implement connection pooling (SQLAlchemy pool_size=5)
2. Close connections properly after each request
3. If still exceeded, evaluate Neon paid tier

### Edge Case 3: Chatbot Query Outside Textbook Scope
**Scenario**: User asks "How do I install Python?" (not in textbook).
**Detection**: Qdrant similarity score <0.5 for all chunks.
**Fallback**: Return message: "This topic is not covered in the current textbook. Please refer to the Further Resources section or search online."

### Edge Case 4: JavaScript Disabled in Browser
**Scenario**: User has JavaScript disabled (rare).
**Detection**: Docusaurus site still renders static HTML.
**Fallback**: Textbook content is readable (core value preserved). Chatbot displays message: "JavaScript required for chatbot. Please enable JavaScript."

### Edge Case 5: Code Block Selection for "Ask AI"
**Scenario**: User selects Python code and clicks "Ask AI".
**Detection**: Selected text contains code fence markers or is within `<pre>` tag.
**Fallback**: Pre-fill chatbot with: "Explain this code: [code]" to provide better context.

---

## Phase Organization & Execution Strategy

### Phase 1: Core MVP (Weeks 1-4)

**Research** (Days 1-3):
- Complete RT-001 through RT-007
- Document decisions in `research.md`
- Resolve all NEEDS CLARIFICATION items

**Foundation** (Days 4-10):
- Initialize Docusaurus project (follow `quickstart.md`)
- Set up FastAPI backend skeleton
- Create Qdrant and Neon accounts, configure credentials
- Write Chapter 1 content (test case for indexing)

**Analysis** (Days 11-14):
- Implement chunking and embedding script
- Test indexing with Chapter 1
- Implement `/v1/chat` endpoint
- Test RAG retrieval with sample queries

**Synthesis** (Days 15-20):
- Implement chatbot widget in Docusaurus
- Integrate frontend with backend API
- Add error handling and fallback messages
- Write remaining chapters (2-6)

**Validation** (Days 21-28):
- Run full indexing for all 6 chapters
- Execute test suite (30-question test set)
- Run Lighthouse CI, verify >90 score
- Deploy to staging (GitHub Pages or Vercel preview)

**Deliverables**:
- [ ] Docusaurus site with 6 chapters deployed
- [ ] RAG chatbot functional with >90% accuracy
- [ ] API response time <3s (p95)
- [ ] All P1 user stories tested and passing

---

### Phase 2: Enhanced UX (Weeks 5-6)

**Research** (Days 1-2):
- Investigate text selection libraries (RT-006)
- Review localStorage best practices for progress tracking

**Foundation** (Days 3-7):
- Implement text selection → Ask AI feature
- Add chapter progress tracking (localStorage)
- Implement progress indicator UI

**Analysis** (Days 8-10):
- Test on mobile devices (iOS, Android)
- Verify touch selection works correctly
- A/B test progress UI placement

**Synthesis** (Days 11-12):
- Refine UX based on testing
- Add documentation for new features

**Validation** (Days 13-14):
- User testing with 5-10 beta users
- Fix bugs and UX issues
- Update test suite

**Deliverables**:
- [ ] Text selection feature working on desktop and mobile
- [ ] Progress tracking persists across sessions
- [ ] All P2 user stories tested and passing

---

### Phase 3: Optional Features (Weeks 7-8, Conditional)

**Decision Point**: Only proceed if:
1. Core MVP metrics are strong (>90% chatbot accuracy, >8min session time)
2. Free-tier budgets have headroom (Qdrant <600 vectors, Neon <300MB)
3. User demand for Urdu or personalization is validated

**Urdu Translation** (if approved):
- Research: Machine translation vs manual translation
- Foundation: Create Urdu MDX files in `docs/ur/` directory
- Analysis: Test Docusaurus i18n plugin
- Synthesis: Implement language toggle, re-index Urdu content
- Validation: Native Urdu speaker review

**Personalized Learning Paths** (if approved):
- Research: Adaptive learning algorithms (simple rule-based)
- Foundation: Track quiz scores in localStorage
- Analysis: Recommend next chapter based on performance
- Synthesis: UI for "Recommended Next Steps"
- Validation: User testing with learners

**Deliverables** (optional):
- [ ] Urdu translation toggle functional
- [ ] Personalized recommendations based on quiz scores

---

## Testing Checklist (Tied to Acceptance Criteria)

### P1 User Stories (Core MVP)

#### US-001: Read Educational Content
- [ ] Homepage loads and displays chapter navigation
- [ ] Chapter 1 page renders with all sections
- [ ] Code blocks have syntax highlighting
- [ ] Copy button works on code blocks
- [ ] Self-assessment questions displayed at end
- [ ] Further Resources links are functional
- [ ] Mobile: Content readable on 320px width
- [ ] Mobile: No horizontal scrolling

#### US-004: Navigate with Auto-Generated Sidebar
- [ ] Sidebar displays all 6 chapters
- [ ] Current chapter/section is highlighted
- [ ] Clicking sidebar link navigates correctly
- [ ] Mobile: Hamburger menu toggles sidebar
- [ ] Mobile: Sidebar drawer dismisses on link click

#### US-007: View Code Examples
- [ ] Python syntax highlighting correct
- [ ] YAML syntax highlighting correct
- [ ] Line numbers displayed
- [ ] Copy button copies exact code (no extra whitespace)
- [ ] Copy button shows "Copied!" feedback

### P2 User Stories (Enhanced Features)

#### US-002: Ask Questions via RAG Chatbot
- [ ] Chatbot widget visible on all pages
- [ ] Clicking widget opens chat modal
- [ ] Typing query and pressing Enter sends request
- [ ] Answer displays within 3 seconds
- [ ] Answer includes citations with chapter links
- [ ] Low-confidence query shows fallback message
- [ ] Out-of-scope query shows appropriate message
- [ ] Chat history preserved during session
- [ ] Rate limit (20/min) enforced, shows error message

#### US-006: Search Textbook Content
- [ ] Search box visible in navbar
- [ ] Typing query shows live results
- [ ] Clicking result navigates to correct page
- [ ] Search highlights matched text on page
- [ ] "No results" message for non-existent terms

#### US-008: Access Further Resources
- [ ] Further Resources section at end of each chapter
- [ ] Links categorized (Official Docs, Tutorials, Community)
- [ ] Links open in new tab
- [ ] All links return 200 status (link checker)

### P3 User Stories (Nice-to-Have)

#### US-003: Select Text and Ask AI
- [ ] Selecting text shows "Ask AI" tooltip
- [ ] Tooltip positioned near selection
- [ ] Clicking tooltip opens chatbot with text pre-filled
- [ ] Works on mobile with long-press selection
- [ ] Works inside code blocks

#### US-005: Toggle Dark Mode
- [ ] Dark mode toggle button visible
- [ ] Clicking toggles theme immediately
- [ ] Theme persists on page refresh
- [ ] Syntax highlighting adapts to theme
- [ ] No FOUC (flash of unstyled content)

### P4 User Stories (Optional)

#### US-009: Urdu Translation (if implemented)
- [ ] Language toggle visible in navbar
- [ ] Switching to Urdu changes all content
- [ ] Urdu chatbot answers in Urdu
- [ ] Language preference persists

#### US-010: Personalize Learning Path (if implemented)
- [ ] Marking chapter complete shows checkmark
- [ ] Progress percentage calculated correctly
- [ ] Progress persists across sessions
- [ ] "Recommended Next" based on quiz scores

---

## Next Steps

1. **Run `/sp.tasks`**: Generate actionable task breakdown from this plan
2. **Create ADRs**: Document key architectural decisions (search provider, RAG strategy, deployment)
3. **Set up repositories**: Initialize Docusaurus and FastAPI projects per `quickstart.md`
4. **Begin Chapter 1 Content**: Write first chapter to validate content pipeline
5. **Implement indexing script**: Test RAG system with real content

---

**Plan Status**: Complete and ready for task generation.
**Branch**: `textbook-generation`
**Date**: 2025-12-06
