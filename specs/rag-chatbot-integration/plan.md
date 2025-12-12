# Implementation Plan: RAG Chatbot Integration

**Branch**: `rag-chatbot-integration` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)

## Executive Summary

Implement a three-phase RAG chatbot system integrating FastAPI backend with Docusaurus frontend. Phase 1 establishes backend infrastructure (FastAPI + Qdrant + Neon). Phase 2 builds the content ingestion pipeline (MD → embeddings → vector DB). Phase 3 creates React UI components and deploys to production. Estimated timeline: 2-3 weeks for full implementation.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────────┐
│                          USER BROWSER                                 │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │              Docusaurus Frontend (React 18.x)                  │  │
│  │                                                                 │  │
│  │  ┌──────────────────┐  ┌──────────────────────────────────┐   │  │
│  │  │  Chapter Pages   │  │  ChatWidget Component            │   │  │
│  │  │  (MDX Content)   │  │  ┌────────────────────────────┐  │   │  │
│  │  │                  │  │  │ • FloatingButton           │  │   │  │
│  │  │  [User selects   │  │  │ • MessageList              │  │   │  │
│  │  │   text: "Isaac   │  │  │ • InputBox                 │  │   │  │
│  │  │   Sim"]          │  │  │ • CitationChip             │  │   │  │
│  │  │         │        │  │  │ • SelectToAskTooltip       │  │   │  │
│  │  │         ▼        │  │  └──────────┬─────────────────┘  │   │  │
│  │  │  ┌─────────────┐ │  │             │                    │   │  │
│  │  │  │ Tooltip:    │ │  │             │                    │   │  │
│  │  │  │ "Ask AI     │◄┼──┤─────────────┘                    │   │  │
│  │  │  │  about this"│ │  │                                  │   │  │
│  │  │  └──────┬──────┘ │  │                                  │   │  │
│  │  │         │        │  │                                  │   │  │
│  │  └─────────┼────────┘  └──────────────┬───────────────────┘   │  │
│  │            │                           │                       │  │
│  │            └───────────────────────────┘                       │  │
│  │                          │                                     │  │
│  │                          │ HTTP POST /api/query               │  │
│  │                          │ {query, context, top_k}            │  │
│  └──────────────────────────┼──────────────────────────────────────┘
│                             │                                       │
└─────────────────────────────┼───────────────────────────────────────┘
                              │
                              │ CORS: Allow origin from GH Pages/Vercel
                              │ Rate Limit: 10 req/min per IP
                              ▼
         ┌────────────────────────────────────────────────────┐
         │       FastAPI Backend (Python 3.10+)               │
         │       Deployed on Railway.app (Free Tier)          │
         │                                                    │
         │  ┌──────────────────────────────────────────────┐ │
         │  │  POST /api/query                             │ │
         │  │  ┌────────────────────────────────────────┐  │ │
         │  │  │ 1. Receive user query + context        │  │ │
         │  │  │ 2. Generate embedding:                 │  │ │
         │  │  │    - Load Sentence Transformer model   │  │ │
         │  │  │    - Encode query → 384-dim vector     │  │ │
         │  │  │    - Latency: ~80ms                    │  │ │
         │  │  │ 3. Search Qdrant:                      │  │ │
         │  │  │    - collection="textbook_embeddings"  │  │ │
         │  │  │    - top_k=5, score_threshold=0.7      │  │ │
         │  │  │    - Latency: ~400ms                   │  │ │
         │  │  │ 4. Format response:                    │  │ │
         │  │  │    - Combine top chunks into answer    │  │ │
         │  │  │    - Extract citations (chapter, URL)  │  │ │
         │  │  │    - Calculate confidence score        │  │ │
         │  │  │ 5. Log to Neon Postgres (async)        │  │ │
         │  │  │ 6. Return JSON response                │  │ │
         │  │  └────────────────────────────────────────┘  │ │
         │  └─────────────┬────────────┬───────────────────┘ │
         │                │            │                      │
         │  ┌─────────────┴──────┐  ┌──┴─────────────────┐   │
         │  │ GET /api/health    │  │ Middleware:        │   │
         │  │ - Check services   │  │ - CORS             │   │
         │  │ - Return status    │  │ - Rate Limiting    │   │
         │  └────────────────────┘  │ - Request Logging  │   │
         │                          └────────────────────┘   │
         └────────────────┬────────────┬──────────────────────┘
                          │            │
                          │            │
         ┌────────────────▼──────┐  ┌─▼──────────────────────┐
         │  Qdrant Cloud         │  │  Neon Postgres         │
         │  (Free Tier)          │  │  (Free Tier)           │
         │                       │  │                        │
         │  Collection:          │  │  Tables:               │
         │  "textbook_embeddings"│  │  - chat_sessions       │
         │                       │  │  - user_feedback       │
         │  Vectors:             │  │  - query_analytics     │
         │  - 384 dimensions     │  │                        │
         │  - ~800 chunks        │  │  Storage: <512MB       │
         │  - Cosine distance    │  │  Connections: 10 max   │
         │  - HNSW index         │  │                        │
         │                       │  │                        │
         │  Storage: ~200MB      │  │                        │
         └───────────────────────┘  └────────────────────────┘
                  ▲
                  │
                  │ Ingestion Pipeline (Offline)
                  │
         ┌────────┴──────────────────────────────────────────┐
         │  Content Ingestion Script (Python)                │
         │  Run locally or via GitHub Actions                │
         │                                                   │
         │  1. Read all MD files from docs/ directory        │
         │  2. Parse frontmatter (chapter, sidebar_position) │
         │  3. Chunk content (300-500 tokens per chunk)      │
         │  4. Generate embeddings (Sentence Transformers)   │
         │  5. Upload to Qdrant with metadata                │
         │  6. Log ingestion stats                           │
         │                                                   │
         │  Execution Time: ~3-5 minutes for 13 chapters     │
         └───────────────────────────────────────────────────┘
```

---

## Phase 1: Backend Infrastructure Setup

**Goal**: Establish FastAPI backend with database connections and basic health checks.

**Duration**: 3-4 days

**Deliverables**:
1. FastAPI project initialized with proper structure
2. Qdrant Cloud account + collection created
3. Neon Postgres database provisioned + tables created
4. Health check endpoint operational
5. CORS and rate limiting middleware configured
6. Deployed to Railway.app with environment variables

### Technical Decisions

#### Backend Project Structure
```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py                 # FastAPI app initialization
│   ├── config.py               # Environment variables, settings
│   ├── models/
│   │   ├── __init__.py
│   │   ├── query.py            # Pydantic models for API
│   │   └── database.py         # SQLAlchemy models
│   ├── services/
│   │   ├── __init__.py
│   │   ├── embeddings.py       # Sentence Transformer wrapper
│   │   ├── qdrant_client.py    # Vector search logic
│   │   └── postgres_client.py  # Database operations
│   ├── routes/
│   │   ├── __init__.py
│   │   ├── query.py            # POST /api/query endpoint
│   │   └── health.py           # GET /api/health endpoint
│   └── middleware/
│       ├── __init__.py
│       ├── cors.py              # CORS configuration
│       └── rate_limit.py        # Rate limiting logic
├── tests/
│   ├── test_query.py
│   ├── test_embeddings.py
│   └── test_qdrant.py
├── requirements.txt
├── .env.example
├── Dockerfile                   # For Railway deployment
└── README.md
```

#### Environment Variables (.env)
```bash
# FastAPI
ENVIRONMENT=production
DEBUG=False
API_HOST=0.0.0.0
API_PORT=8000

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_api_key_here
QDRANT_COLLECTION=textbook_embeddings

# Neon Postgres
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# CORS
ALLOWED_ORIGINS=https://yourusername.github.io,http://localhost:3000

# Rate Limiting
RATE_LIMIT_PER_MINUTE=10

# Embeddings
EMBEDDING_MODEL=sentence-transformers/all-MiniLM-L6-v2
```

#### Qdrant Collection Setup
```python
# Executed during Phase 1 setup
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

client.create_collection(
    collection_name="textbook_embeddings",
    vectors_config=VectorParams(
        size=384,  # all-MiniLM-L6-v2 dimension
        distance=Distance.COSINE
    )
)
```

#### Neon Postgres Schema Creation
```sql
-- Executed during Phase 1 setup

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

---

## Phase 2: Content Ingestion Pipeline

**Goal**: Process all textbook markdown files into vector embeddings and upload to Qdrant.

**Duration**: 2-3 days

**Deliverables**:
1. Python script to read and parse MD files
2. Chunking logic (300-500 tokens per chunk)
3. Embedding generation pipeline
4. Qdrant bulk upload with metadata
5. Ingestion validation and stats logging

### Chunking Strategy

**Approach**: Section-based chunking with overlap

```python
# Pseudocode for chunking logic

def chunk_markdown(md_content, max_tokens=500, overlap=50):
    """
    Split markdown into semantically meaningful chunks.

    Strategy:
    1. Split by H2 headings (## Section Title)
    2. If section > max_tokens, split by paragraphs
    3. Add overlap (last 50 tokens of prev chunk prepended to next)
    4. Preserve code blocks intact (don't split mid-code)
    """
    chunks = []

    # Split by ## headings
    sections = split_by_heading(md_content, level=2)

    for section in sections:
        if token_count(section) <= max_tokens:
            chunks.append(section)
        else:
            # Split large sections by paragraphs
            paragraphs = split_by_paragraph(section)
            current_chunk = ""

            for para in paragraphs:
                if token_count(current_chunk + para) <= max_tokens:
                    current_chunk += para
                else:
                    if current_chunk:
                        chunks.append(current_chunk)
                    # Add overlap from previous chunk
                    overlap_text = get_last_n_tokens(current_chunk, overlap)
                    current_chunk = overlap_text + para

            if current_chunk:
                chunks.append(current_chunk)

    return chunks
```

### Metadata Extraction

```python
# Pseudocode for metadata extraction

def extract_metadata(md_file_path):
    """
    Parse frontmatter and file path to extract metadata.

    Returns:
    {
        "chapter": "3",
        "chapter_title": "ROS 2 Fundamentals",
        "section": "Nodes and Communication",
        "sidebar_position": 3,
        "source_file": "docs/chapter-03/nodes-topics.md",
        "url": "/chapter-03/nodes-topics"
    }
    """
    # Read frontmatter (YAML between --- delimiters)
    with open(md_file_path) as f:
        content = f.read()

    frontmatter = parse_yaml_frontmatter(content)

    # Extract chapter number from file path
    chapter_match = re.search(r'chapter-(\d+)', md_file_path)
    chapter = chapter_match.group(1) if chapter_match else "unknown"

    # Build URL from file path
    url = md_file_path.replace('docs/', '/').replace('.md', '')

    return {
        "chapter": chapter,
        "chapter_title": frontmatter.get("title", ""),
        "sidebar_position": frontmatter.get("sidebar_position", 0),
        "source_file": md_file_path,
        "url": url
    }
```

### Ingestion Script Workflow

```python
# High-level ingestion script (ingest.py)

def main():
    # 1. Initialize services
    embedding_model = load_embedding_model("all-MiniLM-L6-v2")
    qdrant_client = connect_to_qdrant()

    # 2. Read all markdown files
    md_files = glob.glob("docs/chapter-*/*.md")

    total_chunks = 0

    for md_file in md_files:
        print(f"Processing {md_file}...")

        # 3. Parse and chunk
        content = read_file(md_file)
        metadata = extract_metadata(md_file)
        chunks = chunk_markdown(content, max_tokens=500)

        # 4. Generate embeddings
        embeddings = []
        for i, chunk in enumerate(chunks):
            vector = embedding_model.encode(chunk)

            # 5. Prepare payload
            payload = {
                "chunk_id": f"ch{metadata['chapter']}-s{i+1}",
                "content": chunk,
                "token_count": count_tokens(chunk),
                **metadata
            }

            embeddings.append({
                "id": f"{md_file}-{i}",
                "vector": vector.tolist(),
                "payload": payload
            })

        # 6. Upload to Qdrant (batch)
        qdrant_client.upsert(
            collection_name="textbook_embeddings",
            points=embeddings
        )

        total_chunks += len(chunks)
        print(f"  → Uploaded {len(chunks)} chunks")

    print(f"\n✅ Ingestion complete! Total chunks: {total_chunks}")

if __name__ == "__main__":
    main()
```

### Triggering Re-Indexing

**Option A: Manual Execution (Initial Phase)**
```bash
cd backend
python scripts/ingest.py
```

**Option B: GitHub Actions (Future Enhancement)**
```yaml
# .github/workflows/reindex-vectors.yml
name: Re-index Vector Database

on:
  push:
    paths:
      - 'docs/**/*.md'
    branches: [main]

jobs:
  reindex:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-python@v4
        with:
          python-version: '3.10'
      - run: pip install -r backend/requirements.txt
      - run: python backend/scripts/ingest.py
        env:
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
```

---

## Phase 3: Frontend Integration & Deployment

**Goal**: Build React UI components, integrate with backend API, deploy to production.

**Duration**: 5-7 days

**Deliverables**:
1. ChatWidget React component
2. Select-to-Ask tooltip component
3. Citation rendering component
4. API integration with error handling
5. Mobile-responsive UI
6. Production deployment (GH Pages/Vercel + Railway)

### React Component Hierarchy

```
<ChatWidget>
  ├── <FloatingButton />              # Bottom-right corner trigger
  ├── <ChatModal open={isOpen}>       # Overlay modal
  │   ├── <ChatHeader />              # Title + close button
  │   ├── <MessageList>               # Scrollable message container
  │   │   ├── <UserMessage />         # User query bubble
  │   │   └── <BotMessage>            # Bot response bubble
  │   │       ├── <AnswerText />      # Formatted answer
  │   │       └── <CitationList>      # Clickable citations
  │   │           └── <CitationChip /> # Individual citation badge
  │   ├── <LoadingIndicator />        # Spinner during API call
  │   └── <InputBox>                  # Text input + send button
  │       ├── <TextArea />
  │       └── <SendButton />
  └── <SelectToAskTooltip>            # Appears on text selection
      └── <AskAIButton />              # "Ask AI about this"
```

### Component State Management

```javascript
// ChatWidget.jsx - State structure

import { useState, useEffect } from 'react';

function ChatWidget() {
  // UI state
  const [isOpen, setIsOpen] = useState(false);
  const [isLoading, setIsLoading] = useState(false);

  // Chat state
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');

  // Selection state
  const [selectedText, setSelectedText] = useState(null);
  const [tooltipPosition, setTooltipPosition] = useState({ x: 0, y: 0 });

  // API integration
  const handleSendQuery = async (query, context = null) => {
    setIsLoading(true);

    try {
      const response = await fetch('https://your-backend.railway.app/api/query', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ query, context, top_k: 5 })
      });

      const data = await response.json();

      // Add to message history
      setMessages([...messages,
        { type: 'user', content: query },
        { type: 'bot', content: data.answer, citations: data.citations }
      ]);
    } catch (error) {
      console.error('API Error:', error);
      setMessages([...messages,
        { type: 'user', content: query },
        { type: 'bot', content: 'Sorry, the service is temporarily unavailable.' }
      ]);
    } finally {
      setIsLoading(false);
      setInputValue('');
    }
  };

  return (
    <>
      <FloatingButton onClick={() => setIsOpen(true)} />
      {isOpen && (
        <ChatModal>
          {/* Render components... */}
        </ChatModal>
      )}
    </>
  );
}
```

### Select-to-Ask Implementation

```javascript
// SelectToAsk.jsx

import { useEffect, useState } from 'react';

function SelectToAskHandler({ onAskAI }) {
  const [selection, setSelection] = useState(null);
  const [tooltipVisible, setTooltipVisible] = useState(false);

  useEffect(() => {
    const handleTextSelection = () => {
      const selectedText = window.getSelection().toString().trim();

      if (selectedText.length >= 5 && selectedText.length <= 1000) {
        const range = window.getSelection().getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelection({
          text: selectedText,
          position: { x: rect.right, y: rect.bottom }
        });
        setTooltipVisible(true);
      } else {
        setTooltipVisible(false);
      }
    };

    document.addEventListener('mouseup', handleTextSelection);
    document.addEventListener('touchend', handleTextSelection);

    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
      document.removeEventListener('touchend', handleTextSelection);
    };
  }, []);

  const handleAskAI = () => {
    onAskAI({
      query: `Explain: ${selection.text}`,
      context: selection.text
    });
    setTooltipVisible(false);
  };

  return (
    tooltipVisible && (
      <Tooltip position={selection.position}>
        <button onClick={handleAskAI}>
          Ask AI about this 🤖
        </button>
      </Tooltip>
    )
  );
}
```

### Docusaurus Integration

**Option 1: Swizzle Theme Component (Recommended)**
```bash
# Eject Root layout to add ChatWidget
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

Then modify `src/theme/Root.js`:
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

**Option 2: Custom Plugin**
```javascript
// docusaurus.config.js
module.exports = {
  plugins: [
    function chatWidgetPlugin() {
      return {
        name: 'chat-widget-plugin',
        injectHtmlTags() {
          return {
            postBodyTags: ['<div id="chat-widget-root"></div>'],
          };
        },
      };
    },
  ],
};
```

### Styling (CSS Modules)

```css
/* ChatWidget.module.css */

.floatingButton {
  position: fixed;
  bottom: 24px;
  right: 24px;
  width: 60px;
  height: 60px;
  border-radius: 50%;
  background: var(--ifm-color-primary);
  box-shadow: 0 4px 12px rgba(0,0,0,0.15);
  z-index: 9999;
  cursor: pointer;
  transition: transform 0.2s;
}

.floatingButton:hover {
  transform: scale(1.05);
}

.chatModal {
  position: fixed;
  bottom: 100px;
  right: 24px;
  width: 400px;
  max-width: calc(100vw - 48px);
  height: 600px;
  max-height: calc(100vh - 150px);
  background: var(--ifm-background-color);
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0,0,0,0.2);
  display: flex;
  flex-direction: column;
  z-index: 9998;
}

.messageList {
  flex: 1;
  overflow-y: auto;
  padding: 16px;
}

.userMessage {
  background: var(--ifm-color-primary-lightest);
  padding: 12px;
  border-radius: 12px;
  margin-bottom: 12px;
  align-self: flex-end;
}

.botMessage {
  background: var(--ifm-color-gray-100);
  padding: 12px;
  border-radius: 12px;
  margin-bottom: 12px;
}

.citationChip {
  display: inline-block;
  background: var(--ifm-color-primary-lightest);
  padding: 4px 8px;
  margin: 4px;
  border-radius: 4px;
  font-size: 0.875rem;
  cursor: pointer;
}

.citationChip:hover {
  background: var(--ifm-color-primary-light);
}

/* Dark mode support */
[data-theme='dark'] .botMessage {
  background: var(--ifm-color-gray-800);
}

/* Mobile responsive */
@media (max-width: 768px) {
  .chatModal {
    width: 100%;
    height: 100%;
    bottom: 0;
    right: 0;
    border-radius: 0;
    max-width: none;
    max-height: none;
  }

  .floatingButton {
    bottom: 16px;
    right: 16px;
    width: 56px;
    height: 56px;
  }
}
```

---

## Deployment Strategy

### Frontend Deployment (GitHub Pages)

```yaml
# .github/workflows/deploy.yml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - run: npm ci
      - run: npm run build
      - uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

### Backend Deployment (Railway)

1. **Create Railway Project**:
   - Connect GitHub repo
   - Select `backend/` as root directory
   - Railway auto-detects Dockerfile

2. **Set Environment Variables**:
   ```
   QDRANT_URL=https://...
   QDRANT_API_KEY=...
   DATABASE_URL=postgresql://...
   ALLOWED_ORIGINS=https://yourusername.github.io
   ```

3. **Deploy**:
   - Railway auto-deploys on git push
   - Assigns public URL: `https://your-backend.railway.app`

4. **Update Frontend API URL**:
   ```javascript
   // src/components/ChatWidget.jsx
   const API_BASE_URL = 'https://your-backend.railway.app';
   ```

---

## Testing Strategy

### Unit Tests (Backend)
```python
# tests/test_embeddings.py
def test_embedding_generation():
    model = load_embedding_model("all-MiniLM-L6-v2")
    vector = model.encode("ROS 2 nodes communicate via topics")
    assert vector.shape == (384,)
    assert isinstance(vector, np.ndarray)

# tests/test_query.py
def test_query_endpoint():
    response = client.post("/api/query", json={
        "query": "What is ROS 2?",
        "top_k": 5
    })
    assert response.status_code == 200
    assert "answer" in response.json()
    assert len(response.json()["citations"]) > 0
```

### Integration Tests (Frontend)
```javascript
// tests/ChatWidget.test.jsx
import { render, fireEvent, waitFor } from '@testing-library/react';
import ChatWidget from '../src/components/ChatWidget';

test('sends query and displays response', async () => {
  const { getByPlaceholderText, getByText } = render(<ChatWidget />);

  const input = getByPlaceholderText('Ask a question...');
  fireEvent.change(input, { target: { value: 'What is ROS 2?' } });
  fireEvent.click(getByText('Send'));

  await waitFor(() => {
    expect(getByText(/ROS 2 is a robot operating system/)).toBeInTheDocument();
  });
});
```

### E2E Tests (Playwright - Optional)
```javascript
// tests/e2e/chatbot.spec.js
test('full chatbot interaction flow', async ({ page }) => {
  await page.goto('http://localhost:3000/chapter-03/nodes-topics');

  // Open chatbot
  await page.click('[data-testid="chat-button"]');

  // Send query
  await page.fill('textarea', 'Explain ROS 2 nodes');
  await page.click('button:has-text("Send")');

  // Wait for response
  await page.waitForSelector('.bot-message');

  // Click citation
  await page.click('.citation-chip');

  // Verify navigation to source
  expect(page.url()).toContain('chapter-03');
});
```

---

## Performance Optimization

### Backend Optimizations
1. **Connection Pooling**: Reuse Qdrant/Neon connections
2. **Caching**: Cache frequently asked queries (Redis/in-memory)
3. **Batch Embedding**: Process multiple queries in parallel
4. **Async Operations**: Use FastAPI async for non-blocking I/O

### Frontend Optimizations
1. **Code Splitting**: Lazy load ChatWidget component
2. **Debouncing**: Delay search until user stops typing (300ms)
3. **Virtual Scrolling**: For long message lists (react-window)
4. **Service Worker**: Cache API responses (optional)

---

## Rollback Plan

If issues arise post-deployment:

1. **Frontend**: Revert GitHub Pages deployment
   ```bash
   git revert <commit-hash>
   git push origin main
   ```

2. **Backend**: Rollback Railway deployment
   - Railway dashboard → Deployments → Select previous version → Redeploy

3. **Database**: Restore Qdrant snapshot (if available)
   ```python
   qdrant_client.delete_collection("textbook_embeddings")
   qdrant_client.create_collection(...)  # Re-run ingestion
   ```

---

## Success Criteria

### Phase 1 Complete When:
- ✅ `/api/health` returns 200 with all services connected
- ✅ Qdrant collection created and accessible
- ✅ Neon Postgres tables created
- ✅ Backend deployed on Railway with public URL

### Phase 2 Complete When:
- ✅ All 13 chapters ingested into Qdrant
- ✅ Total vectors <1000 (free tier limit)
- ✅ Sample queries return relevant results (manual testing)
- ✅ Ingestion script documented and reproducible

### Phase 3 Complete When:
- ✅ ChatWidget visible on all pages
- ✅ User can send query and receive response
- ✅ Citations clickable and navigate to source
- ✅ Select-to-ask functional on desktop and mobile
- ✅ Mobile responsive (tested on iOS Safari, Android Chrome)
- ✅ Production deployment verified (GH Pages + Railway)

---

## Timeline Summary

| Phase | Duration | Key Milestones |
|-------|----------|----------------|
| **Phase 1** | 3-4 days | Backend infrastructure + deployment |
| **Phase 2** | 2-3 days | Content ingestion + vector DB population |
| **Phase 3** | 5-7 days | Frontend integration + production deploy |
| **Total** | **10-14 days** | Full RAG chatbot operational |

---

**Version**: 1.0.0 | **Status**: Ready for Implementation | **Last Updated**: 2025-12-10
