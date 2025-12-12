# RAG Chatbot Backend

FastAPI-based backend for the Physical AI & Humanoid Robotics textbook chatbot. Provides Retrieval-Augmented Generation (RAG) capabilities using Qdrant vector search and Neon Postgres for analytics.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    FastAPI Backend                          │
│                                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────┐  │
│  │   API Routes │  │   Services   │  │   Core Modules  │  │
│  │              │  │              │  │                 │  │
│  │  /health     │─▶│  Embeddings  │─▶│  Config         │  │
│  │  /api/query  │  │  RAG Logic   │  │  Database       │  │
│  │              │  │              │  │  Vector Store   │  │
│  └──────────────┘  └──────────────┘  └─────────────────┘  │
│                                                             │
└──────────────┬────────────────────────────┬─────────────────┘
               │                            │
               ▼                            ▼
    ┌──────────────────┐        ┌──────────────────────┐
    │  Qdrant Cloud    │        │  Neon Postgres       │
    │  Vector Database │        │  Analytics DB        │
    │                  │        │                      │
    │  - 384-dim vecs  │        │  - chat_sessions     │
    │  - Cosine dist   │        │  - user_feedback     │
    │  - HNSW index    │        │  - query_analytics   │
    └──────────────────┘        └──────────────────────┘
```

## Project Structure

```
backend/
├── app/
│   ├── __init__.py
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py           # Environment configuration (Pydantic Settings)
│   │   ├── database.py         # Neon Postgres connection manager
│   │   └── vector_store.py     # Qdrant client wrapper
│   ├── api/
│   │   └── __init__.py         # API routes (future: query.py)
│   ├── services/
│   │   └── __init__.py         # Business logic (future: embeddings.py, rag.py)
│   └── models/
│       └── __init__.py         # Pydantic models (future: query.py, database.py)
├── main.py                     # FastAPI application entry point
├── requirements.txt            # Python dependencies
├── .env.example                # Environment variables template
└── README.md                   # This file
```

## Prerequisites

- **Python 3.10+** (tested with 3.10, 3.11, 3.12)
- **Qdrant Cloud Account** (free tier: 1GB storage, <1000 vectors)
- **Neon Serverless Postgres** (free tier: 512MB storage, 10 connections)

## Local Development Setup

### 1. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Edit `.env` and fill in your actual credentials:

```bash
# Qdrant Cloud
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Neon Postgres
DATABASE_URL=postgresql://user:password@host.neon.tech/dbname?sslmode=require

# CORS (for local Docusaurus development)
ALLOWED_ORIGINS=http://localhost:3000,http://127.0.0.1:3000
```

#### Getting Qdrant Credentials:
1. Sign up at [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a new cluster (free tier)
3. Copy the cluster URL and API key
4. Collection will be auto-created during ingestion (Phase 2)

#### Getting Neon Credentials:
1. Sign up at [neon.tech](https://neon.tech)
2. Create a new project (free tier)
3. Copy the connection string from the dashboard
4. Run the SQL schema from `specs/rag-chatbot-integration/plan.md` (Phase 1, Section: Neon Postgres Schema Creation)

### 3. Run the Development Server

```bash
# From backend/ directory
python main.py
```

Or using uvicorn directly:

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### 4. Verify Health Check

Open your browser to:
- **API Root**: http://localhost:8000
- **Health Check**: http://localhost:8000/health
- **Interactive Docs**: http://localhost:8000/docs

Expected health check response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "environment": "development",
  "services": {
    "neon_postgres": "connected",
    "qdrant_vector_store": "connected"
  }
}
```

## API Endpoints

### Current Endpoints (Phase 1)

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/` | API metadata and available endpoints |
| GET | `/health` | Service health check |
| GET | `/docs` | Interactive API documentation (Swagger UI) |
| GET | `/openapi.json` | OpenAPI schema |

### Future Endpoints (Phase 2-3)

| Method | Endpoint | Description |
|--------|----------|-------------|
| POST | `/api/query` | Submit query and get RAG response |
| POST | `/api/feedback` | Submit user feedback on answers |
| GET | `/api/analytics` | Query analytics dashboard (admin) |

## Configuration

All configuration is managed via environment variables and Pydantic Settings (`app/core/config.py`).

### Key Configuration Options

| Variable | Description | Default |
|----------|-------------|---------|
| `ENVIRONMENT` | Deployment environment | `development` |
| `DEBUG` | Enable debug mode | `True` |
| `API_PORT` | Server port | `8000` |
| `QDRANT_URL` | Qdrant Cloud endpoint | *(required)* |
| `DATABASE_URL` | Neon Postgres connection string | *(required)* |
| `ALLOWED_ORIGINS` | CORS allowed origins (comma-separated) | `http://localhost:3000` |
| `RATE_LIMIT_PER_MINUTE` | API rate limit per IP | `10` |
| `EMBEDDING_MODEL` | Sentence Transformers model | `all-MiniLM-L6-v2` |

## Database Setup

### Neon Postgres Schema

Run the following SQL in your Neon console to create required tables:

```sql
-- Enable UUID extension
CREATE EXTENSION IF NOT EXISTS "uuid-ossp";

-- Chat sessions table
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

-- User feedback table
CREATE TABLE user_feedback (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    chat_session_id UUID REFERENCES chat_sessions(id) ON DELETE CASCADE,
    feedback_type VARCHAR(50) CHECK (feedback_type IN ('helpful', 'not_helpful', 'inaccurate')),
    comment TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);

-- Query analytics table
CREATE TABLE query_analytics (
    id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
    date DATE NOT NULL UNIQUE,
    total_queries INT DEFAULT 0,
    avg_confidence FLOAT,
    avg_response_time_ms FLOAT,
    no_results_count INT DEFAULT 0
);
```

### Qdrant Collection

The Qdrant collection will be created automatically during content ingestion (Phase 2). Collection configuration:
- **Name**: `textbook_embeddings`
- **Vector Dimension**: 384 (all-MiniLM-L6-v2 model)
- **Distance Metric**: Cosine similarity
- **Index**: HNSW for fast approximate search

## Testing

### Manual Testing

```bash
# Test health endpoint
curl http://localhost:8000/health

# Test root endpoint
curl http://localhost:8000/
```

### Unit Tests (Future)

```bash
pytest tests/ -v
```

## Deployment

### Railway.app (Recommended for Free Tier)

1. **Connect GitHub Repository**:
   - Sign up at [railway.app](https://railway.app)
   - Create new project from GitHub repo
   - Set root directory to `backend/`

2. **Configure Environment Variables**:
   - Add all variables from `.env.example`
   - Use Railway's secret manager (not committed to repo)

3. **Deploy**:
   - Railway auto-deploys on git push to main branch
   - Assigns public URL: `https://your-app.railway.app`

4. **Update Frontend**:
   - Set `API_BASE_URL` in Docusaurus ChatWidget to Railway URL
   - Add Railway URL to `ALLOWED_ORIGINS` in backend .env

### Alternative: Docker Deployment

```bash
# Build Docker image
docker build -t rag-chatbot-backend .

# Run container
docker run -p 8000:8000 --env-file .env rag-chatbot-backend
```

## Troubleshooting

### Database Connection Fails

**Symptom**: Health check shows `"neon_postgres": "disconnected"`

**Solutions**:
- Verify `DATABASE_URL` is correct (check Neon dashboard)
- Ensure SSL mode is enabled: `?sslmode=require`
- Check Neon project is not suspended (free tier auto-suspends after 7 days inactivity)
- Test connection with `psql`: `psql $DATABASE_URL`

### Qdrant Connection Fails

**Symptom**: Health check shows `"qdrant_vector_store": "disconnected"`

**Solutions**:
- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct
- Check Qdrant cluster is running (cloud.qdrant.io dashboard)
- Ensure firewall/network allows HTTPS connections
- Test with Qdrant Python client directly

### CORS Errors in Browser

**Symptom**: Frontend shows "CORS policy" error in console

**Solutions**:
- Add frontend URL to `ALLOWED_ORIGINS` in `.env`
- Restart backend after changing CORS settings
- Verify frontend is making requests to correct backend URL
- Check browser developer tools Network tab for preflight OPTIONS request

### Import Errors

**Symptom**: `ModuleNotFoundError: No module named 'app'`

**Solutions**:
- Run from `backend/` directory, not from root
- Ensure `PYTHONPATH` includes backend directory
- Activate virtual environment if using one
- Reinstall dependencies: `pip install -r requirements.txt`

## Next Steps

- **Phase 2**: Content Ingestion Pipeline (Tasks T014-T020)
  - Implement markdown chunking logic
  - Generate embeddings for all chapters
  - Upload vectors to Qdrant

- **Phase 3**: Frontend Integration (Tasks T021-T035)
  - Implement POST `/api/query` endpoint
  - Create React ChatWidget component
  - Deploy to production

## Support

For issues or questions:
- Review `specs/rag-chatbot-integration/tasks.md` for task details
- Check `specs/rag-chatbot-integration/plan.md` for architecture
- Refer to `specs/rag-chatbot-integration/spec.md` for requirements

---

**Version**: 1.0.0 | **Status**: Phase 1 Complete | **Last Updated**: 2025-12-10
