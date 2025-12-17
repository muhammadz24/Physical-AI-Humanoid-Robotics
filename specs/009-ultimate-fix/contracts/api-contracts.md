# API Contracts: Ultimate Fix Release

**Feature**: 009-ultimate-fix
**Date**: 2025-12-17

## Overview

This document defines the API contracts affected by the Ultimate Fix Release. Most changes are internal (configuration, migrations), but some endpoints may be added or modified for chat history.

---

## Modified Endpoints

### Health Check Endpoint

**Endpoint**: `GET /api/health`

**Purpose**: Verify backend is running and database is connected

**Request**: None

**Response**: `200 OK`
```json
{
  "status": "healthy",
  "database": "connected",
  "timestamp": "2025-12-17T10:30:00Z"
}
```

**Response**: `503 Service Unavailable`
```json
{
  "status": "unhealthy",
  "database": "disconnected",
  "error": "Database connection failed"
}
```

**Changes**:
- Now checks database connectivity via `db_manager.health_check()`
- Returns proper HTTP 503 status if database is down

---

## New Endpoints

### Save Chat Message

**Endpoint**: `POST /api/chat/messages`

**Purpose**: Save a chat message to the database

**Headers**:
```
Content-Type: application/json
Authorization: Bearer <token> (optional, for authenticated users)
```

**Request Body**:
```json
{
  "role": "user",
  "content": "What is ROS 2?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Validation**:
- `role`: Required, must be one of: "user", "assistant", "system"
- `content`: Required, non-empty string, max 10,000 characters
- `session_id`: Required, valid UUID v4 format

**Response**: `201 Created`
```json
{
  "id": 42,
  "role": "user",
  "content": "What is ROS 2?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "user_id": null,
  "created_at": "2025-12-17T10:30:00Z"
}
```

**Error Responses**:

`400 Bad Request` - Invalid request body
```json
{
  "error": "Validation failed",
  "details": {
    "role": "Must be one of: user, assistant, system",
    "content": "Cannot be empty"
  }
}
```

`500 Internal Server Error` - Database error
```json
{
  "error": "Failed to save message",
  "message": "Database connection error"
}
```

---

### Get Chat History

**Endpoint**: `GET /api/chat/messages`

**Purpose**: Retrieve chat history for a session

**Query Parameters**:
- `session_id` (required): UUID of the conversation session
- `limit` (optional): Maximum number of messages to return (default: 50, max: 100)
- `offset` (optional): Number of messages to skip for pagination (default: 0)

**Example**:
```
GET /api/chat/messages?session_id=550e8400-e29b-41d4-a716-446655440000&limit=20
```

**Response**: `200 OK`
```json
{
  "messages": [
    {
      "id": 41,
      "role": "user",
      "content": "Hello!",
      "created_at": "2025-12-17T10:29:00Z"
    },
    {
      "id": 42,
      "role": "assistant",
      "content": "Hello! How can I help you today?",
      "created_at": "2025-12-17T10:29:05Z"
    }
  ],
  "total": 2,
  "limit": 20,
  "offset": 0
}
```

**Error Responses**:

`400 Bad Request` - Missing or invalid session_id
```json
{
  "error": "Invalid session_id",
  "message": "session_id must be a valid UUID"
}
```

`404 Not Found` - No messages found
```json
{
  "messages": [],
  "total": 0,
  "limit": 20,
  "offset": 0
}
```

---

## Configuration Endpoints (No Changes)

Existing endpoints remain unchanged:
- `POST /api/chat` - Send chat message and get AI response
- `GET /api/chapters` - Get textbook chapters
- `POST /api/auth/signup` - User registration
- `POST /api/auth/signin` - User authentication

---

## CORS Configuration

**Changes**:
- CORS now uses dynamic origins list
- Localhost origins hardcoded for development
- Production origin loaded from `ALLOWED_ORIGIN` environment variable

**Allowed Origins**:
- `http://localhost:3000` (hardcoded)
- `http://127.0.0.1:3000` (hardcoded)
- `process.env.ALLOWED_ORIGIN` (production, e.g., `https://yourapp.vercel.app`)

**Allowed Methods**: `GET`, `POST`, `PUT`, `DELETE`, `OPTIONS`

**Allowed Headers**: `*`

**Credentials**: `true`

---

## Environment Variables Required

### Backend

| Variable | Required | Example | Description |
|----------|----------|---------|-------------|
| `DATABASE_URL` | Yes | `postgresql://user:pass@host:5432/db` | PostgreSQL connection string |
| `OPENAI_API_KEY` | Yes | `sk-...` | OpenAI API key for chat |
| `QDRANT_API_KEY` | No | `...` | Qdrant vector DB API key |
| `ALLOWED_ORIGIN` | Production | `https://yourapp.vercel.app` | Frontend origin for CORS |
| `DEBUG` | No | `false` | Enable debug logging |
| `PORT` | No | `8000` | Server port |

### Frontend

| Variable | Required | Example | Description |
|----------|----------|---------|-------------|
| `NEXT_PUBLIC_API_URL` | Production | `https://yourapp.vercel.app/api` | Backend API base URL |

**Local Development**: Frontend uses `http://localhost:8000` if `NEXT_PUBLIC_API_URL` is not set.

---

## Error Handling Standards

All endpoints follow these error response conventions:

**Format**:
```json
{
  "error": "Short error code or title",
  "message": "Human-readable error description",
  "details": {
    "field_name": "Specific validation error"
  }
}
```

**HTTP Status Codes**:
- `200 OK` - Successful request
- `201 Created` - Resource created successfully
- `400 Bad Request` - Invalid request (validation errors)
- `401 Unauthorized` - Authentication required
- `403 Forbidden` - Insufficient permissions
- `404 Not Found` - Resource not found
- `500 Internal Server Error` - Server-side error
- `503 Service Unavailable` - Service temporarily unavailable (e.g., database down)

---

## Versioning

**API Version**: v1 (implicit, no version prefix in URLs)

**Backwards Compatibility**: This release does not break existing endpoints. New endpoints are additive.

**Future Versioning**: If breaking changes are needed, introduce `/api/v2/` prefix.

---

## Security Considerations

1. **Environment Variables**: Never expose secrets in responses or logs
2. **Input Validation**: All user inputs sanitized and validated
3. **SQL Injection**: Use parameterized queries (asyncpg automatically handles this)
4. **Rate Limiting**: Not implemented in this release (future enhancement)
5. **Authentication**: Optional for chat endpoints (user_id can be NULL)

---

## Testing Contract Compliance

### Health Check
```bash
curl https://yourapp.vercel.app/api/health
# Expected: {"status": "healthy", "database": "connected", ...}
```

### Save Message
```bash
curl -X POST https://yourapp.vercel.app/api/chat/messages \
  -H "Content-Type: application/json" \
  -d '{
    "role": "user",
    "content": "Test message",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'
# Expected: 201 Created with message object
```

### Get History
```bash
curl "https://yourapp.vercel.app/api/chat/messages?session_id=550e8400-e29b-41d4-a716-446655440000"
# Expected: 200 OK with messages array
```

---

## Summary

- **Modified Endpoints**: 1 (health check now includes database status)
- **New Endpoints**: 2 (save message, get history)
- **Breaking Changes**: None
- **CORS Changes**: Dynamic origins list (backwards compatible)
- **Environment Variables**: 7 total (2 required for backend)
