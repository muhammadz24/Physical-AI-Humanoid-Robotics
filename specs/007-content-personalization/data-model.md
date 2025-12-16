# Data Model: Content Personalization

**Feature**: 007-content-personalization
**Date**: 2025-12-16
**Phase**: 1 (Design)

## Overview

This document defines the data structures and models used in the content personalization feature.

## Backend Models (Pydantic)

### PersonalizationRequest

**Purpose**: Input model for POST /api/personalize endpoint

**File**: `backend/app/models/personalize.py`

```python
from pydantic import BaseModel, Field
from typing import Literal

class PersonalizationRequest(BaseModel):
    """
    Request model for content personalization.

    Sent from frontend to backend with chapter text to be personalized.
    User context (experience levels) is extracted from authenticated JWT token.
    """

    chapter_text: str = Field(
        ...,
        min_length=100,
        max_length=50000,
        description="Chapter content in markdown format to be personalized"
    )

    chapter_id: str | None = Field(
        None,
        max_length=100,
        description="Optional identifier for the chapter (e.g., 'chapter-1', 'intro-to-ros')"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_text": "# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2)...",
                "chapter_id": "chapter-1"
            }
        }
```

**Validation Rules**:
- `chapter_text` MUST be between 100 and 50,000 characters
- `chapter_text` MUST not be empty or whitespace-only
- `chapter_id` is optional (used for logging/analytics)

---

### PersonalizationResponse

**Purpose**: Output model for POST /api/personalize endpoint

**File**: `backend/app/models/personalize.py`

```python
from pydantic import BaseModel, Field
from datetime import datetime

class PersonalizationResponse(BaseModel):
    """
    Response model for content personalization.

    Contains the LLM-generated personalized content and metadata.
    """

    personalized_text: str = Field(
        ...,
        description="Chapter content rewritten for user's experience level"
    )

    original_length: int = Field(
        ...,
        description="Character count of original chapter text"
    )

    personalized_length: int = Field(
        ...,
        description="Character count of personalized text"
    )

    processing_time_ms: int = Field(
        ...,
        description="Time taken to generate personalization (milliseconds)"
    )

    user_experience: dict = Field(
        ...,
        description="User's experience levels used for personalization"
    )

    generated_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when personalization was generated (UTC)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "personalized_text": "# Introduction to ROS 2 (Simplified)\n\nROS 2...",
                "original_length": 5420,
                "personalized_length": 6100,
                "processing_time_ms": 3245,
                "user_experience": {
                    "software_experience": "beginner",
                    "hardware_experience": "none"
                },
                "generated_at": "2025-12-16T10:30:00Z"
            }
        }
```

**Field Descriptions**:
- `personalized_text`: The LLM-rewritten content
- `original_length`: Used for analytics (content expansion/compression analysis)
- `personalized_length`: Used for analytics
- `processing_time_ms`: For performance monitoring
- `user_experience`: For transparency and debugging
- `generated_at`: Timestamp for caching validation

---

### UserExperienceContext (Internal)

**Purpose**: Internal model for passing user context to LLM service

**File**: `backend/app/models/personalize.py`

```python
from pydantic import BaseModel
from typing import Literal

SoftwareExperience = Literal["beginner", "intermediate", "pro"]
HardwareExperience = Literal["none", "arduino", "ros", "professional"]

class UserExperienceContext(BaseModel):
    """
    User experience profile for LLM prompt construction.

    Retrieved from database users table based on authenticated user_id.
    """

    user_id: str
    software_experience: SoftwareExperience
    hardware_experience: HardwareExperience

    def to_prompt_context(self) -> str:
        """
        Convert experience levels to human-readable prompt text.

        Returns:
            Formatted string for LLM prompt inclusion
        """
        return f"""- Software Development Experience: {self.software_experience}
- Hardware/Robotics Experience: {self.hardware_experience}"""

    def get_adaptation_guidelines(self) -> str:
        """
        Get specific guidelines for LLM based on experience combination.

        Returns:
            Tailored instructions for content adaptation
        """
        # Determine primary guideline based on experience
        if self.software_experience == "beginner" and self.hardware_experience == "none":
            return "Use very simple language, explain all technical terms, include real-world analogies. Assume no prior robotics knowledge."
        elif self.software_experience == "pro" and self.hardware_experience == "professional":
            return "Use concise, technical language. Skip basic explanations. Focus on implementation details, advanced concepts, and best practices."
        elif self.software_experience == "intermediate":
            return "Balance technical depth with clarity. Assume some programming knowledge but explain robotics-specific concepts."
        else:
            return "Adapt content to be accessible yet informative. Explain key concepts without oversimplifying."
```

---

## Frontend Models (TypeScript Interfaces)

### PersonalizeButtonState

**Purpose**: Component state management for PersonalizeButton

**File**: `frontend/src/components/PersonalizeButton/types.ts`

```typescript
export type PersonalizationState =
  | 'idle'           // Initial state, button ready to click
  | 'loading'        // Personalization request in progress
  | 'personalized'   // Personalized content available
  | 'error';         // Error occurred

export interface PersonalizeButtonState {
  state: PersonalizationState;
  originalContent: string | null;
  personalizedContent: string | null;
  errorMessage: string | null;
  isShowingPersonalized: boolean; // For toggle
  processingTimeMs: number | null;
}
```

---

### PersonalizationApiResponse

**Purpose**: TypeScript type for API response

**File**: `frontend/src/utils/api.ts`

```typescript
export interface PersonalizationApiResponse {
  personalized_text: string;
  original_length: number;
  personalized_length: number;
  processing_time_ms: number;
  user_experience: {
    software_experience: 'beginner' | 'intermediate' | 'pro';
    hardware_experience: 'none' | 'arduino' | 'ros' | 'professional';
  };
  generated_at: string; // ISO 8601 timestamp
}

export interface PersonalizationApiError {
  detail: string;
  error_code?: string;
}
```

---

## Database Schema

### No New Tables Required

The personalization feature does NOT create new database tables. It reuses existing `users` table from Feature 006 (Authentication).

**Existing Schema (from Feature 006)**:
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY,
    name VARCHAR(100),
    email VARCHAR(255) UNIQUE,
    hashed_password VARCHAR(255),
    software_experience VARCHAR(20) CHECK (software_experience IN ('beginner', 'intermediate', 'pro')),
    hardware_experience VARCHAR(20) CHECK (hardware_experience IN ('none', 'arduino', 'ros', 'professional')),
    created_at TIMESTAMP,
    updated_at TIMESTAMP
);
```

**Usage**: Query `software_experience` and `hardware_experience` fields based on authenticated user's `id` from JWT token.

---

## Data Flow

```
1. User clicks "Personalize This Chapter" button
   ↓
2. Frontend extracts chapter text from DOM
   ↓
3. Frontend sends PersonalizationRequest to POST /api/personalize
   {
     chapter_text: "# Chapter 1...",
     chapter_id: "chapter-1"
   }
   ↓
4. Backend validates JWT token (extracts user_id)
   ↓
5. Backend queries users table for experience levels
   SELECT software_experience, hardware_experience
   FROM users
   WHERE id = {user_id}
   ↓
6. Backend constructs UserExperienceContext
   ↓
7. Backend builds LLM prompt with chapter_text + experience context
   ↓
8. Backend calls Gemini API (existing llm.py service)
   ↓
9. LLM returns personalized content
   ↓
10. Backend constructs PersonalizationResponse
   ↓
11. Frontend receives response, caches in component state
   ↓
12. Frontend replaces DOM with personalized content
   ↓
13. User toggles between original and personalized (client-side only)
```

---

## Validation Rules

### Backend Validation

**PersonalizationRequest**:
- `chapter_text` length: 100 ≤ len ≤ 50,000 characters
- `chapter_text` NOT empty after stripping whitespace
- `chapter_id` (if provided): max 100 characters, alphanumeric + hyphens only

**Authentication**:
- JWT token MUST be present in cookies
- JWT token MUST be valid and not expired
- User MUST exist in database

**Rate Limiting**:
- Maximum 10 requests per hour per user_id
- Enforced via SlowAPI or in-memory counter

### Frontend Validation

**Pre-Request**:
- User MUST be authenticated (check `isAuthenticated` from AuthContext)
- `chapter_text` extraction MUST succeed (non-empty result from DOM)

**Post-Response**:
- `personalized_text` MUST not be empty
- `personalized_text` SHOULD preserve markdown structure (basic heuristic check)

---

## Error Models

### Backend Error Responses

```python
from pydantic import BaseModel

class ErrorResponse(BaseModel):
    """Standard error response format."""
    detail: str
    error_code: str | None = None

class ValidationErrorResponse(BaseModel):
    """Validation error with field-level details."""
    detail: list[dict]
    # FastAPI default validation error format
```

**Error Codes**:
- `UNAUTHORIZED`: Missing or invalid JWT token
- `INVALID_INPUT`: Request validation failed
- `LLM_TIMEOUT`: Gemini API did not respond within 30s
- `QUOTA_EXCEEDED`: Gemini API quota exhausted
- `RATE_LIMITED`: User exceeded 10 requests/hour
- `INTERNAL_ERROR`: Unexpected server error

---

## Example API Payloads

### Request Example (from frontend)

```json
POST /api/personalize
Content-Type: application/json
Cookie: access_token=eyJhbGc...

{
  "chapter_text": "# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2) is the next generation of the Robot Operating System...",
  "chapter_id": "chapter-1-intro-to-ros"
}
```

### Success Response Example

```json
HTTP/1.1 200 OK
Content-Type: application/json

{
  "personalized_text": "# Introduction to ROS 2 (Beginner-Friendly)\n\nROS 2 is like a communication system for robots...",
  "original_length": 5420,
  "personalized_length": 6180,
  "processing_time_ms": 3245,
  "user_experience": {
    "software_experience": "beginner",
    "hardware_experience": "none"
  },
  "generated_at": "2025-12-16T10:30:45.123Z"
}
```

### Error Response Example (Rate Limited)

```json
HTTP/1.1 429 Too Many Requests
Content-Type: application/json

{
  "detail": "Rate limit exceeded. You can personalize up to 10 chapters per hour. Try again in 45 minutes.",
  "error_code": "RATE_LIMITED"
}
```

---

## Performance Considerations

**Model Size**:
- PersonalizationRequest: ~5-50 KB (depends on chapter length)
- PersonalizationResponse: ~6-60 KB (slightly larger due to metadata)
- Typical chapter: 3000 words ≈ 20 KB

**Network**:
- Request time: <100ms (local network)
- LLM processing: 2-8 seconds (Gemini API)
- Response time: <100ms (local network)
- **Total**: 2-10 seconds end-to-end

**Memory**:
- Frontend state: ~100 KB per personalization (cached in component)
- Backend processing: ~500 KB peak (request + response + LLM context)

---

## Next Steps

1. Implement Pydantic models in `backend/app/models/personalize.py`
2. Create TypeScript interfaces (optional for POC - use inline types)
3. Write API contract (OpenAPI spec) in `contracts/personalize-api.yaml`
4. Proceed to implementation phase (/sp.tasks)
