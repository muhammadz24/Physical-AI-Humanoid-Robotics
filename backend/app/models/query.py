"""
Pydantic Models for Query API

Request and response models for the chat/query endpoint.
"""

from typing import List, Optional
from pydantic import BaseModel, Field


class ChatRequest(BaseModel):
    """Request model for chat endpoint."""

    query: str = Field(..., min_length=1, max_length=1000, description="User question")
    context: Optional[str] = Field(None, description="Optional selected text for contextual queries")
    chapter_filter: Optional[str] = Field(None, description="Optional chapter number to filter (e.g., '3')")
    top_k: int = Field(5, ge=1, le=10, description="Number of chunks to retrieve")


class Citation(BaseModel):
    """Citation model with source information."""

    chapter: str = Field(..., description="Chapter number")
    chapter_title: str = Field(..., description="Chapter title")
    section: str = Field(..., description="Section name")
    chunk_id: str = Field(..., description="Unique chunk identifier")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score 0-1")
    url: str = Field(..., description="Deep link to source section")
    source_file: str = Field(..., description="Source markdown file path")


class ChatResponse(BaseModel):
    """Response model for chat endpoint."""

    status: str = Field(..., description="Response status: success, no_results, or error")
    answer: str = Field(..., description="Generated answer from LLM")
    citations: List[Citation] = Field(default_factory=list, description="Source citations")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Overall confidence score")
    retrieved_chunks: int = Field(..., description="Number of chunks retrieved")
    response_time_ms: int = Field(..., description="Total response time in milliseconds")
    model: Optional[str] = Field(None, description="LLM model used")
    tokens_used: Optional[int] = Field(None, description="Total tokens consumed")


class ErrorResponse(BaseModel):
    """Error response model."""

    status: str = Field(default="error", description="Status: error")
    message: str = Field(..., description="Error message")
    error_code: Optional[str] = Field(None, description="Machine-readable error code")
