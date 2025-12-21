from typing import List, Optional, Any
from pydantic import BaseModel

class ChatRequest(BaseModel):
    query: str

class Citation(BaseModel):
    chapter: str
    chapter_title: Optional[str] = "Unknown"
    section: Optional[str] = "Unknown"
    chunk_id: str
    similarity_score: float
    source_file: str = ""
    url: Optional[str] = None

class ChatResponse(BaseModel):
    status: str
    answer: str
    citations: List[Citation]
    confidence: float
    retrieved_chunks: int
    response_time_ms: float
    model: str
    tokens_used: int = 0
