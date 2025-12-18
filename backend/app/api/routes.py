"""
API Routes for RAG Chatbot

Implements POST /api/chat endpoint for question answering.
"""

import time
from typing import List, Dict, Any
from fastapi import APIRouter, HTTPException, status

from app.models.query import ChatRequest, ChatResponse, Citation, ErrorResponse
from app.services.embedding import embedding_service
from app.services.llm import llm_service
from app.core.vector_store import vector_store


# Create API router (prefix added in main.py for explicit routing)
router = APIRouter(tags=["chat"])


@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """
    Process user query and return AI-generated answer with citations.

    Flow:
    1. Generate embedding for user query
    2. Search Qdrant for top-k relevant chunks
    3. Pass query + chunks to LLM for answer generation
    4. Return answer with citations

    Args:
        request: ChatRequest with user query and options

    Returns:
        ChatResponse with answer and source citations

    Raises:
        HTTPException: If query processing fails
    """
    start_time = time.time()

    try:
        # Step 1: Generate query embedding
        if not embedding_service._is_loaded:
            embedding_service.load_model()

        query_embedding = embedding_service.encode(request.query)

        # Step 2: Search Qdrant for relevant chunks
        search_results = vector_store.search(
            query_vector=query_embedding.tolist(),
            top_k=request.top_k,
            score_threshold=0.35,  # Tuned threshold to include moderate-relevance chunks
            chapter_filter=request.chapter_filter
        )

        # Check if we got results
        if not search_results:
            elapsed_ms = int((time.time() - start_time) * 1000)
            return ChatResponse(
                status="no_results",
                answer="I couldn't find relevant information in the textbook to answer your question. Please try rephrasing or ask about a different topic.",
                citations=[],
                confidence=0.0,
                retrieved_chunks=0,
                response_time_ms=elapsed_ms
            )

        # Step 3: Generate answer using LLM
        llm_response = llm_service.generate_response(
            query=request.query,
            context_chunks=search_results,
            max_tokens=500,
            temperature=0.3
        )

        # Step 4: Build citations from search results
        citations = _build_citations(search_results)

        # Calculate response time
        elapsed_ms = int((time.time() - start_time) * 1000)

        # Return response
        return ChatResponse(
            status="success",
            answer=llm_response["answer"],
            citations=citations,
            confidence=llm_response["confidence"],
            retrieved_chunks=len(search_results),
            response_time_ms=elapsed_ms,
            model=llm_response.get("model"),
            tokens_used=llm_response.get("tokens_used")
        )

    except Exception as e:
        print(f"❌ Chat endpoint error: {e}")
        import traceback
        traceback.print_exc()

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}"
        )


def _build_citations(search_results: List[Dict[str, Any]]) -> List[Citation]:
    """
    Build citation objects from Qdrant search results.

    Args:
        search_results: List of search result dictionaries from Qdrant

    Returns:
        List of Citation objects
    """
    citations = []

    for result in search_results:
        payload = result.get("payload", {})
        score = result.get("score", 0.0)

        citation = Citation(
            chapter=payload.get("chapter", "Unknown"),
            chapter_title=payload.get("chapter_title", "Unknown Chapter"),
            section=payload.get("section", "Unknown Section"),
            chunk_id=payload.get("chunk_id", "unknown"),
            similarity_score=round(score, 3),
            url=payload.get("url", "/"),
            source_file=payload.get("source_file", "unknown")
        )
        citations.append(citation)

    return citations
