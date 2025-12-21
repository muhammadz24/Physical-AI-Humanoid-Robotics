"""
API Routes for RAG Chatbot

Implements POST /chat endpoint for question answering.
"""

from fastapi import APIRouter, HTTPException, status
from app.models.chat import ChatRequest, ChatResponse
from app.services.chat_service import chat_service

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
    try:
        # Delegate to chat_service which handles the full RAG pipeline
        response = await chat_service.process_query(request.query)
        return response

    except Exception as e:
        print(f"❌ Chat endpoint error: {e}")
        import traceback
        traceback.print_exc()

        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to process query: {str(e)}"
        )
