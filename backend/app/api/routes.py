from fastapi import APIRouter, HTTPException
from backend.app.models.chat import ChatRequest, ChatResponse
from backend.app.services.chat_service import chat_service

router = APIRouter()

# FIXED: Empty string route to match prefix exactly
# Main.py sets prefix="/api/chat", route="" = /api/chat (no trailing slash)
@router.post("", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process user query via ChatService.
    Final URL: POST /api/chat

    DEBUG MODE: Exposes real errors instead of generic messages.
    """
    try:
        response = await chat_service.process_query(request.query)

        # DEBUG: If service returned an error response, expose it
        if response.status == "error":
            error_msg = response.answer
            print(f"🐛 CHAT SERVICE ERROR DETECTED: {error_msg}")
            raise HTTPException(status_code=500, detail=f"Chat Service Error: {error_msg}")

        return response
    except HTTPException:
        # Re-raise HTTP exceptions (from the error check above)
        raise
    except Exception as e:
        import traceback
        full_trace = traceback.format_exc()
        print(f"🐛 ROUTE EXCEPTION:\n{full_trace}")
        raise HTTPException(status_code=500, detail=f"Route Error: {type(e).__name__}: {str(e)}")
