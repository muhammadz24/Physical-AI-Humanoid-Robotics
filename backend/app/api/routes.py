from fastapi import APIRouter, HTTPException
from app.models.chat import ChatRequest, ChatResponse
from app.services.chat_service import chat_service

router = APIRouter()

# Fixed path: prefix in main.py is "/api/chat", so path here should be "/"
# Final URL: POST /api/chat
@router.post("/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Process user query via ChatService.
    """
    try:
        # Delegate purely to the service layer
        # No manual embedding checks here
        return await chat_service.process_query(request.query)
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Route Error: {str(e)}")
