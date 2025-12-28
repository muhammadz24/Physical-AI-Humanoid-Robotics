from fastapi import APIRouter, HTTPException, Depends, Cookie
from typing import Optional
from uuid import UUID
import traceback
from backend.app.models.chat import ChatRequest, ChatResponse
from backend.app.services.chat_service import chat_service
from backend.app.core.database import db_manager

# Import get_current_user dependency from auth module
from backend.app.api.auth import get_current_user

router = APIRouter()

# FIXED: Empty string route to match prefix exactly
# Main.py sets prefix="/chat", Vercel rewrite adds /api → Final URL: POST /api/chat
@router.post("", response_model=ChatResponse)
async def chat(
    request: ChatRequest,
    access_token: Optional[str] = Cookie(None)
):
    """
    Process user query via ChatService.
    Final URL: POST /api/chat

    Hybrid Storage:
    - If user is logged in (has valid JWT): Save to database
    - If guest (no JWT): Frontend handles sessionStorage

    DEBUG MODE: Exposes real errors instead of generic messages.
    """
    # Try to get current user (optional - don't fail if not authenticated)
    user_id = None

    # DEBUG: Log cookie presence
    print(f"[CHAT DEBUG] access_token cookie: {'Present' if access_token else 'Missing'}")

    if access_token:
        try:
            # Attempt to get current user (for logged-in users)
            current_user = await get_current_user(access_token)
            user_id = current_user['id']
            print(f"[CHAT DEBUG] ✅ User authenticated: {user_id} ({current_user.get('name', 'Unknown')})")
        except HTTPException as e:
            # User not authenticated - treat as guest
            print(f"[CHAT DEBUG] ❌ Authentication failed: {e.detail}")
            pass
    else:
        print("[CHAT DEBUG] ⚠️ No access_token cookie - treating as guest")

    try:
        # Process query with optional user_id (saves to DB if logged in)
        print(f"[CHAT DEBUG] Processing query with user_id: {user_id}")
        response = await chat_service.process_query(request.query, user_id=user_id)

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
        # Full traceback for debugging (traceback imported at top)
        full_trace = traceback.format_exc()
        print(f"🐛 ROUTE EXCEPTION:\n{full_trace}")
        raise HTTPException(status_code=500, detail=f"Route Error: {type(e).__name__}: {str(e)}")


# ============================================================================
# CHAT HISTORY MANAGEMENT ENDPOINTS
# ============================================================================

@router.get("/history", status_code=200)
async def get_chat_history(
    current_user: dict = Depends(get_current_user),
    limit: int = 50
):
    """
    Get chat history for the current authenticated user.

    Endpoint: GET /api/chat/history?limit=50

    Query Parameters:
        - limit: Maximum number of messages to retrieve (default: 50, max: 100)

    Requires:
        - Valid JWT token in httpOnly cookie

    Returns:
        JSON: {
            "status": "success",
            "data": [
                {
                    "id": "uuid",
                    "query": "user question",
                    "response": "ai answer",
                    "metadata": {...},
                    "created_at": "ISO timestamp"
                },
                ...
            ],
            "count": N
        }

    Raises:
        401 Unauthorized: If not authenticated
    """
    # Validate limit
    if limit > 100:
        limit = 100
    if limit < 1:
        limit = 50

    try:
        # Fetch chat history from database
        chat_history = await chat_service.get_user_chat_history(
            user_id=current_user['id'],
            limit=limit
        )

        return {
            "status": "success",
            "data": chat_history,
            "count": len(chat_history)
        }

    except Exception as e:
        print(f"[ERROR] Get chat history error: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to retrieve chat history. Please try again later."
        )


@router.delete("/history", status_code=200)
async def delete_all_chat_history(current_user: dict = Depends(get_current_user)):
    """
    Delete ALL chat history for the current authenticated user.

    Endpoint: DELETE /api/chat/history

    CRITICAL: This action is irreversible. Deletes all chat messages
    associated with the user's account.

    Requires:
        - Valid JWT token in httpOnly cookie

    Returns:
        JSON: {"status": "success", "message": "Chat history deleted", "deleted_count": N}

    Raises:
        401 Unauthorized: If not authenticated
    """
    # Validate database connection
    if db_manager.pool is None:
        raise HTTPException(
            status_code=503,
            detail="Database service unavailable. Please try again later."
        )

    try:
        # Delete all chat messages for this user
        query = """
            DELETE FROM chats
            WHERE user_id = $1
            RETURNING id
        """

        async with db_manager.pool.acquire() as conn:
            deleted_rows = await conn.fetch(query, current_user['id'])

        deleted_count = len(deleted_rows)

        return {
            "status": "success",
            "message": f"All chat history deleted successfully.",
            "deleted_count": deleted_count
        }

    except Exception as e:
        print(f"Delete chat history error: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to delete chat history. Please try again later."
        )


@router.delete("/{message_id}", status_code=200)
async def delete_single_message(
    message_id: UUID,
    current_user: dict = Depends(get_current_user)
):
    """
    Delete a specific chat message by ID.

    Endpoint: DELETE /api/chat/{message_id}

    CRITICAL: Users can only delete their own messages.

    Requires:
        - Valid JWT token in httpOnly cookie

    Path Parameters:
        - message_id: UUID of the chat message to delete

    Returns:
        JSON: {"status": "success", "message": "Message deleted"}

    Raises:
        401 Unauthorized: If not authenticated
        404 Not Found: If message doesn't exist or doesn't belong to user
    """
    # Validate database connection
    if db_manager.pool is None:
        raise HTTPException(
            status_code=503,
            detail="Database service unavailable. Please try again later."
        )

    try:
        # Delete message only if it belongs to the current user
        query = """
            DELETE FROM chats
            WHERE id = $1 AND user_id = $2
            RETURNING id
        """

        async with db_manager.pool.acquire() as conn:
            row = await conn.fetchrow(query, message_id, current_user['id'])

        if row is None:
            raise HTTPException(
                status_code=404,
                detail="Message not found or you don't have permission to delete it."
            )

        return {
            "status": "success",
            "message": "Message deleted successfully."
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        print(f"Delete message error: {e}")
        raise HTTPException(
            status_code=500,
            detail="Failed to delete message. Please try again later."
        )
