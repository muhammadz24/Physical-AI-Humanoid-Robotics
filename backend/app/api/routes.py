from fastapi import APIRouter, HTTPException, Depends
from uuid import UUID
from backend.app.models.chat import ChatRequest, ChatResponse
from backend.app.services.chat_service import chat_service
from backend.app.core.database import db_manager

# Import get_current_user dependency from auth module
from backend.app.api.auth import get_current_user

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


# ============================================================================
# CHAT HISTORY MANAGEMENT ENDPOINTS
# ============================================================================

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
