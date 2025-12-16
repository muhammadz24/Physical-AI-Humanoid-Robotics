"""
Content Personalization API Endpoint

Feature: 007-content-personalization
Handles POST /api/personalize requests to personalize chapter content
based on authenticated user's experience levels.
"""

from fastapi import APIRouter, Request, HTTPException, status
from fastapi.responses import JSONResponse
from typing import Dict, Any
from uuid import UUID
import time

from app.models.personalize import (
    PersonalizationRequest,
    PersonalizationResponse,
    ErrorResponse
)
from app.services import user_service
from app.services.llm import llm_service
from app.core.database import db_manager
from app.core.security import decode_access_token


router = APIRouter()


def get_current_user_id(request: Request) -> UUID:
    """
    Extract user_id from JWT access_token cookie.

    Args:
        request: FastAPI request object

    Returns:
        User's UUID from JWT token

    Raises:
        HTTPException 401: If token is missing, invalid, or expired
    """
    # Get access_token from cookies
    access_token = request.cookies.get("access_token")

    if not access_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required. Please log in."
        )

    # Decode JWT token
    payload = decode_access_token(access_token)

    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired authentication token."
        )

    # Extract user_id from payload
    user_id_str = payload.get("user_id")

    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload."
        )

    try:
        return UUID(user_id_str)
    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID in token."
        )


@router.post("/personalize", response_model=PersonalizationResponse)
async def personalize_content(
    request: Request,
    personalization_request: PersonalizationRequest
):
    """
    Personalize chapter content based on user's experience level.

    **Authentication Required**: JWT token in access_token cookie

    **Rate Limit**: 10 requests per hour per user (future implementation)

    **Workflow**:
    1. Validate JWT token and extract user_id
    2. Retrieve user's experience levels from database
    3. Build personalization prompt with experience context
    4. Call Gemini LLM with 30-second timeout
    5. Return personalized content with metadata

    Args:
        request: FastAPI request (for accessing cookies)
        personalization_request: Chapter text and optional chapter_id

    Returns:
        PersonalizationResponse with personalized text and metadata

    Raises:
        400: Invalid input (validation errors)
        401: Authentication required or invalid token
        503: LLM timeout or quota exceeded
        500: Unexpected server error
    """
    start_time = time.time()

    # Step 1: Authenticate user
    try:
        user_id = get_current_user_id(request)
    except HTTPException as e:
        # Re-raise authentication errors
        raise e

    # Step 2: Retrieve user's experience levels
    try:
        user_experience = await user_service.get_user_experience(user_id, db_manager.pool)

        if not user_experience:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="User profile not found. Please update your profile with experience levels."
            )

    except HTTPException as e:
        raise e
    except Exception as e:
        print(f"Database error retrieving user experience: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve user profile."
        )

    # Step 3: Call LLM for personalization
    try:
        personalized_text = llm_service.personalize_content(
            chapter_text=personalization_request.chapter_text,
            user_context=user_experience,
            timeout_seconds=30
        )

        # Basic validation: ensure personalized text is not empty
        if not personalized_text or not personalized_text.strip():
            raise ValueError("LLM returned empty personalized content")

    except TimeoutError as e:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Personalization timed out. Please try again with a shorter chapter or retry later."
        )
    except Exception as e:
        error_message = str(e).lower()

        # Check for quota/rate limit errors
        if "quota" in error_message or "rate limit" in error_message:
            raise HTTPException(
                status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
                detail="AI service quota exceeded. Please try again later."
            )

        # Generic LLM error
        print(f"LLM error during personalization: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to generate personalized content. Please try again."
        )

    # Step 4: Build response with metadata
    end_time = time.time()
    processing_time_ms = int((end_time - start_time) * 1000)

    response = PersonalizationResponse(
        personalized_text=personalized_text,
        original_length=len(personalization_request.chapter_text),
        personalized_length=len(personalized_text),
        processing_time_ms=processing_time_ms,
        user_experience={
            "software_experience": user_experience.software_experience,
            "hardware_experience": user_experience.hardware_experience
        }
    )

    # Optional: Log personalization for analytics
    print(f"Personalization completed for user {user_id}, chapter_id: {personalization_request.chapter_id}, time: {processing_time_ms}ms")

    return response
