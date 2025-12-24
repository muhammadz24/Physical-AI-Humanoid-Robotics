"""
Authentication API endpoints.

Handles user signup, signin, and session management using JWT tokens.
"""

from fastapi import APIRouter, HTTPException, status, Response, Cookie, Depends
from asyncpg.exceptions import UniqueViolationError
from typing import Optional
from uuid import UUID

from backend.app.models.user import UserCreate, UserLogin, UserResponse
from backend.app.services import user_service
from backend.app.core.database import db_manager
from backend.app.core.security import create_access_token, verify_password, decode_access_token


# Router (prefix added in main.py for explicit routing)
router = APIRouter(tags=["Authentication"])


# ============================================================================
# DEPENDENCY: Get Current User from JWT Token
# ============================================================================

async def get_current_user(access_token: Optional[str] = Cookie(None)) -> dict:
    """
    Dependency to extract and validate the current user from JWT token.

    Args:
        access_token: JWT token from httpOnly cookie

    Returns:
        User data dictionary

    Raises:
        401 Unauthorized: If token is missing, invalid, or user not found
    """
    # Check if token exists
    if not access_token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated. Please sign in."
        )

    # Decode JWT token
    payload = decode_access_token(access_token)
    if not payload:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token. Please sign in again."
        )

    # Extract user_id from token
    user_id_str = payload.get("user_id")
    if not user_id_str:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid token payload. Please sign in again."
        )

    # Validate database connection
    if db_manager.pool is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service unavailable. Please try again later."
        )

    # Fetch user from database
    try:
        user_id = UUID(user_id_str)
        user_data = await user_service.get_user_by_id(user_id, db_manager.pool)

        if not user_data:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found. Please sign in again."
            )

        return user_data

    except ValueError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid user ID format in token."
        )


@router.post("/signup", response_model=UserResponse, status_code=status.HTTP_201_CREATED)
async def signup(user: UserCreate, response: Response):
    """
    Register a new user.

    Endpoint: POST /api/auth/signup

    Request Body:
        - name: User's full name (1-100 chars)
        - email: Valid email address (must be unique)
        - password: Password (minimum 8 characters)
        - software_experience: "beginner" | "intermediate" | "pro"
        - hardware_experience: "none" | "arduino" | "ros" | "professional"

    Returns:
        UserResponse: Created user data (excluding hashed password)

    Sets Cookie:
        - access_token: JWT token (httpOnly, 7-day expiration)

    Raises:
        409 Conflict: Email already registered
        422 Validation Error: Invalid input data
    """
    # Validate database connection is available
    if db_manager.pool is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service unavailable. Please try again later."
        )

    try:
        # Create user in database (password will be hashed by user_service)
        created_user = await user_service.create_user(user, db_manager.pool)

        # Create JWT access token with user data
        access_token = create_access_token(
            data={
                "user_id": str(created_user.id),
                "email": created_user.email,
                "name": created_user.name
            }
        )

        # Set httpOnly cookie with JWT token (secure in production)
        response.set_cookie(
            key="access_token",
            value=access_token,
            httponly=True,
            secure=False,  # Set to True in production with HTTPS
            samesite="lax",
            max_age=7 * 24 * 60 * 60  # 7 days in seconds
        )

        return created_user

    except UniqueViolationError:
        # Email already exists in database
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered. Please use a different email or sign in."
        )
    except Exception as e:
        # Log unexpected errors for debugging
        print(f"Signup error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create user. Please try again later."
        )


@router.post("/signin", response_model=UserResponse, status_code=status.HTTP_200_OK)
async def signin(credentials: UserLogin, response: Response):
    """
    Authenticate an existing user.

    Endpoint: POST /api/auth/signin

    Request Body:
        - email: Valid email address
        - password: User's password

    Returns:
        UserResponse: Authenticated user data (excluding hashed password)

    Sets Cookie:
        - access_token: JWT token (httpOnly, 7-day expiration)

    Raises:
        401 Unauthorized: Invalid email or password
        422 Validation Error: Invalid input data
    """
    # Validate database connection is available
    if db_manager.pool is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service unavailable. Please try again later."
        )

    try:
        # Get user by email
        user_data = await user_service.get_user_by_email(credentials.email, db_manager.pool)

        # Check if user exists
        if user_data is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password."
            )

        # Verify password
        if not verify_password(credentials.password, user_data['hashed_password']):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password."
            )

        # Create JWT access token with user data
        access_token = create_access_token(
            data={
                "user_id": str(user_data['id']),
                "email": user_data['email'],
                "name": user_data['name']
            }
        )

        # Set httpOnly cookie with JWT token (secure in production)
        response.set_cookie(
            key="access_token",
            value=access_token,
            httponly=True,
            secure=False,  # Set to True in production with HTTPS
            samesite="lax",
            max_age=7 * 24 * 60 * 60  # 7 days in seconds
        )

        # Return user data (excluding hashed password)
        return UserResponse(
            id=user_data['id'],
            name=user_data['name'],
            email=user_data['email'],
            software_experience=user_data['software_experience'],
            hardware_experience=user_data['hardware_experience'],
            created_at=user_data['created_at']
        )

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        # Log unexpected errors for debugging
        print(f"Signin error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to sign in. Please try again later."
        )


# ============================================================================
# USER PROFILE MANAGEMENT ENDPOINTS
# ============================================================================

@router.get("/me", response_model=UserResponse, status_code=status.HTTP_200_OK)
async def get_current_user_profile(current_user: dict = Depends(get_current_user)):
    """
    Get current authenticated user's profile.

    Endpoint: GET /api/auth/me

    Requires:
        - Valid JWT token in httpOnly cookie

    Returns:
        UserResponse: Current user data (id, name, email, experience levels, created_at)

    Raises:
        401 Unauthorized: If not authenticated or token invalid
    """
    return UserResponse(
        id=current_user['id'],
        name=current_user['name'],
        email=current_user['email'],
        software_experience=current_user['software_experience'],
        hardware_experience=current_user['hardware_experience'],
        created_at=current_user['created_at']
    )


@router.put("/update", status_code=status.HTTP_200_OK)
async def update_user_profile(
    name: str,
    current_user: dict = Depends(get_current_user)
):
    """
    Update current user's profile (name only).

    Endpoint: PUT /api/auth/update

    Request Body:
        - name: New name (1-100 chars)

    Note: Email is immutable and cannot be changed.

    Requires:
        - Valid JWT token in httpOnly cookie

    Returns:
        JSON: {"status": "success", "data": {"name": "Updated Name"}}

    Raises:
        401 Unauthorized: If not authenticated
        422 Validation Error: If name is invalid
    """
    # Validate database connection
    if db_manager.pool is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service unavailable. Please try again later."
        )

    # Validate name
    if not name or not name.strip():
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Name cannot be empty."
        )

    if len(name.strip()) > 100:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Name must be 100 characters or less."
        )

    try:
        # Update user name in database
        updated_name = await user_service.update_user_name(
            user_id=current_user['id'],
            name=name.strip(),
            db_pool=db_manager.pool
        )

        return {
            "status": "success",
            "data": {
                "name": updated_name
            }
        }

    except Exception as e:
        print(f"Update user error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update profile. Please try again later."
        )


@router.delete("/account", status_code=status.HTTP_200_OK)
async def delete_user_account(
    response: Response,
    current_user: dict = Depends(get_current_user)
):
    """
    Delete current user's account and all associated data.

    Endpoint: DELETE /api/auth/account

    CRITICAL: This action is irreversible. Deletes:
    - User account
    - All chat history
    - All associated data

    Requires:
        - Valid JWT token in httpOnly cookie

    Returns:
        JSON: {"status": "success", "message": "Account deleted successfully"}

    Clears Cookie:
        - access_token: JWT token removed after deletion

    Raises:
        401 Unauthorized: If not authenticated
    """
    # Validate database connection
    if db_manager.pool is None:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail="Database service unavailable. Please try again later."
        )

    try:
        # Delete user account (cascades to delete chat history)
        await user_service.delete_user(
            user_id=current_user['id'],
            db_pool=db_manager.pool
        )

        # Clear JWT cookie
        response.delete_cookie(key="access_token")

        return {
            "status": "success",
            "message": "Account deleted successfully. All associated data has been removed."
        }

    except Exception as e:
        print(f"Delete user error: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to delete account. Please try again later."
        )
