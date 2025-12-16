"""
Authentication API endpoints.

Handles user signup, signin, and session management using JWT tokens.
"""

from fastapi import APIRouter, HTTPException, status, Response
from asyncpg.exceptions import UniqueViolationError

from app.models.user import UserCreate, UserLogin, UserResponse
from app.services import user_service
from app.core.database import db_manager
from app.core.security import create_access_token, verify_password


router = APIRouter(prefix="/api/auth", tags=["Authentication"])


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
