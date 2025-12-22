"""
User service for database operations.

Handles user creation, retrieval, and authentication operations
against the Neon Postgres database.
"""

from typing import Optional
from uuid import UUID
import asyncpg

from backend.app.models.user import UserCreate, UserResponse
from backend.app.models.personalize import UserExperienceContext
from backend.app.core.security import hash_password


async def create_user(user: UserCreate, db_pool: asyncpg.Pool) -> UserResponse:
    """
    Create a new user in the database.

    Args:
        user: UserCreate model with user registration data
        db_pool: Database connection pool

    Returns:
        UserResponse model with created user data

    Raises:
        asyncpg.UniqueViolationError: If email already exists
    """
    # Hash the password before storing
    hashed_password = hash_password(user.password)

    # Insert user into database
    query = """
        INSERT INTO users (name, email, hashed_password, software_experience, hardware_experience)
        VALUES ($1, $2, $3, $4, $5)
        RETURNING id, name, email, software_experience, hardware_experience, created_at
    """

    async with db_pool.acquire() as conn:
        row = await conn.fetchrow(
            query,
            user.name,
            user.email,
            hashed_password,
            user.software_experience,
            user.hardware_experience
        )

    # Convert database row to UserResponse model
    return UserResponse(
        id=row['id'],
        name=row['name'],
        email=row['email'],
        software_experience=row['software_experience'],
        hardware_experience=row['hardware_experience'],
        created_at=row['created_at']
    )


async def get_user_by_email(email: str, db_pool: asyncpg.Pool) -> Optional[dict]:
    """
    Retrieve a user by email address.

    Args:
        email: User's email address
        db_pool: Database connection pool

    Returns:
        Dictionary with user data including hashed_password, or None if not found
    """
    query = """
        SELECT id, name, email, hashed_password, software_experience, hardware_experience, created_at
        FROM users
        WHERE email = $1
    """

    async with db_pool.acquire() as conn:
        row = await conn.fetchrow(query, email)

    if row is None:
        return None

    return dict(row)


async def get_user_by_id(user_id: UUID, db_pool: asyncpg.Pool) -> Optional[dict]:
    """
    Retrieve a user by ID.

    Args:
        user_id: User's UUID
        db_pool: Database connection pool

    Returns:
        Dictionary with user data (excluding hashed_password), or None if not found
    """
    query = """
        SELECT id, name, email, software_experience, hardware_experience, created_at
        FROM users
        WHERE id = $1
    """

    async with db_pool.acquire() as conn:
        row = await conn.fetchrow(query, user_id)

    if row is None:
        return None

    return dict(row)


async def get_user_experience(user_id: UUID, db_pool: asyncpg.Pool) -> Optional[UserExperienceContext]:
    """
    Retrieve user's experience levels for content personalization.

    Args:
        user_id: User's UUID from JWT token
        db_pool: Database connection pool

    Returns:
        UserExperienceContext model with experience levels, or None if user not found
    """
    query = """
        SELECT id, software_experience, hardware_experience
        FROM users
        WHERE id = $1
    """

    async with db_pool.acquire() as conn:
        row = await conn.fetchrow(query, user_id)

    if row is None:
        return None

    return UserExperienceContext(
        user_id=str(row['id']),
        software_experience=row['software_experience'],
        hardware_experience=row['hardware_experience']
    )
