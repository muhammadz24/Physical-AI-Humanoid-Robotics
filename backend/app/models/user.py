"""
User models for authentication.

Pydantic models for user data validation and serialization.
"""

from pydantic import BaseModel, EmailStr, Field, validator
from typing import Literal
from datetime import datetime
from uuid import UUID


# Enum types for experience levels (PDF compliance - bonus points)
SoftwareExperience = Literal["beginner", "intermediate", "pro"]
HardwareExperience = Literal["none", "arduino", "ros", "professional"]


class UserCreate(BaseModel):
    """
    Model for user registration (signup).

    PDF Compliance: Includes software_experience and hardware_experience fields
    required for bonus points.
    """
    name: str = Field(..., min_length=1, max_length=100, description="User's full name")
    email: EmailStr = Field(..., description="User's email address (must be unique)")
    password: str = Field(..., min_length=8, description="Password (minimum 8 characters)")
    software_experience: SoftwareExperience = Field(..., description="Software development experience level")
    hardware_experience: HardwareExperience = Field(..., description="Hardware/robotics experience level")

    @validator('name')
    def name_not_empty(cls, v):
        """Validate name is not just whitespace."""
        if not v.strip():
            raise ValueError('Name cannot be empty or whitespace')
        return v.strip()

    @validator('software_experience')
    def validate_software_experience(cls, v):
        """Validate software experience is one of the allowed values."""
        allowed = ["beginner", "intermediate", "pro"]
        if v not in allowed:
            raise ValueError(f'Software experience must be one of: {", ".join(allowed)}')
        return v

    @validator('hardware_experience')
    def validate_hardware_experience(cls, v):
        """Validate hardware experience is one of the allowed values."""
        allowed = ["none", "arduino", "ros", "professional"]
        if v not in allowed:
            raise ValueError(f'Hardware experience must be one of: {", ".join(allowed)}')
        return v


class UserLogin(BaseModel):
    """Model for user login (signin)."""
    email: EmailStr = Field(..., description="User's email address")
    password: str = Field(..., description="User's password")


class UserResponse(BaseModel):
    """
    Model for user data in API responses.

    Excludes sensitive fields like hashed_password.
    """
    id: UUID
    name: str
    email: str
    software_experience: SoftwareExperience
    hardware_experience: HardwareExperience
    created_at: datetime

    class Config:
        from_attributes = True  # Allows conversion from database records
