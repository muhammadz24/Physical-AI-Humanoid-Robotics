"""
Pydantic models for content personalization feature.

Feature: 007-content-personalization
Created: 2025-12-16
"""

from pydantic import BaseModel, Field, validator
from typing import Literal
from datetime import datetime


# Type definitions for experience levels
SoftwareExperience = Literal["beginner", "intermediate", "pro"]
HardwareExperience = Literal["none", "arduino", "ros", "professional"]


class PersonalizationRequest(BaseModel):
    """
    Request model for POST /api/personalize endpoint.

    Sent from frontend to backend with chapter text to be personalized.
    User context (experience levels) is extracted from authenticated JWT token.
    """

    chapter_text: str = Field(
        ...,
        min_length=100,
        max_length=50000,
        description="Chapter content in markdown format to be personalized"
    )

    chapter_id: str | None = Field(
        None,
        max_length=100,
        description="Optional identifier for the chapter (e.g., 'chapter-1', 'intro-to-ros')"
    )

    @validator('chapter_text')
    def validate_chapter_text_not_empty(cls, v):
        """Ensure chapter_text is not just whitespace."""
        if not v.strip():
            raise ValueError("chapter_text must not be empty or whitespace-only")
        return v

    @validator('chapter_id')
    def validate_chapter_id_pattern(cls, v):
        """Ensure chapter_id contains only alphanumeric characters and hyphens."""
        if v is not None:
            import re
            if not re.match(r'^[a-zA-Z0-9-]+$', v):
                raise ValueError("chapter_id must contain only alphanumeric characters and hyphens")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "chapter_text": "# Introduction to ROS 2\\n\\nROS 2 (Robot Operating System 2)...",
                "chapter_id": "chapter-1"
            }
        }


class PersonalizationResponse(BaseModel):
    """
    Response model for POST /api/personalize endpoint.

    Contains the LLM-generated personalized content and metadata.
    """

    personalized_text: str = Field(
        ...,
        description="Chapter content rewritten for user's experience level"
    )

    original_length: int = Field(
        ...,
        description="Character count of original chapter text"
    )

    personalized_length: int = Field(
        ...,
        description="Character count of personalized text"
    )

    processing_time_ms: int = Field(
        ...,
        description="Time taken to generate personalization (milliseconds)"
    )

    user_experience: dict = Field(
        ...,
        description="User's experience levels used for personalization"
    )

    generated_at: datetime = Field(
        default_factory=datetime.utcnow,
        description="Timestamp when personalization was generated (UTC)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "personalized_text": "# Introduction to ROS 2 (Simplified)\\n\\nROS 2...",
                "original_length": 5420,
                "personalized_length": 6100,
                "processing_time_ms": 3245,
                "user_experience": {
                    "software_experience": "beginner",
                    "hardware_experience": "none"
                },
                "generated_at": "2025-12-16T10:30:00Z"
            }
        }


class UserExperienceContext(BaseModel):
    """
    Internal model for passing user context to LLM service.

    Retrieved from database users table based on authenticated user_id.
    """

    user_id: str
    software_experience: SoftwareExperience
    hardware_experience: HardwareExperience

    def to_prompt_context(self) -> str:
        """
        Convert experience levels to human-readable prompt text.

        Returns:
            Formatted string for LLM prompt inclusion
        """
        return f"""- Software Development Experience: {self.software_experience}
- Hardware/Robotics Experience: {self.hardware_experience}"""

    def get_adaptation_guidelines(self) -> str:
        """
        Get specific guidelines for LLM based on experience combination.

        Returns:
            Tailored instructions for content adaptation
        """
        # Determine primary guideline based on experience
        if self.software_experience == "beginner" and self.hardware_experience == "none":
            return "Use very simple language, explain all technical terms, include real-world analogies. Assume no prior robotics knowledge."
        elif self.software_experience == "pro" and self.hardware_experience == "professional":
            return "Use concise, technical language. Skip basic explanations. Focus on implementation details, advanced concepts, and best practices."
        elif self.software_experience == "intermediate":
            return "Balance technical depth with clarity. Assume some programming knowledge but explain robotics-specific concepts."
        else:
            return "Adapt content to be accessible yet informative. Explain key concepts without oversimplifying."


class ErrorResponse(BaseModel):
    """Standard error response format for personalization endpoint."""
    detail: str
    error_code: str | None = None
