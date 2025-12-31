import os
from typing import List, Union, Optional
from pydantic import AnyHttpUrl, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    # 1. CORE API SETTINGS
    API_V1_STR: str = "/api"
    PROJECT_NAME: str = "Physical AI Humanoid Robotics"

    # 2. CORS ORIGINS (Auto-parsed)
    # Default allows localhost and Vercel deployments
    ALLOWED_ORIGINS: str = "http://localhost:3000,https://physical-ai-humanoid-robotics.vercel.app,*"

    @property
    def cors_origins_list(self) -> List[str]:
        return [origin.strip() for origin in self.ALLOWED_ORIGINS.split(",")]

    # 3. DATABASE & VECTOR DB (Required)
    # We use Optional + Default to prevent crash if env is missing, but log warning later
    # Uses a dummy local string as fallback to prevent Pydantic validation error on startup
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql://user:pass@localhost:5432/db")

    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")

    # 4. AI MODELS (Google Gemini)
    GOOGLE_API_KEY: Optional[str] = os.getenv("GOOGLE_API_KEY")
    GEMINI_API_KEY: Optional[str] = os.getenv("GEMINI_API_KEY")
    # Force stable model ID to avoid 404
    GEMINI_MODEL: str = "gemini-1.5-flash-001"

    EMBEDDING_MODEL: str = "models/text-embedding-004"
    EMBEDDING_DIMENSION: int = 768

    # 5. SECURITY & AUTH (THE CRITICAL FIX)
    # If Vercel Env Var is missing, use this HARDCODED fallback to stop the crash.
    JWT_SECRET_KEY: str = os.getenv("JWT_SECRET_KEY", "fallback_secret_key_for_vercel_bypass_2025_secure")
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30

    # 6. CONFIG
    model_config = SettingsConfigDict(
        case_sensitive=True,
        env_file=".env",
        extra="ignore"  # Critical: Ignores extra fields instead of crashing
    )

    # Backward compatibility properties (lowercase field names)
    @property
    def gemini_api_key(self) -> str:
        # Fallback logic for keys
        return self.GEMINI_API_KEY or self.GOOGLE_API_KEY or ""

    @property
    def gemini_model(self) -> str:
        return self.GEMINI_MODEL

    @property
    def qdrant_url(self) -> Optional[str]:
        return self.QDRANT_URL

    @property
    def qdrant_api_key(self) -> Optional[str]:
        return self.QDRANT_API_KEY

    @property
    def database_url(self) -> str:
        return self.DATABASE_URL

    @property
    def jwt_secret_key(self) -> str:
        return self.JWT_SECRET_KEY

    @property
    def qdrant_collection(self) -> str:
        return "textbook"  # Hardcoded default

settings = Settings()
