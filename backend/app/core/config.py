"""
Configuration Management using Pydantic BaseSettings

Loads environment variables from .env file and provides type-safe access
to configuration values.
"""

from typing import List
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.

    All settings are loaded from .env file or environment variables.
    Use .env.example as a template for creating your .env file.
    """

    # FastAPI Configuration
    environment: str = "development"
    debug: bool = True
    api_host: str = "0.0.0.0"
    api_port: int = 8000
    api_version: str = "1.0.0"

    # Qdrant Vector Database
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "textbook"

    # Neon Postgres Database
    database_url: str

    # CORS Configuration
    allowed_origins: str = "http://localhost:3000"

    # Rate Limiting
    rate_limit_per_minute: int = 10

    # Embeddings Configuration
    embedding_model: str = "models/text-embedding-004"
    embedding_dimension: int = 768

    # Google Gemini Configuration
    gemini_api_key: str
    gemini_model: str = "gemini-1.5-flash"

    # JWT Authentication
    jwt_secret_key: str

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    @property
    def allowed_origins_list(self) -> List[str]:
        """Parse comma-separated ALLOWED_ORIGINS into a list."""
        return [origin.strip() for origin in self.allowed_origins.split(",")]

    @property
    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment.lower() == "production"

    def __repr__(self) -> str:
        """Safe representation without exposing secrets."""
        return (
            f"Settings(environment={self.environment}, "
            f"api_host={self.api_host}, "
            f"api_port={self.api_port}, "
            f"qdrant_collection={self.qdrant_collection})"
        )


# Global settings instance
settings = Settings()
