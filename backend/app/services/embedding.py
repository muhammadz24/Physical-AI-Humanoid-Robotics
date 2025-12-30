"""
Embedding Service using Official Google Generative AI SDK (Synchronous)

Uses synchronous SDK calls for maximum stability in Vercel serverless.
The async function signature is maintained for compatibility.
"""

import google.generativeai as genai
from backend.app.core.config import settings


class EmbeddingService:
    def __init__(self):
        """Initialize the embedding service with Google Gemini SDK."""
        genai.configure(api_key=settings.gemini_api_key)
        self.model = "models/text-embedding-004"

    async def get_embedding(self, text: str) -> list[float]:
        """
        Generate embedding vector for the given text.

        Args:
            text: Input text to embed

        Returns:
            List of 768 floating-point values (embedding vector)

        Raises:
            Exception: If embedding generation fails
        """
        try:
            text = text.replace("\n", " ")

            # USE SYNCHRONOUS CALL (Most stable for Vercel)
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_document",
                title="Embedding"
            )

            return result['embedding']

        except Exception as e:
            print(f"🔥 EMBEDDING SYNC ERROR: {str(e)}")
            raise e


# Create singleton instance
embedding_service = EmbeddingService()


# Backward compatibility wrapper
async def get_embedding(text: str) -> list[float]:
    """
    Wrapper function for backward compatibility.

    This allows existing code to continue using:
        from backend.app.services.embedding import get_embedding
    """
    return await embedding_service.get_embedding(text)
