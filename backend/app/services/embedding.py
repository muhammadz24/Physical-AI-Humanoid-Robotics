"""
Embedding Service using Official Google Generative AI SDK (Native Async)

Uses the SDK's built-in async methods (embed_content_async) instead of
wrapping synchronous calls. This prevents event loop issues in serverless
environments like Vercel.
"""

import google.generativeai as genai
from backend.app.core.config import settings


class EmbeddingService:
    def __init__(self):
        """Initialize the embedding service with Google Gemini SDK."""
        # Configure SDK using the same key as LLM
        genai.configure(api_key=settings.gemini_api_key)

        # Official free embedding model
        self.model = "models/text-embedding-004"

    async def get_embedding(self, text: str) -> list[float]:
        """
        Generate embedding vector for the given text using native async SDK.

        Args:
            text: Input text to embed

        Returns:
            List of 768 floating-point values (embedding vector)

        Raises:
            Exception: If embedding generation fails
        """
        try:
            # Clean input text
            text = text.replace("\n", " ")

            # USE NATIVE ASYNC METHOD (No asyncio.run_in_executor needed)
            result = await genai.embed_content_async(
                model=self.model,
                content=text,
                task_type="retrieval_document",
                title="Embedding"
            )

            # Extract and return embedding vector
            return result['embedding']

        except Exception as e:
            print(f"🔥 EMBEDDING ASYNC ERROR: {str(e)}")
            # Re-raise to be caught by chat service error handler
            raise e


# Create singleton instance
embedding_service = EmbeddingService()


# Backward compatibility: Export function that uses the service instance
async def get_embedding(text: str) -> list[float]:
    """
    Wrapper function for backward compatibility.

    This allows existing code to continue using:
        from backend.app.services.embedding import get_embedding
    """
    return await embedding_service.get_embedding(text)
