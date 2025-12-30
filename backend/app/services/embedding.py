from google import genai
from backend.app.core.config import settings

class EmbeddingService:
    def __init__(self):
        # Initialize new Client
        self.client = genai.Client(api_key=settings.gemini_api_key)
        self.model_id = "text-embedding-004"

    # Async definition for route compatibility, but SYNCHRONOUS execution for Vercel stability
    async def get_embedding(self, text: str) -> list[float]:
        try:
            text = text.replace("\n", " ")

            # DIRECT SYNCHRONOUS CALL (New SDK Syntax)
            result = self.client.models.embed_content(
                model=self.model_id,
                contents=text,
            )

            # New SDK returns object, access embeddings list
            return result.embeddings[0].values

        except Exception as e:
            print(f"🔥 EMBEDDING NEW-SDK ERROR: {str(e)}")
            raise e

embedding_service = EmbeddingService()

# Backward compatibility wrapper
async def get_embedding(text: str) -> list[float]:
    return await embedding_service.get_embedding(text)
