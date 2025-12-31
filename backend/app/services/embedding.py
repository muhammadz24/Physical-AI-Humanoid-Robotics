import google.generativeai as genai
from backend.app.core.config import settings

class EmbeddingService:
    def __init__(self):
        # Configure SDK with API key
        genai.configure(api_key=settings.gemini_api_key)
        self.model = "models/text-embedding-004"

    # Async definition for route compatibility, but SYNCHRONOUS execution for Vercel stability
    async def get_embedding(self, text: str) -> list[float]:
        try:
            text = text.replace("\n", " ")

            # Use synchronous SDK call (stable for Vercel)
            result = genai.embed_content(
                model=self.model,
                content=text,
                task_type="retrieval_document"
            )

            # Return the embedding vector
            return result['embedding']

        except Exception as e:
            print(f"[EMBEDDING ERROR] {str(e)}")
            raise e

embedding_service = EmbeddingService()

# Backward compatibility wrapper
async def get_embedding(text: str) -> list[float]:
    return await embedding_service.get_embedding(text)
