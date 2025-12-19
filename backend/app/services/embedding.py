from langchain_openai import OpenAIEmbeddings
import os

class EmbeddingService:
    def __init__(self):
        # Uses OPENAI_API_KEY from environment variables automatically
        # Using text-embedding-3-small as it is cheap and efficient
        self.embeddings = OpenAIEmbeddings(model="text-embedding-3-small")

    async def get_embedding(self, text: str):
        """Generate embedding for a single text string."""
        try:
            # Langchain's embed_query is synchronous, but fast enough for Vercel
            return self.embeddings.embed_query(text)
        except Exception as e:
            print(f"Error generating embedding: {e}")
            raise e

# Create a singleton instance
embedding_service = EmbeddingService()
