from langchain_google_genai import GoogleGenerativeAIEmbeddings
import os

class EmbeddingService:
    def __init__(self):
        # Ensure GOOGLE_API_KEY is set in Vercel Environment Variables
        # Using the standard free embedding model
        self.embeddings = GoogleGenerativeAIEmbeddings(model="models/embedding-001")

    async def get_embedding(self, text: str):
        """Generate embedding for a single text string."""
        try:
            # Generate embedding using Google Gemini
            return self.embeddings.embed_query(text)
        except Exception as e:
            print(f"Error generating embedding: {e}")
            # Fallback (768 dimensions for Gemini embedding-001) just in case
            return [0.0] * 768

# Create a singleton instance
embedding_service = EmbeddingService()
