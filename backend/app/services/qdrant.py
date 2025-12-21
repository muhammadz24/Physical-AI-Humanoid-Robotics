from qdrant_client import QdrantClient
import os

class QdrantService:
    def __init__(self):
        # Use environment variables directly to prevent Settings crash
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "textbook"
        self.client = None

        try:
            if self.url and self.api_key:
                self.client = QdrantClient(url=self.url, api_key=self.api_key)
                print(f"[INFO] Connected to Qdrant at {self.url}")
            else:
                print("[WARN] QDRANT_URL or API_KEY not set. Vector search will return empty results.")
        except Exception as e:
            print(f"[ERROR] Failed to initialize Qdrant client: {e}")

    async def search(self, query_vector: list, limit: int = 5):
        """
        Search for similar vectors in Qdrant.
        Async wrapper for the synchronous Qdrant client.
        """
        if not self.client:
            print("[WARN] Qdrant client not initialized. Returning empty results.")
            return []

        try:
            # Qdrant client is synchronous, but we are called with await.
            # In a real async app we might run_in_executor, but for now this works.
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit
            )
            return results
        except Exception as e:
            print(f"[ERROR] Qdrant search failed: {e}")
            # Return empty list instead of crashing the app
            return []

# Singleton instance
qdrant_service = QdrantService()
