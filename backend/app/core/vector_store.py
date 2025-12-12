"""
Qdrant Vector Store Client

Manages connection to Qdrant Cloud for vector similarity search.
Handles embedding storage and retrieval for RAG chatbot.
"""

from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter

from app.core.config import settings


class VectorStoreManager:
    """
    Manages Qdrant Cloud vector database client.

    Provides methods for:
    - Initializing collection
    - Inserting embeddings (during ingestion)
    - Searching for similar vectors (during query)
    - Health checking connection
    """

    def __init__(self):
        self.client: Optional[QdrantClient] = None
        self.collection_name: str = settings.qdrant_collection
        self._is_connected: bool = False

    def connect(self) -> None:
        """
        Initialize Qdrant client with cloud credentials.

        Connection is synchronous (Qdrant client uses GRPC).
        """
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=10.0
            )
            self._is_connected = True
            print("✅ Qdrant vector store connected")
        except Exception as e:
            self._is_connected = False
            print(f"❌ Qdrant connection failed: {e}")
            raise

    def disconnect(self) -> None:
        """Close Qdrant client connection."""
        if self.client is not None:
            self.client.close()
            self.client = None
            self._is_connected = False
            print("🔌 Qdrant vector store disconnected")

    def create_collection(self) -> None:
        """
        Create Qdrant collection if it doesn't exist.

        Collection configuration:
        - Vector size: 384 (all-MiniLM-L6-v2 model dimension)
        - Distance metric: Cosine similarity
        - HNSW index for fast search

        This method is called during setup/ingestion, not during normal API operation.
        """
        if self.client is None:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [col.name for col in collections]

            if self.collection_name in collection_names:
                print(f"✅ Collection '{self.collection_name}' already exists")
                return

            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=settings.embedding_dimension,
                    distance=Distance.COSINE
                )
            )
            print(f"✅ Collection '{self.collection_name}' created successfully")
        except Exception as e:
            print(f"❌ Failed to create collection: {e}")
            raise

    def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.7,
        chapter_filter: Optional[str] = None
    ) -> List[Dict[str, Any]]:
        """
        Search for similar vectors in Qdrant collection.

        Args:
            query_vector: Embedding vector to search for (384-dim)
            top_k: Number of results to return (default: 5)
            score_threshold: Minimum similarity score (default: 0.7)
            chapter_filter: Optional chapter number to filter by (e.g., "3")

        Returns:
            List of search results with payload and similarity scores
        """
        if self.client is None:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        try:
            # Build filter if chapter specified
            search_filter = None
            if chapter_filter:
                search_filter = Filter(
                    must=[
                        {"key": "chapter", "match": {"value": chapter_filter}}
                    ]
                )

            # Perform vector search
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
                query_filter=search_filter,
                with_payload=True
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                })

            return formatted_results
        except Exception as e:
            print(f"❌ Vector search failed: {e}")
            raise

    def upsert(self, points: List[PointStruct]) -> None:
        """
        Insert or update vectors in Qdrant collection.

        Args:
            points: List of PointStruct objects with id, vector, and payload

        This method is used during content ingestion (Phase 2).
        """
        if self.client is None:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
        except Exception as e:
            print(f"❌ Vector upsert failed: {e}")
            raise

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get collection statistics (vector count, size, etc.).

        Returns:
            Dict with collection metadata
        """
        if self.client is None:
            raise RuntimeError("Qdrant client not initialized. Call connect() first.")

        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": info.name,
                "vectors_count": info.vectors_count,
                "points_count": info.points_count,
                "status": info.status
            }
        except Exception as e:
            print(f"❌ Failed to get collection info: {e}")
            return {}

    def health_check(self) -> bool:
        """
        Check if Qdrant connection is healthy.

        Returns:
            bool: True if Qdrant is reachable, False otherwise
        """
        if not self._is_connected or self.client is None:
            return False

        try:
            # Try to get collection info as health check
            self.client.get_collections()
            return True
        except Exception as e:
            print(f"Qdrant health check failed: {e}")
            return False


# Global vector store manager instance
vector_store = VectorStoreManager()
