"""
Embedding Service using Sentence Transformers

Generates 384-dimensional embeddings using the all-MiniLM-L6-v2 model.
This model is lightweight, fast, and runs locally without API costs.
"""

from typing import List, Union
import numpy as np
from sentence_transformers import SentenceTransformer

from app.core.config import settings


class EmbeddingService:
    """
    Service for generating text embeddings using Sentence Transformers.

    Uses the all-MiniLM-L6-v2 model which produces 384-dimensional embeddings.
    Model is loaded once on initialization and cached for efficiency.

    Performance:
    - Single encoding: ~80-100ms
    - Batch encoding (32 texts): ~500-800ms
    - Memory usage: ~400MB for model
    """

    def __init__(self):
        """Initialize and load the Sentence Transformer model."""
        self.model = None
        self.model_name = settings.embedding_model
        self.dimension = settings.embedding_dimension
        self._is_loaded = False

    def load_model(self) -> None:
        """
        Load the Sentence Transformer model into memory.

        Only loads once - subsequent calls are no-ops.
        """
        if self._is_loaded:
            return

        try:
            print(f"📥 Loading embedding model: {self.model_name}...")
            self.model = SentenceTransformer(self.model_name)
            self._is_loaded = True
            print(f"✅ Embedding model loaded successfully (dimension: {self.dimension})")
        except Exception as e:
            print(f"❌ Failed to load embedding model: {e}")
            raise

    def encode(self, text: str) -> np.ndarray:
        """
        Generate embedding for a single text string.

        Args:
            text: Input text to encode

        Returns:
            numpy array of shape (384,) containing the embedding

        Raises:
            RuntimeError: If model is not loaded
            ValueError: If text is empty or invalid
        """
        if not self._is_loaded or self.model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        if not text or not text.strip():
            raise ValueError("Text cannot be empty")

        # Encode single text
        embedding = self.model.encode(text, convert_to_numpy=True)

        # Verify dimension
        if embedding.shape[0] != self.dimension:
            raise ValueError(
                f"Unexpected embedding dimension: {embedding.shape[0]} "
                f"(expected {self.dimension})"
            )

        return embedding

    def encode_batch(
        self,
        texts: List[str],
        batch_size: int = 32,
        show_progress: bool = False
    ) -> np.ndarray:
        """
        Generate embeddings for multiple texts in batches.

        Args:
            texts: List of input texts to encode
            batch_size: Number of texts to encode per batch (default: 32)
            show_progress: Show progress bar during encoding (default: False)

        Returns:
            numpy array of shape (len(texts), 384) containing embeddings

        Raises:
            RuntimeError: If model is not loaded
            ValueError: If texts list is empty
        """
        if not self._is_loaded or self.model is None:
            raise RuntimeError("Model not loaded. Call load_model() first.")

        if not texts:
            raise ValueError("Texts list cannot be empty")

        # Filter out empty strings
        valid_texts = [text for text in texts if text and text.strip()]

        if len(valid_texts) != len(texts):
            print(
                f"⚠️  Warning: Filtered out {len(texts) - len(valid_texts)} "
                "empty texts"
            )

        # Encode batch
        embeddings = self.model.encode(
            valid_texts,
            batch_size=batch_size,
            show_progress_bar=show_progress,
            convert_to_numpy=True
        )

        # Verify dimensions
        if embeddings.shape[1] != self.dimension:
            raise ValueError(
                f"Unexpected embedding dimension: {embeddings.shape[1]} "
                f"(expected {self.dimension})"
            )

        return embeddings

    def get_model_info(self) -> dict:
        """
        Get information about the loaded model.

        Returns:
            dict with model metadata
        """
        return {
            "model_name": self.model_name,
            "dimension": self.dimension,
            "is_loaded": self._is_loaded,
            "max_seq_length": self.model.max_seq_length if self._is_loaded else None
        }


# Global embedding service instance
embedding_service = EmbeddingService()
