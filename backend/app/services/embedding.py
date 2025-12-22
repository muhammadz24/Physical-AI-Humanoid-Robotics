async def get_embedding(text: str):
    """Return dummy embedding vector while AI backend is being optimized."""
    # Return 768-dimensional zero vector to match expected embedding dimension
    return [0.0] * 768
