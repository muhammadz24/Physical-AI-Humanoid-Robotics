import httpx
from backend.app.core.config import settings
import os

async def get_embedding(text: str):
    """Generate embedding using Google Gemini API via direct HTTP."""
    try:
        # Get API key
        api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("CRITICAL: GEMINI_API_KEY is missing in environment variables")

        api_url = f"https://generativelanguage.googleapis.com/v1beta/models/text-embedding-004:embedContent?key={api_key}"

        # Prepare request payload
        payload = {
            "model": "models/text-embedding-004",
            "content": {
                "parts": [{
                    "text": text
                }]
            }
        }

        # Make HTTP request
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(api_url, json=payload)
            response.raise_for_status()

            data = response.json()
            # Extract embedding from response
            if "embedding" in data and "values" in data["embedding"]:
                return data["embedding"]["values"]

            # Fallback: return zero vector if extraction fails
            return [0.0] * 768

    except Exception as e:
        print(f"Error generating embedding: {e}")
        # Return zero vector on error to maintain compatibility
        return [0.0] * 768
