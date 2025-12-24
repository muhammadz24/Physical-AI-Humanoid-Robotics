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

    except httpx.HTTPStatusError as e:
        # HTTP error from Gemini API (403, 429, 404, etc.)
        error_body = e.response.text
        print(f"🐛 GEMINI EMBEDDING API HTTP ERROR: {e.response.status_code} - {error_body}")
        raise Exception(f"Gemini Embedding API returned {e.response.status_code}: {error_body}")
    except Exception as e:
        # Other errors (network, timeout, missing API key, etc.)
        import traceback
        full_trace = traceback.format_exc()
        print(f"🐛 EMBEDDING SERVICE ERROR:\n{full_trace}")
        raise Exception(f"Embedding Error ({type(e).__name__}): {str(e)}")
