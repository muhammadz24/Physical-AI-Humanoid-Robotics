import httpx
from backend.app.core.config import settings
import os
import json

class LLMService:
    def __init__(self):
        # Get API key from settings
        self.api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY")
        if not self.api_key:
            raise ValueError("CRITICAL: GEMINI_API_KEY is missing in environment variables")

        self.api_url = f"https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key={self.api_key}"

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI assistant for humanoid robotics."):
        """Generate a response using Google Gemini via direct HTTP."""
        try:
            # Combine system prompt and user prompt
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # Prepare request payload
            payload = {
                "contents": [{
                    "parts": [{
                        "text": full_prompt
                    }]
                }]
            }

            # Make HTTP request
            async with httpx.AsyncClient(timeout=30.0) as client:
                response = await client.post(self.api_url, json=payload)
                response.raise_for_status()

                data = response.json()
                # Extract text from response
                if "candidates" in data and len(data["candidates"]) > 0:
                    candidate = data["candidates"][0]
                    if "content" in candidate and "parts" in candidate["content"]:
                        parts = candidate["content"]["parts"]
                        if len(parts) > 0 and "text" in parts[0]:
                            return parts[0]["text"]

                return "I apologize, but I couldn't generate a response. Please try again."

        except Exception as e:
            print(f"Error generating LLM response: {e}")
            return "I apologize, but I encountered an error connecting to the AI model. Please check your API configuration."

# Create a singleton instance
llm_service = LLMService()
