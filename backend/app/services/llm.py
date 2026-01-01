import os
import httpx
import json
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # Key uthao (Jahan se bhi mile)
        self.api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
        
        # Latest Stable Model
        self.model_name = "gemini-1.5-flash"

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        if not self.api_key:
            return "CRITICAL ERROR: Google API Key is missing in Environment Variables."

        # DIRECT GOOGLE API URL (Bina kisi library ke)
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{self.model_name}:generateContent?key={self.api_key}"
        
        headers = {"Content-Type": "application/json"}
        
        # Simple JSON Payload
        payload = {
            "contents": [{
                "parts": [{"text": f"{system_prompt}\n\nUser Question: {prompt}"}]
            }]
        }

        async with httpx.AsyncClient() as client:
            try:
                # Direct Post Request
                response = await client.post(url, headers=headers, json=payload, timeout=30.0)
                
                # Agar Google ne Error diya (e.g. 400, 403, 404)
                if response.status_code != 200:
                     error_data = response.text
                     print(f"[GOOGLE RAW ERROR] {error_data}")
                     return f"GOOGLE API ERROR ({response.status_code}): {error_data}"
                
                # Agar Success hua to data nikalo
                data = response.json()
                try:
                    return data["candidates"][0]["content"]["parts"][0]["text"]
                except (KeyError, IndexError):
                    return "Error: Empty response from Google."
                
            except Exception as e:
                print(f"[CONNECTION ERROR] {str(e)}")
                return f"CONNECTION ERROR: {str(e)}"

llm_service = LLMService()