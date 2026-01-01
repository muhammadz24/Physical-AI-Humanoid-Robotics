import os
import httpx
import json
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # API Key uthao
        self.api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
        self.base_url = "https://generativelanguage.googleapis.com/v1beta"
        self.cached_model = None  # Model yaad rakhne ke liye

    async def get_working_model(self):
        """Google se available models ki list mangwa kar pehla valid model chuno"""
        if self.cached_model:
            return self.cached_model

        url = f"{self.base_url}/models?key={self.api_key}"
        print(f"[DEBUG] Fetching available models from: {url}")
        
        async with httpx.AsyncClient() as client:
            try:
                resp = await client.get(url, timeout=10.0)
                if resp.status_code == 200:
                    data = resp.json()
                    # List mein se pehla Gemini model dhoondo jo content generate kar sake
                    for m in data.get('models', []):
                        if 'generateContent' in m.get('supportedGenerationMethods', []) and 'gemini' in m.get('name'):
                            found_model = m['name'].replace("models/", "")
                            print(f"[SUCCESS] Found working model: {found_model}")
                            self.cached_model = found_model
                            return found_model
                
                print(f"[WARNING] Could not list models. Status: {resp.status_code}. Response: {resp.text}")
            except Exception as e:
                print(f"[ERROR] Model list failed: {e}")
        
        # Agar kuch na mile to fallback
        return "gemini-pro"

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        if not self.api_key:
            return "CRITICAL ERROR: Google API Key is missing."

        # 1. Sahi Model Dhoondo (Auto-Detect)
        model_name = await self.get_working_model()
        
        # 2. Request Bhejo
        url = f"{self.base_url}/models/{model_name}:generateContent?key={self.api_key}"
        headers = {"Content-Type": "application/json"}
        payload = {
            "contents": [{"parts": [{"text": f"{system_prompt}\n\nUser Question: {prompt}"}]}]
        }

        async with httpx.AsyncClient() as client:
            try:
                response = await client.post(url, headers=headers, json=payload, timeout=30.0)
                
                if response.status_code != 200:
                     return f"GOOGLE API ERROR ({response.status_code}): Using model '{model_name}' - {response.text}"
                
                data = response.json()
                return data["candidates"][0]["content"]["parts"][0]["text"]
                
            except Exception as e:
                return f"CONNECTION ERROR: {str(e)}"

llm_service = LLMService()