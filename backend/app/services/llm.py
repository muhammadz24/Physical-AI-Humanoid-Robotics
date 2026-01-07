import os
import httpx
import json
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # API Keys uthao
        self.gemini_api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
        self.groq_api_key = os.getenv("GROQ_API_KEY") or os.getenv("GROQ_API_KEY")
        self.gemini_base_url = "https://generativelanguage.googleapis.com/v1beta"
        self.cached_model = None  # Model yaad rakhne ke liye

    async def get_working_model(self):
        """Google se available models ki list mangwa kar pehla valid model chuno"""
        if self.cached_model:
            return self.cached_model

        if not self.gemini_api_key:
            return "gemini-pro"  # fallback

        url = f"{self.gemini_base_url}/models?key={self.gemini_api_key}"
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

    async def get_groq_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        """Try to get response from Groq if available"""
        if not self.groq_api_key:
            raise Exception("GROQ_API_KEY not set")

        # Use a fast Groq model
        model_name = "llama3-8b-8192"  # Fast and reliable Groq model

        url = "https://api.groq.com/openai/v1/chat/completions"
        headers = {
            "Authorization": f"Bearer {self.groq_api_key}",
            "Content-Type": "application/json"
        }
        payload = {
            "model": model_name,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.7,
            "max_tokens": 1000
        }

        async with httpx.AsyncClient() as client:
            try:
                response = await client.post(url, headers=headers, json=payload, timeout=30.0)

                if response.status_code != 200:
                    raise Exception(f"GROQ API ERROR ({response.status_code}): {response.text}")

                data = response.json()
                return data["choices"][0]["message"]["content"]

            except Exception as e:
                raise Exception(f"GROQ CONNECTION ERROR: {str(e)}")

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        # First, try Gemini
        if self.gemini_api_key:
            try:
                return await self._get_gemini_response(prompt, system_prompt)
            except Exception as e:
                print(f"[FALLBACK] Gemini failed: {e}, trying Groq...")
                # If Gemini fails, try Groq
                if self.groq_api_key:
                    try:
                        return await self.get_groq_response(prompt, system_prompt)
                    except Exception as groq_error:
                        print(f"[ERROR] Both Gemini and Groq failed. Gemini: {e}, Groq: {groq_error}")
                        return f"GEMINI API ERROR: {str(e)}. GROQ FALLBACK ALSO FAILED: {str(groq_error)}"
                else:
                    return f"GEMINI API ERROR: {str(e)}. GROQ API key not configured for fallback."
        else:
            # No Gemini key, try Groq directly
            if self.groq_api_key:
                try:
                    return await self.get_groq_response(prompt, system_prompt)
                except Exception as groq_error:
                    return f"GROQ API ERROR: {str(groq_error)}"
            else:
                return "CRITICAL ERROR: No API keys configured (neither Gemini nor Groq)."

    async def _get_gemini_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        if not self.gemini_api_key:
            return "CRITICAL ERROR: Google API Key is missing."

        # 1. Sahi Model Dhoondo (Auto-Detect)
        model_name = await self.get_working_model()

        # 2. Request Bhejo
        url = f"{self.gemini_base_url}/models/{model_name}:generateContent?key={self.gemini_api_key}"
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
                raise Exception(f"CONNECTION ERROR: {str(e)}")

llm_service = LLMService()