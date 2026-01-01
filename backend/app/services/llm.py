import os
import google.generativeai as genai
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # 1. API Key Load Karo
        api_key = settings.gemini_api_key
        if not api_key:
            api_key = os.getenv("GEMINI_API_KEY")
        if not api_key:
            api_key = os.getenv("GOOGLE_API_KEY")

        if api_key:
            print(f"DEBUG: API Key Found (Length: {len(api_key)})")
        else:
            print("CRITICAL ERROR: No API Key found!")

        genai.configure(api_key=api_key)

        # 2. DEBUG: Available Models ki List Print Karo (Taake pata chale asliyat kya hai)
        print("----- CHECKING AVAILABLE GOOGLE MODELS -----")
        try:
            for m in genai.list_models():
                if 'generateContent' in m.supported_generation_methods:
                    print(f"AVAILABLE: {m.name}")
        except Exception as e:
            print(f"COULD NOT LIST MODELS: {e}")
        print("--------------------------------------------")

        # 3. FIX: Use 'gemini-pro' (Sabse Stable Model) instead of Flash
        # Agar Flash nahi mil raha, to Pro zaroor milega.
        self.model = genai.GenerativeModel("gemini-pro")

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        try:
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"
            response = self.model.generate_content(full_prompt)

            if response and response.text:
                return response.text
            return "No response generated."

        except Exception as e:
            error_msg = str(e)
            print(f"[LLM RAW ERROR] {error_msg}")
            return f"GOOGLE API ERROR: {error_msg}"

llm_service = LLMService()