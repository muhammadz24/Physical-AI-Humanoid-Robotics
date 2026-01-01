import os
import google.generativeai as genai
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # STEP 1: Key dhundne ki koshish (Settings se ya Direct Env se)
        api_key = settings.gemini_api_key
        
        # Agar settings mein key nahi mili, to direct Environment check karo
        if not api_key:
            print("DEBUG: Settings failed, checking os.getenv for GEMINI_API_KEY")
            api_key = os.getenv("GEMINI_API_KEY")
        
        # Agar abhi bhi nahi mili, to GOOGLE_API_KEY check karo
        if not api_key:
            print("DEBUG: checking os.getenv for GOOGLE_API_KEY")
            api_key = os.getenv("GOOGLE_API_KEY")

        # Console mein batao ke key mili ya nahi (Security: Key print nahi hogi)
        if api_key:
            print(f"DEBUG: API Key Loaded Successfully! (Length: {len(api_key)})")
        else:
            print("CRITICAL ERROR: No API Key found in Environment Variables!")

        genai.configure(api_key=api_key)
        # Standard Stable Model
        self.model = genai.GenerativeModel("gemini-1.5-flash")

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        try:
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"
            
            # Request send karo
            response = self.model.generate_content(full_prompt)

            if response and response.text:
                return response.text
            return "No response generated."

        except Exception as e:
            # ERROR CHUPANA NAHI HAI - ASLI ERROR DIKHAO
            error_msg = str(e)
            print(f"[LLM RAW ERROR] {error_msg}")
            
            return f"GOOGLE API ERROR: {error_msg}"

llm_service = LLMService()