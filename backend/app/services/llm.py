import google.generativeai as genai
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # Configure SDK with API key
        genai.configure(api_key=settings.gemini_api_key)
        
        # FIX: "gemini-1.5-flash-001" hata kar simple "gemini-1.5-flash" kar diya
        # Yeh model har jagah chalta hai
        self.model = genai.GenerativeModel("gemini-1.5-flash")

    # Async definition for route compatibility, but SYNCHRONOUS execution for Vercel stability
    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        try:
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # Use synchronous SDK call (stable for Vercel)
            response = self.model.generate_content(full_prompt)

            if response and response.text:
                return response.text
            return "No response generated."

        except Exception as e:
            print(f"[LLM ERROR] {str(e)}")
            # Friendly error mapping
            if "404" in str(e):
                return "Error: Model not found. Check if API Key is correct and has Gemini API enabled."
            if "429" in str(e):
                return "Quota exceeded. Please try again later."
            # Agar koi aur error ho to detail dikhaye
            return f"Error: {str(e)}"

llm_service = LLMService()