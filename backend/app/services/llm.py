import google.generativeai as genai
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # Configure SDK with API key
        genai.configure(api_key=settings.gemini_api_key)
        # Use stable model version
        self.model = genai.GenerativeModel("gemini-1.5-flash-001")

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
                return "Error: Model not found. Please check if 'Generative Language API' is enabled in Google Cloud Console."
            if "429" in str(e):
                return "Quota exceeded. Please try again later."
            raise e

llm_service = LLMService()
