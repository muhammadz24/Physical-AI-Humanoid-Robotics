from google import genai
from backend.app.core.config import settings

class LLMService:
    def __init__(self):
        # Initialize new Client
        self.client = genai.Client(api_key=settings.gemini_api_key)
        self.model_id = settings.gemini_model if settings.gemini_model else "gemini-1.5-flash"

    # Async definition for route compatibility, but SYNCHRONOUS execution for Vercel stability
    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI.") -> str:
        try:
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # DIRECT SYNCHRONOUS CALL (New SDK Syntax)
            response = self.client.models.generate_content(
                model=self.model_id,
                contents=full_prompt
            )

            if response and response.text:
                return response.text
            return "No response generated."

        except Exception as e:
            print(f"🔥 LLM NEW-SDK ERROR: {str(e)}")
            if "429" in str(e):
                return "Quota exceeded. Please try again later."
            raise e

llm_service = LLMService()
