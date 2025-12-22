import google.generativeai as genai
from backend.app.core.config import settings
import os

class LLMService:
    def __init__(self):
        # Configure with API key
        api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY")
        if not api_key:
            raise ValueError("CRITICAL: GEMINI_API_KEY is missing in environment variables")

        genai.configure(api_key=api_key)
        # Using gemini-1.5-flash because it is free, fast, and smart
        self.model = genai.GenerativeModel("gemini-1.5-flash")

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI assistant for humanoid robotics."):
        """Generate a response using Google Gemini."""
        try:
            # Combine system prompt and user prompt
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # Use generate_content for synchronous call (gemini SDK doesn't have native async)
            response = self.model.generate_content(full_prompt)
            return response.text
        except Exception as e:
            print(f"Error generating LLM response: {e}")
            return "I apologize, but I encountered an error connecting to the AI model. Please check your API configuration."

# Create a singleton instance
llm_service = LLMService()
