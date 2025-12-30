"""
LLM Service using Official Google Generative AI SDK (Synchronous)

Uses synchronous SDK calls for maximum stability in Vercel serverless.
The async function signature is maintained for compatibility.
"""

import google.generativeai as genai
from backend.app.core.config import settings


class LLMService:
    def __init__(self):
        """Initialize the LLM service with Google Gemini SDK."""
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel(settings.gemini_model)

    async def get_response(
        self,
        prompt: str,
        system_prompt: str = "You are a helpful AI assistant for humanoid robotics."
    ) -> str:
        """
        Generate a response using Google Gemini.

        Args:
            prompt: The user's question/prompt
            system_prompt: System instructions for the model

        Returns:
            Generated text response from the model

        Raises:
            Exception: If the API call fails
        """
        try:
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # USE SYNCHRONOUS CALL (Most stable for Vercel)
            response = self.model.generate_content(full_prompt)

            if response and response.text:
                return response.text

            return "I apologize, but I couldn't generate a response. Please try again."

        except Exception as e:
            print(f"🔥 LLM SYNC ERROR: {str(e)}")
            raise e


# Create a singleton instance
llm_service = LLMService()
