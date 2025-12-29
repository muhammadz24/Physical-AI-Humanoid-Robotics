"""
LLM Service using Official Google Generative AI SDK (Native Async)

Uses the SDK's built-in async methods (generate_content_async) instead of
wrapping synchronous calls. This prevents event loop issues in serverless
environments like Vercel.
"""

import google.generativeai as genai
from backend.app.core.config import settings


class LLMService:
    def __init__(self):
        """Initialize the LLM service with Google Gemini SDK."""
        # Configure the official SDK with API key from settings
        genai.configure(api_key=settings.gemini_api_key)

        # Initialize the generative model
        # SDK automatically handles model versioning and URL construction
        self.model = genai.GenerativeModel(settings.gemini_model)

    async def get_response(
        self,
        prompt: str,
        system_prompt: str = "You are a helpful AI assistant for humanoid robotics."
    ) -> str:
        """
        Generate a response using Google Gemini via native async SDK method.

        Args:
            prompt: The user's question/prompt
            system_prompt: System instructions for the model

        Returns:
            Generated text response from the model

        Raises:
            Exception: If the API call fails (preserves original error for debugging)
        """
        try:
            # Combine system prompt and user prompt
            full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

            # USE NATIVE ASYNC METHOD (No asyncio.run_in_executor needed)
            response = await self.model.generate_content_async(full_prompt)

            # Extract and return the generated text
            if response and response.text:
                return response.text

            # Fallback if no text generated
            return "I apologize, but I couldn't generate a response. Please try again."

        except Exception as e:
            print(f"🔥 LLM ASYNC ERROR: {str(e)}")
            # Re-raise to be caught by chat service error handler
            raise e


# Create a singleton instance
llm_service = LLMService()
