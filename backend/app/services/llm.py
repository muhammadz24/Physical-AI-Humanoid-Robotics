"""
LLM Service using Official Google Generative AI SDK

This module provides a clean interface to Google's Gemini models using
the official google-generativeai library instead of manual HTTP requests.
The SDK handles URL construction, versioning, and API compatibility automatically.
"""

import asyncio
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
        Generate a response using Google Gemini via official SDK.

        Args:
            prompt: The user's question/prompt
            system_prompt: System instructions for the model

        Returns:
            Generated text response from the model

        Raises:
            Exception: If the API call fails (preserves original error for debugging)
        """
        # Combine system prompt and user prompt
        full_prompt = f"{system_prompt}\n\nUser Question: {prompt}"

        # Run the synchronous SDK call in a thread pool to maintain async compatibility
        loop = asyncio.get_event_loop()
        response = await loop.run_in_executor(
            None,
            self.model.generate_content,
            full_prompt
        )

        # Extract and return the generated text
        if response and response.text:
            return response.text

        # Fallback if no text generated
        return "I apologize, but I couldn't generate a response. Please try again."


# Create a singleton instance
llm_service = LLMService()
