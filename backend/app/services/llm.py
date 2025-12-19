from langchain_google_genai import ChatGoogleGenerativeAI
from langchain_core.messages import HumanMessage, SystemMessage
import os

class LLMService:
    def __init__(self):
        # Uses GOOGLE_API_KEY from environment variables automatically
        # Using gemini-1.5-flash because it is free, fast, and smart
        self.llm = ChatGoogleGenerativeAI(
            model="gemini-1.5-flash",
            temperature=0.7
        )

    async def get_response(self, prompt: str, system_prompt: str = "You are a helpful AI assistant for humanoid robotics."):
        """Generate a response using Google Gemini."""
        try:
            messages = []
            if system_prompt:
                messages.append(SystemMessage(content=system_prompt))
            messages.append(HumanMessage(content=prompt))

            # ainvoke is the async method to call the model
            response = await self.llm.ainvoke(messages)
            return response.content
        except Exception as e:
            print(f"Error generating LLM response: {e}")
            return "I apologize, but I encountered an error connecting to the AI model. Please check your API configuration."

# Create a singleton instance
llm_service = LLMService()
