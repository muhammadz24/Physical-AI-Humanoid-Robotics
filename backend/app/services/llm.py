class LLMService:
    def __init__(self):
        # AI backend temporarily disabled to meet serverless size limits
        pass

    async def get_response(self, prompt: str, system_prompt: str = ""):
        """Return static message while AI backend is being optimized."""
        return "AI Backend is currently being optimized for browser-side execution. Your Login and Textbook are fully operational."

# Create a singleton instance
llm_service = LLMService()
