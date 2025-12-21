import google.generativeai as genai
from app.core.config import settings
import os

# Safety check for runtime
api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY")
if not api_key:
    raise ValueError("CRITICAL: GEMINI_API_KEY is missing in environment variables")

genai.configure(api_key=api_key)

async def get_embedding(text: str):
    result = genai.embed_content(model="models/text-embedding-004", content=text, task_type="retrieval_document")
    return result['embedding']
