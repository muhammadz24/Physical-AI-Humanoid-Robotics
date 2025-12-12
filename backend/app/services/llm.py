"""
LLM Service using OpenAI API

Generates contextual responses using OpenAI's GPT models.
Enforces strict context-only answering for RAG use case.
"""

from typing import List, Dict, Any, Optional
from openai import OpenAI

from app.core.config import settings


class LLMService:
    """
    Service for generating responses using OpenAI GPT models.

    Uses GPT-4 or GPT-3.5-turbo for response generation.
    Enforces strict context-only answering to prevent hallucinations.
    """

    def __init__(self, api_key: Optional[str] = None, model: Optional[str] = None):
        """
        Initialize OpenAI client.

        Args:
            api_key: OpenAI API key (defaults to settings.openai_api_key)
            model: Model to use (defaults to settings.openai_model)
        """
        self.client = OpenAI(api_key=api_key or settings.openai_api_key)
        self.model = model or settings.openai_model

    def generate_response(
        self,
        query: str,
        context_chunks: List[Dict[str, Any]],
        max_tokens: int = 500,
        temperature: float = 0.3
    ) -> Dict[str, Any]:
        """
        Generate a response to user query using retrieved context.

        Args:
            query: User's question
            context_chunks: List of retrieved chunks with metadata
            max_tokens: Maximum tokens in response (default: 500)
            temperature: Sampling temperature 0-1 (default: 0.3 for factual)

        Returns:
            Dictionary with answer, confidence, and metadata
        """
        # Build context from chunks
        context_text = self._build_context(context_chunks)

        # Construct system prompt
        system_prompt = self._get_system_prompt()

        # Construct user prompt
        user_prompt = f"""Context from the Physical AI & Humanoid Robotics textbook:

{context_text}

Question: {query}

Instructions:
- Answer ONLY using the information provided in the context above
- If the answer is not in the context, say "I don't have enough information in the textbook to answer this question"
- Be concise but complete
- Cite specific sections when possible
- Use technical terms from the context accurately"""

        try:
            # Call OpenAI API
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=max_tokens,
                temperature=temperature,
                n=1
            )

            # Extract answer
            answer = response.choices[0].message.content.strip()

            # Calculate confidence based on finish reason and chunk scores
            avg_score = sum(c.get("score", 0) for c in context_chunks) / len(context_chunks)
            confidence = min(avg_score, 0.95)  # Cap at 95%

            return {
                "answer": answer,
                "confidence": confidence,
                "model": self.model,
                "tokens_used": response.usage.total_tokens
            }

        except Exception as e:
            print(f"❌ LLM generation failed: {e}")
            return {
                "answer": "I'm sorry, I encountered an error generating a response.",
                "confidence": 0.0,
                "error": str(e)
            }

    def _build_context(self, chunks: List[Dict[str, Any]]) -> str:
        """
        Build formatted context string from retrieved chunks.

        Args:
            chunks: List of chunk dictionaries from Qdrant search

        Returns:
            Formatted context string
        """
        context_parts = []

        for i, chunk in enumerate(chunks, 1):
            payload = chunk.get("payload", {})
            chapter = payload.get("chapter", "Unknown")
            chapter_title = payload.get("chapter_title", "")
            section = payload.get("section", "")
            content = payload.get("content", "")

            context_parts.append(
                f"[Source {i}: Chapter {chapter} - {chapter_title}, Section: {section}]\n"
                f"{content}\n"
            )

        return "\n---\n".join(context_parts)

    def _get_system_prompt(self) -> str:
        """
        Get the system prompt that enforces context-only answering.

        Returns:
            System prompt string
        """
        return """You are an AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role:
- Answer questions ONLY using the provided context from the textbook
- Be accurate and cite specific chapters/sections when relevant
- If the answer is not in the context, clearly state you don't have that information
- Never make up or infer information beyond what's explicitly in the context
- Be helpful but stay strictly within the bounds of the provided material

Your responses should be:
- Concise but complete
- Technically accurate using terms from the textbook
- Clear about which chapter/section information comes from
- Honest when information is not available"""


# Global LLM service instance
llm_service = LLMService()
