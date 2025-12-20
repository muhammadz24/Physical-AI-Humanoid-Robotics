from typing import List, Dict, Any
import time
from app.services.embedding import embedding_service
from app.services.qdrant import qdrant_service
from app.services.llm import llm_service
from app.models.chat import ChatResponse, Citation

class ChatService:
    async def process_query(self, query: str) -> ChatResponse:
        """
        Process a user query through the RAG pipeline.

        Flow:
        1. Generate embedding for query (using Gemini)
        2. Search Qdrant for relevant context
        3. Generate answer using LLM (Gemini)
        4. Return formatted response
        """
        start_time = time.time()

        try:
            # 1. Generate embedding
            # Note: We no longer need to check _is_loaded as we use API
            query_vector = await embedding_service.get_embedding(query)

            # 2. Search Vector DB
            # Ensure query_vector is valid list of floats
            search_results = await qdrant_service.search(query_vector)

            # 3. Prepare Context for LLM
            context_chunks = []
            citations = []

            for result in search_results:
                payload = result.payload or {}
                chunk_text = payload.get("content", "")

                # Add to context
                context_chunks.append({
                    "content": chunk_text,
                    "score": result.score,
                    "metadata": payload
                })

                # Add to citations
                citations.append(Citation(
                    chapter=str(payload.get("chapter", "Unknown")),
                    chapter_title=payload.get("chapter_title", "Unknown"),
                    section=str(payload.get("section", "")),
                    chunk_id=str(result.id),
                    similarity_score=result.score,
                    source_file=payload.get("source_file", "")
                ))

            # 4. Generate Answer with LLM
            # Create a context-aware prompt
            context_str = "\n\n".join([f"Context {i+1}: {c['content']}" for i, c in enumerate(context_chunks)])

            system_prompt = (
                "You are a helpful AI assistant for a robotics textbook. "
                "Answer the user's question based ONLY on the context provided below. "
                "If the answer is not in the context, say you don't know. "
                f"\n\nContext:\n{context_str}"
            )

            answer = await llm_service.get_response(query, system_prompt=system_prompt)

            # 5. Construct Response
            execution_time = (time.time() - start_time) * 1000

            return ChatResponse(
                status="success",
                answer=answer,
                citations=citations,
                confidence=citations[0].similarity_score if citations else 0.0,
                retrieved_chunks=len(citations),
                response_time_ms=execution_time,
                model="gemini-1.5-flash"
            )

        except Exception as e:
            print(f"Error in chat service: {e}")
            # Return a graceful error response instead of crashing
            return ChatResponse(
                status="error",
                answer="I encountered an issue processing your request. Please try again.",
                citations=[],
                confidence=0.0,
                retrieved_chunks=0,
                response_time_ms=(time.time() - start_time) * 1000,
                model="gemini-1.5-flash"
            )

# Singleton instance
chat_service = ChatService()
