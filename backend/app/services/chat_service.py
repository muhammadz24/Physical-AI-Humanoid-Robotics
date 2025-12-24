from typing import List, Dict, Any
import time
from backend.app.services.embedding import get_embedding
from backend.app.services.qdrant import qdrant_service
from backend.app.services.llm import llm_service
from backend.app.models.chat import ChatResponse, Citation

class ChatService:
    async def process_query(self, query: str) -> ChatResponse:
        """
        Process a user query through the RAG pipeline.
        """
        start_time = time.time()

        try:
            # 1. Generate embedding using Gemini API
            query_vector = await get_embedding(query)

            # 2. Search Vector DB
            search_results = await qdrant_service.search(query_vector)

            # 3. Prepare Context
            context_chunks = []
            citations = []

            for result in search_results:
                payload = result.payload or {}
                chunk_text = payload.get("content", "")

                context_chunks.append({
                    "content": chunk_text,
                    "score": result.score,
                    "metadata": payload
                })

                citations.append(Citation(
                    chapter=str(payload.get("chapter", "Unknown")),
                    chapter_title=payload.get("chapter_title", "Unknown"),
                    section=str(payload.get("section", "")),
                    chunk_id=str(result.id),
                    similarity_score=result.score,
                    source_file=payload.get("source_file", "")
                ))

            # 4. Generate Answer
            context_str = "\n\n".join([f"Context {i+1}: {c['content']}" for i, c in enumerate(context_chunks)])

            system_prompt = (
                "You are a helpful AI assistant for a robotics textbook. "
                "Answer the user's question based ONLY on the context provided below. "
                f"\n\nContext:\n{context_str}"
            )

            answer = await llm_service.get_response(query, system_prompt=system_prompt)

            # 5. Response
            return ChatResponse(
                status="success",
                answer=answer,
                citations=citations,
                confidence=citations[0].similarity_score if citations else 0.0,
                retrieved_chunks=len(citations),
                response_time_ms=(time.time() - start_time) * 1000,
                model="gemini-1.5-flash-001"
            )

        except Exception as e:
            import traceback
            error_trace = traceback.format_exc()
            print(f"🐛 CHAT SERVICE EXCEPTION:\n{error_trace}")

            # DEBUG MODE: Include actual error details
            error_details = f"{type(e).__name__}: {str(e)}"
            return ChatResponse(
                status="error",
                answer=f"DEBUG ERROR: {error_details}",
                citations=[],
                confidence=0.0,
                retrieved_chunks=0,
                response_time_ms=0,
                model="gemini-1.5-flash-001"
            )

chat_service = ChatService()
