from typing import List, Dict, Any, Optional
import time
import json
import traceback
from uuid import UUID
from fastapi import HTTPException
from backend.app.services.embedding import get_embedding
from backend.app.services.qdrant import qdrant_service
from backend.app.services.llm import llm_service
from backend.app.models.chat import ChatResponse, Citation
from backend.app.core.database import db_manager

class ChatService:
    async def process_query(self, query: str, user_id: Optional[UUID] = None) -> ChatResponse:
        """
        Process a user query through the RAG pipeline.
        """
        start_time = time.time()

        try:
            # 1. Generate embedding using Gemini API (with defensive error handling)
            try:
                query_vector = await get_embedding(query)
            except Exception as embedding_error:
                # Log detailed error for developer
                print("=" * 80)
                print("🔥 GEMINI EMBEDDING ERROR")
                print("=" * 80)
                print(f"Error Type: {type(embedding_error).__name__}")
                print(f"Error Message: {str(embedding_error)}")
                print("\nFull Traceback:")
                print(traceback.format_exc())
                print("=" * 80)

                # Return user-friendly error (keeping existing error flow)
                raise HTTPException(
                    status_code=503,
                    detail="Embedding service is currently unavailable. Please try again later."
                )

            # 2. Search Vector DB (with defensive error handling)
            try:
                search_results = await qdrant_service.search(query_vector)
            except Exception as qdrant_error:
                # Log detailed error for developer
                print("=" * 80)
                print("🔥 QDRANT SEARCH ERROR")
                print("=" * 80)
                print(f"Error Type: {type(qdrant_error).__name__}")
                print(f"Error Message: {str(qdrant_error)}")
                print("\nFull Traceback:")
                print(traceback.format_exc())
                print("=" * 80)

                # Return user-friendly error (keeping existing error flow)
                raise HTTPException(
                    status_code=503,
                    detail="Vector search service is currently unavailable. Please try again later."
                )

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

            # 4. Generate Answer (with defensive error handling)
            context_str = "\n\n".join([f"Context {i+1}: {c['content']}" for i, c in enumerate(context_chunks)])

            system_prompt = (
                "You are a helpful AI assistant for a robotics textbook. "
                "Answer the user's question based ONLY on the context provided below. "
                f"\n\nContext:\n{context_str}"
            )

            try:
                answer = await llm_service.get_response(query, system_prompt=system_prompt)
            except Exception as llm_error:
                # Log detailed error for developer
                print("=" * 80)
                print("🔥 GEMINI LLM ERROR")
                print("=" * 80)
                print(f"Error Type: {type(llm_error).__name__}")
                print(f"Error Message: {str(llm_error)}")
                print("\nFull Traceback:")
                print(traceback.format_exc())
                print("=" * 80)

                # Return user-friendly error (keeping existing error flow)
                raise HTTPException(
                    status_code=503,
                    detail="AI language model is currently unavailable. Please try again later."
                )

            # 5. Save to database if user is logged in
            print(f"[CHAT SERVICE DEBUG] Checking DB save: user_id={user_id}, pool={bool(db_manager.pool)}")

            if user_id and db_manager.pool:
                print(f"[CHAT SERVICE DEBUG] ✅ Saving chat for user: {user_id}")
                try:
                    # Prepare metadata (citations, confidence, etc.)
                    metadata = {
                        "citations": [
                            {
                                "chapter": c.chapter,
                                "chapter_title": c.chapter_title,
                                "section": c.section,
                                "similarity_score": c.similarity_score
                            }
                            for c in citations
                        ],
                        "confidence": citations[0].similarity_score if citations else 0.0,
                        "retrieved_chunks": len(citations),
                        "model": "gemini-1.5-flash-001"
                    }

                    # Insert chat into database
                    async with db_manager.pool.acquire() as conn:
                        await conn.execute(
                            """
                            INSERT INTO chats (user_id, query, response, metadata)
                            VALUES ($1, $2, $3, $4)
                            """,
                            user_id,
                            query,
                            answer,
                            json.dumps(metadata)
                        )

                    print(f"[CHAT SERVICE DEBUG] ✅ Chat saved to database successfully")

                except Exception as db_error:
                    # Don't fail the request if DB save fails, just log it
                    print(f"[WARNING] Failed to save chat to database: {db_error}")

            # 6. Response
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

    async def get_user_chat_history(self, user_id: UUID, limit: int = 50) -> List[Dict[str, Any]]:
        """
        Fetch chat history for a specific user.

        Args:
            user_id: User's UUID
            limit: Maximum number of messages to retrieve (default: 50)

        Returns:
            List of chat messages sorted by created_at ASC (oldest first)
        """
        if not db_manager.pool:
            raise Exception("Database pool not initialized")

        try:
            async with db_manager.pool.acquire() as conn:
                rows = await conn.fetch(
                    """
                    SELECT id, query, response, metadata, created_at
                    FROM chats
                    WHERE user_id = $1
                    ORDER BY created_at ASC
                    LIMIT $2
                    """,
                    user_id,
                    limit
                )

            # Convert rows to list of dictionaries
            chat_history = []
            for row in rows:
                chat_history.append({
                    "id": str(row['id']),
                    "query": row['query'],
                    "response": row['response'],
                    "metadata": row['metadata'],  # Already parsed as dict by asyncpg
                    "created_at": row['created_at'].isoformat() if row['created_at'] else None
                })

            return chat_history

        except Exception as e:
            print(f"[ERROR] Failed to fetch chat history: {e}")
            raise

chat_service = ChatService()
