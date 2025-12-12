"""
Text Chunking Service

Splits markdown content into semantically meaningful chunks with overlap.
Implements a recursive character splitter optimized for markdown structure.
"""

import re
from typing import List, Dict, Any
import tiktoken


class TextChunker:
    """
    Service for chunking text with token-aware splitting.

    Strategy:
    1. Split by H2 headings (## Section Title) first
    2. If section > max_tokens, split by paragraphs
    3. Add overlap from previous chunk
    4. Preserve code blocks intact
    """

    def __init__(
        self,
        max_tokens: int = 500,
        overlap_tokens: int = 50,
        encoding_name: str = "cl100k_base"
    ):
        """
        Initialize chunker with token limits.

        Args:
            max_tokens: Maximum tokens per chunk (default: 500)
            overlap_tokens: Tokens to overlap between chunks (default: 50)
            encoding_name: Tiktoken encoding to use (default: cl100k_base for GPT-4)
        """
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        self.encoding = tiktoken.get_encoding(encoding_name)

    def count_tokens(self, text: str) -> int:
        """
        Count tokens in text using tiktoken.

        Args:
            text: Input text

        Returns:
            Number of tokens
        """
        return len(self.encoding.encode(text))

    def get_last_n_tokens(self, text: str, n: int) -> str:
        """
        Get the last N tokens from text.

        Args:
            text: Input text
            n: Number of tokens to extract

        Returns:
            String containing last N tokens
        """
        tokens = self.encoding.encode(text)
        if len(tokens) <= n:
            return text

        last_tokens = tokens[-n:]
        return self.encoding.decode(last_tokens)

    def split_by_headings(self, content: str) -> List[str]:
        """
        Split content by H2 headings (## Title).

        Args:
            content: Markdown content

        Returns:
            List of sections (each starting with ## or plain text)
        """
        # Split by ## headings (H2 level)
        sections = re.split(r'\n(?=##\s)', content)

        # Filter out empty sections
        sections = [s.strip() for s in sections if s.strip()]

        return sections

    def split_by_paragraphs(self, text: str) -> List[str]:
        """
        Split text by paragraphs (double newline).

        Args:
            text: Input text

        Returns:
            List of paragraphs
        """
        # Split by double newline
        paragraphs = re.split(r'\n\n+', text)

        # Filter out empty paragraphs
        paragraphs = [p.strip() for p in paragraphs if p.strip()]

        return paragraphs

    def is_code_block(self, text: str) -> bool:
        """
        Check if text contains a code block.

        Args:
            text: Input text

        Returns:
            True if text contains code block markers
        """
        return '```' in text or text.strip().startswith('    ')

    def chunk_section(self, section: str) -> List[str]:
        """
        Chunk a single section into smaller pieces if needed.

        Args:
            section: Text section to chunk

        Returns:
            List of chunks
        """
        chunks = []
        token_count = self.count_tokens(section)

        # If section fits in max_tokens, return as is
        if token_count <= self.max_tokens:
            return [section]

        # If section is a code block, try to keep it intact
        if self.is_code_block(section):
            # If code block is too large, we still need to split
            if token_count > self.max_tokens * 1.5:  # Allow 50% overflow for code
                print(f"⚠️  Warning: Large code block ({token_count} tokens) will be split")
            else:
                return [section]

        # Split by paragraphs
        paragraphs = self.split_by_paragraphs(section)
        current_chunk = ""
        previous_chunk = ""

        for paragraph in paragraphs:
            # Try adding paragraph to current chunk
            test_chunk = current_chunk + "\n\n" + paragraph if current_chunk else paragraph
            test_token_count = self.count_tokens(test_chunk)

            if test_token_count <= self.max_tokens:
                # Paragraph fits, add to current chunk
                current_chunk = test_chunk
            else:
                # Paragraph doesn't fit, save current chunk and start new one
                if current_chunk:
                    chunks.append(current_chunk)
                    previous_chunk = current_chunk

                # Start new chunk with overlap
                if previous_chunk and self.overlap_tokens > 0:
                    overlap_text = self.get_last_n_tokens(
                        previous_chunk,
                        self.overlap_tokens
                    )
                    current_chunk = overlap_text + "\n\n" + paragraph
                else:
                    current_chunk = paragraph

                # If single paragraph is too large, split by sentences
                if self.count_tokens(current_chunk) > self.max_tokens:
                    sentences = re.split(r'(?<=[.!?])\s+', paragraph)
                    for sentence in sentences:
                        if self.count_tokens(current_chunk + " " + sentence) <= self.max_tokens:
                            current_chunk += " " + sentence
                        else:
                            if current_chunk:
                                chunks.append(current_chunk)
                            current_chunk = sentence

        # Add final chunk
        if current_chunk:
            chunks.append(current_chunk)

        return chunks

    def chunk_text(
        self,
        content: str,
        metadata: Dict[str, Any]
    ) -> List[Dict[str, Any]]:
        """
        Chunk text content into semantically meaningful pieces.

        Args:
            content: Text content to chunk
            metadata: Metadata dictionary to attach to each chunk

        Returns:
            List of chunk dictionaries with content, metadata, and token count
        """
        if not content or not content.strip():
            return []

        # Step 1: Split by H2 headings
        sections = self.split_by_headings(content)

        # Step 2: Chunk each section if needed
        all_chunks = []
        for section in sections:
            section_chunks = self.chunk_section(section)
            all_chunks.extend(section_chunks)

        # Step 3: Create chunk objects with metadata
        chunk_objects = []
        for i, chunk_text in enumerate(all_chunks):
            chunk_obj = {
                "chunk_id": f"ch{metadata['chapter']}-{i+1:03d}",
                "content": chunk_text,
                "token_count": self.count_tokens(chunk_text),
                "metadata": metadata
            }
            chunk_objects.append(chunk_obj)

        return chunk_objects

    def get_stats(self, chunks: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Calculate statistics for a list of chunks.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            Dictionary with statistics
        """
        if not chunks:
            return {
                "total_chunks": 0,
                "total_tokens": 0,
                "avg_tokens": 0,
                "min_tokens": 0,
                "max_tokens": 0
            }

        token_counts = [chunk["token_count"] for chunk in chunks]

        return {
            "total_chunks": len(chunks),
            "total_tokens": sum(token_counts),
            "avg_tokens": sum(token_counts) // len(token_counts),
            "min_tokens": min(token_counts),
            "max_tokens": max(token_counts)
        }
