#!/usr/bin/env python3
"""
Content Ingestion Script - Updated for Gemini API Embeddings

Orchestrates the complete ingestion pipeline:
1. Read markdown files from docs directory
2. Parse frontmatter and extract metadata
3. Chunk content into semantic pieces
4. Generate embeddings using Gemini API (768 dimensions)
5. Upload vectors to Qdrant Cloud

Usage:
    python backend/scripts/ingest.py --dry-run          # Test without uploading
    python backend/scripts/ingest.py                     # Full ingestion
    python backend/scripts/ingest.py --limit 5           # Limit to first 5 files
"""

import sys
import os
import argparse
import time
import asyncio
from pathlib import Path
from typing import List, Dict, Any

# Fix Windows console UTF-8 encoding
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer, 'strict')

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.services.embedding import get_embedding
from app.services.parser import MarkdownParser
from app.services.chunker import TextChunker
from app.services.qdrant import qdrant_service

from qdrant_client.models import PointStruct
from tqdm import tqdm


class IngestionPipeline:
    """Orchestrates the complete content ingestion pipeline."""

    def __init__(self, docs_directory: str, dry_run: bool = False, limit: int = None):
        self.docs_directory = docs_directory
        self.dry_run = dry_run
        self.limit = limit

        # Initialize services
        self.parser = MarkdownParser(docs_directory)
        self.chunker = TextChunker(max_tokens=500, overlap_tokens=50)

        # Statistics
        self.stats = {
            "files_processed": 0,
            "total_chunks": 0,
            "total_tokens": 0,
        }

    def parse_files(self) -> List[Dict[str, Any]]:
        """Parse all markdown files from docs directory."""
        print("\n[Stage 1] Parsing Markdown Files")
        print("=" * 60)

        pattern = "**/*.md"
        parsed_files = self.parser.parse_all(pattern)

        if self.limit:
            parsed_files = parsed_files[:self.limit]
            print(f"[INFO] Limited to first {self.limit} files")

        print(f"[OK] Parsed {len(parsed_files)} files")
        return parsed_files

    def chunk_files(self, parsed_files: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Chunk all parsed files into semantic pieces."""
        print("\n[Stage 2] Chunking Content")
        print("=" * 60)

        all_chunks = []
        for file_data in tqdm(parsed_files, desc="Chunking files"):
            content = file_data["content"]
            metadata = file_data["metadata"]
            chunks = self.chunker.chunk_text(content, metadata)
            all_chunks.extend(chunks)

        self.stats["files_processed"] = len(parsed_files)
        self.stats["total_chunks"] = len(all_chunks)
        self.stats["total_tokens"] = sum(c["token_count"] for c in all_chunks)

        print(f"[OK] Generated {len(all_chunks)} chunks")
        return all_chunks

    async def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Generate embeddings for all chunks using Gemini API."""
        print("\n[Stage 3] Generating Embeddings (Gemini text-embedding-004 - 768 dimensions)")
        print("=" * 60)

        start_time = time.time()

        for chunk in tqdm(chunks, desc="Generating embeddings"):
            text = chunk["content"]
            embedding = await get_embedding(text)
            chunk["embedding"] = embedding
            # Rate limiting: 1 second between requests to avoid quota issues
            await asyncio.sleep(1)

        elapsed = time.time() - start_time
        print(f"[OK] Embeddings generated in {elapsed:.2f}s")
        print(f"    Throughput: {len(chunks)/elapsed:.1f} chunks/second")

        return chunks

    def upload_to_qdrant(self, chunks: List[Dict[str, Any]]) -> None:
        """Upload chunks with embeddings to Qdrant."""
        print("\n[Stage 4] Uploading to Qdrant")
        print("=" * 60)

        if self.dry_run:
            print("[DRY RUN] Skipping Qdrant upload")
            print(f"          Would upload {len(chunks)} vectors")
            return

        # Connect directly to Qdrant
        from qdrant_client import QdrantClient
        import os
        from dotenv import load_dotenv

        load_dotenv()
        client = QdrantClient(url=os.getenv("QDRANT_URL"), api_key=os.getenv("QDRANT_API_KEY"))
        collection_name = "textbook"

        # Prepare Qdrant points
        points = []
        for i, chunk in enumerate(chunks):
            point = PointStruct(
                id=i,
                vector=chunk["embedding"],
                payload={
                    "chunk_id": chunk["chunk_id"],
                    "content": chunk["content"],
                    "token_count": chunk["token_count"],
                    "chapter": chunk["metadata"].get("chapter", "Unknown"),
                    "chapter_title": chunk["metadata"].get("chapter_title", "Unknown"),
                    "section": chunk["metadata"].get("section", ""),
                    "url": chunk["metadata"].get("url", ""),
                    "source_file": chunk["metadata"].get("source_file", "")
                }
            )
            points.append(point)

        # Upload in batches
        batch_size = 100
        print(f"[INFO] Uploading {len(points)} vectors in batches of {batch_size}...")

        for i in tqdm(range(0, len(points), batch_size), desc="Uploading batches"):
            batch = points[i:i + batch_size]
            client.upsert(collection_name=collection_name, points=batch)

        print("[OK] Upload complete!")

        # Verify count
        count = client.count(collection_name)
        print(f"[INFO] Total vectors in collection: {count.count}")

    def print_summary(self) -> None:
        """Print final ingestion summary."""
        print("\n" + "=" * 60)
        print("[COMPLETE] INGESTION PIPELINE")
        print("=" * 60)
        print(f"Files processed: {self.stats['files_processed']}")
        print(f"Total chunks: {self.stats['total_chunks']}")
        print(f"Total tokens: {self.stats['total_tokens']:,}")

        if self.dry_run:
            print("\n[DRY RUN] No data was uploaded to Qdrant")
        else:
            print("\n[OK] All data successfully uploaded to Qdrant Cloud")

    async def run(self) -> None:
        """Execute the complete ingestion pipeline."""
        try:
            # Stage 1: Parse files
            parsed_files = self.parse_files()
            if not parsed_files:
                print("[ERROR] No files found to process")
                return

            # Stage 2: Chunk content
            chunks = self.chunk_files(parsed_files)
            if not chunks:
                print("[ERROR] No chunks generated")
                return

            # Stage 3: Generate embeddings
            chunks_with_embeddings = await self.generate_embeddings(chunks)

            # Stage 4: Upload to Qdrant
            self.upload_to_qdrant(chunks_with_embeddings)

            # Print summary
            self.print_summary()

        except KeyboardInterrupt:
            print("\n[WARN] Ingestion interrupted by user")
            sys.exit(1)
        except Exception as e:
            print(f"\n[ERROR] Ingestion failed: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)


async def main():
    """Main entry point for ingestion script."""
    parser = argparse.ArgumentParser(description="Ingest markdown files into Qdrant")
    parser.add_argument("--dry-run", action="store_true", help="Test mode (no upload)")
    parser.add_argument("--limit", type=int, help="Limit number of files to process")
    parser.add_argument("--docs-dir", type=str, default="../docs", help="Path to docs directory")

    args = parser.parse_args()

    # Print configuration
    print("\n" + "=" * 60)
    print("[START] CONTENT INGESTION PIPELINE")
    print("=" * 60)
    print(f"Docs directory: {args.docs_dir}")
    print(f"Dry run: {args.dry_run}")
    if args.limit:
        print(f"File limit: {args.limit}")
    print("=" * 60)

    # Run pipeline
    pipeline = IngestionPipeline(
        docs_directory=args.docs_dir,
        dry_run=args.dry_run,
        limit=args.limit
    )
    await pipeline.run()


if __name__ == "__main__":
    asyncio.run(main())
