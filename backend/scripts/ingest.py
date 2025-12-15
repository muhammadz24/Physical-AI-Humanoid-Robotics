#!/usr/bin/env python3
"""
Content Ingestion Script

Orchestrates the complete ingestion pipeline:
1. Read markdown files from docs directory
2. Parse frontmatter and extract metadata
3. Chunk content into semantic pieces
4. Generate embeddings using Sentence Transformers
5. Upload vectors to Qdrant Cloud

Usage:
    python scripts/ingest.py --dry-run          # Test without uploading
    python scripts/ingest.py                     # Full ingestion
    python scripts/ingest.py --chapter 3         # Ingest specific chapter
    python scripts/ingest.py --limit 5           # Limit to first 5 files
"""

import sys
import os
import argparse
import time
from pathlib import Path
from typing import List, Dict, Any

# Fix Windows console UTF-8 encoding for emojis
if sys.platform == 'win32':
    import codecs
    sys.stdout = codecs.getwriter('utf-8')(sys.stdout.buffer, 'strict')
    sys.stderr = codecs.getwriter('utf-8')(sys.stderr.buffer, 'strict')

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.core.config import settings
from app.core.vector_store import vector_store
from app.services.embedding import embedding_service
from app.services.parser import MarkdownParser
from app.services.chunker import TextChunker

from qdrant_client.models import PointStruct
from tqdm import tqdm


class IngestionPipeline:
    """
    Orchestrates the complete content ingestion pipeline.

    Pipeline stages:
    1. Parse markdown files
    2. Chunk content
    3. Generate embeddings
    4. Upload to Qdrant
    """

    def __init__(
        self,
        docs_directory: str,
        dry_run: bool = False,
        chapter_filter: str = None,
        limit: int = None
    ):
        """
        Initialize ingestion pipeline.

        Args:
            docs_directory: Path to docs folder
            dry_run: If True, skip Qdrant upload
            chapter_filter: Optional chapter number to filter (e.g., "3")
            limit: Optional limit on number of files to process
        """
        self.docs_directory = docs_directory
        self.dry_run = dry_run
        self.chapter_filter = chapter_filter
        self.limit = limit

        # Initialize services
        self.parser = MarkdownParser(docs_directory)
        self.chunker = TextChunker(max_tokens=500, overlap_tokens=50)

        # Statistics
        self.stats = {
            "files_processed": 0,
            "files_failed": 0,
            "total_chunks": 0,
            "total_tokens": 0,
            "chapters": {}
        }

    def parse_files(self) -> List[Dict[str, Any]]:
        """
        Parse all markdown files from docs directory.

        Returns:
            List of parsed file dictionaries
        """
        print("\n📚 Stage 1: Parsing Markdown Files")
        print("=" * 60)

        # Build glob pattern based on filter
        if self.chapter_filter:
            pattern = f"chapter-{self.chapter_filter.zfill(2)}/*.md"
            print(f"🔍 Filter: Chapter {self.chapter_filter} only")
        else:
            pattern = "chapter-*/*.md"
            print("🔍 Filter: All chapters")

        # Parse files
        parsed_files = self.parser.parse_all(pattern)

        # Apply limit if specified
        if self.limit:
            parsed_files = parsed_files[:self.limit]
            print(f"📊 Limited to first {self.limit} files")

        return parsed_files

    def chunk_files(self, parsed_files: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Chunk all parsed files into semantic pieces.

        Args:
            parsed_files: List of parsed file dictionaries

        Returns:
            List of chunk dictionaries
        """
        print("\n✂️  Stage 2: Chunking Content")
        print("=" * 60)

        all_chunks = []

        for file_data in tqdm(parsed_files, desc="Chunking files"):
            content = file_data["content"]
            metadata = file_data["metadata"]

            # Generate chunks
            chunks = self.chunker.chunk_text(content, metadata)
            all_chunks.extend(chunks)

            # Update statistics
            chapter = metadata["chapter"]
            if chapter not in self.stats["chapters"]:
                self.stats["chapters"][chapter] = {
                    "files": 0,
                    "chunks": 0,
                    "tokens": 0
                }

            self.stats["chapters"][chapter]["files"] += 1
            self.stats["chapters"][chapter]["chunks"] += len(chunks)
            self.stats["chapters"][chapter]["tokens"] += sum(
                c["token_count"] for c in chunks
            )

        # Calculate overall stats
        self.stats["files_processed"] = len(parsed_files)
        self.stats["total_chunks"] = len(all_chunks)
        self.stats["total_tokens"] = sum(c["token_count"] for c in all_chunks)

        # Print chunking stats
        chunk_stats = self.chunker.get_stats(all_chunks)
        print(f"\n📊 Chunking Statistics:")
        print(f"   Total chunks: {chunk_stats['total_chunks']}")
        print(f"   Total tokens: {chunk_stats['total_tokens']:,}")
        print(f"   Avg tokens/chunk: {chunk_stats['avg_tokens']}")
        print(f"   Min tokens: {chunk_stats['min_tokens']}")
        print(f"   Max tokens: {chunk_stats['max_tokens']}")

        return all_chunks

    def generate_embeddings(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Generate embeddings for all chunks.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            List of chunks with embeddings added
        """
        print("\n🧠 Stage 3: Generating Embeddings")
        print("=" * 60)

        # Load embedding model
        embedding_service.load_model()

        # Extract chunk texts
        chunk_texts = [chunk["content"] for chunk in chunks]

        # Generate embeddings in batches
        print(f"📥 Encoding {len(chunk_texts)} chunks...")
        start_time = time.time()

        embeddings = embedding_service.encode_batch(
            chunk_texts,
            batch_size=32,
            show_progress=True
        )

        elapsed = time.time() - start_time
        print(f"✅ Embeddings generated in {elapsed:.2f}s")
        print(f"   Throughput: {len(chunks)/elapsed:.1f} chunks/second")

        # Attach embeddings to chunks
        for i, chunk in enumerate(chunks):
            chunk["embedding"] = embeddings[i].tolist()

        return chunks

    def upload_to_qdrant(self, chunks: List[Dict[str, Any]]) -> None:
        """
        Upload chunks with embeddings to Qdrant.

        Args:
            chunks: List of chunk dictionaries with embeddings
        """
        print("\n☁️  Stage 4: Uploading to Qdrant")
        print("=" * 60)

        if self.dry_run:
            print("🔍 DRY RUN MODE - Skipping Qdrant upload")
            print(f"   Would upload {len(chunks)} vectors")
            return

        # Connect to Qdrant
        vector_store.connect()

        # Create collection if it doesn't exist
        try:
            vector_store.create_collection()
        except Exception as e:
            print(f"⚠️  Collection creation skipped: {e}")

        # Prepare Qdrant points
        points = []
        for i, chunk in enumerate(chunks):
            point = PointStruct(
                id=i,  # Use integer ID as required by Qdrant
                vector=chunk["embedding"],
                payload={
                    "chunk_id": chunk["chunk_id"],
                    "content": chunk["content"],
                    "token_count": chunk["token_count"],
                    "chapter": chunk["metadata"]["chapter"],
                    "chapter_title": chunk["metadata"]["chapter_title"],
                    "section": chunk["metadata"]["section"],
                    "url": chunk["metadata"]["url"],
                    "source_file": chunk["metadata"]["source_file"]
                }
            )
            points.append(point)

        # Upload in batches
        batch_size = 100
        total_batches = (len(points) + batch_size - 1) // batch_size

        print(f"📤 Uploading {len(points)} vectors in {total_batches} batches...")

        for i in tqdm(range(0, len(points), batch_size), desc="Uploading batches"):
            batch = points[i:i + batch_size]
            vector_store.upsert(batch)

        print("✅ Upload complete!")

        # Get collection info
        info = vector_store.get_collection_info()
        print(f"\n📊 Qdrant Collection Info:")
        print(f"   Collection: {info.get('name')}")
        print(f"   Total vectors: {info.get('vectors_count', 0)}")
        print(f"   Status: {info.get('status')}")

    def print_summary(self) -> None:
        """Print final ingestion summary."""
        print("\n" + "=" * 60)
        print("🎉 INGESTION PIPELINE COMPLETE")
        print("=" * 60)

        print(f"\n📈 Overall Statistics:")
        print(f"   Files processed: {self.stats['files_processed']}")
        print(f"   Files failed: {self.stats['files_failed']}")
        print(f"   Total chunks: {self.stats['total_chunks']}")
        print(f"   Total tokens: {self.stats['total_tokens']:,}")
        print(f"   Avg tokens/file: {self.stats['total_tokens']//max(self.stats['files_processed'],1):,}")

        print(f"\n📚 Per-Chapter Breakdown:")
        for chapter, data in sorted(self.stats["chapters"].items()):
            print(f"   Chapter {chapter}:")
            print(f"      Files: {data['files']}")
            print(f"      Chunks: {data['chunks']}")
            print(f"      Tokens: {data['tokens']:,}")

        if self.dry_run:
            print(f"\n🔍 DRY RUN MODE - No data was uploaded to Qdrant")
        else:
            print(f"\n✅ All data successfully uploaded to Qdrant Cloud")

    def run(self) -> None:
        """Execute the complete ingestion pipeline."""
        try:
            # Stage 1: Parse files
            parsed_files = self.parse_files()

            if not parsed_files:
                print("❌ No files found to process")
                return

            # Stage 2: Chunk content
            chunks = self.chunk_files(parsed_files)

            if not chunks:
                print("❌ No chunks generated")
                return

            # Stage 3: Generate embeddings
            chunks_with_embeddings = self.generate_embeddings(chunks)

            # Stage 4: Upload to Qdrant
            self.upload_to_qdrant(chunks_with_embeddings)

            # Print summary
            self.print_summary()

        except KeyboardInterrupt:
            print("\n\n⚠️  Ingestion interrupted by user")
            sys.exit(1)
        except Exception as e:
            print(f"\n❌ Ingestion failed: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)


def main():
    """Main entry point for ingestion script."""
    parser = argparse.ArgumentParser(
        description="Ingest Docusaurus markdown files into Qdrant vector database"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Run without uploading to Qdrant (test mode)"
    )
    parser.add_argument(
        "--chapter",
        type=str,
        help="Filter to specific chapter number (e.g., 3)"
    )
    parser.add_argument(
        "--limit",
        type=int,
        help="Limit number of files to process"
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="../docs",
        help="Path to docs directory (default: ../docs)"
    )

    args = parser.parse_args()

    # Print configuration
    print("\n" + "=" * 60)
    print("🚀 CONTENT INGESTION PIPELINE")
    print("=" * 60)
    print(f"📁 Docs directory: {args.docs_dir}")
    print(f"🔍 Dry run: {args.dry_run}")
    if args.chapter:
        print(f"📖 Chapter filter: {args.chapter}")
    if args.limit:
        print(f"📊 File limit: {args.limit}")
    print("=" * 60)

    # Run pipeline
    pipeline = IngestionPipeline(
        docs_directory=args.docs_dir,
        dry_run=args.dry_run,
        chapter_filter=args.chapter,
        limit=args.limit
    )
    pipeline.run()


if __name__ == "__main__":
    main()
