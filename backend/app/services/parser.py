"""
Markdown File Parser and Metadata Extractor

Recursively finds markdown files, parses frontmatter, and extracts metadata.
Cleans HTML/JSX tags to prepare text for embedding.
"""

import os
import re
from typing import List, Dict, Any, Optional
from pathlib import Path
import yaml


class MarkdownParser:
    """
    Parser for Docusaurus markdown files with frontmatter extraction.

    Features:
    - Recursively finds all .md files in docs directory
    - Extracts YAML frontmatter metadata
    - Cleans HTML/JSX tags from content
    - Generates URLs from file paths
    - Extracts chapter numbers from directory structure
    """

    def __init__(self, docs_directory: str):
        """
        Initialize parser with docs directory path.

        Args:
            docs_directory: Path to docs folder (e.g., "../docs")
        """
        self.docs_directory = Path(docs_directory).resolve()

        if not self.docs_directory.exists():
            raise FileNotFoundError(
                f"Docs directory not found: {self.docs_directory}"
            )

    def find_markdown_files(self, pattern: str = "chapter-*/*.md") -> List[Path]:
        """
        Find all markdown files matching the pattern.

        Args:
            pattern: Glob pattern for matching files (default: "chapter-*/*.md")

        Returns:
            List of Path objects for matched files
        """
        files = list(self.docs_directory.glob(pattern))
        files.sort()  # Sort for consistent processing order

        print(f"📁 Found {len(files)} markdown files matching '{pattern}'")
        return files

    def parse_frontmatter(self, content: str) -> tuple[Dict[str, Any], str]:
        """
        Extract YAML frontmatter from markdown content.

        Args:
            content: Full markdown file content

        Returns:
            Tuple of (frontmatter_dict, remaining_content)

        Frontmatter format:
        ---
        title: "Chapter Title"
        sidebar_position: 1
        ---
        Content here...
        """
        frontmatter = {}
        remaining_content = content

        # Match YAML frontmatter between --- delimiters
        frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n'
        match = re.match(frontmatter_pattern, content, re.DOTALL)

        if match:
            yaml_content = match.group(1)
            try:
                frontmatter = yaml.safe_load(yaml_content) or {}
            except yaml.YAMLError as e:
                print(f"⚠️  Failed to parse YAML frontmatter: {e}")
                frontmatter = {}

            # Remove frontmatter from content
            remaining_content = content[match.end():]

        return frontmatter, remaining_content

    def clean_content(self, content: str) -> str:
        """
        Clean markdown content by removing HTML/JSX tags and excessive whitespace.

        Args:
            content: Raw markdown content

        Returns:
            Cleaned text suitable for embedding
        """
        # Remove JSX/React components (e.g., <Link to="...">text</Link>)
        content = re.sub(r'<Link[^>]*>(.*?)</Link>', r'\1', content)

        # Remove HTML tags (keep the text content)
        content = re.sub(r'<[^>]+>', '', content)

        # Remove code block markers but keep code content
        content = re.sub(r'```[\w]*\n', '\n', content)
        content = re.sub(r'```', '', content)

        # Remove excessive whitespace
        content = re.sub(r'\n{3,}', '\n\n', content)
        content = re.sub(r' {2,}', ' ', content)

        # Remove leading/trailing whitespace
        content = content.strip()

        return content

    def extract_metadata(
        self,
        file_path: Path,
        frontmatter: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Extract metadata from file path and frontmatter.

        Args:
            file_path: Path to markdown file
            frontmatter: Parsed frontmatter dictionary

        Returns:
            Dictionary with complete metadata
        """
        # Extract chapter number from path (e.g., "docs/chapter-03/...")
        chapter_match = re.search(r'chapter-(\d+)', str(file_path))
        chapter = chapter_match.group(1) if chapter_match else "unknown"

        # Generate URL from file path
        # docs/chapter-03/nodes-topics.md -> /chapter-03/nodes-topics
        relative_path = file_path.relative_to(self.docs_directory)
        url = "/" + str(relative_path).replace('\\', '/').replace('.md', '')

        # Extract section from filename (e.g., "nodes-topics.md" -> "Nodes Topics")
        section = file_path.stem.replace('-', ' ').title()

        metadata = {
            "chapter": chapter,
            "chapter_title": frontmatter.get("title", f"Chapter {chapter}"),
            "section": section,
            "sidebar_position": frontmatter.get("sidebar_position", 0),
            "source_file": str(relative_path),
            "url": url,
            "tags": frontmatter.get("tags", [])
        }

        return metadata

    def parse_file(self, file_path: Path) -> Optional[Dict[str, Any]]:
        """
        Parse a single markdown file and extract all data.

        Args:
            file_path: Path to markdown file

        Returns:
            Dictionary with content and metadata, or None if parsing fails
        """
        try:
            # Read file content
            with open(file_path, 'r', encoding='utf-8') as f:
                raw_content = f.read()

            # Parse frontmatter
            frontmatter, content = self.parse_frontmatter(raw_content)

            # Clean content
            cleaned_content = self.clean_content(content)

            # Extract metadata
            metadata = self.extract_metadata(file_path, frontmatter)

            return {
                "content": cleaned_content,
                "metadata": metadata,
                "file_path": str(file_path)
            }

        except Exception as e:
            print(f"❌ Failed to parse {file_path}: {e}")
            return None

    def parse_all(self, pattern: str = "chapter-*/*.md") -> List[Dict[str, Any]]:
        """
        Parse all markdown files matching the pattern.

        Args:
            pattern: Glob pattern for matching files

        Returns:
            List of parsed file dictionaries
        """
        files = self.find_markdown_files(pattern)
        parsed_files = []

        for file_path in files:
            parsed = self.parse_file(file_path)
            if parsed:
                parsed_files.append(parsed)

        print(f"✅ Successfully parsed {len(parsed_files)}/{len(files)} files")
        return parsed_files
