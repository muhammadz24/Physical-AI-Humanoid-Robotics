#!/usr/bin/env python3
"""
SP SKILL: VerifyVectorDensity
=============================

PURPOSE: Logic-gated audit of Qdrant vector database density and integrity.

COMPLIANCE: SP Constitution Rule 1 (Zero Vibe Coding - Logic-Gated Verification)

This skill performs deep inspection of the Qdrant vector store to ensure:
1. Collection exists and is accessible
2. Sufficient vector density (minimum viable data)
3. Correct embedding dimensions (768 for Gemini)
4. Data distribution across chapters
5. Vector quality metrics (score distribution)

USAGE:
    python .claude/skills/verify_vector_density.py

    Or import:
    from .claude.skills.verify_vector_density import VectorDensityAudit
    audit = VectorDensityAudit()
    report = audit.run()
"""

import os
import sys
from pathlib import Path
from typing import Dict, Any, List

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from qdrant_client import QdrantClient
from backend.app.core.config import settings


class VectorDensityAudit:
    """Deep audit of Qdrant vector database"""

    def __init__(self):
        self.client = None
        self.collection_name = settings.qdrant_collection
        self.results: Dict[str, Any] = {}

    def connect(self) -> bool:
        """Establish connection to Qdrant"""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=15.0
            )
            return True
        except Exception as e:
            self.results["connection_error"] = str(e)
            return False

    def check_collection_exists(self) -> bool:
        """Verify collection exists"""
        try:
            collections = self.client.get_collections().collections
            exists = any(col.name == self.collection_name for col in collections)
            self.results["collection_exists"] = exists
            return exists
        except Exception as e:
            self.results["collection_check_error"] = str(e)
            return False

    def get_density_metrics(self) -> Dict[str, Any]:
        """Get vector density statistics"""
        try:
            info = self.client.get_collection(self.collection_name)

            metrics = {
                "points_count": info.points_count,
                "vectors_count": info.vectors_count,
                "indexed_vectors_count": getattr(info, 'indexed_vectors_count', 0),
                "status": info.status,
            }

            # Quality assessment
            metrics["density_grade"] = self._grade_density(metrics["points_count"])
            metrics["meets_minimum"] = metrics["points_count"] >= 10

            self.results["density_metrics"] = metrics
            return metrics

        except Exception as e:
            self.results["density_error"] = str(e)
            return {}

    def check_dimensions(self) -> Dict[str, Any]:
        """Verify embedding dimensions match configuration"""
        try:
            info = self.client.get_collection(self.collection_name)

            # Extract vector dimension
            if hasattr(info.config.params.vectors, 'size'):
                actual_dim = info.config.params.vectors.size
            else:
                actual_dim = None

            expected_dim = settings.EMBEDDING_DIMENSION

            check = {
                "expected_dimension": expected_dim,
                "actual_dimension": actual_dim,
                "match": actual_dim == expected_dim,
                "distance_metric": str(info.config.params.vectors.distance) if hasattr(info.config.params.vectors, 'distance') else "unknown"
            }

            self.results["dimension_check"] = check
            return check

        except Exception as e:
            self.results["dimension_error"] = str(e)
            return {}

    def sample_vector_quality(self, sample_size: int = 10) -> Dict[str, Any]:
        """Sample random vectors to check quality"""
        try:
            # Get collection info first
            info = self.client.get_collection(self.collection_name)
            total_points = info.points_count

            if total_points == 0:
                return {"error": "No vectors in collection"}

            # Scroll to get sample points
            sample_points, _ = self.client.scroll(
                collection_name=self.collection_name,
                limit=min(sample_size, total_points),
                with_payload=True,
                with_vectors=False  # Don't fetch vectors, just metadata
            )

            # Analyze sample
            chapters = set()
            chunk_types = set()

            for point in sample_points:
                if point.payload:
                    if "chapter" in point.payload:
                        chapters.add(point.payload["chapter"])
                    if "chunk_type" in point.payload:
                        chunk_types.add(point.payload["chunk_type"])

            quality = {
                "sample_size": len(sample_points),
                "unique_chapters": len(chapters),
                "chapter_list": sorted(list(chapters)),
                "chunk_types": list(chunk_types),
                "has_metadata": len(sample_points) > 0 and sample_points[0].payload is not None
            }

            self.results["quality_sample"] = quality
            return quality

        except Exception as e:
            self.results["quality_error"] = str(e)
            return {"error": str(e)}

    def _grade_density(self, points_count: int) -> str:
        """Grade vector density"""
        if points_count >= 500:
            return "A (Excellent)"
        elif points_count >= 100:
            return "B (Good)"
        elif points_count >= 50:
            return "C (Adequate)"
        elif points_count >= 10:
            return "D (Minimal)"
        else:
            return "F (Insufficient)"

    def run(self) -> Dict[str, Any]:
        """Execute full audit"""
        print("\n" + "="*80)
        print("SP SKILL: VerifyVectorDensity - Qdrant Database Audit")
        print("="*80 + "\n")

        # Step 1: Connect
        print("[1/5] Connecting to Qdrant Cloud...")
        if not self.connect():
            print("   [FAIL] Connection failed")
            return self._generate_report()

        print("   [PASS] Connected successfully")

        # Step 2: Check collection
        print("[2/5] Verifying collection exists...")
        if not self.check_collection_exists():
            print(f"   [FAIL] Collection '{self.collection_name}' not found")
            return self._generate_report()

        print(f"   [PASS] Collection '{self.collection_name}' exists")

        # Step 3: Density metrics
        print("[3/5] Analyzing vector density...")
        metrics = self.get_density_metrics()
        if metrics:
            print(f"   [INFO] Points: {metrics['points_count']} | Grade: {metrics['density_grade']}")

        # Step 4: Dimension check
        print("[4/5] Verifying embedding dimensions...")
        dim_check = self.check_dimensions()
        if dim_check and dim_check.get("match"):
            print(f"   [PASS] Dimensions match: {dim_check['actual_dimension']}D")
        elif dim_check:
            print(f"   [FAIL] Dimension mismatch: Expected {dim_check['expected_dimension']}D, Got {dim_check['actual_dimension']}D")

        # Step 5: Quality sampling
        print("[5/5] Sampling vector quality...")
        quality = self.sample_vector_quality()
        if quality and "unique_chapters" in quality:
            print(f"   [INFO] Unique chapters: {quality['unique_chapters']} | Chapters: {quality['chapter_list']}")

        # Generate report
        return self._generate_report()

    def _generate_report(self) -> Dict[str, Any]:
        """Generate comprehensive report"""
        report = {
            "timestamp": str(Path(__file__).stat().st_mtime),
            "qdrant_url": settings.qdrant_url[:30] + "..." if settings.qdrant_url else "NOT_SET",
            "collection_name": self.collection_name,
            "results": self.results,
            "overall_status": self._determine_status()
        }

        print("\n" + "-"*80)
        print(f"OVERALL STATUS: {report['overall_status']}")
        print("-"*80 + "\n")

        return report

    def _determine_status(self) -> str:
        """Determine overall audit status"""
        if "connection_error" in self.results:
            return "[CRITICAL] Connection Failed"

        if not self.results.get("collection_exists", False):
            return "[CRITICAL] Collection Missing"

        metrics = self.results.get("density_metrics", {})
        if metrics.get("points_count", 0) < 10:
            return "[WARNING] Insufficient Data - Run Ingestion"

        dim_check = self.results.get("dimension_check", {})
        if not dim_check.get("match", False):
            return "[CRITICAL] Dimension Mismatch - Recreate Collection"

        if metrics.get("points_count", 0) >= 100:
            return "[HEALTHY] Production Ready"
        elif metrics.get("points_count", 0) >= 50:
            return "[HEALTHY] Adequate for Testing"
        else:
            return "[WARNING] Minimal Data - Consider More Ingestion"


def main():
    """CLI entry point"""
    auditor = VectorDensityAudit()
    report = auditor.run()

    # Save report
    report_path = project_root / ".claude" / "skills" / "vector_density_report.json"
    import json
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\nFull report saved to: {report_path}\n")


if __name__ == "__main__":
    main()
