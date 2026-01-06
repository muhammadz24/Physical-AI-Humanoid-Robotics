"""
Physical AI & Humanoid Robotics - Infrastructure Health Test Suite
==================================================================

SP Framework Compliance: Test-Driven Repair (Constitution Rule 2)

This master test file performs logic-gated verification of all critical infrastructure:
1. Environment variables validation
2. Qdrant vector store connectivity & data density
3. Neon Postgres database connectivity
4. Google Gemini API (LLM + Embeddings)
5. JWT auth system integrity

Run this BEFORE any fixes to establish baseline health score.
Run AFTER fixes to verify 100% connectivity.

Usage:
    python -m pytest tests/test_infra.py -v
    python tests/test_infra.py  # Direct execution for health report
"""

import os
import sys
import asyncio
from typing import Dict, Any
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

import pytest
import httpx
from qdrant_client import QdrantClient
from sqlalchemy import create_engine, text

# Import project modules
from backend.app.core.config import settings
from backend.app.core.vector_store import vector_store
from backend.app.services.llm import llm_service
from backend.app.services.embedding import embedding_service


class InfrastructureHealthCheck:
    """Master health check orchestrator"""

    def __init__(self):
        self.results: Dict[str, Dict[str, Any]] = {}
        self.total_tests = 0
        self.passed_tests = 0

    def record_result(self, category: str, test_name: str, status: bool, details: str = ""):
        """Record test result"""
        if category not in self.results:
            self.results[category] = {}

        self.results[category][test_name] = {
            "status": "[PASS]" if status else "[FAIL]",
            "details": details
        }

        self.total_tests += 1
        if status:
            self.passed_tests += 1

    def get_health_score(self) -> int:
        """Calculate health score (0-100)"""
        if self.total_tests == 0:
            return 0
        return int((self.passed_tests / self.total_tests) * 100)

    def generate_scorecard(self) -> str:
        """Generate comprehensive health scorecard"""
        score = self.get_health_score()

        report = "\n" + "="*80 + "\n"
        report += "*** PHYSICAL AI & HUMANOID ROBOTICS - SYSTEM HEALTH SCORECARD ***\n"
        report += "="*80 + "\n\n"

        # Overall score
        report += f"OVERALL HEALTH SCORE: {score}/100"
        if score == 100:
            report += " [PERFECT]\n"
        elif score >= 80:
            report += " [GOOD]\n"
        elif score >= 60:
            report += " [NEEDS ATTENTION]\n"
        else:
            report += " [CRITICAL]\n"

        report += f"   Tests Passed: {self.passed_tests}/{self.total_tests}\n\n"

        # Category-by-category breakdown
        for category, tests in self.results.items():
            report += f"\n{'-'*80}\n"
            report += f">> {category.upper()}\n"
            report += f"{'-'*80}\n"

            for test_name, result in tests.items():
                report += f"  {result['status']} {test_name}\n"
                if result['details']:
                    report += f"      -> {result['details']}\n"

        report += "\n" + "="*80 + "\n"
        return report


# Global health checker
health = InfrastructureHealthCheck()


# ============================================================================
# CATEGORY 1: ENVIRONMENT VARIABLES
# ============================================================================

def test_env_gemini_api_key():
    """Verify Gemini API key is set"""
    key = settings.gemini_api_key
    status = bool(key and len(key) > 20)
    health.record_result(
        "Environment Variables",
        "Gemini API Key",
        status,
        f"Length: {len(key) if key else 0} chars" if status else "MISSING or INVALID"
    )
    assert status, "GEMINI_API_KEY not set or too short"


def test_env_qdrant_credentials():
    """Verify Qdrant credentials are set"""
    url = settings.qdrant_url
    key = settings.qdrant_api_key

    url_valid = bool(url and url.startswith("https://"))
    key_valid = bool(key and len(key) > 10)
    status = url_valid and key_valid

    health.record_result(
        "Environment Variables",
        "Qdrant Credentials",
        status,
        f"URL: {url[:30]}... | Key: {'SET' if key_valid else 'MISSING'}" if status else "INVALID"
    )
    assert status, "Qdrant credentials incomplete"


def test_env_database_url():
    """Verify Neon database URL is set"""
    db_url = settings.database_url
    status = bool(db_url and "postgresql://" in db_url and "neon.tech" in db_url)

    # Mask password in output
    display_url = db_url.split("@")[1] if "@" in db_url else "INVALID"

    health.record_result(
        "Environment Variables",
        "Neon Database URL",
        status,
        f"Host: {display_url}" if status else "MISSING or INVALID FORMAT"
    )
    assert status, "DATABASE_URL not set or invalid format"


def test_env_jwt_secret():
    """Verify JWT secret is set"""
    secret = settings.jwt_secret_key
    status = bool(secret and len(secret) >= 32)

    health.record_result(
        "Environment Variables",
        "JWT Secret Key",
        status,
        f"Length: {len(secret)} chars" if status else "TOO SHORT (min 32)"
    )
    assert status, "JWT_SECRET_KEY too short or missing"


# ============================================================================
# CATEGORY 2: QDRANT VECTOR STORE
# ============================================================================

def test_qdrant_connection():
    """Test Qdrant cloud connectivity"""
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )

        # Try to list collections
        collections = client.get_collections()
        status = True
        details = f"Connected | Collections: {len(collections.collections)}"

    except Exception as e:
        status = False
        details = f"Connection failed: {str(e)[:50]}"

    health.record_result("Qdrant Vector Store", "Cloud Connectivity", status, details)
    assert status, f"Qdrant connection failed: {details}"


def test_qdrant_collection_exists():
    """Verify 'textbook' collection exists"""
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )

        collections = client.get_collections().collections
        collection_names = [col.name for col in collections]

        status = settings.qdrant_collection in collection_names
        details = f"Collection '{settings.qdrant_collection}' found" if status else f"NOT FOUND. Available: {collection_names}"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Qdrant Vector Store", "Collection Exists", status, details)
    assert status, details


def test_qdrant_vector_density():
    """Check vector data density (points count)"""
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )

        collection_info = client.get_collection(settings.qdrant_collection)
        points_count = collection_info.points_count
        vectors_count = collection_info.vectors_count

        # Success if we have at least 10 vectors (minimal viable data)
        status = points_count >= 10
        details = f"Points: {points_count} | Vectors: {vectors_count}"

        if not status:
            details += " [WARNING] LOW DENSITY - Run ingestion script!"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Qdrant Vector Store", "Vector Density", status, details)
    # Don't assert - low density is a warning, not a failure
    if not status:
        print(f"[WARNING] {details}")


def test_qdrant_embedding_dimensions():
    """Verify collection is configured for 768 dimensions (Gemini embedding)"""
    try:
        client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            timeout=10.0
        )

        collection_info = client.get_collection(settings.qdrant_collection)

        # Access vector config
        if hasattr(collection_info.config.params.vectors, 'size'):
            actual_dim = collection_info.config.params.vectors.size
        else:
            # Handle named vectors case
            actual_dim = None

        expected_dim = settings.EMBEDDING_DIMENSION  # 768
        status = actual_dim == expected_dim

        details = f"Expected: {expected_dim}D | Actual: {actual_dim}D"
        if not status:
            details += " [ERROR] MISMATCH - Must recreate collection!"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Qdrant Vector Store", "Embedding Dimensions", status, details)
    assert status, f"Dimension mismatch: {details}"


# ============================================================================
# CATEGORY 3: NEON POSTGRES DATABASE
# ============================================================================

def test_database_connection():
    """Test Neon Postgres connectivity"""
    try:
        engine = create_engine(settings.database_url, pool_pre_ping=True)

        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            row = result.fetchone()

            status = row[0] == 1
            details = "Connected successfully"

    except Exception as e:
        status = False
        details = f"Connection failed: {str(e)[:50]}"

    health.record_result("Neon Postgres Database", "Connectivity", status, details)
    assert status, f"Database connection failed: {details}"


def test_database_users_table():
    """Verify 'users' table exists"""
    try:
        engine = create_engine(settings.database_url, pool_pre_ping=True)

        with engine.connect() as conn:
            result = conn.execute(text(
                "SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name='users')"
            ))
            exists = result.fetchone()[0]

            status = bool(exists)
            details = "Table 'users' found" if status else "Table NOT FOUND - Run migrations!"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Neon Postgres Database", "Users Table", status, details)
    assert status, details


def test_database_chatlogs_table():
    """Verify 'chatlogs' table exists"""
    try:
        engine = create_engine(settings.database_url, pool_pre_ping=True)

        with engine.connect() as conn:
            result = conn.execute(text(
                "SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name='chatlogs')"
            ))
            exists = result.fetchone()[0]

            status = bool(exists)
            details = "Table 'chatlogs' found" if status else "Table NOT FOUND - Run migrations!"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Neon Postgres Database", "ChatLogs Table", status, details)
    assert status, details


# ============================================================================
# CATEGORY 4: GOOGLE GEMINI API
# ============================================================================

@pytest.mark.asyncio
async def test_gemini_model_list():
    """Test Gemini API accessibility and model listing"""
    try:
        url = f"{llm_service.base_url}/models?key={llm_service.api_key}"

        async with httpx.AsyncClient() as client:
            resp = await client.get(url, timeout=10.0)

            status = resp.status_code == 200

            if status:
                data = resp.json()
                models = [m.get('name', '').replace('models/', '') for m in data.get('models', [])]
                gemini_models = [m for m in models if 'gemini' in m.lower()]
                details = f"Found {len(gemini_models)} Gemini models"
            else:
                details = f"HTTP {resp.status_code}: {resp.text[:50]}"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Google Gemini API", "Model List Access", status, details)
    assert status, f"Gemini API model list failed: {details}"


@pytest.mark.asyncio
async def test_gemini_llm_generation():
    """Test LLM content generation"""
    try:
        response = await llm_service.get_response(
            prompt="Say 'OK' if you're working.",
            system_prompt="Reply with exactly 'OK'"
        )

        # Check if response is valid (not an error message)
        status = (
            "OK" in response.upper() or
            len(response) < 100  # Simple non-error response
        ) and "ERROR" not in response.upper()

        details = f"Response: {response[:50]}" if status else f"FAILED: {response[:100]}"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Google Gemini API", "LLM Generation", status, details)
    assert status, f"LLM generation failed: {details}"


@pytest.mark.asyncio
async def test_gemini_embedding_service():
    """Test embedding generation (768-dim vectors)"""
    try:
        test_text = "Physical AI and humanoid robotics"
        embedding = await embedding_service.get_embedding(test_text)

        # Verify embedding is a list of floats with correct dimension
        status = (
            isinstance(embedding, list) and
            len(embedding) == settings.EMBEDDING_DIMENSION and
            all(isinstance(x, (int, float)) for x in embedding)
        )

        details = f"Generated {len(embedding)}-dim vector" if status else f"INVALID: type={type(embedding)}, len={len(embedding) if isinstance(embedding, list) else 'N/A'}"

    except Exception as e:
        status = False
        details = f"Error: {str(e)[:50]}"

    health.record_result("Google Gemini API", "Embedding Generation", status, details)
    assert status, f"Embedding generation failed: {details}"


# ============================================================================
# CATEGORY 5: AUTHENTICATION SYSTEM
# ============================================================================

def test_jwt_config():
    """Verify JWT configuration"""
    status = (
        settings.jwt_secret_key and
        len(settings.jwt_secret_key) >= 32 and
        settings.ALGORITHM == "HS256" and
        settings.ACCESS_TOKEN_EXPIRE_MINUTES > 0
    )

    details = f"Algorithm: {settings.ALGORITHM} | Expiry: {settings.ACCESS_TOKEN_EXPIRE_MINUTES}min" if status else "INVALID CONFIG"

    health.record_result("Authentication System", "JWT Configuration", status, details)
    assert status, "JWT configuration invalid"


# ============================================================================
# MAIN EXECUTION
# ============================================================================

def run_health_check():
    """Run all tests and generate health scorecard"""
    print("\n*** Starting Infrastructure Health Check...\n")

    # Run pytest programmatically
    pytest.main([__file__, "-v", "--tb=short"])

    # Generate and print scorecard
    scorecard = health.generate_scorecard()
    print(scorecard)

    # Save scorecard to file
    report_path = project_root / "tests" / "health_report.txt"
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(scorecard)

    print(f"\n*** Full report saved to: {report_path}\n")

    # Return exit code based on score
    score = health.get_health_score()
    return 0 if score == 100 else 1


if __name__ == "__main__":
    exit_code = run_health_check()
    sys.exit(exit_code)
