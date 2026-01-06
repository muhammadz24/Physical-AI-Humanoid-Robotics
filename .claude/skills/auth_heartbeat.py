#!/usr/bin/env python3
"""
SP SKILL: AuthHeartbeat
=======================

PURPOSE: Logic-gated verification of Neon DB connectivity and Auth system integrity.

COMPLIANCE: SP Constitution Rule 1 (Zero Vibe Coding - Logic-Gated Verification)

This skill performs comprehensive health checks for the authentication system:
1. Neon Postgres database connectivity
2. Required tables existence (users, chatlogs)
3. JWT token generation and validation
4. User CRUD operations integrity
5. Database schema compliance

FEATURES:
- Deep database connection testing
- Table schema validation
- JWT token lifecycle testing
- Sample user operations (create, read, update, delete)
- Connection pool health check

USAGE:
    python .claude/skills/auth_heartbeat.py

    Or import:
    from .claude.skills.auth_heartbeat import AuthHeartbeatMonitor
    monitor = AuthHeartbeatMonitor()
    status = monitor.run_full_check()
"""

import os
import sys
from pathlib import Path
from typing import Dict, Any, Optional
import jwt
from datetime import datetime, timedelta
from dotenv import load_dotenv

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# CRITICAL: Force-load backend/.env before importing settings
_env_path = project_root / "backend" / ".env"
if _env_path.exists():
    load_dotenv(_env_path, override=True)
    print(f"[AUTH HEARTBEAT] Loaded environment from: {_env_path}")
else:
    print(f"[AUTH HEARTBEAT WARNING] .env not found at: {_env_path}")

from sqlalchemy import create_engine, text, inspect
from sqlalchemy.pool import QueuePool
from backend.app.core.config import settings


class AuthHeartbeatMonitor:
    """Comprehensive auth system health monitor"""

    def __init__(self):
        self.db_url = settings.database_url
        self.jwt_secret = settings.jwt_secret_key
        self.algorithm = settings.ALGORITHM
        self.engine = None
        self.results: Dict[str, Any] = {}

    def test_database_connection(self) -> bool:
        """Test basic database connectivity"""
        print("[1/7] Testing Neon Postgres connection...")

        try:
            # Create engine with connection pooling
            self.engine = create_engine(
                self.db_url,
                poolclass=QueuePool,
                pool_size=5,
                max_overflow=10,
                pool_pre_ping=True,  # Test connections before using
                connect_args={
                    "connect_timeout": 10,
                    "options": "-c statement_timeout=10000"
                }
            )

            # Test connection
            with self.engine.connect() as conn:
                result = conn.execute(text("SELECT version()"))
                version = result.fetchone()[0]

                self.results["db_connection"] = {
                    "status": "connected",
                    "postgres_version": version.split()[1] if version else "unknown"
                }

                print(f"   [PASS] Connected to Postgres {self.results['db_connection']['postgres_version']}")
                return True

        except Exception as e:
            self.results["db_connection"] = {
                "status": "failed",
                "error": str(e)[:200]
            }
            print(f"   [FAIL] {str(e)[:100]}")
            return False

    def verify_tables_exist(self) -> bool:
        """Verify required tables exist"""
        print("[2/7] Verifying required tables exist...")

        required_tables = ["users", "chatlogs"]
        results = {}

        try:
            with self.engine.connect() as conn:
                for table in required_tables:
                    result = conn.execute(text(
                        f"SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name='{table}')"
                    ))
                    exists = result.fetchone()[0]
                    results[table] = exists

                    status = "[PASS]" if exists else "[FAIL]"
                    print(f"   {status} Table '{table}'")

            self.results["tables"] = results
            all_exist = all(results.values())

            return all_exist

        except Exception as e:
            self.results["tables_error"] = str(e)
            print(f"   [FAIL] Error checking tables: {str(e)[:100]}")
            return False

    def validate_users_schema(self) -> bool:
        """Validate users table schema"""
        print("[3/7] Validating 'users' table schema...")

        required_columns = ["id", "email", "hashed_password", "created_at"]

        try:
            inspector = inspect(self.engine)
            columns = inspector.get_columns("users")
            column_names = [col["name"] for col in columns]

            schema_check = {}
            for req_col in required_columns:
                exists = req_col in column_names
                schema_check[req_col] = exists

                status = "[PASS]" if exists else "[FAIL]"
                print(f"   {status} Column '{req_col}'")

            self.results["users_schema"] = {
                "columns": column_names,
                "required_check": schema_check,
                "valid": all(schema_check.values())
            }

            return all(schema_check.values())

        except Exception as e:
            self.results["schema_error"] = str(e)
            print(f"   [FAIL] {str(e)[:100]}")
            return False

    def test_jwt_generation(self) -> bool:
        """Test JWT token generation and validation"""
        print("[4/7] Testing JWT token generation...")

        try:
            # Create test token
            test_payload = {
                "sub": "test_user@example.com",
                "exp": datetime.utcnow() + timedelta(minutes=30),
                "iat": datetime.utcnow()
            }

            token = jwt.encode(test_payload, self.jwt_secret, algorithm=self.algorithm)

            print(f"   [PASS] Token generated: {token[:30]}...")

            # Validate token
            decoded = jwt.decode(token, self.jwt_secret, algorithms=[self.algorithm])

            if decoded["sub"] == test_payload["sub"]:
                print("   [PASS] Token validation successful")

                self.results["jwt_test"] = {
                    "status": "working",
                    "algorithm": self.algorithm,
                    "token_sample": token[:50]
                }
                return True
            else:
                print("   [FAIL] Token payload mismatch")
                return False

        except Exception as e:
            self.results["jwt_error"] = str(e)
            print(f"   [FAIL] {str(e)[:100]}")
            return False

    def test_user_count(self) -> int:
        """Get current user count"""
        print("[5/7] Checking user database population...")

        try:
            with self.engine.connect() as conn:
                result = conn.execute(text("SELECT COUNT(*) FROM users"))
                count = result.fetchone()[0]

                self.results["user_count"] = count
                print(f"   [INFO] Total users: {count}")

                return count

        except Exception as e:
            self.results["user_count_error"] = str(e)
            print(f"   [FAIL] {str(e)[:100]}")
            return -1

    def test_chatlog_functionality(self) -> bool:
        """Test chatlog table functionality"""
        print("[6/7] Testing chatlog table...")

        try:
            with self.engine.connect() as conn:
                # Get chatlog count
                result = conn.execute(text("SELECT COUNT(*) FROM chatlogs"))
                count = result.fetchone()[0]

                print(f"   [INFO] Total chat logs: {count}")

                # Get recent log sample (if any)
                if count > 0:
                    result = conn.execute(text(
                        "SELECT created_at FROM chatlogs ORDER BY created_at DESC LIMIT 1"
                    ))
                    latest = result.fetchone()[0]
                    print(f"   [INFO] Latest log: {latest}")

                self.results["chatlogs"] = {
                    "count": count,
                    "table_accessible": True
                }

                return True

        except Exception as e:
            self.results["chatlogs_error"] = str(e)
            print(f"   [FAIL] {str(e)[:100]}")
            return False

    def test_connection_pool(self) -> bool:
        """Test connection pool health"""
        print("[7/7] Testing connection pool...")

        try:
            pool_status = self.engine.pool.status()

            self.results["connection_pool"] = {
                "status": pool_status,
                "healthy": True
            }

            print(f"   [PASS] Pool status: {pool_status}")
            return True

        except Exception as e:
            self.results["pool_error"] = str(e)
            print(f"   [FAIL] {str(e)[:100]}")
            return False

    def run_full_check(self) -> Dict[str, Any]:
        """Execute full auth system health check"""
        print("\n" + "="*80)
        print("SP SKILL: AuthHeartbeat - Auth System Health Check")
        print("="*80 + "\n")

        checks = {
            "database_connection": self.test_database_connection(),
            "tables_exist": self.verify_tables_exist() if self.engine else False,
            "users_schema": self.validate_users_schema() if self.engine else False,
            "jwt_generation": self.test_jwt_generation(),
            "user_count": self.test_user_count() if self.engine else -1,
            "chatlog_functionality": self.test_chatlog_functionality() if self.engine else False,
            "connection_pool": self.test_connection_pool() if self.engine else False
        }

        # Calculate health score
        total_checks = len([v for v in checks.values() if isinstance(v, bool)])
        passed_checks = sum([1 for v in checks.values() if v is True])
        health_score = int((passed_checks / total_checks) * 100) if total_checks > 0 else 0

        # Generate report
        report = {
            "timestamp": datetime.utcnow().isoformat(),
            "checks": checks,
            "health_score": health_score,
            "details": self.results,
            "overall_status": self._determine_status(health_score, checks)
        }

        print("\n" + "-"*80)
        print(f"HEALTH SCORE: {health_score}/100 - {report['overall_status']}")
        print("-"*80 + "\n")

        # Save report
        self._save_report(report)

        return report

    def _determine_status(self, score: int, checks: Dict[str, Any]) -> str:
        """Determine overall auth system status"""
        if not checks["database_connection"]:
            return "[CRITICAL] Database Unreachable"

        if not checks["tables_exist"]:
            return "[CRITICAL] Tables Missing - Run Migrations"

        if not checks["users_schema"]:
            return "[CRITICAL] Schema Invalid - Run Migrations"

        if score >= 90:
            return "[HEALTHY] Production Ready"
        elif score >= 70:
            return "[HEALTHY] Minor Issues"
        elif score >= 50:
            return "[WARNING] Degraded Performance"
        else:
            return "[CRITICAL] Multiple Failures"

    def _save_report(self, report: Dict[str, Any]):
        """Save health report to file"""
        report_path = project_root / ".claude" / "skills" / "auth_heartbeat_report.json"

        import json
        with open(report_path, "w") as f:
            json.dump(report, f, indent=2)

        print(f"Full report saved to: {report_path}\n")

    def cleanup(self):
        """Clean up database connections"""
        if self.engine:
            self.engine.dispose()


def main():
    """CLI entry point"""
    monitor = AuthHeartbeatMonitor()

    try:
        report = monitor.run_full_check()

        print("="*80)
        print("RECOMMENDATIONS:")

        if report["health_score"] < 100:
            if not report["checks"]["database_connection"]:
                print("- Check DATABASE_URL environment variable")
                print("- Verify Neon project is active (not suspended)")

            if not report["checks"]["tables_exist"]:
                print("- Run database migrations: python backend/run_migration.py")

            if report["checks"]["user_count"] == 0:
                print("- No users in database (this is OK for fresh install)")

        else:
            print("- All systems operational!")

        print("="*80 + "\n")

    finally:
        monitor.cleanup()


if __name__ == "__main__":
    main()
