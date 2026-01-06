#!/usr/bin/env python3
"""
SP SKILL: GeminiModelAutoDetect
================================

PURPOSE: Logic-gated detection and selection of stable Gemini models.

COMPLIANCE: SP Constitution Rule 1 (Zero Vibe Coding - Logic-Gated Verification)

This skill intelligently detects available Gemini models and selects the most
stable option for content generation, avoiding 404 and rate limit errors.

FEATURES:
1. Auto-fetch available models from Google API
2. Filter for generation-capable models
3. Prioritize stable models (flash, pro)
4. Test model with simple prompt
5. Cache working model for future use

USAGE:
    python .claude/skills/gemini_model_autodetect.py

    Or import:
    from .claude.skills.gemini_model_autodetect import GeminiModelDetector
    detector = GeminiModelDetector()
    best_model = detector.find_best_model()
"""

import os
import sys
import asyncio
from pathlib import Path
from typing import Dict, List, Optional, Any
import httpx

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from backend.app.core.config import settings


class GeminiModelDetector:
    """Intelligent Gemini model detection and selection"""

    def __init__(self):
        self.api_key = settings.gemini_api_key
        self.base_url = "https://generativelanguage.googleapis.com/v1beta"
        self.available_models: List[Dict[str, Any]] = []
        self.tested_models: Dict[str, Dict[str, Any]] = {}

    async def fetch_available_models(self) -> List[Dict[str, Any]]:
        """Fetch list of available models from Google API"""
        url = f"{self.base_url}/models?key={self.api_key}"

        print("[1/4] Fetching available models from Google API...")

        try:
            async with httpx.AsyncClient() as client:
                resp = await client.get(url, timeout=10.0)

                if resp.status_code == 200:
                    data = resp.json()
                    models = data.get('models', [])

                    # Filter for Gemini models with generateContent support
                    gemini_models = [
                        m for m in models
                        if 'gemini' in m.get('name', '').lower() and
                        'generateContent' in m.get('supportedGenerationMethods', [])
                    ]

                    self.available_models = gemini_models
                    print(f"   [PASS] Found {len(gemini_models)} Gemini models")
                    return gemini_models

                else:
                    print(f"   [FAIL] HTTP {resp.status_code}: {resp.text[:100]}")
                    return []

        except Exception as e:
            print(f"   [FAIL] Error: {str(e)[:100]}")
            return []

    def prioritize_models(self) -> List[str]:
        """Prioritize models by stability (flash > pro > experimental)"""
        print("[2/4] Prioritizing models by stability...")

        if not self.available_models:
            print("   [WARN] No models available to prioritize")
            return []

        # Extract model names
        model_names = [m['name'].replace('models/', '') for m in self.available_models]

        # Priority tiers
        tier_1_flash = [m for m in model_names if 'flash' in m.lower() and 'exp' not in m.lower()]
        tier_2_pro = [m for m in model_names if 'pro' in m.lower() and 'exp' not in m.lower()]
        tier_3_exp = [m for m in model_names if 'exp' in m.lower()]
        tier_4_other = [m for m in model_names if m not in tier_1_flash + tier_2_pro + tier_3_exp]

        # Sort each tier (prefer newer versions)
        prioritized = (
            sorted(tier_1_flash, reverse=True) +
            sorted(tier_2_pro, reverse=True) +
            sorted(tier_3_exp, reverse=True) +
            sorted(tier_4_other, reverse=True)
        )

        print(f"   [INFO] Prioritized {len(prioritized)} models")
        print(f"   [INFO] Top 3: {prioritized[:3]}")

        return prioritized

    async def test_model(self, model_name: str) -> Dict[str, Any]:
        """Test a model with a simple generation request"""
        url = f"{self.base_url}/models/{model_name}:generateContent?key={self.api_key}"
        headers = {"Content-Type": "application/json"}
        payload = {
            "contents": [{"parts": [{"text": "Say 'OK' if you're working."}]}]
        }

        result = {
            "model": model_name,
            "status": "unknown",
            "latency_ms": 0,
            "error": None
        }

        try:
            async with httpx.AsyncClient() as client:
                import time
                start = time.time()

                resp = await client.post(url, headers=headers, json=payload, timeout=15.0)

                latency = int((time.time() - start) * 1000)
                result["latency_ms"] = latency

                if resp.status_code == 200:
                    data = resp.json()
                    response_text = data["candidates"][0]["content"]["parts"][0]["text"]

                    result["status"] = "working"
                    result["response_sample"] = response_text[:50]

                elif resp.status_code == 429:
                    result["status"] = "rate_limited"
                    result["error"] = "Quota exceeded"

                elif resp.status_code == 404:
                    result["status"] = "not_found"
                    result["error"] = "Model not found"

                else:
                    result["status"] = "error"
                    result["error"] = f"HTTP {resp.status_code}"

        except asyncio.TimeoutError:
            result["status"] = "timeout"
            result["error"] = "Request timeout"

        except Exception as e:
            result["status"] = "error"
            result["error"] = str(e)[:100]

        return result

    async def find_best_model(self, test_top_n: int = 3) -> Optional[str]:
        """Find the best working model by testing top candidates"""
        print("\n" + "="*80)
        print("SP SKILL: GeminiModelAutoDetect - Finding Best Model")
        print("="*80 + "\n")

        # Step 1: Fetch available models
        models = await self.fetch_available_models()
        if not models:
            print("\n[CRITICAL] No models available\n")
            return None

        # Step 2: Prioritize models
        prioritized = self.prioritize_models()
        if not prioritized:
            print("\n[CRITICAL] Failed to prioritize models\n")
            return None

        # Step 3: Test top models
        print(f"[3/4] Testing top {test_top_n} models...")

        for i, model_name in enumerate(prioritized[:test_top_n], 1):
            print(f"   [{i}/{test_top_n}] Testing {model_name}...", end=" ")

            test_result = await self.test_model(model_name)
            self.tested_models[model_name] = test_result

            if test_result["status"] == "working":
                print(f"[PASS] Latency: {test_result['latency_ms']}ms")
            elif test_result["status"] == "rate_limited":
                print(f"[WARN] Rate limited")
            else:
                print(f"[FAIL] {test_result['error']}")

        # Step 4: Select best model
        print("[4/4] Selecting best model...")

        working_models = [
            (name, result) for name, result in self.tested_models.items()
            if result["status"] == "working"
        ]

        if working_models:
            # Sort by latency (fastest first)
            working_models.sort(key=lambda x: x[1]["latency_ms"])
            best_model = working_models[0][0]

            print(f"   [PASS] Best model: {best_model}")
            print(f"   [INFO] Latency: {working_models[0][1]['latency_ms']}ms")

            self._save_result(best_model, working_models[0][1])
            return best_model

        else:
            print("   [FAIL] No working models found")

            # Check for rate limiting
            rate_limited = [
                name for name, result in self.tested_models.items()
                if result["status"] == "rate_limited"
            ]

            if rate_limited:
                print(f"   [INFO] Rate limited models: {rate_limited}")
                print("   [SUGGESTION] Try again later or check quota")

            return None

    def _save_result(self, best_model: str, result: Dict[str, Any]):
        """Save detection result to config file"""
        report_path = project_root / ".claude" / "skills" / "gemini_model_detection.json"

        import json
        data = {
            "best_model": best_model,
            "latency_ms": result["latency_ms"],
            "all_tested": self.tested_models,
            "timestamp": str(Path(__file__).stat().st_mtime)
        }

        with open(report_path, "w") as f:
            json.dump(data, f, indent=2)

        print(f"\n   Report saved to: {report_path}")

    async def get_fallback_model(self) -> str:
        """Get fallback model if detection fails"""
        fallbacks = [
            "gemini-1.5-flash-001",
            "gemini-1.5-flash",
            "gemini-pro",
            "gemini-1.0-pro"
        ]

        for fallback in fallbacks:
            result = await self.test_model(fallback)
            if result["status"] == "working":
                print(f"\n[FALLBACK] Using {fallback}")
                return fallback

        print("\n[CRITICAL] All fallbacks failed")
        return "gemini-pro"  # Last resort


async def main():
    """CLI entry point"""
    detector = GeminiModelDetector()
    best_model = await detector.find_best_model()

    print("\n" + "="*80)
    if best_model:
        print(f"RESULT: Best model is '{best_model}'")
        print("\nUpdate backend/app/core/config.py:")
        print(f'    GEMINI_MODEL: str = "{best_model}"')
    else:
        print("RESULT: No working model found")
        print("\nTroubleshooting:")
        print("1. Check GEMINI_API_KEY is valid")
        print("2. Verify you haven't exceeded quota")
        print("3. Try again later (rate limit reset)")

    print("="*80 + "\n")


if __name__ == "__main__":
    asyncio.run(main())
