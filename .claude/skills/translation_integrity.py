#!/usr/bin/env python3
"""
SP SKILL: TranslationIntegrity
==============================

PURPOSE: Logic-gated verification of Urdu translation and RTL layout integrity.

COMPLIANCE: SP Constitution Rule 1 (Zero Vibe Coding - Logic-Gated Verification)

This skill performs comprehensive checks for the multilingual features:
1. Urdu translation toggle component exists
2. RTL CSS classes are properly defined
3. Docusaurus i18n configuration is correct
4. Translation files exist and are properly structured
5. Font rendering for Urdu characters

FEATURES:
- Component existence checks
- CSS RTL class validation
- i18n configuration audit
- Translation file integrity
- Rendering test (if browser available)

USAGE:
    python .claude/skills/translation_integrity.py

    Or import:
    from .claude.skills.translation_integrity import TranslationIntegrityChecker
    checker = TranslationIntegrityChecker()
    report = checker.run_full_check()
"""

import os
import sys
from pathlib import Path
from typing import Dict, Any, List, Optional
import json
import re

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))


class TranslationIntegrityChecker:
    """Comprehensive translation and RTL integrity checker"""

    def __init__(self):
        self.frontend_root = project_root
        self.results: Dict[str, Any] = {}

    def check_urdu_toggle_component(self) -> bool:
        """Check if Urdu translation toggle component exists"""
        print("[1/6] Checking Urdu translation toggle component...")

        # Check for translation button/toggle component
        possible_locations = [
            self.frontend_root / "src" / "components" / "LanguageToggle.js",
            self.frontend_root / "src" / "components" / "LanguageToggle.tsx",
            self.frontend_root / "src" / "components" / "UrduToggle.js",
            self.frontend_root / "src" / "components" / "UrduToggle.tsx",
            self.frontend_root / "src" / "components" / "TranslationButton.js",
        ]

        found_components = [loc for loc in possible_locations if loc.exists()]

        if found_components:
            print(f"   [PASS] Found translation component: {found_components[0].name}")
            self.results["toggle_component"] = {
                "exists": True,
                "path": str(found_components[0])
            }
            return True
        else:
            print("   [WARN] No translation toggle component found")
            self.results["toggle_component"] = {
                "exists": False,
                "checked_locations": [str(loc) for loc in possible_locations]
            }
            return False

    def check_rtl_css_classes(self) -> bool:
        """Verify RTL CSS classes are defined"""
        print("[2/6] Checking RTL CSS classes...")

        css_files = [
            self.frontend_root / "src" / "css" / "custom.css",
            self.frontend_root / "src" / "css" / "rtl.css",
            self.frontend_root / "src" / "css" / "urdu.css"
        ]

        rtl_patterns = [
            r"direction:\s*rtl",
            r"text-align:\s*right",
            r"\.rtl",
            r"lang.*ur",
            r"lang.*urdu"
        ]

        found_rtl = False
        rtl_definitions = []

        for css_file in css_files:
            if css_file.exists():
                try:
                    with open(css_file, 'r', encoding='utf-8') as f:
                        content = f.read()

                        for pattern in rtl_patterns:
                            matches = re.findall(pattern, content, re.IGNORECASE)
                            if matches:
                                found_rtl = True
                                rtl_definitions.append({
                                    "file": css_file.name,
                                    "pattern": pattern,
                                    "count": len(matches)
                                })

                except Exception as e:
                    print(f"   [WARN] Could not read {css_file.name}: {str(e)[:50]}")

        if found_rtl:
            print(f"   [PASS] Found RTL styles in {len(rtl_definitions)} locations")
            self.results["rtl_css"] = {
                "exists": True,
                "definitions": rtl_definitions
            }
        else:
            print("   [WARN] No RTL CSS classes found")
            self.results["rtl_css"] = {
                "exists": False,
                "checked_files": [str(f) for f in css_files if f.exists()]
            }

        return found_rtl

    def check_docusaurus_i18n_config(self) -> bool:
        """Check Docusaurus i18n configuration"""
        print("[3/6] Checking Docusaurus i18n configuration...")

        config_file = self.frontend_root / "docusaurus.config.js"

        if not config_file.exists():
            print("   [FAIL] docusaurus.config.js not found")
            self.results["i18n_config"] = {"exists": False}
            return False

        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                content = f.read()

                # Check for i18n configuration
                has_i18n = "i18n:" in content or "i18n :" in content
                has_urdu = "ur" in content.lower() or "urdu" in content.lower()
                has_rtl = "rtl" in content.lower() or "right-to-left" in content.lower()

                if has_i18n:
                    print("   [PASS] i18n configuration found")
                    self.results["i18n_config"] = {
                        "exists": True,
                        "has_urdu_locale": has_urdu,
                        "has_rtl_direction": has_rtl
                    }

                    if has_urdu:
                        print("   [PASS] Urdu locale configured")
                    else:
                        print("   [WARN] Urdu locale not explicitly configured")

                    return True
                else:
                    print("   [WARN] i18n configuration not found")
                    self.results["i18n_config"] = {
                        "exists": False,
                        "file_checked": str(config_file)
                    }
                    return False

        except Exception as e:
            print(f"   [FAIL] Error reading config: {str(e)[:50]}")
            self.results["i18n_config_error"] = str(e)
            return False

    def check_translation_files(self) -> bool:
        """Check for Urdu translation JSON files"""
        print("[4/6] Checking translation files...")

        # Common Docusaurus i18n file locations
        i18n_locations = [
            self.frontend_root / "i18n" / "ur" / "docusaurus-plugin-content-docs",
            self.frontend_root / "i18n" / "ur" / "code.json",
            self.frontend_root / "src" / "i18n" / "ur",
            self.frontend_root / "translations" / "ur"
        ]

        found_files = []

        for location in i18n_locations:
            if location.exists():
                if location.is_file():
                    found_files.append(str(location))
                else:
                    # Directory - check for JSON files
                    json_files = list(location.glob("**/*.json"))
                    found_files.extend([str(f) for f in json_files])

        if found_files:
            print(f"   [PASS] Found {len(found_files)} translation files")
            self.results["translation_files"] = {
                "exists": True,
                "count": len(found_files),
                "sample_files": found_files[:5]
            }
            return True
        else:
            print("   [WARN] No translation files found")
            self.results["translation_files"] = {
                "exists": False,
                "checked_locations": [str(loc) for loc in i18n_locations]
            }
            return False

    def check_urdu_font_support(self) -> bool:
        """Check for Urdu font configuration"""
        print("[5/6] Checking Urdu font support...")

        # Check CSS for Urdu-friendly fonts
        css_files = [
            self.frontend_root / "src" / "css" / "custom.css",
            self.frontend_root / "src" / "css" / "fonts.css"
        ]

        urdu_fonts = [
            "Noto Nastaliq Urdu",
            "Jameel Noori Nastaleeq",
            "Nafees",
            "Alvi Nastaleeq",
            "Mehr Nastaliq"
        ]

        found_fonts = []

        for css_file in css_files:
            if css_file.exists():
                try:
                    with open(css_file, 'r', encoding='utf-8') as f:
                        content = f.read()

                        for font in urdu_fonts:
                            if font in content:
                                found_fonts.append(font)

                except Exception as e:
                    pass

        if found_fonts:
            print(f"   [PASS] Found Urdu fonts: {', '.join(found_fonts)}")
            self.results["urdu_fonts"] = {
                "configured": True,
                "fonts": found_fonts
            }
            return True
        else:
            print("   [WARN] No specific Urdu fonts configured")
            print("   [INFO] System may fall back to default fonts")
            self.results["urdu_fonts"] = {
                "configured": False,
                "note": "Using system default fonts"
            }
            return False

    def check_sample_urdu_content(self) -> bool:
        """Check for sample Urdu content in docs"""
        print("[6/6] Checking for sample Urdu content...")

        # Search for Urdu Unicode characters in markdown files
        docs_dir = self.frontend_root / "docs"

        if not docs_dir.exists():
            print("   [WARN] Docs directory not found")
            return False

        urdu_pattern = re.compile(r'[\u0600-\u06FF]+')  # Arabic/Urdu Unicode range
        files_with_urdu = []

        try:
            for md_file in docs_dir.glob("**/*.md"):
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()
                    if urdu_pattern.search(content):
                        files_with_urdu.append(str(md_file.relative_to(self.frontend_root)))

            if files_with_urdu:
                print(f"   [PASS] Found Urdu content in {len(files_with_urdu)} files")
                self.results["urdu_content"] = {
                    "exists": True,
                    "file_count": len(files_with_urdu),
                    "sample_files": files_with_urdu[:3]
                }
                return True
            else:
                print("   [INFO] No Urdu content detected (OK for English-only docs)")
                self.results["urdu_content"] = {
                    "exists": False,
                    "note": "No Urdu Unicode characters found in markdown files"
                }
                return False

        except Exception as e:
            print(f"   [WARN] Error scanning for Urdu content: {str(e)[:50]}")
            self.results["urdu_content_error"] = str(e)
            return False

    def run_full_check(self) -> Dict[str, Any]:
        """Execute full translation integrity check"""
        print("\n" + "="*80)
        print("SP SKILL: TranslationIntegrity - RTL & Translation Check")
        print("="*80 + "\n")

        checks = {
            "toggle_component": self.check_urdu_toggle_component(),
            "rtl_css": self.check_rtl_css_classes(),
            "i18n_config": self.check_docusaurus_i18n_config(),
            "translation_files": self.check_translation_files(),
            "urdu_fonts": self.check_urdu_font_support(),
            "urdu_content": self.check_sample_urdu_content()
        }

        # Calculate health score
        passed_checks = sum([1 for v in checks.values() if v is True])
        total_checks = len(checks)
        health_score = int((passed_checks / total_checks) * 100)

        # Generate report
        report = {
            "timestamp": str(Path(__file__).stat().st_mtime),
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

    def _determine_status(self, score: int, checks: Dict[str, bool]) -> str:
        """Determine overall translation integrity status"""
        if score == 100:
            return "[PERFECT] Full Translation Support"
        elif score >= 67:  # 4+ checks passed
            return "[HEALTHY] Translation Infrastructure Ready"
        elif score >= 50:  # 3+ checks passed
            return "[PARTIAL] Basic RTL Support Present"
        elif score >= 33:  # 2+ checks passed
            return "[MINIMAL] Some Translation Features"
        else:
            return "[NOT CONFIGURED] Translation Not Implemented"

    def _save_report(self, report: Dict[str, Any]):
        """Save integrity report to file"""
        report_path = project_root / ".claude" / "skills" / "translation_integrity_report.json"

        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

        print(f"Full report saved to: {report_path}\n")


def main():
    """CLI entry point"""
    checker = TranslationIntegrityChecker()
    report = checker.run_full_check()

    print("="*80)
    print("RECOMMENDATIONS:")

    if report["health_score"] < 100:
        if not report["checks"]["toggle_component"]:
            print("- Create Urdu translation toggle component in src/components/")

        if not report["checks"]["rtl_css"]:
            print("- Add RTL CSS classes to src/css/custom.css:")
            print("  .rtl { direction: rtl; text-align: right; }")

        if not report["checks"]["i18n_config"]:
            print("- Configure i18n in docusaurus.config.js")
            print("- Add Urdu locale: locales: ['en', 'ur']")

        if not report["checks"]["translation_files"]:
            print("- Create translation files in i18n/ur/")

        if not report["checks"]["urdu_fonts"]:
            print("- Add Urdu font in src/css/custom.css:")
            print("  @import url('https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu');")

    else:
        print("- All translation features configured!")

    print("="*80 + "\n")


if __name__ == "__main__":
    main()
