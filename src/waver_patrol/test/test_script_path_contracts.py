from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
FORBIDDEN = ("src/FSD_Vehicle/src/waver_patrol", "src/src/waver_patrol")
SCAN_ROOTS = ("README.md", "docs", "scripts", "src/waver_patrol")


def test_no_current_docs_or_scripts_use_obsolete_waver_patrol_paths() -> None:
    offenders: list[str] = []
    for rel in SCAN_ROOTS:
        base = ROOT / rel
        paths = [base] if base.is_file() else list(base.rglob("*"))
        for path in paths:
            if not path.is_file() or path.suffix in {".pyc", ".png", ".jpg", ".jpeg", ".pgm", ".db3", ".mcap"}:
                continue
            text = path.read_text(encoding="utf-8", errors="ignore")
            if any(token in text for token in FORBIDDEN) and "LEGACY_PATH_ALLOWED" not in text:
                offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []
