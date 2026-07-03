import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
TARGETS = [ROOT / "README.md", ROOT / "docs", ROOT / "src/waver_patrol/docs"]
FORBIDDEN_PATTERNS = (
    r"\b10-run\b",
    r"\b10runs\b",
    r"\b10/10\b",
    r"precision mean:\s*1\.000",
    r"recall mean:\s*1\.000",
    r"\bn\s*=\s*5\b",
    r"\b5x\b",
    r"5회",
)


def iter_markdown():
    for target in TARGETS:
        if target.is_file():
            yield target
        elif target.exists():
            yield from target.rglob("*.md")


def test_paper_facing_docs_do_not_claim_repeated_or_perfect_performance():
    offenders = []
    for path in iter_markdown():
        text = path.read_text(errors="replace")
        for pattern in FORBIDDEN_PATTERNS:
            if re.search(pattern, text, flags=re.IGNORECASE):
                offenders.append(f"{path.relative_to(ROOT)} matches {pattern}")
    assert not offenders, "\n".join(offenders)
