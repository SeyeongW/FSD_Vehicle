from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Mapping


DEFAULT_CLASS_ALIASES = {
    "bird": "bird",
    "avian": "bird",
    "drone": "drone",
    "uav": "drone",
    "person": "person",
    "human": "person",
    "car": "vehicle",
    "truck": "vehicle",
    "vehicle": "vehicle",
    "robot": "robot",
    "airplane": "irrelevant",
    "kite": "unknown",
}

DEFAULT_NON_BIRD_CLASSES = {"person", "vehicle", "robot", "drone", "irrelevant", "none"}


@dataclass(frozen=True)
class ClassificationDecision:
    normalized_class: str
    confidence: float
    bird_confirmed: bool
    non_bird: bool
    unknown: bool
    reason: str


def parse_class_aliases(items: Iterable[str] | None) -> dict[str, str]:
    aliases = dict(DEFAULT_CLASS_ALIASES)
    for item in items or []:
        text = str(item).strip()
        if not text:
            continue
        if ":" in text:
            src, dst = text.split(":", 1)
        elif "=" in text:
            src, dst = text.split("=", 1)
        else:
            continue
        src = src.strip().lower()
        dst = dst.strip().lower()
        if src and dst:
            aliases[src] = dst
    return aliases


def normalize_class_name(name: str, aliases: Mapping[str, str] | None = None) -> str:
    raw = str(name or "").strip().lower()
    if not raw:
        return "unknown"
    table = aliases if aliases is not None else DEFAULT_CLASS_ALIASES
    return str(table.get(raw, raw)).strip().lower() or "unknown"


def classify_candidate(
    class_name: str,
    confidence: float,
    *,
    bird_threshold: float = 0.75,
    non_bird_threshold: float = 0.70,
    aliases: Mapping[str, str] | None = None,
    accepted_bird_classes: Iterable[str] = ("bird",),
    non_bird_classes: Iterable[str] = DEFAULT_NON_BIRD_CLASSES,
) -> ClassificationDecision:
    normalized = normalize_class_name(class_name, aliases)
    score = max(float(confidence), 0.0)
    accepted = {normalize_class_name(name, aliases) for name in accepted_bird_classes}
    rejected = {normalize_class_name(name, aliases) for name in non_bird_classes}
    if normalized in accepted and score >= bird_threshold:
        return ClassificationDecision(normalized, score, True, False, False, "BIRD_CONFIRMED")
    if normalized in rejected and score >= non_bird_threshold:
        return ClassificationDecision(normalized, score, False, True, False, "NON_BIRD")
    if normalized in accepted:
        return ClassificationDecision(normalized, score, False, False, True, "LOW_CONFIDENCE_BIRD")
    return ClassificationDecision(normalized, score, False, False, True, "UNKNOWN_OR_LOW_CONFIDENCE")


def detector_model_state(model_path: str, *, required: bool = True) -> str:
    text = str(model_path or "").strip()
    if not text:
        return "MODEL_MISSING" if required else "MODEL_OPTIONAL_MISSING"
    path = Path(text).expanduser()
    if not path.exists():
        return f"MODEL_NOT_FOUND path={text}"
    if not path.is_file():
        return f"MODEL_NOT_FILE path={text}"
    return "MODEL_READY"
