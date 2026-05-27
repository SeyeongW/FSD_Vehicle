#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import yaml


def read_pgm(path: Path) -> tuple[int, int, list[int]]:
    data = path.read_bytes()
    idx = 0

    def token() -> bytes:
        nonlocal idx
        while idx < len(data) and data[idx] in b" \t\r\n":
            idx += 1
        if idx < len(data) and data[idx] == ord("#"):
            while idx < len(data) and data[idx] not in b"\r\n":
                idx += 1
            return token()
        start = idx
        while idx < len(data) and data[idx] not in b" \t\r\n":
            idx += 1
        return data[start:idx]

    magic = token()
    if magic not in {b"P5", b"P2"}:
        raise ValueError(f"unsupported PGM magic {magic!r}")
    width = int(token())
    height = int(token())
    maxval = int(token())
    if maxval <= 0 or maxval > 255:
        raise ValueError(f"unsupported PGM maxval {maxval}")
    while idx < len(data) and data[idx] in b" \t\r\n":
        idx += 1
    if magic == b"P5":
        pixels = list(data[idx : idx + width * height])
    else:
        pixels = [int(token()) for _ in range(width * height)]
    if len(pixels) != width * height:
        raise ValueError(f"expected {width * height} pixels, got {len(pixels)}")
    return width, height, pixels


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: waver_map_quality_check.py <map.yaml>", file=sys.stderr)
        return 2
    yaml_path = Path(sys.argv[1]).expanduser().resolve()
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
    image = Path(str(meta.get("image", "")))
    if not image.is_absolute():
        image = yaml_path.parent / image
    width, height, pixels = read_pgm(image)
    total = width * height
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.25))
    occupied = 0
    free = 0
    unknown = 0
    non_unknown = []
    for i, px in enumerate(pixels):
        occ = (255 - int(px)) / 255.0
        if occ > occupied_thresh:
            occupied += 1
            non_unknown.append(i)
        elif occ < free_thresh:
            free += 1
            non_unknown.append(i)
        else:
            unknown += 1
    known = occupied + free
    known_ratio = known / float(total or 1)
    if non_unknown:
        xs = [i % width for i in non_unknown]
        ys = [i // width for i in non_unknown]
        bbox_area = (max(xs) - min(xs) + 1) * (max(ys) - min(ys) + 1)
    else:
        bbox_area = 0
    reasons = []
    if occupied < 100:
        reasons.append("OCCUPIED_TOO_FEW")
    if free < 1000:
        reasons.append("FREE_TOO_FEW")
    if known_ratio < 0.02:
        reasons.append("KNOWN_RATIO_LOW")
    if bbox_area < 2500:
        reasons.append("KNOWN_BBOX_TOO_SMALL")
    print(f"MAP_YAML={yaml_path}")
    print(f"MAP_IMAGE={image}")
    print(f"MAP_WIDTH={width}")
    print(f"MAP_HEIGHT={height}")
    print(f"OCCUPIED_COUNT={occupied}")
    print(f"FREE_COUNT={free}")
    print(f"UNKNOWN_COUNT={unknown}")
    print(f"KNOWN_COUNT={known}")
    print(f"KNOWN_RATIO={known_ratio:.6f}")
    print(f"KNOWN_BBOX_AREA={bbox_area}")
    if reasons:
        print(f"MAP_QUALITY=FAIL reason={','.join(reasons)}")
        return 1
    print("MAP_QUALITY=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
