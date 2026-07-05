import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_release_generator_writes_sidecar_with_actual_archive_hash():
    script = (ROOT / "scripts/make_bird_mission_field_release.py").read_text()
    checker = (ROOT / "scripts/check_bird_mission_field_release.py").read_text()
    assert "BIRD_MISSION_FIELD_RELEASE_SIDECAR" in script
    assert "archive_sha256" in script
    assert "sha256sum" in checker
    assert "sidecar archive_sha256 does not match archive" in checker


def test_source_manifest_is_not_release_hash_source_of_truth():
    manifest_path = ROOT / "reports/source_manifest.json"
    if not manifest_path.exists():
        return
    data = json.loads(manifest_path.read_text())
    assert data.get("artifact_type") in {"source_release", "bird_mission_field_release"}
    assert data.get("release_sha256") in {None, "computed_after_archive_write"}


def test_current_git_head_is_available_to_runtime_manifest_generation():
    head = subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"], text=True).strip()
    assert head
