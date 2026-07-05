import hashlib
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.exists() else "MISSING"


def test_bird_release_generation_does_not_mutate_source_manifest_by_default(tmp_path):
    manifest = ROOT / "reports/source_manifest.json"
    before = digest(manifest)
    archive = tmp_path / "bird_release.tar.gz"
    proc = subprocess.run(
        ["python3", str(ROOT / "scripts/make_bird_mission_field_release.py"), "--output", str(archive)],
        text=True,
        capture_output=True,
        check=False,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    assert archive.exists()
    assert digest(manifest) == before
    assert "BIRD_MISSION_FIELD_RELEASE_SHA256=" in proc.stdout
