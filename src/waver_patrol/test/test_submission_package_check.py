import io
import tarfile
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/check_submission_package.py"


def _make_tar(path: Path, names: list[str]) -> None:
    with tarfile.open(path, "w:gz") as tar:
        for name in names:
            data = b"test\n"
            info = tarfile.TarInfo(name)
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))


def test_submission_checker_rejects_raw_workspace_artifacts(tmp_path):
    archive = tmp_path / "bad.tar.gz"
    _make_tar(archive, ["FSD_Vehicle/.git/config", "FSD_Vehicle/build/foo", "FSD_Vehicle/.env"])
    result = subprocess.run(["python3", str(SCRIPT), "--path", str(archive)], text=True, capture_output=True)
    assert result.returncode != 0
    assert "SUBMISSION_PACKAGE_CHECK=FAIL" in result.stdout


def test_submission_checker_allows_examples_and_docs(tmp_path):
    archive = tmp_path / "good.tar.gz"
    _make_tar(
        archive,
        [
            "FSD_Vehicle/README.md",
            "FSD_Vehicle/.env.example",
            "FSD_Vehicle/config/waver_field_env.example",
            "FSD_Vehicle/reports/rosbag_replay/README.md",
        ],
    )
    result = subprocess.run(["python3", str(SCRIPT), "--path", str(archive)], text=True, capture_output=True)
    assert result.returncode == 0
    assert "SUBMISSION_PACKAGE_CHECK=PASS" in result.stdout
