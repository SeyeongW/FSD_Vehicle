import subprocess
import tarfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def run(cmd, cwd):
    return subprocess.run(cmd, cwd=cwd, text=True, capture_output=True, check=False)


def test_field_release_extracts_and_quickstart_dry_run_passes(tmp_path):
    archive = tmp_path / "waver_field_release.tar.gz"
    result = run(["python3", "scripts/make_field_release.py", "--output", str(archive)], ROOT)
    assert result.returncode == 0, result.stdout + result.stderr
    assert archive.exists()

    extract_dir = tmp_path / "extract"
    extract_dir.mkdir()
    with tarfile.open(archive, "r:*") as tf:
        tf.extractall(extract_dir)
    tree = next(p for p in extract_dir.iterdir() if p.is_dir())

    assert not (tree / "config/waver_field_env").exists()
    quickstart = run(
        [
            "bash",
            "scripts/waver_quickstart_field.sh",
            "--dry-run",
            "--jetson-host",
            "192.0.2.10",
            "--jetson-user",
            "waver",
            "--jetson-ws",
            "/home/waver/ros2_ws5/FSD_Vehicle",
            "--container",
            "fsd_dev_jetson",
            "--serial-port",
            "/dev/serial/by-id/usb-WAVER_BASE_TEST",
        ],
        tree,
    )
    assert quickstart.returncode == 0, quickstart.stdout + quickstart.stderr

    (tree / "config/waver_field_env.local").unlink(missing_ok=True)
    acceptance = run(["bash", "scripts/waver_clone_to_run_acceptance.sh", "--release-mode"], tree)
    assert acceptance.returncode == 0, acceptance.stdout + acceptance.stderr

    check = run(["python3", "scripts/check_field_release.py", "--path", str(archive)], ROOT)
    assert check.returncode == 0, check.stdout + check.stderr
