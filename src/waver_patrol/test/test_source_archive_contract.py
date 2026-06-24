import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts/make_source_archive.py"
spec = importlib.util.spec_from_file_location("make_source_archive", SCRIPT)
archive = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(archive)


def excluded(path: str) -> bool:
    return archive.should_exclude(ROOT / path, ROOT)


def test_archive_excludes_generated_and_local_secret_paths():
    for path in (
        ".git/config",
        ".env",
        ".env.local",
        "build/waver_patrol/file",
        "install/waver_patrol/file",
        "log/latest.log",
        "src/waver_patrol/__pycache__/node.cpython-310.pyc",
        "experiment_results/trial/run.db3",
        "reports/quality_gate/latest/contract_report.json",
        "bags/field_test.mcap",
    ):
        assert excluded(path), path


def test_archive_includes_real_source_files():
    for path in (
        "src/waver_patrol/package.xml",
        "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py",
        "README.md",
        ".env.example",
    ):
        assert not excluded(path), path
