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
        "reports/gazebo_functional_validation/20260630_190314/summary.csv",
        "reports/field_docker_ssh_check/20260630_190314/summary.csv",
        "reports/remote_ui_validation/20260630_190314/summary.csv",
        "reports/full_readiness_loop/20260630_190110/summary.csv",
        "reports/pre_existing_git_status.txt",
        "bags/field_test.mcap",
        "rosbag/run1/data.db3",
        "rosbag_2026_07_04/run.mcap",
        "reports/rosbag_replay/sample.db3",
        "config/waver_field_env",
        "config/waver_field_env.local",
    ):
        assert excluded(path), path


def test_archive_includes_real_source_files():
    for path in (
        "src/waver_patrol/package.xml",
        "src/waver_patrol/launch/waver_real_bird_autonomy.launch.py",
        "README.md",
        ".env.example",
        "config/waver_field_env.example",
        "config/waver_field_env.local.example",
        "config/real_profiles/wheel_on_low_speed.yaml",
        "config/waver_base_feedback_schema.yaml",
        "reports/rosbag_replay/README.md",
        "ROSBAG_REGRESSION_GUIDE.md",
    ):
        assert not excluded(path), path


def test_archive_includes_map_metadata_when_maps_are_present():
    maps = list((ROOT / "maps").glob("*.pgm")) + list((ROOT / "maps").glob("*.yaml"))
    if maps:
        assert (ROOT / "docs/sample_map_metadata.md").exists() or (ROOT / "maps/README.md").exists()
