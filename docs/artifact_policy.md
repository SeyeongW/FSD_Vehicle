# Artifact Policy

This repository separates day-to-day development files from paper submission
artifacts. Do not submit a raw workspace backup as the paper source package.

## Artifact Types

### A. `dev_workspace_backup`

Internal backup only.

May contain:

- `.git/`
- `build/`, `install/`, `log/`
- local caches
- local maps, bags, screenshots, temporary reports

Must not be used as the paper/public source artifact.

### B. `source_release`

Paper/public source package.

Created only with:

```bash
python3 scripts/make_source_archive.py --root .
python3 scripts/check_submission_package.py --path <archive>
```

Must exclude:

- `.git/`
- `build/`, `install/`, `log/`
- `.env` and local field env files
- rosbag/db3/mcap/sqlite/log/cache files
- stale generated validation outputs
- absolute host symlinks

Must include:

- source code
- launch/config/test files
- example env files
- documented sample map fixtures
- `reports/source_manifest.json`
- `reports/source_sha256_manifest.csv`

### C. `evidence_package`

Raw experiment evidence package.

Stored separately from `source_release`. It may contain rosbags, screenshots,
maps, raw CSV files, and ground truth tables. It is required only when the paper
claims experimental performance, detector accuracy, real deterrence effect, or
field operation.

## Claim Boundary

This source package can support source-level, unit-level, static-contract, and
optional single-run Gazebo smoke validation claims. It does not claim repeated
performance, real bird detector accuracy, or wheel-on safety unless a separate
evidence package proves those claims.
