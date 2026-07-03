# Paper Readiness Audit

Final judgement: **PAPER_READY_SOURCE_ONLY**

- Generated at: 2026-07-04T01:27:58
- Repository root: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`
- Paper mode: `source_only`
- Branch: `jo`
- Commit: `41c5b8e`
- Dirty file count: `72`
- Untracked file count: `0`
- Raw experiment data present: `False`
- Source manifest present: `True`

## Blocking Reasons

- none for the selected source-only policy

## Limitations / Non-Blockers

- livox_ros_driver2 submodule has local/dirty changes; patch policy B is documented
- raw experiment data absent; experimental performance claims are unavailable
- working tree has dirty/untracked files; HEAD commit alone does not reproduce this artifact

## Evidence Summary

| Item | Status | Notes |
|---|---|---|
| Code validation | PASS_RECORDED | `reports/local_validation_summary.md` records the latest static/package validation commands. |
| Gazebo validation | NOT_PROVEN_BY_THIS_AUDIT | Current-schema single-run smoke PASS is required for simulation-smoke claims. |
| Real wheel-on readiness | NOT_PROVEN | No automatic motor/serial/wheel-on execution is performed by this audit. |
| Paper raw data | ABSENT_PERFORMANCE_CLAIM_UNAVAILABLE | Raw data is required only for experimental performance claims. |
| Livox/MID-360 sim plugin | LIMITED | Dirty submodule patch captured at `patches/livox_ros_driver2_humble_mid360.patch`. |
| Repeated performance claims | NONE_FOUND | This package does not claim repeated-count or perfect-metric performance. |

## Source Manifest

- Manifest path: `reports/source_manifest.json`
- SHA256 manifest path: `reports/source_sha256_manifest.csv`
- Manifest artifact type: `source_release`
- Manifest generated at: `2026-07-04T01:26:36+0900`
- Manifest git branch: `jo`
- Manifest git commit: `41c5b8e`
- Manifest included file count: `1877`

HEAD commit alone does not reproduce this artifact when the working tree is dirty or has untracked files; use the source manifest and sha256 manifest for artifact-level reproducibility.

## Repeated-Claim Scan

- none found in paper-facing markdown docs

## Submodule Status

```text
6a940156dd7151c3ab6a52442d86bc83613bd11b src/Livox-SDK2 (v1.2.5)
 58ae16b43cc90423d3f8dc2ae3018a7c178c330a src/livox_laser_simulation_RO2 (heads/main)
 6b9356cadf77084619ba406e6a0eb41163b08039 src/livox_ros_driver2 (1.2.4-2-g6b9356c)
```

## Claim Boundary

- Allowed: static/unit contract checks, Gazebo smoke or simulation claims only when logs are PASS.
- Forbidden: real wheel-on safety, real bird detection accuracy, real deterrence effect, or Livox simulation success when plugin logs show failure.

See `reports/paper_evidence_manifest.csv` for artifact-level evidence rows.
