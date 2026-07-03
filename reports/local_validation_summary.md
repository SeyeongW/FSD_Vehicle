# Local Validation Summary

Generated: 2026-07-04

Workspace: `/home/chotaehyun/ros2_ws5/FSD_Vehicle`

Branch: `jo`

## Result

| Check | Result | Notes |
|---|---:|---|
| Python compileall | PASS | `scripts`, `src/waver_patrol/waver_patrol`, launch files, and Waver scripts compiled. |
| Shell syntax | PASS | All `.sh` files under `scripts` and `src/waver_patrol/scripts` passed `bash -n`. |
| Focused paper-readiness tests | PASS | Covered archive, submission package, claim hygiene, and status classification contracts. |
| no-ROS unit test suite | PASS | 66 tests passed. |
| `waver_patrol` pytest suite | PASS | 86 passed, 10 skipped. |
| clone-to-run acceptance | PASS | No warnings, no skipped checks. |
| source archive filter | PASS | Local field env, generated validation CSVs, build/install/log, bags, and caches excluded. |
| Waver contract check | PASS | 0 critical/high/medium/low issues; build/install/log are informational generated artifacts. |
| source release extraction self-test | PASS | Extracted archive passes contract check without `.git`, no-ROS tests, and submission-package privacy scan. |
| paper result preparation | SOURCE_ONLY | Raw experiment data is absent, so performance claims are unavailable. This is not a blocker for source-only paper packaging. |

## Claim Boundary

This validation proves static/package readiness only. It does not prove real wheel-on safety, real bird classification accuracy, real sound deterrence effect, or publishable experimental performance without raw experiment data in `experiments_result/`.
