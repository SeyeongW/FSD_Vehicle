# Paper Outline

## Title

Waver: A ROS 2 UGV Pipeline for Gazebo-Based Elevated Dynamic Target Patrol

## Abstract Draft

This paper describes a ROS 2 Humble Waver UGV software stack that integrates
guarded command muxing, Gazebo validation, elevated dynamic target filtering,
remote UI monitoring, and reproducible evidence packaging. Current evidence is
simulation/source-level unless raw experiment data and strict gates are supplied.

## Contributions

1. Safety-critical command authority structure for manual, mission, and autonomy commands.
2. Evidence-aware Gazebo validation workflow with PASS/FAIL/SKIP separation.
3. Height and dynamic-motion target filtering pipeline for elevated target scenarios.
4. Paper artifact generation that preserves raw rows, limitations, and failure reasons.

## System Architecture

Remote UI and mission commands feed guarded ROS topics. The safety command mux is
the final `/cmd_vel` authority. Perception and target mission nodes may request
goals but must not publish final motor commands directly.

## Safety Design

The source package supports static/unit contract checks and guarded real launch
configuration. Real wheel-on safety is not proven by source artifacts alone.

## Perception And Elevated Dynamic Target Filtering

The Gazebo/synthetic pipeline distinguishes elevated dynamic targets from static
or low-altitude targets using height and motion gates. Bird classification claims
require external ground truth and cannot be inferred from fake detector rows.

## Mission State Machine

Patrol, target inspection, sound-event simulation, and return-to-patrol should be
reported only when corresponding evidence rows exist and pass strict validation.

## Experimental Setup

This paper package does not claim repeated-trial performance. It reports
source-level, unit-level, static-contract, and optional single-run Gazebo smoke
validation only.

## Results Table Placeholders

Experimental performance tables are intentionally absent unless a separate
`evidence_package` supplies raw data and ground truth.

Optional future evidence files:

- `experiments_result/paper_ready/<run>/tables/paper_metrics.csv`
- `experiments_result/paper_ready/<run>/tables/failure_reason_counts.csv`
- `experiments_result/paper_ready/<run>/tables/evidence_level_summary.csv`


## Limitations

- No automatic real motor, serial bridge, or wheel-on run is executed by the
  paper-preparation scripts.
- Livox Gazebo plugin failures must not be merged with scan-mapper fallback
  smoke results.
- Real bird detection accuracy and deterrence effect require separate external
  ground truth and real-world evidence.

## Reproducibility Package

Use `scripts/make_source_archive.py --dry-run --list` to inspect source-only
release contents. Build/install/log, rosbag/db3/mcap, local env, and raw
experiment result directories are excluded from source archives.
