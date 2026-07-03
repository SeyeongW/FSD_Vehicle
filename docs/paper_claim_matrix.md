# Paper Claim Matrix

| Claim | Evidence level | Current evidence | Allowed wording | Forbidden wording |
|---|---|---|---|---|
| Safety command mux prevents direct final `/cmd_vel` authority drift | L0/L1 static and unit contracts | `waver_contract_check.py`, command mux tests | "verified by static and unit tests" | "proven safe in real field operation" |
| Height-based elevated dynamic target gate separates elevated moving targets from static/low targets | L1/L2 depending on CSV evidence | `prepare_paper_results.py` tables when raw rows exist | "evaluated in Gazebo/synthetic evidence" | "real bird classification is accurate" |
| Gazebo target mission chain can trigger approach/inspection behavior | L2 if Gazebo logs PASS | Gazebo validation loop and raw experiment summaries | "demonstrated in Gazebo scenario" | "validated on real birds" |
| UI visualization shows map/pose/path/target states | L1/L2 depending on UI validation logs | remote UI validation summaries | "UI smoke-tested" | "operator workload is proven reduced" |
| SLAM/map smoke can save and reload sample maps | L2 smoke only | UI SLAM smoke scripts and sample map metadata | "smoke-tested in Gazebo" | "field localization is validated" |
| Real robot readiness | L3 dry-run only unless wheel-on evidence exists | real launch/static contracts, no wheel-on evidence in source package | "pre-real guarded launch exists" | "real wheel-on operation is proven" |
| Livox/MID-360 readiness | L1/L2 limited | config and patch policy; plugin failure must be separated from fallback | "MID-360 configuration path is prepared" | "Livox Gazebo plugin worked" when `libros2_livox.so` failed |
| Bird detection/classification | L0/L1 contract unless external GT exists | detector/fusion contracts, fake detector flags in tables | "classification hook exists" | "bird precision/recall/mAP is proven" without external ground truth |
| Sound deterrence | L1/L2 simulated event only | fake/sim sound flags in evidence manifest | "simulated sound event was triggered" | "real deterrence effect was proven" |

## Evidence Levels

- L0: source/static contract only
- L1: unit or no-ROS test
- L2: Gazebo/simulation run
- L3: hardware dry-run without motion
- L4: wheel-off hardware test
- L5: supervised low-speed wheel-on
- L6: documented real field operation with raw evidence, checklists, and operator sign-off

If real wheel-on evidence, rosbag, safety checklist, and operator sign-off are
absent, L5/L6 claims are forbidden.
