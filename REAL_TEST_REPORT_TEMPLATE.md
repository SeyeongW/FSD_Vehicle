# Waver Real Test Report Template

## Test Metadata

- Date:
- Operator:
- Workspace:
- Branch:
- Commit:
- Test type: dry-run / wheel-off / low-speed wheel-on
- Jetson IP:
- Serial port:
- Battery voltage:
- Physical E-stop verified: yes/no

## Commands

```bash
# Paste exact commands here.
```

## Safety Contract

- `/cmd_vel` publisher count:
- final `/cmd_vel` publisher:
- `/waver/mode` publisher count:
- serial owner count:
- STANDBY zero command: pass/fail
- E-stop immediate zero: pass/fail
- external stop zero: pass/fail
- release timeout zero: pass/fail

## Motion Results

- forward command result:
- reverse command result:
- left pivot result:
- right pivot result:
- 0.3 m square patrol result:
- unexpected movement:
- maximum observed speed:

## Sensor Results

- `/odom`:
- `odom -> base_link` TF:
- `/scan` or `/scan_safety`:
- camera:
- bird detector state:
- bird 3D fusion state:
- voltage:

## Mission Results

- START_PATROL gate:
- target interrupt:
- departure pose save:
- return to departure:
- resume patrol:
- sound gate:

## Logs And Bags

- rosbag path:
- report directory:
- screenshots:
- notes:

## Verdict

- PASS/FAIL:
- blocking issue:
- next required fix:
