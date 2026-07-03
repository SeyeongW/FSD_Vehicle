# Hardware Acceptance Matrix

The readiness checker treats this matrix as the machine-readable checklist
boundary for L4 and above. The table is intentionally conservative; unknown or
missing evidence is not a PASS.

| item | required_for_level | evidence_command | evidence_file | status | notes |
|---|---|---|---|---|---|
| LiDAR mounted and frame axis verified | L2 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | `reports/hardware_calibration/lidar_frame.json` | TODO | Confirm forward/lateral/height axes. |
| LiDAR topic rate verified | L2 | `python3 scripts/waver_field_readiness_check.py --level L2 --strict` | `reports/field_readiness/latest.json` | TODO | `/scan` or `/scan_safety` must meet rate threshold. |
| pointcloud_to_laserscan or Livox adapter verified | L2 | `ros2 topic hz /scan_safety` | `reports/hardware_calibration/scan_adapter.json` | TODO | Sparse obstacle scan is not sufficient for SLAM unless documented. |
| base_link -> livox_frame static TF verified | L2 | `ros2 run tf2_ros tf2_echo base_link livox_frame` | `reports/hardware_calibration/livox_tf.json` | TODO | Required for LiDAR target localization. |
| base_link -> camera_frame static TF verified | L5 | `ros2 run tf2_ros tf2_echo base_link camera_frame` | `reports/hardware_calibration/camera_tf.json` | TODO | Required only if camera/bird fusion is enabled. |
| odom -> base_link TF verified | L3 | `ros2 run tf2_ros tf2_echo odom base_link` | `reports/hardware_calibration/odom_tf.json` | TODO | Required for wheel-off and wheel-on. |
| map -> odom localization verified | L4 | `ros2 run tf2_ros tf2_echo map odom` | `reports/hardware_calibration/localization_tf.json` | TODO | Required for Nav2 localization. |
| wheel radius/wheel base verified | L3 | `python3 scripts/waver_motor_calibration_wizard.py ...` | `config/waver_motor_calibration.yaml` | TODO | No automatic wheel motion without wheel-off confirmation. |
| encoder direction verified | L3 | `python3 scripts/waver_base_feedback_probe.py ...` | `reports/base_feedback/latest.json` | TODO | Required before wheel-on. |
| battery voltage scale verified | L4 | `ros2 topic echo --once /voltage` | `reports/hardware_calibration/battery_scale.json` | TODO | Voltage under load must be calibrated. |
| E-stop physically verified | L4 | manual checklist | `reports/hardware_feedback/WHEEL_OFF_TEST_CHECKLIST.md` | TODO | Human operator and physical E-stop required. |
| sound output physically verified | L5 | manual checklist | `reports/hardware_calibration/sound_ack.json` | OPTIONAL | Experimental only; requires legal and hardware ack. |
| bird detector model path verified | L5 | `ros2 topic echo /waver/bird_detector_state` | `reports/hardware_calibration/bird_detector.json` | OPTIONAL | Experimental only. |
| gimbal/pan-tilt backend verified | L5 | backend-specific command | `reports/hardware_calibration/gimbal.json` | OPTIONAL | Experimental only. |
