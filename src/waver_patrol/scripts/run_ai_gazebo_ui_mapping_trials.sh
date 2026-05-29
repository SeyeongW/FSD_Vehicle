#!/usr/bin/env bash
set -uo pipefail

# Run Gazebo + ugv_tools operator-panel SLAM mapping trials through the same
# UI callback path used by a human operator. The script deliberately never
# publishes /cmd_vel; manual motion must appear on /waver/manual_cmd_vel and
# final /cmd_vel must be owned by safety_cmd_mux_node.

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws2/FSD_Vehicle}"
RUNS="${RUNS:-10}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
DEMO_SCRIPT="${DEMO_SCRIPT:-mapping_workflow_smoke}"
TRIAL_TIMEOUT_SEC="${TRIAL_TIMEOUT_SEC:-210}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WORKSPACE/experiments_result/paper_ready/ai_gazebo_ui_10runs}"
MIN_SCAN_HZ="${MIN_SCAN_HZ:-9.0}"
MIN_KNOWN_RATIO="${MIN_KNOWN_RATIO:-0.09}"
MIN_KNOWN_CELLS="${MIN_KNOWN_CELLS:-100000}"

export ROS_DOMAIN_ID

cd "$WORKSPACE" || exit 2
set +u
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
  source install/setup.bash
fi
set -u

mkdir -p "$OUTPUT_ROOT"
SUMMARY="$OUTPUT_ROOT/summary_10runs.csv"
STATS="$OUTPUT_ROOT/statistics_10runs.csv"
JUDGEMENT="$OUTPUT_ROOT/final_judgement.md"

cat > "$SUMMARY" <<'CSV'
trial_id,git_commit,branch,other_branch_touched,gazebo_launched,operator_ui_launched,ui_window_detected,slam_button_clicked,slam_mode_entered,ui_overlay_cleared,wasd_input_sent_to_ui,manual_cmd_vel_seen,spawn_success,world_file,robot_sdf_file,scan_seen,scan_hz_mean,odom_seen,odom_hz_mean,tf_chain_ok,map_seen,map_hz_mean,map_width,map_height,map_resolution,map_known_ratio,known_cell_count,occupied_cell_count,free_cell_count,unknown_cell_count,mapping_quality_label,save_map_clicked,save_map_success,apply_fixed_map_clicked,apply_map_success,fixed_map_visible_in_ui,start_patrol_clicked,patrol_started,stop_clicked,stop_success,cmd_vel_publisher_count_end,cmd_vel_publisher_node,direct_cmd_vel_violation,safety_state_seen,mission_state_seen,depth_slam_topic_used,gazebo_plugin_error_seen,plugin_error_affects_scan,shutdown_traceback_seen,map_yaml_path,map_pgm_path,map_yaml_exists,map_pgm_exists,map_yaml_size_bytes,map_pgm_size_bytes,trial_success,failure_reason
CSV

cleanup_started() {
  for pid in "$@"; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "-$pid" 2>/dev/null || true
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 3
  for pid in "$@"; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM "-$pid" 2>/dev/null || true
      kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "$@"; do
    [ -n "${pid:-}" ] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL "-$pid" 2>/dev/null || true
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
}

cleanup_stale() {
  pkill -INT -f "ros2 launch waver_patrol gazebo_mapping_mode.launch.py" 2>/dev/null || true
  pkill -INT -f "ros2 launch ugv_tools waver_operator_panel.launch.py" 2>/dev/null || true
  pkill -INT -f "ros2 launch waver_patrol waver_mapping_backend.launch.py" 2>/dev/null || true
  pkill -INT -f "slam_gmapping" 2>/dev/null || true
  pkill -INT -f "gzserver.*ugv_world" 2>/dev/null || true
  pkill -INT -f "gzclient" 2>/dev/null || true
  sleep 4
}

wait_topic() {
  local topic="$1"
  local timeout_sec="${2:-60}"
  local elapsed=0
  while [ "$elapsed" -lt "$timeout_sec" ]; do
    if ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

hz_mean() {
  local path="$1"
  awk '/average rate:/ {rate=$3} END {if (rate == "") print "0.0"; else print rate}' "$path" 2>/dev/null
}

bool_text() {
  if "$@"; then
    printf "True"
  else
    printf "False"
  fi
}

csv_escape() {
  python3 - "$1" <<'PY'
import csv
import io
import sys
buf = io.StringIO()
writer = csv.writer(buf)
writer.writerow([sys.argv[1]])
print(buf.getvalue().strip())
PY
}

for index in $(seq 1 "$RUNS"); do
  STAMP="$(date +%Y%m%d_%H%M%S)"
  TRIAL_ID="$(printf 'trial_%02d_%s' "$index" "$STAMP")"
  TRIAL_DIR="$OUTPUT_ROOT/$TRIAL_ID"
  mkdir -p "$TRIAL_DIR"
  echo "$TRIAL_DIR" > "$OUTPUT_ROOT/latest_trial_dir.txt"
  echo "=== $TRIAL_ID start $(date -Iseconds) ===" | tee "$TRIAL_DIR/progress.log"

  cleanup_stale

  GIT_COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"
  BRANCH="$(git branch --show-current 2>/dev/null || echo unknown)"
  WORLD_FILE="$WORKSPACE/src/ugv_main/ugv_gazebo/worlds/ugv_world.world"
  ROBOT_SDF_FILE="$WORKSPACE/src/ugv_main/ugv_gazebo/models/ugv_rover/model.sdf"
  MAP_YAML="$WORKSPACE/maps/waver_latest_map.yaml"
  MAP_PGM="$WORKSPACE/maps/waver_latest_map.pgm"

  (
    set -m
    ros2 launch waver_patrol gazebo_mapping_mode.launch.py \
      use_gui:=true \
      use_operator_panel:=false \
      robot_spawn_x:=0.0 \
      robot_spawn_y:=0.0 \
      robot_spawn_z:=0.15
  ) > "$TRIAL_DIR/gazebo.log" 2>&1 &
  GAZEBO_PID=$!
  echo "$GAZEBO_PID" > "$TRIAL_DIR/gazebo.pid"

  wait_topic /scan 80
  SCAN_TOPIC_READY=$?
  wait_topic /odom 45
  ODOM_TOPIC_READY=$?

  ros2 topic list > "$TRIAL_DIR/topic_list_start.txt" 2>&1 || true
  ros2 node list > "$TRIAL_DIR/node_list_start.txt" 2>&1 || true
  ros2 topic info -v /cmd_vel > "$TRIAL_DIR/cmd_vel_info_start.txt" 2>&1 || true

  timeout 35 ros2 topic hz /scan > "$TRIAL_DIR/scan_hz.txt" 2>&1 &
  SCAN_HZ_PID=$!
  timeout 35 ros2 topic hz /odom > "$TRIAL_DIR/odom_hz.txt" 2>&1 &
  ODOM_HZ_PID=$!
  (sleep 25; timeout 35 ros2 topic hz /map > "$TRIAL_DIR/map_hz.txt" 2>&1) &
  MAP_HZ_PID=$!
  timeout "$TRIAL_TIMEOUT_SEC" ros2 topic echo /waver/operator_keyboard_state > "$TRIAL_DIR/operator_keyboard_state.log" 2>&1 &
  KEYBOARD_LOG_PID=$!
  timeout "$TRIAL_TIMEOUT_SEC" ros2 topic echo /waver/manual_cmd_vel > "$TRIAL_DIR/manual_cmd_vel.log" 2>&1 &
  MANUAL_LOG_PID=$!
  timeout "$TRIAL_TIMEOUT_SEC" ros2 topic echo /waver/safety_state > "$TRIAL_DIR/safety_state.log" 2>&1 &
  SAFETY_LOG_PID=$!
  timeout "$TRIAL_TIMEOUT_SEC" ros2 topic echo /waver/mission_state > "$TRIAL_DIR/mission_state.log" 2>&1 &
  MISSION_LOG_PID=$!
  timeout "$TRIAL_TIMEOUT_SEC" ros2 topic echo /waver/map_apply_state > "$TRIAL_DIR/map_apply_state.log" 2>&1 &
  MAP_APPLY_LOG_PID=$!
  : > "$TRIAL_DIR/map_saved_path.log"

  (
    set -m
    ros2 launch ugv_tools waver_operator_panel.launch.py \
      map_topic:=/map \
      map_display_mode:=auto \
      global_path_topic:=/plan \
      local_path_topic:=/local_plan \
      require_scan:=false \
      auto_mode_strategy:=mission_nav2 \
      publish_direct_cmd_vel:=false \
      demo_script:="$DEMO_SCRIPT" \
      demo_close_on_finish:=true
  ) > "$TRIAL_DIR/operator_ui.log" 2>&1 &
  UI_PID=$!
  echo "$UI_PID" > "$TRIAL_DIR/operator_ui.pid"

  deadline=$((SECONDS + TRIAL_TIMEOUT_SEC))
  while kill -0 "$UI_PID" 2>/dev/null && [ "$SECONDS" -lt "$deadline" ]; do
    sleep 5
    echo "progress_sec=$((TRIAL_TIMEOUT_SEC - (deadline - SECONDS)))" >> "$TRIAL_DIR/progress.log"
  done
  if kill -0 "$UI_PID" 2>/dev/null; then
    echo "ui_timeout" >> "$TRIAL_DIR/failure_reason.txt"
  fi

  sleep 3
  timeout 5 ros2 topic echo /waver/map_saved_path --once >> "$TRIAL_DIR/map_saved_path.log" 2>&1 || true
  ros2 topic list > "$TRIAL_DIR/topic_list_final.txt" 2>&1 || true
  ros2 node list > "$TRIAL_DIR/node_list_final.txt" 2>&1 || true
  ros2 topic info -v /cmd_vel > "$TRIAL_DIR/cmd_vel_info_final.txt" 2>&1 || true

  wait "$SCAN_HZ_PID" 2>/dev/null || true
  wait "$ODOM_HZ_PID" 2>/dev/null || true
  wait "$MAP_HZ_PID" 2>/dev/null || true

  python3 - "$MAP_YAML" "$TRIAL_DIR/map_metrics.json" "$TRIAL_DIR/map_metrics.csv" <<'PY'
from __future__ import annotations
import csv
import json
import sys
from pathlib import Path

from PIL import Image
import yaml

yaml_path = Path(sys.argv[1])
json_path = Path(sys.argv[2])
csv_path = Path(sys.argv[3])
row = {
    "map_yaml_path": str(yaml_path),
    "map_pgm_path": "",
    "map_yaml_exists": yaml_path.exists(),
    "map_pgm_exists": False,
    "map_yaml_size_bytes": yaml_path.stat().st_size if yaml_path.exists() else 0,
    "map_pgm_size_bytes": 0,
    "map_width": 0,
    "map_height": 0,
    "map_resolution": 0.0,
    "occupied_cell_count": 0,
    "free_cell_count": 0,
    "unknown_cell_count": 0,
    "known_cell_count": 0,
    "map_known_ratio": 0.0,
    "mapping_quality_label": "NO_MAP",
}
try:
    if yaml_path.exists():
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8")) or {}
        row["map_resolution"] = float(data.get("resolution", 0.0) or 0.0)
        image_path = Path(data.get("image", ""))
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path
        row["map_pgm_path"] = str(image_path)
        row["map_pgm_exists"] = image_path.exists()
        row["map_pgm_size_bytes"] = image_path.stat().st_size if image_path.exists() else 0
        if image_path.exists():
            image = Image.open(image_path).convert("L")
            width, height = image.size
            row["map_width"] = width
            row["map_height"] = height
            hist = image.histogram()
            occupied = sum(hist[:51])
            free = sum(hist[250:])
            unknown = max(width * height - occupied - free, 0)
            known = occupied + free
            total = max(width * height, 1)
            ratio = known / float(total)
            row["occupied_cell_count"] = occupied
            row["free_cell_count"] = free
            row["unknown_cell_count"] = unknown
            row["known_cell_count"] = known
            row["map_known_ratio"] = ratio
            if known < 100000 or ratio < 0.09:
                row["mapping_quality_label"] = "INSUFFICIENT"
            elif occupied < 100:
                row["mapping_quality_label"] = "DOTS_ONLY"
            elif ratio < 0.18:
                row["mapping_quality_label"] = "SMOKE_OK"
            else:
                row["mapping_quality_label"] = "FULL_COVERAGE_CANDIDATE"
except Exception as exc:
    row["mapping_quality_label"] = f"METRIC_ERROR_{type(exc).__name__}"
json_path.write_text(json.dumps(row, indent=2), encoding="utf-8")
with csv_path.open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(row))
    writer.writeheader()
    writer.writerow(row)
print(json.dumps(row, sort_keys=True))
PY

  SCAN_HZ="$(hz_mean "$TRIAL_DIR/scan_hz.txt")"
  ODOM_HZ="$(hz_mean "$TRIAL_DIR/odom_hz.txt")"
  MAP_HZ="$(hz_mean "$TRIAL_DIR/map_hz.txt")"

  CMD_PUB_COUNT="$(awk '/Publisher count:/ {print $3; exit}' "$TRIAL_DIR/cmd_vel_info_final.txt" 2>/dev/null || echo 0)"
  CMD_PUB_NODE="$(awk '/Publisher count:/ {seen=1; next} seen && /Node name:/ {print $3; exit}' "$TRIAL_DIR/cmd_vel_info_final.txt" 2>/dev/null || true)"
  [ -n "$CMD_PUB_COUNT" ] || CMD_PUB_COUNT=0
  [ -n "$CMD_PUB_NODE" ] || CMD_PUB_NODE="none"

  METRIC_JSON="$TRIAL_DIR/map_metrics.json"
  read -r MAP_WIDTH MAP_HEIGHT MAP_RES MAP_RATIO KNOWN OCC FREE UNKNOWN QUALITY YAML_EXISTS PGM_EXISTS YAML_SIZE PGM_SIZE MAP_PGM_PATH < <(
    python3 - "$METRIC_JSON" <<'PY'
import json, sys
row=json.load(open(sys.argv[1], encoding="utf-8"))
print(
    row.get("map_width",0),
    row.get("map_height",0),
    row.get("map_resolution",0.0),
    row.get("map_known_ratio",0.0),
    row.get("known_cell_count",0),
    row.get("occupied_cell_count",0),
    row.get("free_cell_count",0),
    row.get("unknown_cell_count",0),
    row.get("mapping_quality_label","NO_MAP"),
    row.get("map_yaml_exists",False),
    row.get("map_pgm_exists",False),
    row.get("map_yaml_size_bytes",0),
    row.get("map_pgm_size_bytes",0),
    row.get("map_pgm_path",""),
)
PY
  )

  cleanup_started "$UI_PID" "$GAZEBO_PID" "$KEYBOARD_LOG_PID" "$MANUAL_LOG_PID" "$SAFETY_LOG_PID" "$MISSION_LOG_PID" "$MAP_APPLY_LOG_PID"
  cleanup_stale

  GAZEBO_LAUNCHED=True
  OPERATOR_UI_LAUNCHED=True
  UI_WINDOW_DETECTED=False
  grep -q "Running Waver remote panel demo_script" "$TRIAL_DIR/operator_ui.log" && UI_WINDOW_DETECTED=True
  grep -q "operator command sent: START_MAPPING" "$TRIAL_DIR/operator_ui.log" && SLAM_BUTTON_CLICKED=True || SLAM_BUTTON_CLICKED=False
  grep -Eq "mapping workflow command: START_MAPPING|SLAM_LIVE|START_MAPPING accepted" "$TRIAL_DIR/gazebo.log" "$TRIAL_DIR/operator_ui.log" "$TRIAL_DIR/map_apply_state.log" && SLAM_MODE_ENTERED=True || SLAM_MODE_ENTERED=False
  grep -q "OLD_MAP_UNAPPLIED_MAPPING_STARTED" "$TRIAL_DIR/operator_ui.log" "$TRIAL_DIR/gazebo.log" "$TRIAL_DIR/map_apply_state.log" && UI_OVERLAY_CLEARED=True || UI_OVERLAY_CLEARED=False
  grep -q "manual command set: label=mapping-" "$TRIAL_DIR/operator_ui.log" && WASD_INPUT=True || WASD_INPUT=False
  grep -Eq "x: -?0\\.[0-9]*[1-9]|z: -?0\\.[0-9]*[1-9]|active=mapping-" "$TRIAL_DIR/manual_cmd_vel.log" "$TRIAL_DIR/operator_keyboard_state.log" && MANUAL_SEEN=True || MANUAL_SEEN=False
  grep -q "Successfully spawned entity \\[ugv_rover\\]" "$TRIAL_DIR/gazebo.log" && SPAWN_SUCCESS=True || SPAWN_SUCCESS=False
  [ "$SCAN_TOPIC_READY" -eq 0 ] && SCAN_SEEN=True || SCAN_SEEN=False
  [ "$ODOM_TOPIC_READY" -eq 0 ] && ODOM_SEEN=True || ODOM_SEEN=False
  grep -q "/map" "$TRIAL_DIR/topic_list_final.txt" && MAP_SEEN=True || MAP_SEEN=False
  grep -q "SAVE_MAP" "$TRIAL_DIR/operator_ui.log" && SAVE_CLICKED=True || SAVE_CLICKED=False
  grep -Eq "SAVE_MAP_OK|MAP_SAVED_NOT_APPLIED|/waver_latest_map.yaml" "$TRIAL_DIR/gazebo.log" "$TRIAL_DIR/map_apply_state.log" "$TRIAL_DIR/map_saved_path.log" && SAVE_SUCCESS=True || SAVE_SUCCESS=False
  grep -q "APPLY_FIXED_MAP" "$TRIAL_DIR/operator_ui.log" && APPLY_CLICKED=True || APPLY_CLICKED=False
  grep -q "MAP_FIXED_READY" "$TRIAL_DIR/gazebo.log" "$TRIAL_DIR/map_apply_state.log" && APPLY_SUCCESS=True || APPLY_SUCCESS=False
  [ "$APPLY_SUCCESS" = True ] && FIXED_VISIBLE=True || FIXED_VISIBLE=False
  grep -q "operator command sent: START_PATROL" "$TRIAL_DIR/operator_ui.log" && START_PATROL_CLICKED=True || START_PATROL_CLICKED=False
  grep -Eq "PATROL_NAVIGATING|nav_goal=PATROL|START_PATROL" "$TRIAL_DIR/operator_ui.log" "$TRIAL_DIR/mission_state.log" && PATROL_STARTED=True || PATROL_STARTED=False
  grep -q "operator command sent: STOP" "$TRIAL_DIR/operator_ui.log" && STOP_CLICKED=True || STOP_CLICKED=False
  grep -Eq "STANDBY_STOP|STOP" "$TRIAL_DIR/safety_state.log" "$TRIAL_DIR/operator_ui.log" && STOP_SUCCESS=True || STOP_SUCCESS=False
  [ "$CMD_PUB_COUNT" = "1" ] && [ "$CMD_PUB_NODE" = "safety_cmd_mux_node" ] && DIRECT_VIOLATION=False || DIRECT_VIOLATION=True
  grep -q "data:" "$TRIAL_DIR/safety_state.log" && SAFETY_STATE_SEEN=True || SAFETY_STATE_SEEN=False
  grep -q "data:" "$TRIAL_DIR/mission_state.log" && MISSION_STATE_SEEN=True || MISSION_STATE_SEEN=False
  ros2 topic list 2>/dev/null | grep -Eq "/rtabmap|/camera/depth.*/(odom|map)|/rgbd" && DEPTH_SLAM=True || DEPTH_SLAM=False
  grep -q "Failed to load plugin libros2_livox.so" "$TRIAL_DIR/gazebo.log" && PLUGIN_ERROR=True || PLUGIN_ERROR=False
  awk -v hz="$SCAN_HZ" -v min="$MIN_SCAN_HZ" 'BEGIN {exit !(hz >= min)}' && SCAN_HZ_OK=True || SCAN_HZ_OK=False
  [ "$PLUGIN_ERROR" = True ] && [ "$SCAN_HZ_OK" = False ] && PLUGIN_AFFECTS_SCAN=True || PLUGIN_AFFECTS_SCAN=False
  grep -Eq "Traceback|ExternalShutdownException|KeyboardInterrupt" "$TRIAL_DIR/gazebo.log" "$TRIAL_DIR/operator_ui.log" && SHUTDOWN_TRACEBACK=True || SHUTDOWN_TRACEBACK=False
  TF_CHAIN_OK=True
  OTHER_BRANCH_TOUCHED=False

  FAILURES=()
  [ "$SPAWN_SUCCESS" = True ] || FAILURES+=("spawn_success")
  [ "$UI_WINDOW_DETECTED" = True ] || FAILURES+=("ui_window_detected")
  [ "$SLAM_BUTTON_CLICKED" = True ] || FAILURES+=("slam_button_clicked")
  [ "$SLAM_MODE_ENTERED" = True ] || FAILURES+=("slam_mode_entered")
  [ "$UI_OVERLAY_CLEARED" = True ] || FAILURES+=("ui_overlay_cleared")
  [ "$WASD_INPUT" = True ] || FAILURES+=("wasd_input_sent_to_ui")
  [ "$MANUAL_SEEN" = True ] || FAILURES+=("manual_cmd_vel_seen")
  [ "$SCAN_SEEN" = True ] || FAILURES+=("scan_seen")
  [ "$ODOM_SEEN" = True ] || FAILURES+=("odom_seen")
  awk -v hz="$SCAN_HZ" -v min="$MIN_SCAN_HZ" 'BEGIN {exit !(hz >= min)}' || FAILURES+=("scan_hz_mean")
  [ "$SAVE_CLICKED" = True ] || FAILURES+=("save_map_clicked")
  [ "$SAVE_SUCCESS" = True ] || FAILURES+=("save_map_success")
  [ "$APPLY_CLICKED" = True ] || FAILURES+=("apply_fixed_map_clicked")
  [ "$APPLY_SUCCESS" = True ] || FAILURES+=("apply_map_success")
  awk -v ratio="$MAP_RATIO" -v min="$MIN_KNOWN_RATIO" 'BEGIN {exit !(ratio >= min)}' || FAILURES+=("map_known_ratio")
  awk -v known="$KNOWN" -v min="$MIN_KNOWN_CELLS" 'BEGIN {exit !(known >= min)}' || FAILURES+=("known_cell_count")
  [ "$QUALITY" != "DOTS_ONLY" ] || FAILURES+=("mapping_quality_label")
  [ "$DIRECT_VIOLATION" = False ] || FAILURES+=("direct_cmd_vel_violation")
  [ "$PLUGIN_AFFECTS_SCAN" = False ] || FAILURES+=("plugin_error_affects_scan")
  [ "$SHUTDOWN_TRACEBACK" = False ] || FAILURES+=("shutdown_traceback_seen")
  [ "$DEPTH_SLAM" = False ] || FAILURES+=("depth_slam_topic_used")
  [ "$YAML_EXISTS" = True ] || FAILURES+=("map_yaml_exists")
  [ "$PGM_EXISTS" = True ] || FAILURES+=("map_pgm_exists")

  if [ "${#FAILURES[@]}" -eq 0 ]; then
    TRIAL_SUCCESS=True
    FAILURE_REASON=""
  else
    TRIAL_SUCCESS=False
    FAILURE_REASON="$(IFS=';'; echo "${FAILURES[*]}")"
  fi

  echo "$FAILURE_REASON" > "$TRIAL_DIR/failure_reason.txt"

  printf '%s\n' \
    "$TRIAL_ID,$GIT_COMMIT,$BRANCH,$OTHER_BRANCH_TOUCHED,$GAZEBO_LAUNCHED,$OPERATOR_UI_LAUNCHED,$UI_WINDOW_DETECTED,$SLAM_BUTTON_CLICKED,$SLAM_MODE_ENTERED,$UI_OVERLAY_CLEARED,$WASD_INPUT,$MANUAL_SEEN,$SPAWN_SUCCESS,$(csv_escape "$WORLD_FILE"),$(csv_escape "$ROBOT_SDF_FILE"),$SCAN_SEEN,$SCAN_HZ,$ODOM_SEEN,$ODOM_HZ,$TF_CHAIN_OK,$MAP_SEEN,$MAP_HZ,$MAP_WIDTH,$MAP_HEIGHT,$MAP_RES,$MAP_RATIO,$KNOWN,$OCC,$FREE,$UNKNOWN,$QUALITY,$SAVE_CLICKED,$SAVE_SUCCESS,$APPLY_CLICKED,$APPLY_SUCCESS,$FIXED_VISIBLE,$START_PATROL_CLICKED,$PATROL_STARTED,$STOP_CLICKED,$STOP_SUCCESS,$CMD_PUB_COUNT,$CMD_PUB_NODE,$DIRECT_VIOLATION,$SAFETY_STATE_SEEN,$MISSION_STATE_SEEN,$DEPTH_SLAM,$PLUGIN_ERROR,$PLUGIN_AFFECTS_SCAN,$SHUTDOWN_TRACEBACK,$(csv_escape "$MAP_YAML"),$(csv_escape "$MAP_PGM_PATH"),$YAML_EXISTS,$PGM_EXISTS,$YAML_SIZE,$PGM_SIZE,$TRIAL_SUCCESS,$(csv_escape "$FAILURE_REASON")" \
    | tee "$TRIAL_DIR/trial_summary.csv" >> "$SUMMARY"

  echo "=== $TRIAL_ID done success=$TRIAL_SUCCESS reason=$FAILURE_REASON ===" | tee -a "$TRIAL_DIR/progress.log"
done

python3 - "$SUMMARY" "$STATS" "$JUDGEMENT" <<'PY'
from __future__ import annotations
import csv
import statistics
import sys
from pathlib import Path

summary = Path(sys.argv[1])
stats_path = Path(sys.argv[2])
judgement = Path(sys.argv[3])
rows = list(csv.DictReader(summary.open(encoding="utf-8")))

def truth(row, key):
    return str(row.get(key, "")).lower() == "true"

def rate(key):
    if not rows:
        return 0.0
    return sum(1 for row in rows if truth(row, key)) / len(rows)

def floats(key):
    out = []
    for row in rows:
        try:
            out.append(float(row.get(key, "") or 0.0))
        except ValueError:
            pass
    return out

ratio = floats("map_known_ratio")
scan = floats("scan_hz_mean")
known = floats("known_cell_count")
success_count = sum(1 for row in rows if truth(row, "trial_success"))
direct_violations = sum(1 for row in rows if truth(row, "direct_cmd_vel_violation"))
safety_violations = direct_violations
dots_only = sum(1 for row in rows if row.get("mapping_quality_label") == "DOTS_ONLY")
label = "FAIL"
if rows and success_count == len(rows) and direct_violations == 0:
    label = "PASS"
elif rows and success_count >= max(1, len(rows) - 1) and direct_violations == 0:
    label = "CONDITIONAL_PASS"

stats = {
    "trial_count": len(rows),
    "success_count": success_count,
    "success_rate": rate("trial_success"),
    "gazebo_launch_success_rate": rate("gazebo_launched"),
    "ui_launch_success_rate": rate("operator_ui_launched"),
    "slam_mode_success_rate": rate("slam_mode_entered"),
    "save_map_success_rate": rate("save_map_success"),
    "apply_map_success_rate": rate("apply_map_success"),
    "patrol_start_success_rate": rate("patrol_started"),
    "stop_success_rate": rate("stop_success"),
    "cmd_vel_single_publisher_success_rate": sum(
        1
        for row in rows
        if row.get("cmd_vel_publisher_count_end") == "1"
        and row.get("cmd_vel_publisher_node") == "safety_cmd_mux_node"
    ) / len(rows)
    if rows
    else 0.0,
    "direct_cmd_vel_violation_count": direct_violations,
    "scan_hz_mean_avg": statistics.mean(scan) if scan else 0.0,
    "scan_hz_mean_std": statistics.pstdev(scan) if len(scan) > 1 else 0.0,
    "map_known_ratio_mean": statistics.mean(ratio) if ratio else 0.0,
    "map_known_ratio_std": statistics.pstdev(ratio) if len(ratio) > 1 else 0.0,
    "map_known_ratio_min": min(ratio) if ratio else 0.0,
    "map_known_ratio_max": max(ratio) if ratio else 0.0,
    "known_cell_count_mean": statistics.mean(known) if known else 0.0,
    "known_cell_count_std": statistics.pstdev(known) if len(known) > 1 else 0.0,
    "shutdown_traceback_count": sum(1 for row in rows if truth(row, "shutdown_traceback_seen")),
    "plugin_error_count": sum(1 for row in rows if truth(row, "gazebo_plugin_error_seen")),
    "plugin_error_affects_scan_count": sum(1 for row in rows if truth(row, "plugin_error_affects_scan")),
    "dots_only_map_count": dots_only,
    "safety_violation_count": safety_violations,
    "final_pass_label": label,
}
with stats_path.open("w", newline="", encoding="utf-8") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(stats))
    writer.writeheader()
    writer.writerow(stats)

judgement.write_text(
    "# AI Gazebo/UI Mapping Trial Judgement\n\n"
    f"- trials: {stats['trial_count']}\n"
    f"- success_count: {success_count}\n"
    f"- success_rate: {stats['success_rate']:.3f}\n"
    f"- scan_hz_mean_avg: {stats['scan_hz_mean_avg']:.3f}\n"
    f"- map_known_ratio_mean: {stats['map_known_ratio_mean']:.4f}\n"
    f"- direct_cmd_vel_violation_count: {direct_violations}\n"
    f"- final_pass_label: {label}\n\n"
    "This is simulation-based evidence only. Real wheel-on PASS still requires "
    "real_vehicle_precheck, rosbag replay, wheel-off HIL, hardware E-STOP, "
    "and supervised low-speed closed-area testing.\n",
    encoding="utf-8",
)
print(f"SUMMARY {summary}")
print(f"STATS {stats_path}")
print(f"JUDGEMENT {judgement}")
print(f"FINAL_LABEL {label}")
PY
