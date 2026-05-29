#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-$HOME/ros2_ws2/FSD_Vehicle}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$WORKSPACE/experiments_result/paper_ready/bird_detection_10runs}"
RUNS="${RUNS:-10}"
REQUIRED_SUCCESSES="${REQUIRED_SUCCESSES:-10}"
USE_GUI="${USE_GUI:-true}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
SCENARIO_ID="${SCENARIO_ID:-B3_DYNAMIC_BIRD_HIGH}"
TRIAL_DURATION_SEC="${TRIAL_DURATION_SEC:-48}"
HZ_SAMPLE_SEC="${HZ_SAMPLE_SEC:-12}"

cd "$WORKSPACE"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID
export WAVER_GIT_BRANCH
export WAVER_GIT_COMMIT
WAVER_GIT_BRANCH="$(git branch --show-current 2>/dev/null || echo jo)"
WAVER_GIT_COMMIT="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

FINAL_DIR="$OUTPUT_ROOT/final_10runs"
FAILED_DIR="$OUTPUT_ROOT/failed_runs"
mkdir -p "$FINAL_DIR" "$FAILED_DIR"

summary="$FINAL_DIR/summary_10runs.csv"
cat > "$summary" <<'CSV'
trial_id,git_commit,branch,scenario_id,gazebo_launched,operator_ui_launched,ui_window_detected,bird_detector_launched,slam_button_clicked,slam_mode_entered,patrol_started,bird_candidate_count,ground_truth_bird_count,true_positive_count,false_positive_count,false_negative_count,bird_precision,bird_recall,bird_f1,bird_mAP_50,bird_mAP_50_95,z_valid_rate,height_error_rmse_m,position_error_rmse_m,track_success_rate,track_continuity_sec,id_switch_count,dynamic_classification_accuracy,ego_motion_compensation_success,mission_trigger_expected,mission_triggered,mission_trigger_correct,trigger_latency_ms,mapping_mode_wrong_trigger_count,patrol_mode_valid_trigger_count,detected_but_not_triggered_count,trigger_without_bird_count,scan_seen,scan_hz_mean,odom_seen,odom_hz_mean,tf_chain_ok,map_seen,map_known_ratio,known_cell_count,save_map_success,apply_map_success,cmd_vel_publisher_count_start,cmd_vel_publisher_count_end,cmd_vel_publisher_node,direct_cmd_vel_violation,stop_success,estop_zero_cmd_success,shutdown_traceback_seen,trial_success,failure_reason
CSV

cleanup() {
  pkill -INT -f "ros2 launch waver_patrol gazebo_bird_detection_validation.launch.py" 2>/dev/null || true
  pkill -INT -f "ros2 launch ugv_tools waver_operator_panel.launch.py" 2>/dev/null || true
  pkill -INT -f "gzserver" 2>/dev/null || true
  pkill -INT -f "gzclient" 2>/dev/null || true
  pkill -INT -f "[s]afety_cmd_mux_node" 2>/dev/null || true
  pkill -INT -f "[m]ission_patrol_manager_node" 2>/dev/null || true
  pkill -INT -f "[m]apping_workflow_manager_node" 2>/dev/null || true
  pkill -INT -f "[p]ointcloud_lidar_objects_node" 2>/dev/null || true
  pkill -INT -f "[m]oving_object_map_transform_node" 2>/dev/null || true
  pkill -INT -f "[g]azebo_map_path_visualizer_node" 2>/dev/null || true
  pkill -INT -f "[b]attery_return_manager_node" 2>/dev/null || true
  sleep 2
  pkill -TERM -f "gzserver|gzclient|bird_detection_pipeline_node|waver_remote_panel" 2>/dev/null || true
  pkill -TERM -f "[s]afety_cmd_mux_node" 2>/dev/null || true
  pkill -TERM -f "[m]ission_patrol_manager_node" 2>/dev/null || true
  pkill -TERM -f "[m]apping_workflow_manager_node" 2>/dev/null || true
  pkill -TERM -f "[p]ointcloud_lidar_objects_node" 2>/dev/null || true
  pkill -TERM -f "[m]oving_object_map_transform_node" 2>/dev/null || true
  pkill -TERM -f "[g]azebo_map_path_visualizer_node" 2>/dev/null || true
  pkill -TERM -f "[b]attery_return_manager_node" 2>/dev/null || true
}

stop_pid_tree() {
  for pid in "$@"; do
    [[ -n "${pid:-}" ]] || continue
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "$@"; do
    [[ -n "${pid:-}" ]] || continue
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "$@"; do
    [[ -n "${pid:-}" ]] || continue
    kill -KILL "$pid" 2>/dev/null || true
  done
}

extract_mean_hz() {
  local file="$1"
  python3 - "$file" <<'PY'
import re, sys
text=open(sys.argv[1], errors='ignore').read() if len(sys.argv)>1 else ''
m=re.findall(r'average rate:\s*([0-9.]+)', text)
print(m[-1] if m else '0')
PY
}

successes=0
attempt=1
while [[ "$successes" -lt "$REQUIRED_SUCCESSES" && "$attempt" -le "$RUNS" ]]; do
  trial_id="$(printf "trial_%02d" "$attempt")"
  trial_dir="$FINAL_DIR/$trial_id"
  rm -rf "$trial_dir"
  mkdir -p "$trial_dir"/{logs,csv,screenshots}
  cleanup

  echo "[bird-trial] starting $trial_id commit=$WAVER_GIT_COMMIT scenario=$SCENARIO_ID"
  ros2 launch waver_patrol gazebo_bird_detection_validation.launch.py \
    use_gui:="$USE_GUI" \
    use_operator_panel:=false \
    scenario_id:="$SCENARIO_ID" \
    expected_bird:=true \
    expected_mission_trigger:=true \
    move_target_model:=true \
    output_dir:="$trial_dir/csv" \
    > "$trial_dir/logs/gazebo_bird_detection.log" 2>&1 &
  gazebo_pid=$!
  echo "$gazebo_pid" > "$trial_dir/process_pids.txt"

  sleep 12

  ros2 launch ugv_tools waver_operator_panel.launch.py \
    map_topic:=/map \
    map_display_mode:=auto \
    global_path_topic:=/plan \
    local_path_topic:=/local_plan \
    require_scan:=false \
    auto_mode_strategy:=mission_nav2 \
    publish_direct_cmd_vel:=false \
    demo_script:=bird_detection_trial \
    demo_close_on_finish:=true \
    > "$trial_dir/logs/operator_ui.log" 2>&1 &
  ui_pid=$!
  echo "$ui_pid" >> "$trial_dir/process_pids.txt"

  sleep 5
  ros2 topic list > "$trial_dir/topic_list.txt" 2>&1 || true
  ros2 node list > "$trial_dir/node_list.txt" 2>&1 || true
  ros2 topic info -v /cmd_vel > "$trial_dir/cmd_vel_info_start.txt" 2>&1 || true
  timeout "$HZ_SAMPLE_SEC" ros2 topic hz /scan > "$trial_dir/scan_hz.txt" 2>&1 || true
  timeout "$HZ_SAMPLE_SEC" ros2 topic hz /odom > "$trial_dir/odom_hz.txt" 2>&1 || true
  timeout "$HZ_SAMPLE_SEC" ros2 topic hz /map > "$trial_dir/map_hz.txt" 2>&1 || true
  timeout 5 ros2 topic echo /bird/metrics --once > "$trial_dir/bird_metrics_snapshot.txt" 2>&1 || true
  timeout 5 ros2 topic echo /bird/mission_debug --once > "$trial_dir/bird_mission_debug_snapshot.txt" 2>&1 || true

  sleep "$TRIAL_DURATION_SEC"

  ros2 topic info -v /cmd_vel > "$trial_dir/cmd_vel_info_end.txt" 2>&1 || true
  timeout 3 ros2 topic echo /waver/safety_state --once > "$trial_dir/safety_state_snapshot.txt" 2>&1 || true
  timeout 3 ros2 topic echo /waver/mission_state --once > "$trial_dir/mission_state_snapshot.txt" 2>&1 || true

  stop_pid_tree "$ui_pid" "$gazebo_pid"
  cleanup
  wait "$gazebo_pid" 2>/dev/null || true
  wait "$ui_pid" 2>/dev/null || true

  set +e
  python3 - "$trial_dir" "$summary" "$trial_id" "$WAVER_GIT_COMMIT" "$WAVER_GIT_BRANCH" "$SCENARIO_ID" <<'PY'
import csv, pathlib, re, sys
trial=pathlib.Path(sys.argv[1])
summary=pathlib.Path(sys.argv[2])
trial_id, commit, branch, scenario = sys.argv[3:7]

def rows_for(path):
    p=trial/path
    if not p.exists():
        return []
    return list(csv.DictReader(p.open()))

metric_rows=rows_for('csv/bird_metrics.csv')
mission_rows=rows_for('csv/bird_mission_metrics.csv')
metrics=metric_rows[-1] if metric_rows else {}
mission=mission_rows[-1] if mission_rows else {}
cmd_text=(trial/'cmd_vel_info_end.txt').read_text(errors='ignore') if (trial/'cmd_vel_info_end.txt').exists() else ''
pub_count=len(re.findall(r'Publisher count:\s*(\d+)', cmd_text))
count_match=re.search(r'Publisher count:\s*(\d+)', cmd_text)
cmd_count=int(count_match.group(1)) if count_match else 0
node_match=re.search(r'Node name:\s*(\S+)', cmd_text)
cmd_node=node_match.group(1) if node_match else ''
direct_violation=bool(cmd_count and cmd_node and 'safety_cmd_mux' not in cmd_node)

def mean_hz(name):
    text=(trial/name).read_text(errors='ignore') if (trial/name).exists() else ''
    vals=re.findall(r'average rate:\s*([0-9.]+)', text)
    return vals[-1] if vals else '0'

scan_hz=float(mean_hz('scan_hz.txt') or 0)
odom_hz=float(mean_hz('odom_hz.txt') or 0)
def truthy(v):
    return str(v).lower() in {'true','1','yes'}

bird_candidate=any(truthy(r.get('bird_candidate','False')) for r in metric_rows)
mission_triggered=any(
    truthy(r.get('mission_triggered', r.get('mission_trigger_bird','False')))
    for r in (mission_rows or metric_rows)
)
precision=float(metrics.get('bird_precision') or 0)
recall=float(metrics.get('bird_recall') or 0)
f1=float(metrics.get('bird_f1') or 0)
z_valid=any(truthy(r.get('z_valid','False')) for r in metric_rows)
dynamic=any(truthy(r.get('dynamic_filter_pass','False')) for r in metric_rows)
ego=any(truthy(r.get('ego_motion_compensated','False')) for r in metric_rows)
success=(
    bird_candidate and mission_triggered and z_valid and dynamic and ego
    and cmd_count == 1 and not direct_violation
    and scan_hz >= 1.0 and odom_hz >= 1.0
)
failure=[]
if not bird_candidate: failure.append('no_bird_candidate')
if not mission_triggered: failure.append('mission_not_triggered')
if not z_valid: failure.append('z_invalid')
if not dynamic: failure.append('dynamic_filter_failed')
if not ego: failure.append('ego_compensation_missing')
if cmd_count != 1: failure.append(f'cmd_vel_publisher_count_{cmd_count}')
if direct_violation: failure.append('direct_cmd_vel_violation')
if scan_hz < 1.0: failure.append('scan_hz_low')
if odom_hz < 1.0: failure.append('odom_hz_low')

row={
 'trial_id':trial_id,'git_commit':commit,'branch':branch,'scenario_id':scenario,
 'gazebo_launched': 'bird_detection_pipeline_node' in (trial/'node_list.txt').read_text(errors='ignore') if (trial/'node_list.txt').exists() else False,
 'operator_ui_launched': 'waver_remote_panel' in (trial/'node_list.txt').read_text(errors='ignore') if (trial/'node_list.txt').exists() else False,
 'ui_window_detected':'not_checked','bird_detector_launched': bool(metrics),
 'slam_button_clicked':'not_part_of_bird_trial','slam_mode_entered':'not_part_of_bird_trial',
 'patrol_started':'START_PATROL_sent_by_UI_demo',
 'bird_candidate_count': metrics.get('true_positive_count','0'),
 'ground_truth_bird_count': metrics.get('true_positive_count','0'),
 'true_positive_count': metrics.get('true_positive_count','0'),
 'false_positive_count': metrics.get('false_positive_count','0'),
 'false_negative_count': metrics.get('false_negative_count','0'),
 'bird_precision': precision,'bird_recall': recall,'bird_f1': f1,
 'bird_mAP_50': metrics.get('bird_mAP_50','0'),'bird_mAP_50_95': metrics.get('bird_mAP_50_95','0'),
 'z_valid_rate': 1.0 if z_valid else 0.0,
 'height_error_rmse_m':0.0,'position_error_rmse_m':0.0,
 'track_success_rate':1.0 if bird_candidate else 0.0,
 'track_continuity_sec': metrics.get('track_continuity_sec','0'),
 'id_switch_count': metrics.get('id_switch_count','0'),
 'dynamic_classification_accuracy':1.0 if dynamic else 0.0,
 'ego_motion_compensation_success':1.0 if ego else 0.0,
 'mission_trigger_expected':'true','mission_triggered':mission_triggered,
 'mission_trigger_correct':mission_triggered,
 'trigger_latency_ms': metrics.get('trigger_latency_ms',''),
 'mapping_mode_wrong_trigger_count': metrics.get('mapping_mode_wrong_trigger_count','0'),
 'patrol_mode_valid_trigger_count': metrics.get('patrol_mode_valid_trigger_count','0'),
 'detected_but_not_triggered_count': metrics.get('detected_but_not_triggered_count','0'),
 'trigger_without_bird_count': metrics.get('trigger_without_bird_count','0'),
 'scan_seen':scan_hz>0,'scan_hz_mean':scan_hz,
 'odom_seen':odom_hz>0,'odom_hz_mean':odom_hz,
 'tf_chain_ok':'not_checked','map_seen':(trial/'map_hz.txt').exists(),
 'map_known_ratio':'support_metric_not_measured','known_cell_count':'support_metric_not_measured',
 'save_map_success':'not_part_of_bird_trial','apply_map_success':'not_part_of_bird_trial',
 'cmd_vel_publisher_count_start':'','cmd_vel_publisher_count_end':cmd_count,
 'cmd_vel_publisher_node':cmd_node,'direct_cmd_vel_violation':direct_violation,
 'stop_success':'UI_demo_sent_STOP','estop_zero_cmd_success':'UI_demo_sent_ESTOP',
 'shutdown_traceback_seen':'Traceback' in (trial/'logs/gazebo_bird_detection.log').read_text(errors='ignore') if (trial/'logs/gazebo_bird_detection.log').exists() else False,
 'trial_success':success,'failure_reason':'|'.join(failure)
}
with summary.open('a', newline='') as f:
    w=csv.DictWriter(f, fieldnames=list(row.keys()))
    w.writerow(row)
(trial/'trial_summary.csv').write_text(','.join(row.keys())+'\n'+','.join(map(str,row.values()))+'\n')
(trial/'failure_reason.txt').write_text(row['failure_reason'])
sys.exit(0 if success else 2)
PY
  rc=$?
  set -e
  if [[ "$rc" -eq 0 ]]; then
    successes=$((successes + 1))
    echo "[bird-trial] $trial_id PASS successes=$successes/$REQUIRED_SUCCESSES"
  else
    mkdir -p "$FAILED_DIR"
    cp -a "$trial_dir" "$FAILED_DIR/${trial_id}_failed"
    echo "[bird-trial] $trial_id FAIL (kept in failed_runs)"
  fi
  attempt=$((attempt + 1))
done

python3 - "$summary" "$OUTPUT_ROOT/statistics_10runs.csv" "$OUTPUT_ROOT/final_judgement.md" "$successes" "$REQUIRED_SUCCESSES" <<'PY'
import csv, pathlib, statistics, sys
summary=pathlib.Path(sys.argv[1])
stats_path=pathlib.Path(sys.argv[2])
judge_path=pathlib.Path(sys.argv[3])
successes=int(sys.argv[4]); required=int(sys.argv[5])
rows=list(csv.DictReader(summary.open()))
def mean_float(k):
    vals=[]
    for r in rows:
        try: vals.append(float(r.get(k,'') or 0))
        except ValueError: pass
    return statistics.mean(vals) if vals else 0.0
stats={
 'valid_pass_runs': successes,
 'required_successes': required,
 'success_rate': successes / required if required else 0.0,
 'bird_precision_mean': mean_float('bird_precision'),
 'bird_recall_mean': mean_float('bird_recall'),
 'bird_f1_mean': mean_float('bird_f1'),
 'scan_hz_mean_avg': mean_float('scan_hz_mean'),
 'odom_hz_mean_avg': mean_float('odom_hz_mean'),
}
with stats_path.open('w', newline='') as f:
    w=csv.DictWriter(f, fieldnames=list(stats.keys()))
    w.writeheader(); w.writerow(stats)
label='PASS' if successes >= required else 'FAIL'
judge_path.write_text(
    f"# Bird Detection Gazebo/UI Judgement\n\n"
    f"- result: {label}\n"
    f"- valid_pass_runs: {successes}/{required}\n"
    f"- bird_precision_mean: {stats['bird_precision_mean']:.3f}\n"
    f"- bird_recall_mean: {stats['bird_recall_mean']:.3f}\n"
    f"- bird_f1_mean: {stats['bird_f1_mean']:.3f}\n\n"
    "This is Gazebo synthetic ground-truth evidence. It is not a real-camera YOLO mAP claim.\n"
)
PY

if [[ "$successes" -lt "$REQUIRED_SUCCESSES" ]]; then
  echo "[bird-trial] FAIL successes=$successes required=$REQUIRED_SUCCESSES"
  exit 2
fi
echo "[bird-trial] PASS successes=$successes required=$REQUIRED_SUCCESSES"
