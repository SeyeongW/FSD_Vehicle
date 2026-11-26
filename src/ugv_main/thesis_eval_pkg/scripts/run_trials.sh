#!/bin/bash
# 졸업논문 단일/다중 LiDAR 비교 배치 실험.
# 각 트라이얼: 시뮬 정리 → thesis_bringup 실행(gui:=true — 헤드리스면 Livox 플러그인이 안 켜지는 버그)
#  → bird_in → 새 등장 후 sim 시간으로 (FAULT_DELAY + POST_FAULT)초 지나면 종료.
#
# 사용: bash run_trials.sh                 (기본 조건 전부)
#       CONDS="multi:lidar1" SEEDS="1" bash run_trials.sh
#  조건 형식 mode:fault  (fault = none | mid360 | lidar1 | lidar2)
#  추격 조향: STEERING=smc(기본) | pure_pursuit | pn  → 결과는 logs/thesis_<STEERING>/ 에 따로 저장

WS=~/ros2_ws/ugv_ws
CONDS=${CONDS:-"single:mid360 multi:mid360 multi:lidar1 multi:lidar2"}
SEEDS=${SEEDS:-"1 2 3"}
FAULT_DELAY=${FAULT_DELAY:-60}
POST_FAULT=${POST_FAULT:-60}
BOOT_WAIT=${BOOT_WAIT:-25}
STEERING=${STEERING:-smc}
LOG_DIR=$WS/logs/thesis_$STEERING
mkdir -p "$LOG_DIR"
RUNLOG=$LOG_DIR/_batch_run.log

source /opt/ros/humble/setup.bash
source $WS/install/setup.bash
export UGV_MODEL=ugv_rover

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$RUNLOG"; }
END=$((FAULT_DELAY + POST_FAULT))

# 융합 노드가 기대 센서 수(multi=3, single=1)를 모두 살아있다고 보고할 때까지 대기.
# Livox 플러그인이 가끔 수십 초 늦게 켜져서(정상 구간 전체가 탐지 0으로 기록된 사례) 필요함.
wait_sensors() {
  local want=$1 limit=$((SECONDS + 120))
  while [ $SECONDS -lt $limit ]; do
    n=$(timeout 4 ros2 topic echo --once --full-length /thesis/fusion_stats std_msgs/msg/String 2>/dev/null \
        | grep -o 'n_alive[^,]*' | grep -o '[0-9]*$')
    [ "$n" = "$want" ] && return 0
    sleep 2
  done
  return 1
}

for seed in $SEEDS; do
  for cond in $CONDS; do
    mode=${cond%%:*}; fault=${cond##*:}
    want=3; [ "$mode" = single ] && want=1
    for attempt in 1 2 3; do
      log "=== mode=$mode fault=$fault seed=$seed 시작 (시도 $attempt) ==="
      bash $WS/kill_sim.sh >> "$RUNLOG" 2>&1
      ros2 launch thesis_eval_pkg thesis_bringup.launch.py \
        mode:=$mode fault:=$fault seed:=$seed fault_delay:=$FAULT_DELAY gui:=true \
        steering_mode:=$STEERING out_dir:=$LOG_DIR \
        > "$LOG_DIR/_launch_${mode}_${fault}_s${seed}.log" 2>&1 &
      lpid=$!
      log "launch PID=$lpid, ${BOOT_WAIT}s 기동 대기"
      sleep $BOOT_WAIT
      wait_sensors $want && break
      log "센서 $want개가 120s 안에 안 켜짐 — 재시작"
      bash $WS/kill_sim.sh >> "$RUNLOG" 2>&1
      rm -f $(ls -t "$LOG_DIR"/thesis_${mode}_${fault}_s${seed}_*.csv 2>/dev/null | head -1)
      sleep 3
    done
    log "센서 준비 완료"
    ros2 topic pub --once /bird_command std_msgs/String "data: bird_in" > /dev/null 2>&1
    log "bird_in 발행, 새 등장 후 sim ${END}s까지 대기"

    csv=""
    deadline=$((SECONDS + END * 4 + 120))   # RTF가 낮아도 버티도록 여유
    while [ $SECONDS -lt $deadline ]; do
      sleep 5
      csv=$(ls -t "$LOG_DIR"/thesis_${mode}_${fault}_s${seed}_*.csv 2>/dev/null | head -1)
      [ -z "$csv" ] && continue
      bt=$(tail -1 "$csv" | cut -d, -f2)
      [[ "$bt" =~ ^[0-9.]+$ ]] && awk "BEGIN{exit !($bt >= $END)}" && break
    done
    log "종료: $csv (마지막 bird_t=${bt:-?})"
    bash $WS/kill_sim.sh >> "$RUNLOG" 2>&1
    sleep 3
  done
done
log "배치 완료"
