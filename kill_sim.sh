#!/bin/bash
# hackathon_bringup.launch.py(및 관련 노드) 전체를 안전하게 정리한다.
# SIGINT로 먼저 곱게 종료를 시도하고, 몇 초 기다려도 안 죽으면 SIGKILL로
# 마무리한 뒤 /dev/shm에 남은 FastDDS 락 파일까지 정리한다.
#
# 순서(SIGKILL부터 바로 쏘지 않는 이유): ros2 launch를 SIGKILL로 죽이면
# 그 시점에 자식 프로세스(gzserver, robot_state_publisher, static_transform_publisher
# 등)한테는 종료 신호가 아예 전파되지 않아 orphan으로 남는다. SIGINT를 줘서
# launch 자체가 정상적으로 자식들을 정리할 기회를 먼저 준다.

PATTERN='gzserver|gzclient|rviz2|ros2 launch ugv_gazebo|ros2 launch thesis_eval_pkg|ft_fusion_node|eval_logger_node|lidar_fusion_node|bird_detector_node|bird_pursuit_node|patrol_node|pointcloud_to_laserscan_node|nav2_amcl|nav2_map_server|nav2_lifecycle_manager|bird_manager\.py|gimbal_bird_tracker|robot_state_publisher|static_transform_publisher|joint_state_publisher|bird_input_node|robot_status_monitor|trajectory_viz_node|data_logger_node'

pids() { ps aux | grep -E "$PATTERN" | grep -v grep | awk '{print $2}'; }

live=$(pids)
if [ -z "$live" ]; then
    echo "정리할 프로세스 없음."
else
    echo "SIGINT로 정상 종료 시도: $live"
    for p in $live; do kill -INT "$p" 2>/dev/null; done
    for i in 1 2 3 4 5; do
        sleep 1
        live=$(pids)
        [ -z "$live" ] && break
    done
fi

live=$(pids)
if [ -n "$live" ]; then
    echo "아직 안 죽은 프로세스 SIGKILL로 정리: $live"
    for p in $live; do kill -9 "$p" 2>/dev/null; done
    sleep 1
fi

echo "포트 11345:"
ss -ltnp 2>/dev/null | grep 11345 || echo "  (비어있음)"

removed=0
cd /dev/shm 2>/dev/null && for f in fastrtps_* sem.fastrtps_*; do
    [ -e "$f" ] || continue
    if ! fuser "$f" >/dev/null 2>&1; then
        rm -f "$f"
        removed=$((removed + 1))
    fi
done
echo "/dev/shm fastrtps 파일 중 사용 안 하는 것만 $removed 개 정리 (다른 세션이 쓰는 건 안 건드림)."
echo "done."
