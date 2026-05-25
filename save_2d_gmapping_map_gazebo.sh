#!/bin/bash
WS="${UGV_WS_PATH:?Error: UGV_WS_PATH 환경변수를 설정해주세요. source setup_local_env.sh 를 먼저 실행하세요.}"
cd "${WS}/src/ugv_main/ugv_gazebo/maps"
ros2 run nav2_map_server map_saver_cli -f ./map
cd -
