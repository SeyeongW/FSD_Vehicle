# 1. apriltag 디렉토리로 이동
cd /home/seyeongw/ros2_ws/ugv_ws/src/ugv_else/apriltag_ros/apriltag/

# 2. 빌드 설정 (기존 build 폴더가 있다면 그대로 진행)
cmake -B build -DCMAKE_BUILD_TYPE=Release

# 3. 설치 (여기서 sudo가 반드시 필요합니다)
sudo cmake --build build --target install

# 4. 워크스페이스 루트로 복귀
cd /home/seyeongw/ros2_ws/ugv_ws/