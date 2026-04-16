cmake_minimum_required(VERSION 3.5)

project(ugv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Livox CSV 경로 자동 설정 ──────────────────────────────
# colcon build 시 현재 PC의 install 경로를 자동으로 계산하여
# model.sdf의 @LIVOX_CSV_PATH@ 를 실제 경로로 치환합니다.
# Docker든 로컬이든 빌드만 하면 자동으로 맞는 경로가 들어갑니다.
set(LIVOX_CSV_PATH "${CMAKE_INSTALL_PREFIX}/../ros2_livox_simulation/share/ros2_livox_simulation/scan_mode/mid360.csv")
get_filename_component(LIVOX_CSV_PATH "${LIVOX_CSV_PATH}" ABSOLUTE)

# model.sdf 파일의 @LIVOX_CSV_PATH@ 플레이스홀더를 실제 경로로 치환
file(GLOB MODEL_SDF_FILES "${CMAKE_CURRENT_SOURCE_DIR}/models/*/model.sdf")
foreach(SDF_FILE ${MODEL_SDF_FILES})
    # model.sdf가 있는 상대 경로 계산 (예: models/rasp_rover/model.sdf)
    file(RELATIVE_PATH SDF_REL_PATH "${CMAKE_CURRENT_SOURCE_DIR}" "${SDF_FILE}")
    # 빌드 디렉토리에 치환된 파일 생성
    configure_file("${SDF_FILE}" "${CMAKE_CURRENT_BINARY_DIR}/${SDF_REL_PATH}" @ONLY)
endforeach()

# 치환된 models를 install (빌드 디렉토리에서)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/models/"
    DESTINATION share/${PROJECT_NAME}/models
)

# 나머지 디렉토리는 소스에서 그대로 install
install(DIRECTORY rviz launch urdf worlds param maps
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(ament_cmake_python)
ament_export_dependencies(rosidl_default_generators)

ament_package()
