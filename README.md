# FSD_Vehicle Project
ROS2-based integrated development environment for UGV autonomous driving research, supporting both x86_64 (PC) and ARM64 (Jetson) architectures.

---

## 1. Prerequisites
- OS: Ubuntu 22.04 LTS (Recommended)
- Docker Engine
- Docker Compose v2.0+

---

## 2. Setup and Execution

### 2.1 Repository Cloning
Create a workspace and clone the repository.
```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/SeyeongW/FSD_Vehicle.git ugv_ws
cd ugv_ws
```

### 2.2 Docker Image Build
Build the image according to the target architecture.
```bash
# PC (Simulation/Development)
bash docker/run.sh build-pc

# Jetson Orin Nano (Deployment)
bash docker/run.sh build-jetson
```

### 2.3 Container Execution
```bash
# PC Development
bash docker/run.sh pc

# Jetson Deployment
bash docker/run.sh jetson
```
Note: Hardware connection warnings in the `run.sh` script can be ignored for simulation-only tasks.

### 2.4 Initial Workspace Build
The following scripts must be executed within the container for the first-time setup.
```bash
# Build Apriltag library
bash build_apriltag.sh

# Build all ROS2 packages
bash build_first.sh
```

---

## 3. Development Workflow

For incremental builds after source code modifications:
```bash
# Incremental build
bash build_common.sh

# Environment sourcing
source install/setup.bash
```

---

## 4. Launching Applications

### 4.1 Gazebo Simulation (PC)
```bash
ros2 launch ugv_gazebo slam_nav.launch.py
```

### 4.2 Physical Hardware Bringup (Jetson/Robot)
```bash
ros2 launch ugv_bringup bringup_lidar.launch.py
```

---

## 5. Project Structure
- `src/ugv_main`: Core logic, control, and perception source code.
- `src/ugv_else`: 3rd party libraries and sensor drivers (Livox SDK2, LDLiDAR, etc.).
- `docker/`: Dockerfiles and system configuration files.
- `.env`: Global project configuration (Image names, serial paths, etc.).

---

## 6. Technical Notes
- **GUI Support**: Automated X11 forwarding is configured in `run.sh` for RViz and Gazebo.
- **Volume Mapping**: Current directory is mounted to `/ros2_ws/ugv_ws` inside the container.
- **Hardware Access**: Correct serial and audio group permissions are pre-configured in the Docker images.