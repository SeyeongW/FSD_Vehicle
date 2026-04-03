# FSD_Vehicle Project
ROS2 Humble-based integrated development environment for UGV (Unmanned Ground Vehicle) autonomous driving research. This project provides a unified workspace supporting both PC-based high-fidelity simulations (Gazebo/RViz) and physical deployment on Jetson Orin Nano hardware.

---

## 1. Prerequisites and Environment Setup
Before proceeding with the installation, ensure the following system dependencies are met:
- **Operating System**: Ubuntu 22.04 LTS (Jammy Jellyfish) is highly recommended for native ROS2 Humble compatibility.
- **Docker Engine**: Required for environment isolation.
- **Docker Compose v2.0+**: Utilized for managing multi-container configurations and environment variable substitution.

---

## 2. Installation and Initial Configuration

### 2.1 Repository Cloning
Initialize the workspace by cloning the repository into your preferred ROS2 workspace directory.
```bash
# Recommended directory structure: ~/ros2_ws/ugv_ws
mkdir -p ~/ros2_ws && cd ~/ros2_ws
git clone https://github.com/SeyeongW/FSD_Vehicle.git ugv_ws
cd ugv_ws
```

### 2.2 Docker Image Reconstruction
Build the environment images tailored for the target architecture. This process includes installing pinned versions of libraries such as OpenCV, PyTorch, and Livox SDK2.
```bash
# For PC-based development (x86_64)
bash docker/run.sh build-pc

# For Jetson-based physical deployment (ARM64)
bash docker/run.sh build-jetson
```

### 2.3 Container Execution and Hardware Verification
Execute the environment using the unified entry script.
```bash
# Launch PC environment
bash docker/run.sh pc

# Launch Jetson environment
bash docker/run.sh jetson
```
Note: The `run.sh` script automatically verifies the connectivity of pre-configured serial ports (e.g., `/dev/ttyTHS1`). If hardware is not connected, a warning will be displayed; however, the container will still initialize for simulation purposes.

---

## 3. Workspace Initialization (Inside Container)
Upon first container entry, binary libraries and core ROS2 packages must be compiled. 

### 3.1 External Library Compilation
```bash
# Compile AprilTag detection library (C-based dependency)
bash build_apriltag.sh
```

### 3.2 Full Workspace Compilation
Execute the primary build script to resolve dependencies and compile all internal packages using `colcon`.
```bash
bash build_first.sh
```

---

## 4. Development and Operational Workflow

### 4.1 Incremental Builds
For frequent source code updates, use the common build script to minimize compilation time.
```bash
bash build_common.sh
source install/setup.bash
```

### 4.2 Simulation (Gazebo/RViz)
Launch the full SLAM and Navigation stack in the virtual Gazebo environment.
```bash
ros2 launch ugv_gazebo slam_nav.launch.py
```

### 4.3 Hardware Bringup
Execute the core driver nodes to initialize LiDAR sensors and motor controllers.
```bash
ros2 launch ugv_bringup bringup_lidar.launch.py
```

---

## 5. Technical Architecture and Configuration

- **Workspace Path Mapping**: The host's current directory is bi-directionally mounted to `/ros2_ws/ugv_ws`, enabling live code editing via host-side IDEs.
- **Hardware Permissions**: Root access to `dialout`, `audio`, and `video` groups is pre-configured to handle serial MCU communication and sensor data acquisition.
- **Environment Variables**: Key parameters such as `ROS_DOMAIN_ID`, `SERIAL_PORT`, and `UGV_MODEL` are managed via the `.env` file located in the project root.
- **Project Structure**:
    - `src/ugv_main`: Primary development area for FSD-specific algorithms and control logic.
    - `src/ugv_else`: Integration layer for 3rd party drivers (Livox, Realsense, LDLiDAR).

---

**Maintainer**: SeyeongW  
For technical inquiries or bug reports, please refer to the project's Issue tracker.