# Jetson Setup

The Jetson runs the Waver backend inside Docker. The local PC starts it through
SSH, so the Jetson must be reachable from the same hotspot network.

## Network

```bash
ssh sw@10.139.225.150
```

If the IP changed, edit `config/waver_field_env.local` on the local PC:

```bash
JETSON_HOST=<current_jetson_ip>
JETSON_USER=sw
JETSON_WS=/home/sw/ros2_ws5/FSD_Vehicle
```

## Docker Bootstrap

From the local PC:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_bootstrap_jetson.sh
```

This command syncs the repository to the Jetson, starts
`fsd_dev_jetson`, and builds the ROS overlay in Docker when
`install_docker/setup.bash` is missing.

## Waver USB Serial

Connect Waver USB serial to the Jetson. The backend prefers a stable by-id path:

```bash
ls -l /dev/serial/by-id/
```

The field backend refuses ambiguous serial ownership. Do not run another serial
bridge, `ugv_driver`, or feedback process on the same port.

## Livox Mid-360

The real LiDAR backend is prepared by:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_setup_livox_mid360_docker.sh
```

Then run:

```bash
START_LIVOX_DRIVER=true bash scripts/waver_field_lidar_nav_backend_start.sh
```

Check the actual Mid-360 IP and broadcast code before wheel-on testing.

