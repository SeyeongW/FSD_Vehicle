# Field Troubleshooting

## Backend Starts But UI Does Not Move Waver

Check the backend contract on the Jetson Docker side:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/manual_cmd_vel
ros2 topic echo --once /waver/safety_state
ros2 topic echo --once --full-length /waver/base_driver_state
```

Expected:

```text
/cmd_vel publisher count = 1
/cmd_vel publisher = safety_cmd_mux_node
/cmd_vel subscriber = waver_base_driver_node
/waver/manual_cmd_vel subscriber = safety_cmd_mux_node
```

If the local UI prints `remote bridge connection failed`, fix SSH first:

```bash
ssh sw@10.139.225.150
```

If password SSH is used, install `sshpass` or switch to SSH keys:

```bash
sudo apt install -y sshpass
ssh-copy-id sw@10.139.225.150
```

## Docker Container Missing

On Jetson:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash docker/run.sh jetson-up
docker ps
```

The container name should be `fsd_dev_jetson` unless overridden in env.

## Serial Not Found

Do not use an ambiguous serial port for wheel-on testing.

```bash
ls -l /dev/serial/by-id/
fuser -v /dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_*
```

Only `waver_base_driver_node` should own the Waver command port.

## Safety Stops

For supervised USB-only micro-patrol, `SCAN_DISABLED_TEST_ONLY` may appear when
the LiDAR is not connected. For real LiDAR/Nav2 operation, scan and odometry
must be live before wheel-on.

Wheel-on rules:

```text
- physical E-stop available
- Waver lifted for first direction test
- /cmd_vel owner is safety_cmd_mux_node only
- battery voltage sane
- serial owner count <= 1
- scan and odom live for autonomous operation
```

