# Waver Clone-To-Field Quickstart

This repository is prepared so a fresh clone can be brought to the Waver field
control path with repository-owned non-secret defaults.

## Topology

```text
Local PC code / remote panel
  -> SSH
  -> Jetson host
  -> Docker container fsd_dev_jetson
  -> ROS 2 nodes
  -> safety_cmd_mux_node
  -> /cmd_vel
  -> waver_base_driver_node
  -> Waver USB serial controller
```

`safety_cmd_mux_node` remains the final `/cmd_vel` owner. The UI, mission,
tracking, and Nav2 nodes must not publish final `/cmd_vel` directly.

## First-Time Setup

```bash
git clone <REPO_URL> ~/ros2_ws5/FSD_Vehicle
cd ~/ros2_ws5/FSD_Vehicle

bash scripts/waver_setup_local_pc.sh
bash scripts/waver_doctor.sh
```

Edit `config/waver_field_env` for shared non-secret defaults, or create the
ignored local override:

```bash
cp config/waver_field_env.local.example config/waver_field_env.local
nano config/waver_field_env.local
```

Passwords and private keys stay out of git. On first field start, the scripts
will ask for the Jetson SSH password once and save it to the ignored local file
`config/waver_field_env.local`. After that, the two normal run commands are
enough.

If you prefer one home-level file that can be handed to a new operator outside
the repository, use:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_create_home_field_env.sh 10.139.225.150 sw /home/sw/ros2_ws5/FSD_Vehicle
```

This writes `~/.waver_field_env` with file mode `600`. The package always reads
this file as a legacy/local override after the repository defaults.

SSH keys are still preferred when available:

```bash
ssh-copy-id sw@10.139.225.150
```

If password SSH is unavoidable and you do not want the first-run prompt, put
`WAVER_ALLOW_PASSWORD_SSH=1` and `JETSON_PASS=...` only in
`config/waver_field_env.local`.

## Jetson Bootstrap

With the Jetson and local PC on the same hotspot:

```bash
ssh sw@10.139.225.150
```

Then from the local PC:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_bootstrap_jetson.sh
```

This syncs the source tree to the Jetson workspace, starts the Docker container,
and builds the selected ROS overlay inside Docker when needed.

## Normal Field Start

After bootstrap, the normal field workflow is:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_docker_backend_start.sh
```

Open a second local PC terminal:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_local_ui_start.sh
```

If the Jetson workspace has not been bootstrapped yet, the backend script now
runs bootstrap automatically before starting Docker backend nodes.

Equivalent wrapper names are also available:

```bash
bash scripts/waver_start_field_backend.sh
bash scripts/waver_start_local_ui.sh
```

## Quickstart Helper

For a new operator:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_quickstart_field.sh \
  --jetson-host 10.139.225.150 \
  --jetson-user sw
```

Then run the backend and UI commands above.

## Validation

```bash
bash scripts/waver_clone_to_run_acceptance.sh
bash scripts/waver_doctor.sh --jetson
bash scripts/waver_docker_env_check.sh
```

Expected field backend contracts:

```bash
ros2 topic info -v /cmd_vel
ros2 topic info -v /waver/manual_cmd_vel
ros2 topic echo --once /waver/safety_state
ros2 topic echo --once /waver/base_driver_state
```

`/cmd_vel` must have exactly one publisher: `safety_cmd_mux_node`.
