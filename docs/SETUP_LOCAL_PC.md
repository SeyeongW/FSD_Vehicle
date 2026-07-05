# Local PC Setup

Run these commands once on the operator laptop or desktop.

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_setup_local_pc.sh --check
bash scripts/waver_setup_local_pc.sh --install-minimal-ui
```

The setup script checks common local dependencies and suggests packages when
something is missing. If you prefer manual installation:

```bash
sudo apt update
sudo apt install -y \
  openssh-client rsync sshpass python3-pip python3-tk \
  python3-colcon-common-extensions python3-rosdep

python3 -m pip install --user -r requirements-local-ui.txt
```

Use SSH keys when possible:

```bash
ssh-copy-id sw@10.139.225.150
ssh sw@10.139.225.150
```

If you cannot use SSH keys, the first backend/UI start will prompt once and
create the ignored local override automatically. To create it manually:

```bash
cd ~/ros2_ws5/FSD_Vehicle
cp config/waver_field_env.local.example config/waver_field_env.local
nano config/waver_field_env.local
```

Only `config/waver_field_env.local` may contain `JETSON_PASS`. Do not commit it.

Alternative home-file handoff:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_create_home_field_env.sh 10.139.225.150 sw /home/sw/ros2_ws5/FSD_Vehicle
```

This creates `~/.waver_field_env`, which is read automatically by all field
scripts and can contain the Jetson password.

Before field operation:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_doctor.sh
```

Open the approved local operator station after the Jetson backend is running:

```bash
cd ~/ros2_ws5/FSD_Vehicle
bash scripts/waver_field_operator_station_start.sh
```

See `docs/LOCAL_OPERATOR_STATION.md` for the local PC / Jetson Docker boundary.
