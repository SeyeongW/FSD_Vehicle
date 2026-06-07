# ROS2 WS3 Experiment Data Archive

This workspace was compacted to keep real-vehicle and Gazebo source files usable while moving heavy experiment artifacts out of the source tree.

Moved full raw experiment data:

`/home/chotaehyun/waver_archives/ros2_ws3_20260607_232254/experiments_result_full`

Archive inventory:

`/home/chotaehyun/waver_archives/ros2_ws3_20260607_232254/experiments_result_file_manifest.tsv`

The workspace keeps lightweight paper-data tables and metadata under:

`experiments_result_compact/`

What was removed from the workspace:

- `build/`
- `install/`
- `log/`
- Python `__pycache__/` and `*.pyc`

These are generated artifacts and can be recreated with:

```bash
cd ~/ros2_ws3/FSD_Vehicle
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

To restore the full raw experiment directory into the workspace:

```bash
cd ~/ros2_ws3/FSD_Vehicle
cp -a /home/chotaehyun/waver_archives/ros2_ws3_20260607_232254/experiments_result_full ./experiments_result
```
