# Zbot

Autonomous diff-drive cart with SO-101 arms. This repo lives at `~/zbot_ws/src/zbot` on every machine.

## Layout

| Folder | What | Built by |
| --- | --- | --- |
| `zbot_description` | URDF/xacro, meshes | colcon |
| `zbot_msgs` | custom messages/services | colcon |
| `zbot_base` | diff drive: RoboClaw, odometry, IMU | colcon |
| `zbot_arm` | SO-101 to ROS 2 bridge | colcon |
| `zbot_nav` | Nav2 config, maps | colcon |
| `zbot_perception` | cameras, ToF | colcon |
| `zbot_bringup` | per-machine launch + config | colcon |
| `arm_lab/` | LeRobot scripts (`lr` venv) | not built |
| `sim/` | MuJoCo models/scripts (`lr` venv) | not built |
| `firmware/` | ESP32 / micro-ROS (PlatformIO) | not built |
| `system/` | udev rules, systemd units, `setup.sh` | run by hand |

## New machine

```bash
mkdir -p ~/zbot_ws/src && cd ~/zbot_ws/src
git clone https://github.com/mzumbrun/zbot.git
./zbot/system/setup.sh
cd ~/zbot_ws && ros && colcon build --symlink-install
```

On a Pi, build only what it runs, e.g. `colcon build --packages-up-to zbot_base zbot_bringup`.

## Two environments, two terminals

- `ros` terminal: colcon, ROS 2 nodes, launch files.
- `lr` terminal: LeRobot, `arm_lab/`, `sim/`.

Not in git: calibration (`~/.cache/huggingface/lerobot/calibration/`), datasets and policies (HF Hub), rosbags.
