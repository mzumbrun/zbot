# Dev environment

Two laptops, set up identically: **Victus** (RTX 4060) and **Upc** (CPU only). Both run
Ubuntu 24.04 + ROS 2 Jazzy.

- Repo: `mzumbrun/zbot` at `~/zbot_ws/src/zbot` (colcon workspace `~/zbot_ws`)
- LeRobot 0.6.2, editable from the fork `mzumbrun/lerobot` at `~/lerobot`
- venv: `~/lerobot-venv`

---

## Every terminal: pick the right environment

| Running | Source first |
|---|---|
| LeRobot scripts (`arm_lab/*.py`, `lerobot-calibrate`) | `lr` |
| Pure ROS (RViz, `ros2 topic/service …`) | `ros && source ~/zbot_ws/install/setup.bash` |
| The bridge (needs both) | `lr && ros && source ~/zbot_ws/install/setup.bash` (in that order) |

ROS is **only** sourced via `ros`, never in `.bashrc`, so a fresh terminal has neither.

## After pulling or editing code

```
cd ~/zbot_ws
colcon build                                 # or --packages-select <pkg>
source install/setup.bash                    # in any terminal already open
```

New ROS package dependencies go in that package's `package.xml` (e.g. `std_srvs` for zbot_arm).

## When switching laptops

1. On the machine you're leaving: commit and push (`git status` should be clean).
2. On the other: `git pull`, then `colcon build`.
3. **Things not in git:** the LeRobot calibration file
   (`~/.cache/huggingface/lerobot/calibration/robots/so_follower/zbot_follower_1.json`) and
   anything changed in the venv or `~/lerobot`. Copy the calibration with `scp` if you
   recalibrated (see `so101_arm.md`). Push changes to the LeRobot fork and pull them on the
   other machine.

## When the arm's USB port isn't `/dev/so101_follower`

The udev rule creates the name. If it's missing: replug, run `ls -l /dev/so101_follower`, and
check `/etc/udev/rules.d/` and `sudo udevadm control --reload-rules && sudo udevadm trigger`.

*Last updated: 2026-10-25*
