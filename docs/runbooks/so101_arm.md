# SO-101 follower arm

Arm: `zbot_follower_1` on `/dev/so101_follower`, 12 V. Driver: `zbot_arm/so101_bridge.py`.
Params: `zbot_arm/config/so101_follower.yaml` (written by `arm_lab/post_calibrate.py`).

**Rule:** only one program can hold the serial port. Stop the bridge before running
`step_joints.py`, `post_calibrate.py`, or `lerobot-calibrate`, and vice versa.

---

## Every session: start the arm in ROS

1. 12 V on. USB plugged in. Check the port exists: `ls -l /dev/so101_follower`
2. **Terminal A — bridge** (starts limp, publishes `/joint_states`):
   ```
   lr && ros && source ~/zbot_ws/install/setup.bash
   cd ~/zbot_ws/src/zbot
   python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/so101_follower.yaml
   ```
   Check the startup log: no ERROR about the calibration stamp, and the soft limits look sane.
3. **Terminal B — RViz** (optional):
   ```
   ros && source ~/zbot_ws/install/setup.bash
   ros2 launch zbot_description display_so101.launch.py gui:=false
   ```
   Move the arm by hand; the model should follow.

## Every time: enable and command

1. Hand-pose the arm near mid-range. Enabling holds wherever it is.
2. Enable:
   ```
   ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
3. Command in **URDF radians** (partial lists OK). Use a real number, not a placeholder:
   ```
   ros2 topic echo --once /joint_states        # current positions
   ros2 topic pub --once /so101_bridge/joint_command sensor_msgs/msg/JointState \
     "{name: [elbow_flex], position: [1.35]}"
   ```
   Targets outside the soft limits are clamped (a warning is logged). Motion is slewed at 30°/s.

## Every session end: stop safely

- **Normal:** `ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: false}"`
  → the arm glides to the rest pose, pauses ~1 s, goes limp. Then Ctrl-C the bridge.
- **Ctrl-C while enabled** also goes to rest and releases.
- **If there is no rest pose, or the arm is faulted:** torque is left ON, holding. Support the
  arm, then cut 12 V or run `python arm_lab/step_joints.py --read` (goes limp).

## After every `lerobot-calibrate`

Calibration moves each joint's zero, so the wrist_roll offset and rest pose go stale. Soft
limits update themselves.

1. Bridge stopped.
2. **Terminal A:** `ros && ros2 launch zbot_description display_so101.launch.py` (sliders at 0).
3. **Terminal B:**
   ```
   lr && cd ~/zbot_ws/src/zbot
   python arm_lab/post_calibrate.py
   ```
   - Check the printed limits look like each joint's real swept range.
   - **[a]** Turn wrist_roll to match the model → Enter.
   - **[b]** Fold the arm to rest → Enter.
4. Commit the result:
   ```
   git add zbot_arm/config/so101_follower.yaml
   git commit -m "Recalibrate zbot_follower_1"
   git push
   ```
5. **Copy the calibration file to the other laptop.** It lives outside git, so the other machine
   still has the old one, and its bridge will flag a stamp mismatch:
   ```
   scp ~/.cache/huggingface/lerobot/calibration/robots/so_follower/zbot_follower_1.json \
       <other-laptop>:~/.cache/huggingface/lerobot/calibration/robots/so_follower/
   ```
   On the other laptop, `git pull` too.
6. If anything other than wrist_roll looks off from the model in RViz, measure its offset the
   same way (reading R → offset −R) and edit `offsets_deg` in the YAML.

## After a FAULT

The log says `FAULT: <joint> tracking error …` or `bus reads failing`. The arm holds where it
is and ignores commands.

1. Look for the cause: obstruction, collision, cable snag, or a target the joint couldn't
   reach under load.
2. Clear it, hand-support if needed, then re-enable (this resets the fault and holds the current
   pose):
   ```
   ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
3. Repeated `bus reads failing`: check 12 V, the USB cable, and that nothing else has the port.

## After changing bridge code or params

- Edited `zbot_arm/*.py`: `cd ~/zbot_ws && colcon build --packages-select zbot_arm`
  (skip if the workspace was built with `--symlink-install`).
- Changed P gain: retest with `step_joints.py --p <n>` and record the result in
  `docs/arm_tuning.md` (current: P = 32; P = 40 makes shoulder_pan oscillate).

## Quick fixes

| Symptom | Fix |
|---|---|
| `Failed to populate field: must be real number, not str` | Replace the placeholder with a number |
| ERROR: calibration changed since the params file was made | Run `post_calibrate.py` (or copy the calibration file from the other laptop) |
| `Calibration … doesn't match the motors` | Run `lerobot-calibrate`, then the post-calibration routine |
| `Command ignored: torque off, faulted, or releasing` | Call `enable true` |
| Port busy / can't open | Another script or bridge is running; stop it |
| `std_srvs` not found | `sudo apt install ros-jazzy-std-srvs` |

*Last updated: 2026-10-25*
