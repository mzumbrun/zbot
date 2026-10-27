# SO-101 follower arm

Arms: `zbot_follower_1` on `/dev/so101_follower`, `zbot_follower_2` on `/dev/so101_follower_2`,
12 V. Driver: `zbot_arm/so101_bridge.py`. Params: `zbot_arm/config/<ID>.yaml`, one file per arm
named after its ID (written by `arm_lab/post_calibrate.py`). Commands below use arm 1; for
another arm use its ID, port and params file.

**Rules**
- Only one program can hold a serial port. Stop the bridge before running `step_joints.py`,
  `post_calibrate.py`, `servo_regs.py`, or `lerobot-calibrate`, and vice versa.
- Always use the udev names (`/dev/so101_follower*`), never `/dev/ttyACM*`: those numbers
  swap with plug order.
- IDs are `zbot_…` (arm and calibration file); ports are `/dev/so101_…`. Don't mix them up:
  the wrong `--id` gives "Calibration … doesn't match the motors".

---

## New arm, or a replaced servo: servo IDs and udev

1. **Unplug every other arm's USB**, so setup can't renumber the wrong arm.
2. 12 V and USB to the new arm's board. Find the port: `lr && lerobot-find-port`.
3. Set IDs, **one servo on the board at a time** (every new servo is ID 1; two chained
   servos look like a missing one):
   ```
   lerobot-setup-motors --robot.type=so101_follower --robot.port=<PORT>
   ```
   Replacing one servo: run it anyway and press through the others; it rewrites the same IDs.
4. New board only: get its serial (`udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`),
   add a line to the repo's `99-so101.rules`, run `./system/setup.sh`, commit, and run
   `setup.sh` on the other laptop after pulling. Check: `ls -l /dev/so101_follower*`.
5. Then **calibrate** (next section) and run the checklist from check 2.

A replaced servo has a new encoder zero, so the **whole arm** must be recalibrated.

## Every session: start the arm in ROS

1. 12 V on. USB plugged in. Check the port exists: `ls -l /dev/so101_follower`
2. **Terminal A — bridge** (starts limp, publishes `/joint_states`):
   ```
   lr && ros && source ~/zbot_ws/install/setup.bash
   cd ~/zbot_ws/src/zbot
   python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/zbot_follower_1.yaml
   ```
   Check the startup log: no ERROR about the calibration stamp, **no warnings** (a
   `rest_pose outside soft limits` warning means the calibration sweep was incomplete), soft
   limits sane, and the `torque_limit` line.
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
   Enable is refused (torque stays off) if a joint is outside its calibrated range or a servo's
   goal doesn't verify by read-back. See Quick fixes.
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

## Every `lerobot-calibrate`

```
lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<ID>
```

- Sweep **every joint stop to stop**: elbow fully folded and fully open, shoulder_lift its whole
  travel, **gripper fully open and fully closed**.
- Check the printed table: each body joint should span ~2,200–2,700 counts. A small range
  (e.g. gripper 1843–2047) means that joint wasn't swept; redo it.
- Verify, torque off: `python arm_lab/step_joints.py --read --id <ID> --port <PORT>` and sweep
  each joint. Readings change smoothly, never jump or flip sign.

## After every `lerobot-calibrate`

Calibration moves each joint's zero, so the wrist_roll offset and rest pose go stale. Soft
limits update themselves.

1. Bridge stopped.
2. **Terminal A:** `ros && ros2 launch zbot_description display_so101.launch.py` (sliders at 0).
3. **Terminal B:**
   ```
   lr && cd ~/zbot_ws/src/zbot
   python arm_lab/post_calibrate.py            # other arms: --id <ID> --port <PORT>
   ```
   - Check the printed limits look like each joint's real swept range.
   - **[a]** Turn wrist_roll to match the model → Enter.
   - **[b]** Fold the arm to rest → Enter. If it reports **"Outside the soft limits"**, press
     **r** and re-pose; don't accept. If the folded pose can't fit, recalibrate with a fuller sweep.
4. Commit the result:
   ```
   git add zbot_arm/config/zbot_follower_1.yaml
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

## First enable on a new or rebuilt arm

Add `torque_limit: 300` (0–1000) to the arm's YAML for the first enable, so an unexpected move
can't strip gears. Remove it once check 5 of the checklist passes. The startup log shows the
value in effect.

## After a FAULT

The log says `FAULT: <joint> tracking error …` or `bus reads failing`, followed by every
servo's raw goal/present counts. The arm holds where it is and ignores commands.

1. Look for the cause: obstruction, collision, cable snag, or a target the joint couldn't
   reach under load. A goal far from present in the logged counts means the servo was told to
   go somewhere unexpected; save the log line.
2. Clear it, hand-support if needed, then re-enable (this resets the fault and holds the current
   pose):
   ```
   ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
3. Repeated `bus reads failing`: check 12 V, the USB cable, and that nothing else has the port.

## Diagnosing servos (nothing moves)

```
python arm_lab/servo_regs.py --id <ID> --port <PORT>                 # raw registers
python arm_lab/servo_regs.py --id <ID> --port <PORT> --write-test 200   # goal write reliability
```

Goal_Position = 0 with torque off is normal (power-up default). Any missed writes in the
write test point to a cable or connector on that joint.

## After changing bridge code or params

- Edited `zbot_arm/*.py`: `cd ~/zbot_ws && colcon build --packages-select zbot_arm`
  (skip if the workspace was built with `--symlink-install`).
- Changed P gain: retest with `step_joints.py --p <n>` and record the result in
  `docs/arm_tuning.md` (current: P = 16, smoothest; 32 tracks tighter but shoulder_pan stutters; 40 oscillates).

## Quick fixes

| Symptom | Fix |
|---|---|
| `Failed to populate field: must be real number, not str` | Replace the placeholder with a number |
| ERROR: calibration changed since the params file was made; enable refused | Run `post_calibrate.py` (or copy the calibration file from the other laptop) |
| `Calibration … doesn't match the motors` | Wrong `--id` (IDs are `zbot_…`), or not calibrated: run `lerobot-calibrate`, then post-calibration |
| `Refused: outside calibrated range` | Move that joint into range by hand and enable again; if its normal pose is outside, recalibrate with a full sweep |
| `Refused: goal did not verify` | Torque stays off. Reseat/replace cables to the named servo; run `servo_regs.py --write-test` |
| `rest_pose outside soft limits` warning at startup | Calibration sweep incomplete: recalibrate, then post_calibrate with no warnings |
| setup-motors can't find a servo | Only one servo may be on the board; check cable and the servo's other port |
| `Command ignored: torque off, faulted, or releasing` | Call `enable true` |
| Port busy / can't open | Another script or bridge is running; stop it |
| `std_srvs` not found | `sudo apt install ros-jazzy-std-srvs` |
| `Couldn't parse params file … Cannot have a value before ros__parameters` | YAML indentation: values need 4 spaces, `ros__parameters:` 2, no tabs |
| `elbow_flex target 5000+ clamped` | You sent degrees; the topic takes radians (90° = 1.571) |

*Last updated: 2026-10-27*
