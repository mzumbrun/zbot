# SO-101 follower arm

Arms: `zbot_follower_1` on `/dev/so101_follower`, `zbot_follower_2` on `/dev/so101_follower_2`,
12 V. Driver: `zbot_arm/so101_bridge.py`, started by `zbot_arm/launch/so101_bridge.launch.py`.
Params: `zbot_arm/config/<ID>.yaml`, one file per arm named after its ID (written by
`arm_lab/post_calibrate.py`, and must contain `robot_id` and `port`). Commands below use arm 1;
for another arm substitute its ID.

Each bridge runs in the namespace of its arm ID:

| Interface | Name |
|---|---|
| joint positions (URDF rad, 30 Hz) | `/<ID>/joint_states` |
| commands (URDF rad, partial OK) | `/<ID>/so101_bridge/joint_command` |
| torque on / go to rest | `/<ID>/so101_bridge/enable` (std_srvs/SetBool) |

**Rules**
- Only one program can hold a serial port. Stop the arm's bridge before running `step_joints.py`,
  `post_calibrate.py`, `servo_regs.py`, or `lerobot-calibrate` on it, and vice versa.
- Always use the udev names (`/dev/so101_follower*`), never `/dev/ttyACM*`: those numbers
  swap with plug order. The launch file refuses any other port.
- IDs are `zbot_…` (arm, calibration file, namespace); ports are `/dev/so101_…`.
- An arm is configured only by its params file and `arm:=<ID>`. Nothing machine-specific goes
  in the code, so the same command works on either laptop or, later, on the Pi it's plugged into.

---

## One-time: build so the launch files and configs are installed

`zbot_arm/setup.py` must install `launch/` and `config/`. In `data_files`:

```python
(os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
(os.path.join("share", package_name, "config"), glob("config/*.yaml")),
```

(`import os` and `from glob import glob` at the top.) `package.xml` needs
`<exec_depend>` for `launch`, `launch_ros`, `joint_state_publisher_gui`, `robot_state_publisher`,
`tf2_ros`, `rviz2`, `python3-yaml`. Then build **with symlinks**, so an edit to a params file by
`post_calibrate.py` takes effect without rebuilding:

```
cd ~/zbot_ws && colcon build --symlink-install --packages-select zbot_arm zbot_description
```

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
5. Then **calibrate** (below) and run the checklist from check 2.

A replaced servo has a new encoder zero, so the **whole arm** must be recalibrated.

## Every session: start the arms in ROS

Every terminal: `ros && source ~/zbot_ws/install/setup.bash`. The venv is not needed in these
terminals; the launch file starts the bridge with `~/lerobot-venv/bin/python`.

1. 12 V on, USB in. `ls -l /dev/so101_follower*`
2. **One terminal per arm — bridge** (starts limp, publishes `/<ID>/joint_states`):
   ```
   ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_1
   ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_2    # second terminal
   ```
   Check the startup log: the `[so101_bridge.launch]` line names the right port; no ERROR about
   the calibration stamp; **no warnings**; soft limits sane; the `torque_limit` line.
3. **RViz, all arms** (optional):
   ```
   ros2 launch zbot_description display_arms.launch.py                 # both arms
   ros2 launch zbot_description display_arms.launch.py arms:=zbot_follower_1
   ```
   Arms stand 0.30 m apart along Y (`spacing:=`), fixed frame `world`. Move each arm by hand;
   only its own model should follow. An arm whose bridge isn't running doesn't appear.

Other options: `config:=<file>` (a different params file), `python:=<venv>/bin/python`.

## Every time: enable and command

1. Hand-pose the arm near mid-range. Enabling holds wherever it is.
2. Enable:
   ```
   ros2 service call /zbot_follower_1/so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
   Refused (torque stays off) if a joint is outside its calibrated range or a servo's goal
   doesn't verify by read-back. See Quick fixes.
3. Command in **URDF radians** (partial lists OK). Use a real number, not a placeholder:
   ```
   ros2 topic echo --once /zbot_follower_1/joint_states
   ros2 topic pub --once /zbot_follower_1/so101_bridge/joint_command sensor_msgs/msg/JointState \
     "{name: [elbow_flex], position: [1.35]}"
   ```
   Targets outside the soft limits are clamped (a warning is logged). Motion is slewed at 30°/s.

## Slider teleop

Needs the arm's bridge and `display_arms.launch.py` running (the sliders are built from
`/<ID>/robot_description`).

1. Torque off. Hand-pose the arm.
2. ```
   ros2 launch zbot_arm so101_sliders.launch.py arm:=zbot_follower_1
   ```
   It reads the arm's current pose and starts every slider there, so opening the window
   commands no move.
3. Enable (above). Drag a slider; the arm follows at up to 30°/s, clamped to the soft limits.
4. **Never press Randomize.** Center sends every joint to 0 (mid-range); it will move.
5. Done: `enable false` (arm goes to rest), then Ctrl-C the sliders.

The sliders publish all the time. While torque is off the bridge ignores them and logs
`Command ignored` every 2 s; that's expected. If the arm is moved by hand while the sliders are
open, re-enabling sends it back to the slider pose: close and relaunch the sliders first.

**Arm 2 until its elbow servo is replaced:** torque-off mirroring only, or `torque_limit: 300`.
No slider teleop on it.

## Every session end: stop safely

- **Normal:** `ros2 service call /zbot_follower_1/so101_bridge/enable std_srvs/srv/SetBool "{data: false}"`
  → the arm glides to the rest pose, pauses ~1 s, goes limp. Then Ctrl-C the bridge launch.
- **Ctrl-C while enabled** also goes to rest and releases. So does closing the bridge's terminal
  window, but Ctrl-C lets you watch it finish. The launch waits up to 30 s for it;
  don't press Ctrl-C a second time while the arm is moving.
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

1. That arm's bridge stopped.
2. **Terminal A:** `ros && ros2 launch zbot_description display_so101.launch.py` (single model,
   sliders at 0).
3. **Terminal B:**
   ```
   lr && cd ~/zbot_ws/src/zbot
   python arm_lab/post_calibrate.py --id <ID> --port <PORT>
   ```
   - Check the printed limits look like each joint's real swept range.
   - **[a]** Turn wrist_roll to match the model → Enter.
   - **[b]** Fold the arm to rest and **let go**: it must sit there on its own, unpowered (that's
     what the bridge parks it at before releasing torque). Don't hold it up. Then Enter. The rest
     pose may sit up to 1° from a calibrated stop (commands still keep the 5° margin). If it
     reports **"Outside the calibrated range"**, the sweep didn't reach the folded pose: press
     **r**, and if it still fails, recalibrate with a fuller sweep. Never accept it.
4. Commit the result:
   ```
   git add zbot_arm/config/<ID>.yaml
   git commit -m "Recalibrate <ID>"
   git push
   ```
5. **Copy the calibration file to the other laptop.** It lives outside git, so the other machine
   still has the old one, and its bridge will flag a stamp mismatch:
   ```
   scp ~/.cache/huggingface/lerobot/calibration/robots/so_follower/<ID>.json \
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
2. Close the slider window if it's open (it would re-send its pose on re-enable).
3. Clear it, hand-support if needed, then re-enable (resets the fault, holds the current pose):
   ```
   ros2 service call /zbot_follower_1/so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
4. Repeated `bus reads failing`: check 12 V, the USB cable, and that nothing else has the port.

## Diagnosing servos (nothing moves)

```
python arm_lab/servo_regs.py --id <ID> --port <PORT>                 # raw registers
python arm_lab/servo_regs.py --id <ID> --port <PORT> --write-test 200   # goal write reliability
```

Goal_Position = 0 with torque off is normal (power-up default). Any missed writes in the
write test point to a cable or connector on that joint.

## Running the bridge without the launch file

Only for debugging, single arm, no namespace (topics at `/joint_states`, `/so101_bridge/…`):

```
lr && ros && source ~/zbot_ws/install/setup.bash && cd ~/zbot_ws/src/zbot
python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/<ID>.yaml
```

Don't add `-r __ns:=…` by hand: the params file's `so101_bridge:` key no longer matches the
namespaced node and every value is silently ignored. The launch file handles that.

## After changing bridge code or params

- With `--symlink-install`, edits to `zbot_arm/*.py`, launch files and params files take effect
  on the next launch. A new file (new arm YAML, new launch file) needs one `colcon build`.
- Changed P gain: retest with `step_joints.py --p <n>` and record the result in
  `docs/arm_tuning.md` (current: P = 16, smoothest; 32 tracks tighter but shoulder_pan stutters; 40 oscillates).

## Quick fixes

| Symptom | Fix |
|---|---|
| `No params file …/config/<ID>.yaml` | Run post_calibrate for that arm; check setup.py installs `config/`; new YAML needs a `colcon build` |
| `…: no 'port'` | Add `port: "/dev/so101_…"` under `ros__parameters`, or re-run post_calibrate |
| `robot_id is '…' but arm:=…` | Wrong `arm:=`, or the YAML belongs to another arm |
| `…/lerobot-venv/bin/python not found` | Pass `python:=<path to venv python>` |
| `Package zbot_arm not found` | `source ~/zbot_ws/install/setup.bash` |
| `Failed to populate field: must be real number, not str` | Replace the placeholder with a number |
| ERROR: calibration changed since the params file was made; enable refused | Run `post_calibrate.py` (or copy the calibration file from the other laptop) |
| `Calibration … doesn't match the motors` | Wrong ID, or not calibrated: run `lerobot-calibrate`, then post-calibration |
| `Refused: outside calibrated range` | Move that joint into range by hand and enable again; if its normal pose is outside, recalibrate with a full sweep |
| `Refused: goal did not verify` | Torque stays off. Reseat/replace cables to the named servo; run `servo_regs.py --write-test` |
| `rest_pose outside the calibrated range` warning at startup | Calibration sweep didn't reach the folded pose: recalibrate with a full sweep, then post_calibrate with no warnings |
| setup-motors can't find a servo | Only one servo may be on the board; check cable and the servo's other port |
| `Command ignored: torque off, faulted, or releasing` | Call `enable true` (expected while the sliders run with torque off) |
| `ros2 service call /so101_bridge/enable` hangs | Old name: services are under `/<ID>/so101_bridge/…` now |
| Sliders: `No /<ID>/joint_states` | Start that arm's bridge first |
| Slider window opens empty / never appears | `display_arms.launch.py` isn't running (no `/<ID>/robot_description`) |
| Arm missing in RViz | Its bridge isn't running, or it's not in `arms:=` |
| Port busy / can't open | Another script or bridge is running; stop it |
| `std_srvs` not found | `sudo apt install ros-jazzy-std-srvs` |
| `Couldn't parse params file …` | YAML indentation: values need 4 spaces, `ros__parameters:` 2, no tabs |
| `elbow_flex target 5000+ clamped` | You sent degrees; the topic takes radians (90° = 1.571) |

*Last updated: 2026-10-27*
