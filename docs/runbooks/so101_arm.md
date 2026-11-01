# SO-101 arms (followers and leaders)

Followers: `zbot_follower_1` on `/dev/so101_follower`, `zbot_follower_2` on `/dev/so101_follower_2`,
**12 V**. Leaders: `zbot_leader_1` on `/dev/so101_leader_1`, **7.4 V**. Never put a 12 V supply on
a leader board; label every supply at the plug end.

Driver: one launch for every arm, `zbot_arm/launch/so101_bridge.launch.py arm:=<ID>`. The arm's
params file `zbot_arm/config/<ID>.yaml` picks the node by its top key:
- `so101_bridge:` → follower bridge `zbot_arm/so101_bridge.py` (file written by
  `arm_lab/post_calibrate.py`).
- `so101_leader:` → read-only leader node `zbot_arm/so101_leader.py` (file written by hand; it
  never enables torque).

Every params file must contain `robot_id` and `port`. Commands below use arm 1; for another arm
substitute its ID.

Each arm runs in the namespace of its ID:

| Interface | Name | Arms |
|---|---|---|
| joint positions (URDF rad, 30 Hz) | `/<ID>/joint_states` | all |
| commands (URDF rad, partial OK) | `/<ID>/so101_bridge/joint_command` | followers |
| torque on / go to rest | `/<ID>/so101_bridge/enable` (std_srvs/SetBool) | followers |

**Rules**
- Only one program can hold a serial port. Stop the arm's node before running `step_joints.py`,
  `post_calibrate.py`, `servo_regs.py`, `lerobot-calibrate` or `lerobot-teleoperate` on it, and
  vice versa.
- Always use the udev names (`/dev/so101_*`), never `/dev/ttyACM*`: those numbers swap with plug
  order. The launch file refuses any other port.
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
2. Power (12 V follower / 7.4 V leader) and USB to the new arm's board. Find the port:
   `lr && lerobot-find-port`.
3. Set IDs, **one servo on the board at a time** (every new servo is ID 1; two chained
   servos look like a missing one):
   ```
   lerobot-setup-motors --robot.type=so101_follower --robot.port=<PORT>    # follower
   lerobot-setup-motors --teleop.type=so101_leader --teleop.port=<PORT>    # leader
   ```
   Replacing one servo: run it anyway and press through the others; it rewrites the same IDs.
   Leader gearing: 1/191 shoulder_pan + elbow_flex, 1/345 shoulder_lift, 1/147 wrist_flex,
   wrist_roll, gripper.
4. New board only: get its serial (`udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT`),
   add a line to the repo's `99-so101.rules` (`SYMLINK+="so101_<name>"`), run `./system/setup.sh`,
   commit, and run `setup.sh` on the other laptop after pulling. Check: `ls -l /dev/so101_*`.
5. Then **calibrate** (below). Follower: run the checklist from check 2. Leader: "Leader arm" below.

A replaced servo has a new encoder zero, so the **whole arm** must be recalibrated.

## Every session: start the arms in ROS

Every terminal: `ros && source ~/zbot_ws/install/setup.bash`. The venv is not needed in these
terminals; the launch file starts the node with `~/lerobot-venv/bin/python`.

1. Power on (check **every** supply: a follower with its 12 V off fails with `Missing motor IDs`),
   USB in. `ls -l /dev/so101_*`
2. **One terminal per arm** (followers start limp; leaders are always limp):
   ```
   ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_1
   ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_2    # second terminal
   ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_leader_1      # third terminal
   ```
   Check the startup log: the `[so101_bridge.launch]` line names the right node and port; no ERROR
   about the calibration stamp; **no warnings**; soft limits sane; the `torque_limit` line.
3. **RViz, all arms** (optional):
   ```
   ros2 launch zbot_description display_arms.launch.py                 # both arms
   ros2 launch zbot_description display_arms.launch.py arms:=zbot_follower_1
   ```
   Arms stand 0.30 m apart along Y (`spacing:=`), fixed frame `world`. Move each arm by hand;
   only its own model should follow. An arm whose node isn't running doesn't appear.

Other options: `config:=<file>` (a different params file), `python:=<venv>/bin/python`.

## Every time: enable and command

1. Hand-pose the arm near mid-range. Enabling holds wherever it is.
2. Enable:
   ```
   ros2 service call /zbot_follower_1/so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
   ```
   Refused (torque stays off) if a joint is more than 2° outside its calibrated range or a
   servo's goal doesn't verify by read-back. See Quick fixes. A joint folded on its stop just
   past the calibrated end is fine: it nudges onto the limit (≤2°) and the reply names it.
3. Command in **URDF radians** (partial lists OK). Use a real number, not a placeholder:
   ```
   ros2 topic echo --once /zbot_follower_1/joint_states
   ros2 topic pub --once /zbot_follower_1/so101_bridge/joint_command sensor_msgs/msg/JointState \
     "{name: [elbow_flex], position: [1.35]}"
   ```
   Targets outside the soft limits are clamped (a warning is logged). Motion is slewed at 30°/s.

## Leader teleop

Leader → follower through ROS, with all the follower bridge's protections (soft limits, slew,
watchdog, rest on stop). Node: `zbot_arm/so101_teleop.py`.

1. Start the leader's node and the follower's bridge (Every session, step 2).
2. Enable the follower (it holds where it is).
3. Start teleop:
   ```
   ros2 launch zbot_arm so101_teleop.launch.py leader:=zbot_leader_1 follower:=zbot_follower_1
   ```
4. **Engage:** nothing is sent yet. The log prints `Leader minus follower (deg)` every 2 s.
   Pose the leader to match the follower until every body joint is within 10°
   (`engage_deg:=`); the log says `ENGAGED` and the follower starts tracking.
5. Move the leader. The follower follows at up to 30°/s (the bridge's slew), so fast leader
   moves lag; that is expected.
6. Done: `enable false` on the follower (it goes to rest), then Ctrl-C teleop, leader, bridge.

**Deadman:** if the leader stops publishing for 0.3 s (`timeout_s:=`), teleop stops sending, the
follower holds, and the log says `DEADMAN`. It re-engages only after you re-match the poses.

**Arm 2 until its elbow servo is replaced:** no teleop on it.

Quick check without ROS (stock LeRobot; no slew, no rest, no watchdog; bridge stopped):
```
lr && lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/so101_follower --robot.id=zbot_follower_1 \
  --robot.max_relative_target=5 \
  --teleop.type=so101_leader --teleop.port=/dev/so101_leader_1 --teleop.id=zbot_leader_1
```

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
Never run sliders and leader teleop on the same follower at once.

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
- Supplies off when you walk away.

## Every `lerobot-calibrate`

Followers are **robots**, leaders are **teleoperators**; each has its own flag prefix:

```
lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<ID>    # follower
lerobot-calibrate --teleop.type=so101_leader --teleop.port=<PORT> --teleop.id=<ID>   # leader
```

- `<PORT>` is a full device path (`/dev/so101_leader_1`), not just the name.
- `<ID>` names the calibration file; the same ID goes in the params file and `arm:=`.
- Files: followers `~/.cache/huggingface/lerobot/calibration/robots/so_follower/<ID>.json`,
  leaders `…/calibration/teleoperators/so_leader/<ID>.json`.
- Sweep **every joint stop to stop**: elbow fully folded and fully open, shoulder_lift its whole
  travel, **gripper fully open and fully closed**.
- Check the printed table: each body joint should span ~2,200–2,700 counts. A small range
  (e.g. gripper 1843–2047) means that joint wasn't swept; redo it.
- Follower verify, torque off: `python arm_lab/step_joints.py --read --id <ID> --port <PORT>` and
  sweep each joint. Readings change smoothly, never jump or flip sign.

## After every `lerobot-calibrate` (follower)

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

## Leader arm: params file and wrist_roll check (after every leader calibrate)

The leader's YAML is hand-written (`so101_leader:` top key; `robot_id`, `port`, `calib_stamp`,
`offsets_deg`). Teleop copies URDF radians across, so the leader's offsets must put it in the
same URDF pose as the follower when the two arms look the same.

1. Leader node and follower 1's bridge running, follower **torque off**.
2. Pose both arms the same by eye (wrist_roll especially), then compare:
   ```
   ros2 topic echo --once /zbot_leader_1/joint_states
   ros2 topic echo --once /zbot_follower_1/joint_states
   ```
   Body joints should agree within a few degrees (0.05 rad ≈ 3°). If wrist_roll is off by
   D degrees (leader − follower), subtract D from the leader's wrist_roll offset.
3. Set `calib_stamp` to the leader calibration's hash (this clears the startup warning):
   ```
   sha1sum ~/.cache/huggingface/lerobot/calibration/teleoperators/so_leader/zbot_leader_1.json | cut -c1-12
   ```
4. Commit the YAML; `scp` the leader's calibration file to the other laptop (same as followers,
   `teleoperators/so_leader/`).

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
2. Close the slider window or teleop if it's running (it would re-send its pose on re-enable).
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

Which servos answer at all, any baud (read-only, `lr`, nothing else on the port):
```
python -c "from lerobot.motors.feetech import FeetechMotorsBus as B; print(B.scan_port('<PORT>'))"
```

## Running a node without the launch file

Only for debugging, single arm, no namespace (topics at `/joint_states`, `/so101_bridge/…`):

```
lr && ros && source ~/zbot_ws/install/setup.bash && cd ~/zbot_ws/src/zbot
python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/<ID>.yaml
python -m zbot_arm.so101_leader --ros-args --params-file zbot_arm/config/zbot_leader_1.yaml
```

Don't add `-r __ns:=…` by hand: the params file's top key no longer matches the namespaced node
and every value is silently ignored. The launch file handles that.

## After changing bridge code or params

- With `--symlink-install`, edits to `zbot_arm/*.py`, launch files and params files take effect
  on the next launch. A new file (new arm YAML, new launch file, new node) needs one `colcon build`.
- Changed P gain: retest with `step_joints.py --p <n>` and record the result in
  `docs/arm_tuning.md` (current: P = 16, smoothest; 32 tracks tighter but shoulder_pan stutters; 40 oscillates).

## Quick fixes

| Symptom | Fix |
|---|---|
| `Missing motor IDs` … found list `{}` | Port opened but no servo answered: that arm's supply is off (most often), or the board-to-base servo cable is loose |
| `invalid choice: 'so101_leader'` for `--robot.type` | A leader is a teleoperator: use `--teleop.type/port/id` |
| `ls: cannot access '/dev/so101_…'` | Check the spelling, then `ls -l /dev/so101_*`; no symlink yet → add the udev rule |
| `No params file …/config/<ID>.yaml` | Follower: run post_calibrate. Leader: write it (Leader arm). New YAML needs a `colcon build` |
| `expected exactly one top-level section` | The YAML's top key must be `so101_bridge:` (follower) or `so101_leader:` (leader) |
| `…: no 'port'` | Add `port: "/dev/so101_…"` under `ros__parameters`, or re-run post_calibrate |
| `robot_id is '…' but arm:=…` | Wrong `arm:=`, or the YAML belongs to another arm |
| `…/lerobot-venv/bin/python not found` | Pass `python:=<path to venv python>` |
| `Package zbot_arm not found` | `source ~/zbot_ws/install/setup.bash` |
| `Failed to populate field: must be real number, not str` | Replace the placeholder with a number |
| ERROR: calibration changed since the params file was made; enable refused | Run `post_calibrate.py` (or copy the calibration file from the other laptop) |
| Leader WARN: calibration changed since offsets_deg was measured | Redo "Leader arm" steps 2–3 |
| `Calibration … doesn't match the motors` | Wrong ID, or not calibrated (the leader error prints the path it looked in): run `lerobot-calibrate`, then post-calibration |
| Teleop stays `Waiting to engage` | Pose the leader closer to the follower; the log shows which joint is off. A constant wrist_roll gap → fix the leader's offset |
| Teleop `DEADMAN` | Leader node stopped or its reads are failing; check its terminal, USB and 7.4 V |
| `Refused: outside calibrated range by more than 2°` | Move that joint into range by hand and enable again; if its normal pose is outside, recalibrate with a full sweep. Up to 2° past an end (a folded joint on its stop) is accepted: the joint moves onto its limit and the enable reply says `moved onto it` |
| `Refused: goal did not verify` | Torque stays off. Reseat/replace cables to the named servo; run `servo_regs.py --write-test` |
| `rest_pose outside the calibrated range` warning at startup | Calibration sweep didn't reach the folded pose: recalibrate with a full sweep, then post_calibrate with no warnings |
| setup-motors can't find a servo | Only one servo may be on the board; check cable and the servo's other port |
| `Command ignored: torque off, faulted, or releasing` | Call `enable true` (expected while sliders or teleop run with torque off) |
| `ros2 service call /so101_bridge/enable` hangs | Old name: services are under `/<ID>/so101_bridge/…` now |
| Sliders: `No /<ID>/joint_states` | Start that arm's bridge first |
| Slider window opens empty / never appears | `display_arms.launch.py` isn't running (no `/<ID>/robot_description`) |
| Arm missing in RViz | Its node isn't running, or it's not in `arms:=` |
| Port busy / can't open | Another script or node is running; stop it |
| `std_srvs` not found | `sudo apt install ros-jazzy-std-srvs` |
| `Couldn't parse params file …` | YAML indentation: values need 4 spaces, `ros__parameters:` 2, no tabs |
| `elbow_flex target 5000+ clamped` | You sent degrees; the topic takes radians (90° = 1.571) |

*Last updated: 2026-10-30*
