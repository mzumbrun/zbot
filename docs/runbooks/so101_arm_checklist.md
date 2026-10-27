# SO-101 Arm Bring-up Checklist

Print one copy per arm. An arm is ready for ROS use when all 10 checks pass on the same calibration.
Re-run from check 2 after any recalibration or servo replacement.

## Arm record

| Field | Value |
| --- | --- |
| Arm ID (robot_id) | |
| Serial port (udev name) | /dev/ |
| Params file | zbot_arm/config/          .yaml |
| Laptop (Victus / Upc) | |
| Calibration date | |
| calib_stamp (from post_calibrate.py) | |
| Measured wrist_roll offset (°) | |
| P gain | 16 |
| torque_limit for first enable | 300 (new or rebuilt arm) / 1000 |
| Test date | |

## Before you start

Each arm needs its own ID and port name; its params file is `zbot_arm/config/<ID>.yaml`. The commands below use `<ID>` and `<PORT>`; substitute this arm's values (`zbot_follower_1` / `/dev/so101_follower`, `zbot_follower_2` / `/dev/so101_follower_2`). IDs are `zbot_…`, ports are `/dev/so101_…`.

- [ ] Arm assembled, 12 V supply connected, USB plugged in
- [ ] New arm or replaced servo: IDs set with `lerobot-setup-motors`, **one servo on the board at a time**, other arms unplugged
- [ ] udev rule gives this arm its own name: `ls -l <PORT>`
- [ ] Calibrated, **every joint swept stop to stop** (gripper fully open and closed): `lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<ID>`
- [ ] Calibration table: every body joint spans roughly 2,200–2,700 counts
- [ ] Two terminals ready: **A** = `ros && source ~/zbot_ws/install/setup.bash`, **B** = `lr && cd ~/zbot_ws/src/zbot`
- [ ] Only one program on the port at a time: stop the bridge before running any `arm_lab` script

## The 10 checks

Do them in order; each relies on the one before. Joint order everywhere: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.

### 1. Repo and build

```
cd ~/zbot_ws/src/zbot && git status
cd ~/zbot_ws && colcon build --packages-select zbot_arm
```

- [ ] Pass: build has no errors and `git status` is clean

### 2. Create the params file

Terminal A: `ros2 launch zbot_description display_so101.launch.py` (sliders at 0). Terminal B:

```
python arm_lab/post_calibrate.py --id <ID> --port <PORT>
```

Check the printed limits, match wrist_roll to the model [a], fold the arm to rest [b]. It writes `zbot_arm/config/<ID>.yaml` (including `robot_id` and `port`); commit it. New or rebuilt arm: add `torque_limit: 300` to the YAML for now.

- [ ] Pass: limits look like each joint's swept range; wrist_roll offset recorded above
- [ ] Pass: no "Outside the soft limits" at [b] (re-pose with **r**; never accept)

### 3. Bridge startup

Stop RViz. Terminal B, with ROS on top (`ros && source ~/zbot_ws/install/setup.bash`):

```
python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/<ID>.yaml
```

- [ ] Pass: soft limits and `torque_limit` logged; no calibration-stamp ERROR
- [ ] Pass: **no warnings at all** (in particular no `rest_pose not set` or `rest_pose outside soft limits`)

### 4. Mirroring, torque off

Terminal A: `ros2 launch zbot_description display_so101.launch.py gui:=false`. Move each joint by hand.

- [ ] Pass: every joint follows in the right direction, including wrist_roll and gripper
- [ ] Pass: `ros2 topic hz /joint_states` shows about 30 Hz

### 5. Enable and nudge

Hand-pose the arm at mid-range, then:

```
ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo --once /joint_states
ros2 topic pub --once /so101_bridge/joint_command sensor_msgs/msg/JointState "{name: [elbow_flex], position: [<current + 0.15>]}"
```

Positions are in radians, not degrees (90° = 1.571). Take the current value from `/joint_states`, not from step_joints.py. Send the original value back afterwards.

- [ ] Pass: enable answers `Torque ON (limit …)`; no jump on enable; elbow moves about 9° smoothly each way and holds
- [ ] If `torque_limit: 300` was set: remove it, restart the bridge, and repeat this check at 1000

### 6. Clamp

```
ros2 topic pub --once /so101_bridge/joint_command sensor_msgs/msg/JointState "{name: [elbow_flex], position: [3.0]}"
```

- [ ] Pass: log shows `target … clamped to …`; joint stops at the soft limit, short of the hard stop

### 7. Watchdog

Hold the forearm firmly, then send elbow_flex 0.5 rad away from its current value.

- [ ] Pass: within about 0.3 s, `FAULT: elbow_flex tracking error` with raw goal/present counts; arm stops pushing and holds
- [ ] Pass: further commands are refused until `enable true`, which clears the fault

### 8. Release

```
ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

- [ ] Pass: arm glides to rest, pauses about 1 s, goes limp

### 9. Ctrl-C

Re-enable, move a joint, then Ctrl-C the bridge while torque is still on. Do not call enable false first.

- [ ] Pass: prints `Returning to rest pose...` then `At rest, torque OFF`

### 10. Refusal checks

**a. Out of range.** Bridge running, torque off. Hand-push one joint a little past its calibrated stop (an end of the range printed at check 2), hold it, and call `enable true`.

- [ ] Pass: `Refused: outside calibrated range …` naming that joint; torque stays off

**b. Stale stamp.** In `zbot_arm/config/<ID>.yaml`, change `calib_stamp` to `"x"` and start the bridge. Change it back afterwards.

- [ ] Pass: bridge logs the "calibration changed" ERROR, and enable true is refused
- [ ] Stamp restored

## Sign-off

If a check fails, note the step and the log line, fix, and re-run from that step. A recalibration or servo replacement restarts the sheet at check 2.

| Item | Entry |
| --- | --- |
| All 10 checks passed | Yes / No |
| Failed checks and fixes | |
| Params file committed (git hash) | |
| `torque_limit: 300` removed from YAML | Yes / N/A |
| Calibration copied to other laptop | Yes / No / N/A |
| Signed / date | |

Notes:
