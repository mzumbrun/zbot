# SO-101 Arm Bring-up Checklist

Print one copy per arm. An arm is ready for ROS use when all 10 checks pass on the same calibration.
Editable version: https://claude.ai/artifact/EdLzenG2Yacx1RKPVsf8Q4 (export to PDF to print).

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
| P gain | 32 |
| Test date | |

## Before you start

Each arm needs its own ID and port name; its params file is `zbot_arm/config/<ID>.yaml`. The commands below use `<ID>` and `<PORT>`; substitute this arm's values (first arm: `zbot_follower_1`, `/dev/so101_follower`).

- [ ] Arm assembled, 12 V supply connected, USB plugged in
- [ ] udev rule gives this arm its own name: `ls -l <PORT>`
- [ ] Calibrated: `lerobot-calibrate --robot.type=so101_follower --robot.port=<PORT> --robot.id=<ID>`
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

Check the printed limits, match wrist_roll to the model [a], fold the arm to rest [b]. It writes `zbot_arm/config/<ID>.yaml` (including `robot_id` and `port`); commit it.

- [ ] Pass: limits look like each joint's swept range; wrist_roll offset recorded above

### 3. Bridge startup

Stop RViz. Terminal B, with ROS on top (`ros && source ~/zbot_ws/install/setup.bash`):

```
python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/<ID>.yaml
```

- [ ] Pass: soft limits logged; no `rest_pose not set` warning; no calibration-stamp ERROR

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

Use a real number for the position. Send the original value back afterwards.

- [ ] Pass: no jump on enable; elbow moves about 9° smoothly each way and holds

### 6. Clamp

```
ros2 topic pub --once /so101_bridge/joint_command sensor_msgs/msg/JointState "{name: [elbow_flex], position: [3.0]}"
```

- [ ] Pass: log shows `target … clamped to …`; joint stops at the soft limit, short of the hard stop

### 7. Watchdog

Hold the forearm firmly, then send elbow_flex 0.5 rad away from its current value.

- [ ] Pass: within about 0.3 s, `FAULT: elbow_flex tracking error`; arm stops pushing and holds
- [ ] Pass: further commands are refused until `enable true`, which clears the fault

### 8. Release

```
ros2 service call /so101_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

- [ ] Pass: arm glides to rest, pauses about 1 s, goes limp

### 9. Ctrl-C

Re-enable, move a joint, then Ctrl-C the bridge.

- [ ] Pass: prints `Returning to rest pose...` then `At rest, torque OFF`

### 10. Stamp check

In `zbot_arm/config/<ID>.yaml`, change `calib_stamp` to `"x"` and start the bridge. Change it back afterwards.

- [ ] Pass: bridge logs the "calibration changed" ERROR
- [ ] Stamp restored

## Sign-off

If a check fails, note the step and the log line, fix, and re-run from that step. A recalibration restarts the sheet at check 2.

| Item | Entry |
| --- | --- |
| All 10 checks passed | Yes / No |
| Failed checks and fixes | |
| Params file committed (git hash) | |
| Calibration copied to other laptop | Yes / No / N/A |
| Signed / date | |

Notes:
