# SO-101 follower: servo P-gain tuning

**Decision: `position_p_coefficient = 32` for all joints** (LeRobot default is 16).
Pass it on every LeRobot command that drives the follower:

```bash
--robot.position_p_coefficient=32
```

Use the same value for teleop, recording and deployment. A policy trained on data recorded at
one gain sees different arm behavior at another.

## Setup

| Item | Value |
| --- | --- |
| Arm | `zbot_follower_1` (12 V STS3215, 1/345), on `/dev/so101_follower` |
| Software | LeRobot 0.6.2 (fork `mzumbrun/lerobot`), `use_degrees=True`, I = 0, D = 32 |
| Supply | 12 V, bus read 12.5–12.6 V throughout |
| Tool | `arm_lab/step_joints.py --p <gain> --joints shoulder_lift elbow_flex` |
| Pose | Arm straight out, upper arm and forearm as flat as possible (shoulder_lift ≈ 86–89°, elbow_flex ≈ −81°) |
| Test | Each joint: start → +10° → start → −10° → start, ramped at 20°/s, 0.5 s settle |
| Date | 2026-10-25, Victus |

Joint angles are LeRobot degrees: 0 is the middle of the range swept during calibration, not
a geometric reference. "Straight out" reads near +87° shoulder_lift and −81° elbow_flex.

## Finding: the error is gravity droop under P-only control

With I = 0, a servo holding a steady load settles where the P term balances that load, so the
steady-state error is roughly load / P.

- **Moves that lift against gravity (− direction for shoulder_lift and elbow_flex) stop short,** and
  the miss grows with load: shoulder 3.6° upright vs. 5.4° straight out at P = 16.
- **Moves with gravity land within about 0.5° at ~0 mA.** Friction in the 1/345 gearbox holds the
  arm once it arrives, so the motor carries no load.
- **Unloaded joints** (shoulder_pan, wrist_flex, wrist_roll, gripper) track within ±0.5°. The
  sign follows the approach direction, which is the servo deadband plus backlash.

## Results (straight-out pose)

Lift error is the mean of the two lift-direction points per run.

| P | shoulder_lift error | predicted 16/P × 5.3° | elbow_flex error | predicted 16/P × 2.9° | shoulder holding current | Stable? |
| --- | --- | --- | --- | --- | --- | --- |
| 16 | 5.3° | 5.3° | 2.9° | 2.9° | 234–240 mA | yes |
| 24 | 3.4° | 3.5° | 1.8° | 1.9° | 188–202 mA | yes |
| 32 | 2.5° | 2.65° | 1.4° | 1.45° | 169–221 mA | yes, no buzzing |
| 40 | 1.8° | 2.1° | 1.05° | 1.2° | 150–176 mA | **no: shoulder_pan oscillated** |

The error follows 1/P closely, which confirms the droop is load over gain. It isn't gearbox
slop or a calibration problem.

At P = 40 the start pose was about 10° further out than in the other runs. The shoulder +step
(stopped at 102.9° with 0 mA) and the elbow −step (stopped at −95.6°) hit the ends of the
calibrated range, so those two points are range limits, not tracking error, and are left out
of the table.

## Why 32

- One P applies to every motor. **shoulder_pan reaches the stability limit first**: it carries no
  gravity load and has little friction to damp it, so it hunts at P = 40 while the loaded
  joints are still fine.
- Backing off about 25% from the first unstable gain (40) gives 30–32. P = 32 ran quietly.
- At P = 32 the gravity droop is about half the default: shoulder 5.3° → 2.5° (roughly 1 cm at
  the gripper with the arm extended), elbow 2.9° → 1.4°.

## Power (first numbers)

- Worst static hold measured: **~260 mA at 12.5 V on shoulder_lift, about 3 W**, arm straight out.
- elbow_flex holds at ~60–85 mA in the same pose: it only carries the forearm, wrist and gripper.
- Static holding won't size the supply. **Open:** measure peak current during fast moves and
  when a joint stalls (clamp meter on the 12 V lead), per arm and for four arms on the Zbot
  NiMH bus.

## Open / later

- Per-joint gains (higher on shoulder_lift and elbow_flex, lower on shoulder_pan). LeRobot takes
  one value, so this needs custom code. Not needed yet.
- Small integral gain or software gravity compensation, to remove the remaining ~2.5° of
  shoulder droop. Integral action may hunt because of backlash, so test carefully.
- Re-check the gain on the second follower before trusting it there.
