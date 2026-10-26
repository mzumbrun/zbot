#!/usr/bin/env python
"""step_joints.py — step each SO-101 follower joint through a small range and log the result.

First scripted motion test. For every joint, one at a time:
    start -> start + step -> start -> start - step -> start
Moves are ramped slowly (--speed), all other joints hold their position, and every
settled point is logged (commanded vs measured, error, motor current, bus voltage).

Run in the LeRobot venv (`lr`), arm on 12 V, from ~/zbot_ws/src/zbot:
    python arm_lab/step_joints.py                       # all joints, defaults
    python arm_lab/step_joints.py --joints elbow_flex   # just one joint
    python arm_lab/step_joints.py --read                # read-only, arm limp
    python arm_lab/step_joints.py --p 32 --joints shoulder_lift elbow_flex   # try a higher P gain

Safety:
  - You pose the arm by hand at mid-range before any motion (torque off),
    so +/- step never runs into a joint limit.
  - Each joint waits for Enter; type 's' to skip it, 'q' to quit.
  - Ctrl-C at any time returns the arm to the start pose.
  - Torque stays ON at the end until you support the arm and press Enter.
"""

import argparse
import csv
import datetime as dt
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
CURRENT_MA_PER_UNIT = 6.5  # STS3215 Present_Current resolution
VOLT_PER_UNIT = 0.1  # STS3215 Present_Voltage resolution


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/so101_follower")
    p.add_argument("--id", default="zbot_follower_1")
    p.add_argument("--joints", nargs="+", default=JOINTS, choices=JOINTS)
    p.add_argument("--step", type=float, default=10.0, help="body-joint step, degrees (default 10)")
    p.add_argument("--gripper-step", type=float, default=20.0, help="gripper step, %% open (default 20)")
    p.add_argument("--speed", type=float, default=20.0, help="ramp speed, units/s (default 20)")
    p.add_argument("--settle", type=float, default=0.5, help="wait after each move, s (default 0.5)")
    p.add_argument("--p", type=int, default=16,
                   help="servo position P gain written at connect (LeRobot default 16; try 24, 32)")
    p.add_argument("--read", action="store_true", help="read-only: print positions live, no motion")
    return p.parse_args()


def read_pos(robot):
    obs = robot.get_observation()
    return {j: obs[f"{j}.pos"] for j in JOINTS}


def read_extra(robot, joint):
    """Motor current (mA) and bus voltage (V) for one joint; blank if unavailable."""
    try:
        cur = robot.bus.sync_read("Present_Current", joint)[joint] * CURRENT_MA_PER_UNIT
        volt = robot.bus.sync_read("Present_Voltage", joint)[joint] * VOLT_PER_UNIT
        return round(cur), round(volt, 1)
    except Exception:
        return "", ""


def ramp_to(robot, hold, joint, target, speed, hz=50):
    """Move one joint to target at `speed` units/s while every other joint holds."""
    start = read_pos(robot)[joint]
    n = max(1, int(abs(target - start) / speed * hz))
    for i in range(1, n + 1):
        cmd = dict(hold)
        cmd[joint] = start + (target - start) * i / n
        robot.send_action({f"{j}.pos": v for j, v in cmd.items()})
        time.sleep(1 / hz)
    hold[joint] = target


def read_only(robot):
    robot.bus.disable_torque()
    print("Torque OFF — move the joints by hand. Ctrl-C to stop.\n")
    print("  ".join(f"{j:>13s}" for j in JOINTS))
    try:
        while True:
            pos = read_pos(robot)
            print("  ".join(f"{pos[j]:13.1f}" for j in JOINTS), end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print()


def main():
    a = parse_args()
    cfg = SO101FollowerConfig(port=a.port, id=a.id, disable_torque_on_disconnect=False,
                              position_p_coefficient=a.p)
    robot = SO101Follower(cfg)
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        sys.exit(f"Calibration for '{a.id}' doesn't match the motors. Run lerobot-calibrate first.")

    if not 1 <= a.p <= 64:
        sys.exit("--p must be between 1 and 64 (16 is the LeRobot default).")

    if a.read:
        read_only(robot)
        robot.disconnect()
        return

    # --- pose by hand
    robot.bus.disable_torque()
    input(
        "\nTorque is OFF. Support the arm and pose every joint near MID-RANGE\n"
        f"(at least {a.step:.0f}° of room each way, gripper half open), then press Enter..."
    )
    hold = read_pos(robot)
    robot.send_action({f"{j}.pos": v for j, v in hold.items()})  # goal = present before torque on
    robot.bus.enable_torque()
    home = dict(hold)
    print(f"Torque ON (P = {a.p}), holding start pose:")
    for j in JOINTS:
        print(f"  {j:14s} {home[j]:7.1f}")

    # --- log file
    log_dir = Path(__file__).resolve().parent / "logs"
    log_dir.mkdir(exist_ok=True)
    log_path = log_dir / f"step_joints_P{a.p}_{dt.datetime.now():%Y%m%d_%H%M%S}.csv"
    f = open(log_path, "w", newline="")
    w = csv.writer(f)
    w.writerow(["time_s", "p_gain", "joint", "phase", "commanded", "measured", "error", "current_mA", "bus_V"])
    t0 = time.time()

    try:
        for joint in a.joints:
            step = a.gripper_step if joint == "gripper" else a.step
            unit = "%" if joint == "gripper" else "°"
            ans = input(f"\n[{joint}] ±{step:g}{unit} — Enter to run, s to skip, q to quit: ").strip().lower()
            if ans == "q":
                break
            if ans == "s":
                continue
            s = home[joint]
            for phase, target in [("+step", s + step), ("return", s), ("-step", s - step), ("return", s)]:
                ramp_to(robot, hold, joint, target, a.speed)
                time.sleep(a.settle)
                meas = read_pos(robot)[joint]
                cur, volt = read_extra(robot, joint)
                err = meas - target
                w.writerow([f"{time.time() - t0:.2f}", a.p, joint, phase, f"{target:.2f}", f"{meas:.2f}",
                            f"{err:.2f}", cur, volt])
                f.flush()
                print(f"  {phase:7s} cmd {target:7.1f}  meas {meas:7.1f}  err {err:+6.2f}{unit}"
                      f"  {cur:>5} mA  {volt} V")
    except KeyboardInterrupt:
        print("\nInterrupted — returning to start pose.")
    finally:
        try:
            for j in JOINTS:
                if abs(hold[j] - home[j]) > 0.5:
                    ramp_to(robot, hold, j, home[j], a.speed)
        finally:
            f.close()
            print(f"\nLog: {log_path}")
            try:
                input("Support the arm, then press Enter to release torque...")
            except (KeyboardInterrupt, EOFError):
                pass
            robot.bus.disable_torque()
            robot.disconnect()


if __name__ == "__main__":
    main()
