#!/usr/bin/env python
"""servo_regs.py — dump raw servo registers for one SO-101 arm. Read-only.

Opens the bus WITHOUT robot.connect() (connect() runs configure(), which writes registers and
toggles torque), so nothing is written and nothing moves. Values are raw counts, not degrees.

    lr && cd ~/zbot_ws/src/zbot
    python arm_lab/servo_regs.py --id zbot_follower_2 --port /dev/so101_follower_2
    python arm_lab/servo_regs.py                         # zbot_follower_1, for comparison

Stop the bridge first (it holds the port).
"""

import argparse

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
REGS = ["Torque_Enable", "Present_Position", "Goal_Position", "Min_Position_Limit",
        "Max_Position_Limit", "Homing_Offset", "Torque_Limit", "Max_Torque_Limit",
        "P_Coefficient", "Status"]


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/so101_follower")
    p.add_argument("--id", default="zbot_follower_1")
    p.add_argument("--write-test", type=int, default=0, metavar="N",
                   help="with torque OFF, write goal = present N times and read it back; "
                        "counts writes that didn't land (nothing moves)")
    a = p.parse_args()

    robot = SO101Follower(SO101FollowerConfig(port=a.port, id=a.id))
    bus = robot.bus
    bus.connect()  # open port + ping only; no configure, no torque change
    try:
        print(f"{a.id} on {a.port} (raw counts)\n")
        print(f"{'register':20s}" + "".join(f"{j:>15s}" for j in JOINTS))
        for reg in REGS:
            row = []
            for j in JOINTS:
                try:
                    row.append(str(bus.read(reg, j, normalize=False)))
                except Exception:
                    row.append("n/a")
            print(f"{reg:20s}" + "".join(f"{v:>15s}" for v in row))
        print("\nGoal - Present (counts): " +
              ", ".join(f"{j} {_diff(bus, j)}" for j in JOINTS))
        if a.write_test:
            write_test(bus, a.write_test)
    finally:
        bus.disconnect(disable_torque=False)


def write_test(bus, n):
    """Same write the bridge does before torque on (sync write, no acknowledgement), read back.

    Refuses to run unless every servo reports torque OFF, so a changed goal can't move anything.
    """
    if any(int(bus.read("Torque_Enable", j, normalize=False)) for j in JOINTS):
        print("\nwrite-test skipped: a servo has torque ON. Cut 12 V or run step_joints.py --read.")
        return
    print(f"\nwrite-test: {n} x sync_write(goal = present), torque OFF")
    missed = {j: 0 for j in JOINTS}
    for i in range(n):
        pres = {j: int(bus.read("Present_Position", j, normalize=False)) for j in JOINTS}
        bus.sync_write("Goal_Position", pres, normalize=False)
        for j in JOINTS:
            if abs(int(bus.read("Goal_Position", j, normalize=False)) - pres[j]) > 5:
                missed[j] += 1
        # scramble so the next pass can't pass on a leftover value
        bus.sync_write("Goal_Position", {j: 0 for j in JOINTS}, normalize=False)
    print("missed writes: " + ", ".join(f"{j} {m}/{n}" for j, m in missed.items()))


def _diff(bus, j):
    try:
        return int(bus.read("Goal_Position", j, normalize=False)) - \
               int(bus.read("Present_Position", j, normalize=False))
    except Exception:
        return "n/a"


if __name__ == "__main__":
    main()
