#!/usr/bin/env python3
"""leader_offsets.py — measure a leader's offsets_deg against a follower.

Teleop copies URDF radians from leader to follower, so when the two arms LOOK the same, their
joint_states must read the same. LeRobot puts each joint's zero at the middle of its
calibration sweep, and a leader's sweep differs from a follower's (handle, trigger, stops), so
a leader joint can read tens of degrees off. This tool measures that and fixes the leader's
offsets_deg (and calib_stamp) in zbot_arm/config/<leader>.yaml.

Run after every leader lerobot-calibrate (runbooks/so101_arm.md, "Leader arm").

Needs running, each in its own terminal: the leader node and the follower's bridge, follower
torque OFF (do not enable it), teleop NOT running. ROS terminal, no venv needed:

    ros && source ~/zbot_ws/install/setup.bash && cd ~/zbot_ws/src/zbot
    python3 arm_lab/leader_offsets.py                     # zbot_leader_1 vs zbot_follower_1
    python3 arm_lab/leader_offsets.py --write             # same, and update the YAML

It prints leader minus follower, in degrees, twice a second. Pose both arms the same by eye
(a mid pose, nothing on a stop) and press Enter to capture. Then change the pose (wrist up
vs down, elbow more bent) and capture again. If a joint's difference is about the same in
both captures, it is an offset and gets fixed. If it changes a lot between captures, that
joint turns the opposite way on the two arms (a sign problem, not an offset): it is reported,
not changed.
"""

import argparse
import hashlib
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
BODY = JOINTS[:-1]
REPO = Path(__file__).resolve().parent.parent
CAL_DIR = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so_leader"
FIX_DEG = 3.0       # differences smaller than this are hand-posing error: left alone
SIGN_DEG = 10.0     # a difference that changes more than this between captures = sign problem


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--leader", default="zbot_leader_1")
    p.add_argument("--follower", default="zbot_follower_1")
    p.add_argument("--captures", type=int, default=2)
    p.add_argument("--write", action="store_true", help="update the leader's YAML")
    return p.parse_args()


def main():
    a = parse_args()
    yaml_path = REPO / "zbot_arm" / "config" / f"{a.leader}.yaml"
    data = yaml.safe_load(yaml_path.read_text())
    params = data["so101_leader"]["ros__parameters"]
    offsets = [float(v) for v in params.get("offsets_deg", [0.0] * 6)]

    rclpy.init()
    node = rclpy.create_node("leader_offsets")
    pose = {}

    def sub(arm):
        def cb(m):
            if len(m.name) == len(m.position):
                pose[arm] = dict(zip(m.name, m.position))
        node.create_subscription(JointState, f"/{arm}/joint_states", cb, 10)

    sub(a.leader)
    sub(a.follower)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    def diff():
        if a.leader not in pose or a.follower not in pose:
            return None
        return {j: math.degrees(pose[a.leader][j] - pose[a.follower][j]) for j in BODY}

    t0 = time.monotonic()
    while diff() is None:
        if time.monotonic() - t0 > 5:
            missing = [x for x in (a.leader, a.follower) if x not in pose]
            sys.exit(f"No joint_states from {missing}: start its node first.")
        time.sleep(0.1)

    entered = threading.Event()
    caps = []
    print(f"\nLeader {a.leader} minus follower {a.follower}, degrees. Follower torque must be OFF.")
    for n in range(a.captures):
        print(f"\nCapture {n + 1}/{a.captures}: pose both arms the same by eye, then press Enter.")
        entered.clear()
        threading.Thread(target=lambda: (input(), entered.set()), daemon=True).start()
        while not entered.is_set():
            d = diff()
            print("\r  " + "  ".join(f"{j} {d[j]:+6.1f}" for j in BODY) + "   ", end="", flush=True)
            time.sleep(0.5)
        caps.append(diff())
        print("\n  captured.")

    node.destroy_node()
    rclpy.shutdown()

    print("\nResult (leader minus follower, deg):")
    new = list(offsets)
    changed, sign_issue = [], []
    for i, j in enumerate(BODY):
        vals = [c[j] for c in caps]
        mean = sum(vals) / len(vals)
        spread = max(vals) - min(vals)
        note = ""
        if spread > SIGN_DEG:
            note = "  <-- changes between poses: SIGN problem, not fixed"
            sign_issue.append(j)
        elif abs(mean) >= FIX_DEG:
            new[i] = round(offsets[i] - mean, 1)
            note = f"  offset {offsets[i]:+.1f} -> {new[i]:+.1f}"
            changed.append(j)
        print(f"  {j:14s} " + " ".join(f"{v:+6.1f}" for v in vals) + note)

    stamp_file = CAL_DIR / f"{a.leader}.json"
    stamp = hashlib.sha1(stamp_file.read_bytes()).hexdigest()[:12] if stamp_file.is_file() else None

    if sign_issue:
        print(f"\nSign problem on {sign_issue}: the leader's signs need -1 there. Not changed here; "
              "check that joint's direction in RViz and set `signs` by hand.")
    if not changed and stamp == params.get("calib_stamp"):
        print("\nNothing to change.")
        return
    print(f"\noffsets_deg: [{', '.join(f'{v:.1f}' for v in new)}]")
    print(f"calib_stamp: \"{stamp or params.get('calib_stamp', '')}\""
          + ("" if stamp else f"   (no {stamp_file}; stamp unchanged)"))
    if not a.write:
        print("\nRe-run with --write to save these to " + str(yaml_path.relative_to(REPO)))
        return

    params["offsets_deg"] = new
    if stamp:
        params["calib_stamp"] = stamp
    header = [ln for ln in yaml_path.read_text().splitlines() if ln.startswith("#")]
    body = yaml.safe_dump({"so101_leader": {"ros__parameters": params}},
                          default_flow_style=None, sort_keys=False)
    yaml_path.write_text("\n".join(header) + "\n" + body)
    print(f"\nWrote {yaml_path.relative_to(REPO)}. Restart the leader node to use it, then commit.")


if __name__ == "__main__":
    main()
