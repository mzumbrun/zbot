#!/usr/bin/env python
"""post_calibrate.py — refresh the SO-101 bridge params after a lerobot-calibrate.

Run this EVERY time you recalibrate the follower. Calibration moves each joint's LeRobot zero,
so anything stored in LeRobot units goes stale: the wrist_roll offset and the rest pose.
(Soft limits are not stored — the bridge derives them from the calibration at startup.)

It writes zbot_arm/config/<id>.yaml (one file per arm, named after its LeRobot id), holding
robot_id, port, the offsets and the rest pose, stamped with a hash of the calibration file.
If that stamp no longer matches the calibration, the bridge logs an ERROR and refuses ~/enable.

Procedure, arm on 12 V, bridge NOT running (it holds the serial port):
    lerobot-calibrate ...                                    # 1. calibrate as usual
    ros && ros2 launch zbot_description display_so101.launch.py   # 2. terminal A: model at zero
    lr && cd ~/zbot_ws/src/zbot && python arm_lab/post_calibrate.py --id <id> --port <port>
                                                             # 3. terminal B (defaults: arm 1)
    git add zbot_arm/config/<id>.yaml && git commit          # 4. keep it with the code

The script keeps the arm limp throughout and asks for two poses:
  a. wrist_roll matched to the RViz model (all sliders at 0)  -> wrist_roll offset
  b. the arm folded into its resting position                 -> rest_pose
     This is where the arm sits on its own, unpowered (folded joints on their stops). It
     must lie within the calibrated range minus REST_MARGIN_DEG; it is exempt from the
     soft-limit margin, which only applies to commands. wrist_roll: within ±90.
Offsets for the other joints are carried over from the existing file (0 by default); if a
calibration ever leaves another joint off from the model, measure it the same way and edit it.
"""

import argparse
import hashlib
import sys
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
REPO = Path(__file__).resolve().parent.parent
CONFIG_DIR = REPO / "zbot_arm" / "config"
OWNED = ("robot_id", "port", "calib_stamp", "margin_deg", "offsets_deg", "rest_pose")
WRIST_ROLL_DEFAULT = (-90.0, 90.0)  # bridge default limit for the un-swept wrist_roll
REST_MARGIN_DEG = 1.0  # same as zbot_arm/so101_bridge.py — keep the two in step


def parse_args():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/so101_follower")
    p.add_argument("--id", default="zbot_follower_1")
    p.add_argument("--out", type=Path, default=None,
                   help="params file (default zbot_arm/config/<id>.yaml)")
    p.add_argument("--margin", type=float, default=None,
                   help="soft-limit margin, degrees (default: keep the file's value, else 5)")
    a = p.parse_args()
    if a.out is None:
        a.out = CONFIG_DIR / f"{a.id}.yaml"
    return a


def calib_stamp(path):
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()[:12]


def calib_path(robot, robot_id):
    p = getattr(robot, "calibration_fpath", None)
    return Path(p) if p else (Path.home() / ".cache/huggingface/lerobot/calibration/robots"
                              / "so_follower" / f"{robot_id}.json")


def calib_limits(bus, margin_deg):
    """Same formula as zbot_arm/so101_bridge.py calib_limits() — keep the two in step."""
    out = {}
    for j in JOINTS:
        if j == "gripper":
            out[j] = (0.0, 100.0)
            continue
        c = bus.calibration[j]
        try:
            max_res = bus.model_resolution_table[bus.motors[j].model] - 1
        except (AttributeError, KeyError):
            max_res = 4095
        half = (c.range_max - c.range_min) / 2.0 * 360.0 / max_res
        out[j] = (-half + margin_deg, half - margin_deg)
    lo, hi = out["wrist_roll"]
    out["wrist_roll"] = (max(lo, WRIST_ROLL_DEFAULT[0]), min(hi, WRIST_ROLL_DEFAULT[1]))
    return out


def load_existing(path):
    """Existing ros__parameters dict, or {} if the file is missing or unreadable."""
    if not path.exists():
        return {}
    try:
        import yaml
        data = yaml.safe_load(path.read_text()) or {}
        return dict(data.get("so101_bridge", {}).get("ros__parameters", {}))
    except Exception as e:
        print(f"(Couldn't read {path}: {e}; starting fresh.)")
        return {}


def fmt_list(vals):
    return "[" + ", ".join(f"{v:.1f}" for v in vals) + "]"


def write_params(path, params, extras):
    lines = [
        "# SO-101 follower bridge params. Written by arm_lab/post_calibrate.py — re-run it after",
        "# every lerobot-calibrate. Values are LeRobot units (degrees, gripper 0-100).",
        "so101_bridge:",
        "  ros__parameters:",
        f'    robot_id: "{params["robot_id"]}"',
        f'    port: "{params["port"]}"',
        f'    calib_stamp: "{params["calib_stamp"]}"',
        f"    margin_deg: {params['margin_deg']:.1f}",
        f"    offsets_deg: {fmt_list(params['offsets_deg'])}",
        f"    rest_pose: {fmt_list(params['rest_pose'])}",
    ]
    if extras:  # hand-added keys (e.g. torque_limit, tighter limit_min / limit_max) are carried over
        import yaml
        dumped = yaml.safe_dump(extras, default_flow_style=None, sort_keys=False)
        lines += ["    " + ln for ln in dumped.splitlines()]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")


def read_pos(robot):
    obs = robot.get_observation()
    return {j: obs[f"{j}.pos"] for j in JOINTS}


def main():
    a = parse_args()
    existing = load_existing(a.out)
    extras = {k: v for k, v in existing.items() if k not in OWNED}
    offsets = list(existing.get("offsets_deg", [0.0] * 6))
    if len(offsets) != 6:
        offsets = [0.0] * 6
    margin = a.margin if a.margin is not None else float(existing.get("margin_deg", 5.0))

    robot = SO101Follower(SO101FollowerConfig(port=a.port, id=a.id))
    robot.connect(calibrate=False)
    try:
        if not robot.is_calibrated:
            sys.exit(f"Calibration for '{a.id}' doesn't match the motors. Run lerobot-calibrate first.")
        robot.bus.disable_torque()
        cpath = calib_path(robot, a.id)
        stamp = calib_stamp(cpath)
        lim = calib_limits(robot.bus, margin)
        rest_lim = calib_limits(robot.bus, REST_MARGIN_DEG)

        print(f"\nCalibration {cpath.name}  stamp {stamp}"
              f"  (file had {existing.get('calib_stamp', 'none')})")
        print(f"Soft limits the bridge will use for commands (margin {margin:g}°):")
        for j, (lo, hi) in lim.items():
            print(f"  {j:14s} [{lo:7.1f}, {hi:7.1f}]")
        if extras:
            print(f"Carrying over hand-added params: {', '.join(extras)}")
        print("\nTorque is OFF for the whole procedure — keep a hand on the arm.")

        # a. wrist_roll offset
        print("\n[a] wrist_roll offset. In RViz (display_so101.launch.py, sliders at 0), look at"
              "\n    the wrist. Turn the real wrist_roll until it matches the model.")
        ans = input(f"    Enter when matched, k to keep the current offset ({offsets[4]:+.1f}°): ")
        if ans.strip().lower() != "k":
            r = read_pos(robot)["wrist_roll"]
            offsets[4] = -r
            print(f"    wrist_roll reads {r:+.1f}°  ->  offset {offsets[4]:+.1f}°")

        # b. rest pose
        while True:
            input("\n[b] Fold the arm into its resting position and LET GO: it should sit there"
                  "\n    on its own, unpowered (gripper nearly closed). Then press Enter: ")
            pos = read_pos(robot)
            rest = [pos[j] for j in JOINTS]
            outside = [f"{j} {v:.1f} not in [{rest_lim[j][0]:.1f}, {rest_lim[j][1]:.1f}]"
                       for j, v in zip(JOINTS, rest) if not rest_lim[j][0] <= v <= rest_lim[j][1]]
            print("    rest_pose " + fmt_list(rest))
            if not outside:
                break
            print("    Outside the calibrated range (the sweep didn't reach this pose):")
            for s in outside:
                print("      " + s)
            if input("    r to re-pose, Enter to accept anyway: ").strip().lower() != "r":
                break
    finally:
        robot.disconnect()

    write_params(a.out, {"robot_id": a.id, "port": a.port, "calib_stamp": stamp, "margin_deg": margin,
                         "offsets_deg": offsets, "rest_pose": rest}, extras)
    try:
        shown = a.out.relative_to(REPO)
    except ValueError:
        shown = a.out
    print(f"\nWrote {a.out}\nStart the bridge with:\n"
          f"  ros2 launch zbot_arm so101_bridge.launch.py arm:={a.id}")


if __name__ == "__main__":
    main()
