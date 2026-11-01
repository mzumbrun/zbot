"""so101_teleop.launch.py — a leader arm drives a follower arm.

    ros2 launch zbot_arm so101_teleop.launch.py leader:=zbot_leader_1 follower:=zbot_follower_1

Needs, already running, each in its own terminal:
    ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_leader_1      # read-only leader
    ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_1    # follower bridge
and the follower enabled (enable true). See zbot_arm/so101_teleop.py for the ENGAGE and
DEADMAN gates, and runbooks/so101_arm.md "Leader teleop" for the procedure.

Arguments: leader, follower, timeout_s (0.3), engage_deg (10), python (venv python; the node
itself needs only ROS, the venv is used so every zbot_arm node starts the same way).
"""

import os
import re
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def node_env():
    """Same as so101_bridge.launch.py: source package first on PYTHONPATH under symlink install."""
    pkg_root = Path(__file__).resolve().parent.parent
    paths = os.environ.get("PYTHONPATH", "")
    if (pkg_root / "zbot_arm" / "so101_teleop.py").is_file():
        paths = str(pkg_root) + (os.pathsep + paths if paths else "")
    return {"PYTHONPATH": paths}


def setup(context):
    get = lambda n: LaunchConfiguration(n).perform(context).strip()
    leader, follower = get("leader"), get("follower")
    for name, v in (("leader", leader), ("follower", follower)):
        if not ID_RE.match(v):
            raise RuntimeError(f"{name}:={v!r} is not a valid arm ID.")
    if leader == follower:
        raise RuntimeError("leader:= and follower:= must be different arms.")
    py = Path(get("python")).expanduser()
    if not os.access(py, os.X_OK):
        raise RuntimeError(f"{py} not found. Pass python:=<venv>/bin/python.")

    print(f"[so101_teleop.launch] {leader} -> {follower}")
    return [ExecuteProcess(
        cmd=[str(py), "-m", "zbot_arm.so101_teleop", "--ros-args",
             "-r", f"__node:={leader}_to_{follower}",
             "-p", f"leader:={leader}", "-p", f"follower:={follower}",
             "-p", f"timeout_s:={float(get('timeout_s'))}",
             "-p", f"engage_deg:={float(get('engage_deg'))}"],
        additional_env=node_env(),
        name=f"{leader}_to_{follower}",
        output="screen",
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("leader", default_value="zbot_leader_1"),
        DeclareLaunchArgument("follower", default_value="zbot_follower_1"),
        DeclareLaunchArgument("timeout_s", default_value="0.3"),
        DeclareLaunchArgument("engage_deg", default_value="10.0"),
        DeclareLaunchArgument("python", default_value="~/lerobot-venv/bin/python"),
        OpaqueFunction(function=setup),
    ])
