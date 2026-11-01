"""so101_bridge.launch.py — one SO-101 arm (follower or leader), namespaced by its arm ID.

    ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_follower_1
    ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_leader_1

The arm is configured ONLY by its params file, zbot_arm/config/<arm>.yaml, and the arm:=
argument. The file's top key picks the node:

    so101_bridge:   follower (zbot_arm/so101_bridge.py; file written by arm_lab/post_calibrate.py)
        pub  /<arm>/joint_states                   sensor_msgs/JointState   URDF radians
        sub  /<arm>/so101_bridge/joint_command     sensor_msgs/JointState   URDF radians
        srv  /<arm>/so101_bridge/enable            std_srvs/SetBool
    so101_leader:   leader, read-only, never torques (zbot_arm/so101_leader.py; hand-written)
        pub  /<arm>/joint_states                   sensor_msgs/JointState   URDF radians

so several arms can run at once (one terminal and one launch per arm), on one machine or
on whichever Raspberry Pi the arm is plugged into.

Arguments:
    arm         arm ID, e.g. zbot_follower_1 (required). Also the LeRobot calibration id.
    config      params file (default: <zbot_arm share>/config/<arm>.yaml; with
                --symlink-install that is the file in the repo)
    python      interpreter that has LeRobot (default ~/lerobot-venv/bin/python)

Run from a terminal with ROS and the workspace sourced (no venv needed in that terminal):
    ros && source ~/zbot_ws/install/setup.bash

Why the node is started this way:
  - It needs LeRobot, which lives in the venv, so it is started with the venv's Python, not
    via `ros2 run` (whose entry point uses the system Python).
  - The params file's top key (`so101_bridge:` / `so101_leader:`) is read by ROS as the node
    /so101_bridge, which does NOT match /<arm>/so101_bridge, so the values would be silently
    ignored. This launch file reads the YAML and rewrites it under the fully qualified name.
  - Ctrl-C ramps a follower to rest before releasing torque. ros2 launch normally escalates to
    SIGTERM after 5 s, which could cut that ramp short, so the timeout here is 30 s.
"""

import os
import re
import subprocess
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# params-file top key -> Python module that runs it
NODES = {"so101_bridge": "zbot_arm.so101_bridge", "so101_leader": "zbot_arm.so101_leader"}
ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")  # valid ROS namespace token


def default_config(arm):
    try:
        share = Path(get_package_share_directory("zbot_arm"))
    except PackageNotFoundError:
        raise RuntimeError("Package zbot_arm not found: source ~/zbot_ws/install/setup.bash")
    return share / "config" / f"{arm}.yaml"


def load_params(path, arm):
    """(node name, ros__parameters) from the params file, checked against the arm ID."""
    if not path.is_file():
        raise RuntimeError(f"No params file {path}. Follower: run arm_lab/post_calibrate.py --id "
                           f"{arm} --port <PORT>. Leader: write it by hand (runbooks/so101_arm.md). "
                           "A new YAML needs one colcon build.")
    data = yaml.safe_load(path.read_text()) or {}
    keys = [k for k in NODES if isinstance(data, dict) and k in data]
    if len(keys) != 1:
        raise RuntimeError(f"{path}: expected exactly one top-level section, "
                           f"{' or '.join(NODES)}, with 'ros__parameters:' under it.")
    node = keys[0]
    try:
        params = dict(data[node]["ros__parameters"])
    except (KeyError, TypeError):
        raise RuntimeError(f"{path}: expected '{node}: ros__parameters:'.")

    rid = params.get("robot_id")
    if rid is None:
        params["robot_id"] = arm
    elif rid != arm:
        raise RuntimeError(f"{path}: robot_id is '{rid}' but arm:={arm}. Each file must match "
                           "its arm ID.")
    if not params.get("port"):
        raise RuntimeError(f"{path}: no 'port'. Add e.g.  port: \"/dev/so101_…\"  under "
                           "ros__parameters.")
    if not str(params["port"]).startswith("/dev/so101_"):
        raise RuntimeError(f"{path}: port {params['port']} is not a udev name (/dev/so101_…). "
                           "Never use /dev/ttyACM*.")
    return node, params


def write_namespaced(node, params, arm):
    """Temp params file keyed by the fully qualified node name."""
    fd, tmp = tempfile.mkstemp(prefix=f"{arm}_{node}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump({f"/{arm}/{node}": {"ros__parameters": params}}, f,
                       default_flow_style=None, sort_keys=False)
    return tmp


def bridge_env():
    """PYTHONPATH so the venv's Python finds the zbot_arm Python package.

    A --symlink-install build links this launch file back to the source tree but exposes the
    zbot_arm package to Python only through a .pth file, which a venv's Python never reads.
    So when the source package sits next to this launch file, put it first on PYTHONPATH.
    A normal (copy) install already has it in install/.../site-packages on PYTHONPATH.
    """
    pkg_root = Path(__file__).resolve().parent.parent   # .../zbot_arm (ament package dir)
    paths = os.environ.get("PYTHONPATH", "")
    if (pkg_root / "zbot_arm" / "so101_bridge.py").is_file():
        paths = str(pkg_root) + (os.pathsep + paths if paths else "")
    return {"PYTHONPATH": paths}


def check_python(py, module, env):
    if not os.access(py, os.X_OK):
        raise RuntimeError(f"{py} not found. Pass python:=<venv>/bin/python (the venv with LeRobot).")
    check = subprocess.run(
        [str(py), "-c", "import importlib.util, sys; "
                        f"sys.exit(importlib.util.find_spec('{module}') is None)"],
        env={**os.environ, **env}, capture_output=True, text=True)
    if check.returncode != 0:
        raise RuntimeError(f"{py} can't find {module}. Rebuild "
                           "(colcon build --symlink-install --packages-select zbot_arm) and "
                           f"source install/setup.bash. {check.stderr.strip()}")


def setup(context):
    arm = LaunchConfiguration("arm").perform(context).strip()
    if not ID_RE.match(arm):
        raise RuntimeError(f"arm:={arm!r} is not a valid arm ID (e.g. arm:=zbot_follower_1).")

    cfg = LaunchConfiguration("config").perform(context).strip()
    path = Path(cfg).expanduser() if cfg else default_config(arm)
    node, params = load_params(path.resolve(), arm)
    module = NODES[node]

    py = Path(LaunchConfiguration("python").perform(context)).expanduser()
    env = bridge_env()
    check_python(py, module, env)

    tmp = write_namespaced(node, params, arm)
    print(f"[so101_bridge.launch] {arm}: {node}, port {params['port']}, params {path.resolve()}")
    return [ExecuteProcess(
        cmd=[str(py), "-m", module, "--ros-args",
             "-r", f"__ns:=/{arm}", "-r", f"__node:={node}", "--params-file", tmp],
        additional_env=env,
        name=f"{arm}_{node}",
        output="screen",
        emulate_tty=True,
        sigterm_timeout="30",   # let Ctrl-C finish a follower's ramp to rest
        sigkill_timeout="5",
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", description="arm ID, e.g. zbot_follower_1 or zbot_leader_1"),
        DeclareLaunchArgument("config", default_value="",
                              description="params file (default: zbot_arm config/<arm>.yaml)"),
        DeclareLaunchArgument("python", default_value="~/lerobot-venv/bin/python",
                              description="Python with LeRobot installed"),
        OpaqueFunction(function=setup),
    ])
