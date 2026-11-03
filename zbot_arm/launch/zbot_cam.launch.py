"""zbot_cam.launch.py — one USB (UVC) camera, namespaced by its camera ID.

    ros2 launch zbot_arm zbot_cam.launch.py cam:=zbot_cam_follower_1

The camera is configured ONLY by its params file, zbot_arm/config/<cam>.yaml, and cam:=.
Same pattern as so101_bridge.launch.py: one terminal and one launch per camera.

    pub  /<cam>/image_raw            sensor_msgs/Image        rgb8
    pub  /<cam>/camera_info          sensor_msgs/CameraInfo
    (plus /<cam>/image_raw/compressed etc. if image_transport plugins are installed)

Arguments:
    cam      camera ID, e.g. zbot_cam_follower_1 (required); also the udev name and namespace
    config   params file (default: <zbot_arm share>/config/<cam>.yaml)

Run with ROS and the workspace sourced (`ros`). Needs: sudo apt install ros-jazzy-usb-cam

Why the YAML is rewritten: its top key `usb_cam:` would be read as the node /usb_cam, which
does not match /<cam>/usb_cam, so every value would be silently ignored. This file re-keys it
under the fully qualified name (as so101_bridge.launch.py does for the arms).
"""

import os
import re
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NODE = "usb_cam"
ID_RE = re.compile(r"^zbot_cam_[A-Za-z0-9_]+$")


def default_config(cam):
    try:
        share = Path(get_package_share_directory("zbot_arm"))
    except PackageNotFoundError:
        raise RuntimeError("Package zbot_arm not found: source ~/zbot_ws/install/setup.bash")
    return share / "config" / f"{cam}.yaml"


def load_params(path, cam):
    if not path.is_file():
        raise RuntimeError(f"No params file {path}. Write it (docs/runbooks/cameras.md); "
                           "a new YAML needs one colcon build.")
    data = yaml.safe_load(path.read_text()) or {}
    try:
        params = dict(data[NODE]["ros__parameters"])
    except (KeyError, TypeError):
        raise RuntimeError(f"{path}: expected top key '{NODE}:' with 'ros__parameters:' under it.")

    name = params.get("camera_name")
    if name is None:
        params["camera_name"] = cam
    elif name != cam:
        raise RuntimeError(f"{path}: camera_name is '{name}' but cam:={cam}. Each file must match "
                           "its camera ID.")
    dev = str(params.get("video_device", ""))
    if not dev.startswith("/dev/zbot_cam_"):
        raise RuntimeError(f"{path}: video_device '{dev}' is not a udev name (/dev/zbot_cam_…). "
                           "Never use /dev/videoN: the numbers change with plug order.")
    if not Path(dev).exists():
        raise RuntimeError(f"{dev} does not exist: camera unplugged, on a different hub port, or "
                           "the udev rule isn't installed. ls -l /dev/zbot_cam_*")
    return params


def setup(context):
    cam = LaunchConfiguration("cam").perform(context).strip()
    if not ID_RE.match(cam):
        raise RuntimeError(f"cam:={cam!r} is not a camera ID (e.g. cam:=zbot_cam_follower_1).")

    cfg = LaunchConfiguration("config").perform(context).strip()
    path = (Path(cfg).expanduser() if cfg else default_config(cam)).resolve()
    params = load_params(path, cam)

    fd, tmp = tempfile.mkstemp(prefix=f"{cam}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump({f"/{cam}/{NODE}": {"ros__parameters": params}}, f, sort_keys=False)

    print(f"[zbot_cam.launch] {cam}: {params['video_device']} "
          f"{params.get('image_width')}x{params.get('image_height')} "
          f"@ {params.get('framerate')} {params.get('pixel_format')}, params {path}")
    return [Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name=NODE,
        namespace=cam,
        parameters=[tmp],
        output="screen",
        emulate_tty=True,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cam", description="camera ID, e.g. zbot_cam_follower_1"),
        DeclareLaunchArgument("config", default_value="",
                              description="params file (default: zbot_arm config/<cam>.yaml)"),
        OpaqueFunction(function=setup),
    ])
