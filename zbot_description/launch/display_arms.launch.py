"""display_arms.launch.py — RViz with several SO-101 arms side by side, each from its own bridge.

    ros2 launch zbot_description display_arms.launch.py
    ros2 launch zbot_description display_arms.launch.py arms:=zbot_follower_1 spacing:=0.35

For each arm ID in arms:= (comma-separated) this starts, in namespace /<ID>:
    robot_state_publisher   reads /<ID>/joint_states (from that arm's bridge), publishes
                            /<ID>/robot_description and TF frames prefixed "<ID>/"
    static transform        world -> <ID>/<URDF root link>, arms spaced along +Y

and one RViz with a RobotModel per arm, fixed frame "world". The RViz config is generated for
the arms given, so nothing needs editing when an arm is added.

The bridges are separate (zbot_arm so101_bridge.launch.py arm:=<ID>). An arm whose bridge isn't
running yet just doesn't appear until it is. The placement here is only for viewing; the real
mount geometry belongs to the Zbot cart URDF later.
"""

import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def root_link(urdf_xml):
    """The URDF's root link: the one link that is no joint's child."""
    root = ET.fromstring(urdf_xml)
    links = [l.get("name") for l in root.findall("link")]
    children = {j.find("child").get("link") for j in root.findall("joint")}
    roots = [l for l in links if l not in children]
    if len(roots) != 1:
        raise RuntimeError(f"URDF should have exactly one root link, found {roots}")
    return roots[0]


def parse_arms(text):
    arms = [a.strip() for a in text.split(",") if a.strip()]
    if not arms:
        raise RuntimeError("arms:= is empty (e.g. arms:=zbot_follower_1,zbot_follower_2)")
    bad = [a for a in arms if not ID_RE.match(a)]
    if bad or len(set(arms)) != len(arms):
        raise RuntimeError(f"arms:= has invalid or repeated IDs: {arms}")
    return arms


def offsets(n, spacing):
    """Y positions, centred on world origin."""
    return [(i - (n - 1) / 2.0) * spacing for i in range(n)]


def rviz_config(arms):
    """RViz2 config: grid + one RobotModel per arm, fixed frame world."""
    models = ""
    for a in arms:
        models += f"""
    - Class: rviz_default_plugins/RobotModel
      Name: {a}
      Enabled: true
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /{a}/robot_description
      TF Prefix: {a}
      Visual Enabled: true
      Collision Enabled: false
      Alpha: 1"""
    return f"""Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Cell Size: 0.05
      Plane Cell Count: 30
      Reference Frame: <Fixed Frame>{models}
  Global Options:
    Fixed Frame: world
    Background Color: 48; 48; 48
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.4
      Focal Point:
        X: 0
        Y: 0
        Z: 0.15
      Pitch: 0.5
      Yaw: 0.8
      Target Frame: <Fixed Frame>
"""


def setup(context):
    arms = parse_arms(LaunchConfiguration("arms").perform(context))
    spacing = float(LaunchConfiguration("spacing").perform(context))
    share = Path(get_package_share_directory("zbot_description"))
    urdf = (share / "urdf" / "so101.urdf").read_text()
    base = root_link(urdf)

    actions = []
    for arm, y in zip(arms, offsets(len(arms), spacing)):
        actions.append(Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            namespace=arm, name="robot_state_publisher",
            parameters=[{"robot_description": urdf, "frame_prefix": f"{arm}/"}],
        ))
        actions.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name=f"{arm}_mount_tf",
            arguments=["--x", "0", "--y", f"{y:.3f}", "--z", "0",
                       "--frame-id", "world", "--child-frame-id", f"{arm}/{base}"],
        ))

    cfg = tempfile.NamedTemporaryFile("w", prefix="zbot_arms_", suffix=".rviz", delete=False)
    cfg.write(rviz_config(arms))
    cfg.close()
    actions.append(Node(package="rviz2", executable="rviz2", arguments=["-d", cfg.name],
                        condition=IfCondition(LaunchConfiguration("rviz"))))
    print(f"[display_arms] {', '.join(arms)} spaced {spacing} m along Y; root link '{base}'")
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arms", default_value="zbot_follower_1,zbot_follower_2",
                              description="comma-separated arm IDs"),
        DeclareLaunchArgument("spacing", default_value="0.30",
                              description="distance between arm bases along Y, metres"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup),
    ])
