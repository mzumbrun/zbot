"""so101_sliders.launch.py — joint sliders that command one arm through its bridge.

    ros2 launch zbot_arm so101_sliders.launch.py arm:=zbot_follower_1

Needs, already running: the arm's bridge (so101_bridge.launch.py arm:=<ID>) and
display_arms.launch.py (which publishes /<ID>/robot_description the sliders are built from).

joint_state_publisher_gui runs in namespace /<ID> with its output remapped from joint_states
to /<ID>/so101_bridge/joint_command. Before it starts, this launch reads the arm's current
pose from /<ID>/joint_states and sets every slider's start value to it, so opening the slider
window doesn't command a jump to the URDF zero pose.

The sliders publish continuously (10 Hz). Order: pose the arm by hand, launch the sliders,
THEN call enable true: the arm holds, and the first slider message is the same pose.
While torque is off the bridge ignores them (a throttled "Command ignored" warning).

Buttons in the slider window:
    Center     commands every joint to 0 = mid-range. It moves (slewed to 30°/s).
    Randomize  NEVER on the real arm: a random pose, collisions likely.
"""

import math
import re
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ID_RE = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


def current_pose(arm, timeout_s):
    """One message from /<arm>/joint_states as {joint: rad}, or None on timeout.

    Uses its own rclpy context, separate from launch_ros's.
    """
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState

    ctx = Context()
    rclpy.init(context=ctx)
    got = {}
    try:
        node = rclpy.create_node("zbot_slider_seed", context=ctx)
        node.create_subscription(
            JointState, f"/{arm}/joint_states",
            lambda m: got.setdefault("pose", dict(zip(m.name, m.position))), 10)
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        end = time.monotonic() + timeout_s
        while "pose" not in got and time.monotonic() < end:
            ex.spin_once(timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)
    return got.get("pose")


def setup(context):
    arm = LaunchConfiguration("arm").perform(context).strip()
    if not ID_RE.match(arm):
        raise RuntimeError(f"arm:={arm!r} is not a valid arm ID (e.g. arm:=zbot_follower_1).")

    pose = current_pose(arm, float(LaunchConfiguration("wait_s").perform(context)))
    if pose is None:
        raise RuntimeError(f"No /{arm}/joint_states. Start the bridge first: "
                           f"ros2 launch zbot_arm so101_bridge.launch.py arm:={arm}")
    pose = {j: float(v) for j, v in pose.items() if math.isfinite(v)}
    print(f"[so101_sliders] {arm}: sliders start at the current pose: " +
          ", ".join(f"{j} {v:+.2f}" for j, v in pose.items()))

    return [Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        namespace=arm, name="sliders",
        parameters=[{"zeros": pose}],
        # Both relative and absolute forms, so it holds whichever name the node uses.
        remappings=[("joint_states", f"/{arm}/so101_bridge/joint_command"),
                    ("/joint_states", f"/{arm}/so101_bridge/joint_command"),
                    ("robot_description", f"/{arm}/robot_description"),
                    ("/robot_description", f"/{arm}/robot_description")],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm", description="arm ID, e.g. zbot_follower_1"),
        DeclareLaunchArgument("wait_s", default_value="5.0",
                              description="how long to wait for the arm's joint_states"),
        OpaqueFunction(function=setup),
    ])
