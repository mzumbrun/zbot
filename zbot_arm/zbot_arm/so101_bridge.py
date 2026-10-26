"""so101_bridge — publish the real SO-101 follower's joint positions on /joint_states.

Read-only for now: the arm is limp (torque off) so you can move it by hand and watch the
RViz model follow. Commanding the arm from ROS comes later.

This node needs BOTH LeRobot (in ~/lerobot-venv) and rclpy (ROS), so it runs from the venv
with ROS sourced on top, in its own terminal:

    lr && ros && source ~/zbot_ws/install/setup.bash
    python -m zbot_arm.so101_bridge
    python -m zbot_arm.so101_bridge --ros-args -p offsets_deg:="[0.,0.,0.,0.,-78.,0.]"

Parameters:
    port          serial port                          (/dev/so101_follower)
    robot_id      LeRobot calibration id               (zbot_follower_1)
    rate_hz       publish rate                         (30.0)
    signs         per-joint sign, URDF order below     ([1,1,1,1,1,1])
    offsets_deg   per-joint offset added after the sign, degrees (DEFAULT_OFFSETS_DEG);
                  gripper offset is ignored. See "Measuring an offset" below.
    p_gain        servo P gain written at connect      (32, see docs/arm_tuning.md)

Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

Measuring an offset (e.g. wrist_roll):
    1. ros2 launch zbot_description display_so101.launch.py   (sliders all at 0: the URDF zero pose)
    2. In an lr terminal: python arm_lab/step_joints.py --read
    3. Turn the real joint until it matches the model; note the reading R (degrees).
    4. Offset for that joint = -R.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# URDF gripper joint limits (so101_new_calib.urdf). LeRobot reports the gripper as
# 0 (closed) .. 100 (open); map that linearly onto the URDF joint range.
GRIPPER_LOWER_RAD = -0.174533
GRIPPER_UPPER_RAD = 1.74533

# Measured offsets for zbot_follower_1 with its 2026-10-25 calibration (Victus).
# wrist_roll has no calibrated range in LeRobot, so its zero is wherever the wrist sat at the
# mid-range calibration step. RE-MEASURE after any lerobot-calibrate of this arm.
DEFAULT_OFFSETS_DEG = [0.0, 0.0, 0.0, 0.0, -78.0, 0.0]


def lerobot_to_urdf(obs, signs, offsets_deg=(0.0,) * 6):
    """LeRobot observation dict (degrees, gripper 0-100) -> URDF joint positions (rad).

    Body joints: urdf_deg = sign * lerobot_deg + offset_deg.
    """
    out = []
    for joint, sign, off in zip(JOINTS, signs, offsets_deg):
        v = obs[f"{joint}.pos"]
        if joint == "gripper":
            frac = min(max(v / 100.0, 0.0), 1.0)
            out.append(sign * (GRIPPER_LOWER_RAD + frac * (GRIPPER_UPPER_RAD - GRIPPER_LOWER_RAD)))
        else:
            out.append(math.radians(sign * v + off))
    return out


class SO101Bridge(Node):
    def __init__(self):
        super().__init__("so101_bridge")
        port = self.declare_parameter("port", "/dev/so101_follower").value
        robot_id = self.declare_parameter("robot_id", "zbot_follower_1").value
        rate = self.declare_parameter("rate_hz", 30.0).value
        self.signs = list(self.declare_parameter("signs", [1.0] * 6).value)
        self.offsets = list(self.declare_parameter("offsets_deg", DEFAULT_OFFSETS_DEG).value)
        p_gain = self.declare_parameter("p_gain", 32).value
        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise ValueError("signs and offsets_deg must each have 6 entries (one per joint)")

        # Import here so a missing venv gives a clear message instead of a bare traceback.
        try:
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        except ImportError as e:
            raise SystemExit(
                f"Can't import LeRobot ({e}). Run from the venv: lr && ros && "
                "source ~/zbot_ws/install/setup.bash, then python -m zbot_arm.so101_bridge"
            )

        cfg = SO101FollowerConfig(port=port, id=robot_id, position_p_coefficient=p_gain)
        self.robot = SO101Follower(cfg)
        self.robot.connect(calibrate=False)
        if not self.robot.is_calibrated:
            self.robot.disconnect()
            raise SystemExit(f"Calibration for '{robot_id}' doesn't match the motors; run lerobot-calibrate.")
        self.robot.bus.disable_torque()  # limp: read-only mirroring

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(f"Connected to {robot_id} on {port}; torque OFF; publishing at {rate:g} Hz")

    def tick(self):
        try:
            obs = self.robot.get_observation()
        except Exception as e:  # a corrupted Feetech packet now and then shouldn't kill the node
            self.get_logger().warn(f"read failed: {e}", throttle_duration_sec=2.0)
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = lerobot_to_urdf(obs, self.signs, self.offsets)
        self.pub.publish(msg)

    def close(self):
        try:
            self.robot.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SO101Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
