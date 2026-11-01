"""so101_leader — ROS 2 node for an SO-101 LEADER arm. Read-only: it never enables torque.

Publishes the leader's joint positions on joint_states in URDF radians, using the same
conversion as so101_bridge, so a leader pose and a follower pose with the same values look the
same in RViz and can be copied straight across by so101_teleop.

Normally started, one per leader and namespaced by its ID, with the same launch file as the
followers (it picks this node from the params file's top key, `so101_leader:`):

    ros2 launch zbot_arm so101_bridge.launch.py arm:=zbot_leader_1

Interfaces (relative names; under the launch file they sit in /<ID>/):
    pub  joint_states    sensor_msgs/JointState   URDF radians, every tick

Parameters (zbot_arm/config/<ID>.yaml, top key so101_leader:):
    port          serial port                                       (/dev/so101_leader_1)
    robot_id      LeRobot calibration id (teleoperator)             (zbot_leader_1)
    rate_hz       publish rate                                      (30.0)
    signs         per-joint sign, URDF order                        ([1,1,1,1,1,1])
    offsets_deg   per-joint offset after the sign, degrees. wrist_roll must be measured the same
                  way as a follower's (matched to the RViz model at 0), see runbooks/so101_arm.md.
    calib_stamp   hash of the calibration the offsets were measured with ("" = not checked).
                  A mismatch is a WARNING, not a block: the leader never moves on its own, but
                  the follower would be offset by the stale amount.

Safety: torque is disabled at connect and never written again. Moving the leader by hand is
always safe. A run of failed reads (~1 s) stops publishing, so so101_teleop's deadman trips and
the follower holds where it is.
"""

import math
import signal

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

from zbot_arm.so101_bridge import JOINTS, calib_stamp, lerobot_to_urdf


class SO101Leader(Node):
    def __init__(self):
        super().__init__("so101_leader")
        log = self.get_logger()
        P = self.declare_parameter
        port = P("port", "/dev/so101_leader_1").value
        robot_id = P("robot_id", "zbot_leader_1").value
        rate = P("rate_hz", 30.0).value
        self.signs = list(P("signs", [1.0] * 6).value)
        self.offsets = list(P("offsets_deg", [0.0] * 6).value)
        stamp_param = P("calib_stamp", "").value
        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise ValueError("signs and offsets_deg must each have 6 entries (one per joint)")

        try:
            from lerobot.motors import MotorNormMode
            from lerobot.teleoperators.so_leader import SO101Leader as Leader, SO101LeaderConfig
        except ImportError as e:
            raise SystemExit(f"Can't import LeRobot ({e}). Start it with the launch file.")

        self.arm = Leader(SO101LeaderConfig(port=port, id=robot_id))
        # calibrate=False: never start LeRobot's interactive calibration from inside ROS.
        self.arm.connect(calibrate=False)
        try:
            if not self.arm.is_calibrated:
                raise SystemExit(
                    f"Calibration for '{robot_id}' doesn't match the motors (looked for "
                    f"{self.arm.calibration_fpath}). Run lerobot-calibrate --teleop.type=so101_leader "
                    f"--teleop.port={port} --teleop.id={robot_id}")
            bad = [j for j in JOINTS[:-1]
                   if self.arm.bus.motors[j].norm_mode != MotorNormMode.DEGREES]
            if bad:
                raise SystemExit(f"{bad} not in DEGREES mode; the radian conversion would be wrong.")
            self.arm.bus.disable_torque()  # belt and braces: configure() already did

            stamp_now = calib_stamp(self.arm.calibration_fpath)
            if not stamp_param:
                log.warn("No calib_stamp in the params file: wrist_roll offset not checked "
                         f"against the calibration (current stamp {stamp_now}).")
            elif stamp_param != stamp_now:
                log.warn(f"Calibration changed since offsets_deg was measured ({stamp_param} -> "
                         f"{stamp_now}). Re-measure the wrist_roll offset and update calib_stamp.")
            self.read_pos()  # fail now, not on the first tick
        except BaseException:
            self.arm.disconnect()
            raise

        self.read_fail = 0
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_timer(1.0 / rate, self.tick)
        log.info(f"Connected to leader {robot_id} on {port}; torque OFF (read-only); {rate:g} Hz.")

    def read_pos(self):
        return self.arm.get_action()  # {"<joint>.pos": value}, same keys as a follower observation

    def tick(self):
        try:
            obs = self.read_pos()
            self.read_fail = 0
        except Exception as e:
            self.read_fail += 1
            self.get_logger().warn(f"read failed: {e}", throttle_duration_sec=2.0)
            return  # publish nothing: a stale leader must look stale to so101_teleop
        if not all(math.isfinite(obs[f"{j}.pos"]) for j in JOINTS):
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = lerobot_to_urdf(obs, self.signs, self.offsets)
        self.pub.publish(msg)

    def close(self):
        try:
            self.arm.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGHUP, lambda *_: rclpy.try_shutdown())
    node = None
    try:
        node = SO101Leader()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
