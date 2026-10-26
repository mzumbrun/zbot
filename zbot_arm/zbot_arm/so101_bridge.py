"""so101_bridge — ROS 2 bridge for the real SO-101 follower.

Always publishes the arm's joint positions on /joint_states. Starts limp (torque off), so it
behaves exactly like the old read-only bridge until you enable it; once enabled it follows
joint targets from ROS.

This node needs BOTH LeRobot (in ~/lerobot-venv) and rclpy (ROS), so it runs from the venv
with ROS sourced on top, in its own terminal, from ~/zbot_ws/src/zbot:

    lr && ros && source ~/zbot_ws/install/setup.bash
    python -m zbot_arm.so101_bridge --ros-args --params-file zbot_arm/config/zbot_follower_1.yaml

Each arm has its own params file, zbot_arm/config/<robot_id>.yaml, holding its robot_id,
port and everything that depends on its calibration (offsets, rest pose). It is written by
arm_lab/post_calibrate.py. Re-run that script after EVERY lerobot-calibrate;
the bridge warns at startup if the calibration has changed since the file was written.

Interfaces:
    pub  /joint_states                  sensor_msgs/JointState  URDF radians, every tick
    sub  /so101_bridge/joint_command    sensor_msgs/JointState  URDF radians; partial OK
    srv  /so101_bridge/enable           std_srvs/SetBool        true  = torque on, hold pose
                                                                false = go to rest, release

Parameters (limits and rest pose are in LeRobot units — degrees, gripper 0-100; topics are
URDF radians):
    port              serial port                                   (/dev/so101_follower)
    robot_id          LeRobot calibration id                        (zbot_follower_1)
    rate_hz           publish + control rate                        (30.0)
    signs             per-joint sign, URDF order below              ([1,1,1,1,1,1])
    offsets_deg       per-joint offset added after the sign, degrees (DEFAULT_OFFSETS_DEG);
                      gripper offset is ignored. Set by post_calibrate.py.
    p_gain            servo P gain written at connect               (32, see docs/arm_tuning.md)
    margin_deg        soft limits sit this far inside the calibrated range (5.0)
    limit_min/max     optional TIGHTER limits, joint order; nan = use calibration
                      (default: wrist_roll ±90, everything else from calibration)
    rest_pose         pose to park at before releasing torque       (unset; post_calibrate.py)
    calib_stamp       hash of the calibration the params file was made from (post_calibrate.py)
    max_vel_deg_s     body-joint slew limit                         (30.0)
    gripper_max_vel   gripper slew limit, %/s                       (50.0)
    track_err_deg     watchdog: body-joint lag that counts as stuck (15.0)
    track_err_time_s  watchdog: how long it may stay stuck          (0.3)

Joint order: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

Soft limits come from the calibration itself: in DEGREES mode LeRobot puts 0 at the middle of
the range swept during lerobot-calibrate, so each joint can go ±(half the swept range) minus
margin_deg. They therefore follow every recalibration automatically. wrist_roll is not swept
(LeRobot gives it the full turn), so it gets a ±90° default; widen it only as far as the wrist
cable allows. limit_min/max can only tighten, never widen past calibration.

Safety:
  - Commands are ignored until ~/enable true. Enable writes goal = present before torque on,
    so nothing moves, and any stale target is discarded.
  - Targets are clamped to the soft limits; unknown joints and non-finite values are rejected.
  - The goal sent to the servos slews toward the target, so a distant target is never a
    full-speed jump.
  - Watchdog: a body joint lagging its setpoint by > track_err_deg for > track_err_time_s
    (collision, stall), or ~1 s of failed bus reads, freezes the arm where it is. Torque stays
    on, so nothing drops; commands are refused until ~/enable true. The gripper is exempt so
    it can squeeze objects.
  - ~/enable false and Ctrl-C ramp to rest_pose, then release torque. Without a rest_pose,
    enable false releases immediately (support the arm) and Ctrl-C leaves torque ON, holding.
"""

import hashlib
import math
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
BODY = JOINTS[:-1]

# URDF gripper joint limits (so101_new_calib.urdf). LeRobot reports the gripper as
# 0 (closed) .. 100 (open); map that linearly onto the URDF joint range.
GRIPPER_LOWER_RAD = -0.174533
GRIPPER_UPPER_RAD = 1.74533

# Fallback offsets if no params file is given. The real values live in
# zbot_arm/config/<robot_id>.yaml, written by arm_lab/post_calibrate.py.
DEFAULT_OFFSETS_DEG = [0.0, 0.0, 0.0, 0.0, -78.0, 0.0]

NAN = float("nan")
# Optional tighter limits (LeRobot units). nan = take the limit from the calibration.
DEFAULT_LIMIT_MIN = [NAN, NAN, NAN, NAN, -90.0, NAN]
DEFAULT_LIMIT_MAX = [NAN, NAN, NAN, NAN, 90.0, NAN]

POST_CAL = "arm_lab/post_calibrate.py"


def calib_stamp(path):
    """Short content hash of a LeRobot calibration file (changes on every recalibration)."""
    return hashlib.sha1(Path(path).read_bytes()).hexdigest()[:12]


def calib_path(robot, robot_id):
    """LeRobot calibration file for this arm."""
    p = getattr(robot, "calibration_fpath", None)
    return Path(p) if p else (Path.home() / ".cache/huggingface/lerobot/calibration/robots"
                              / "so_follower" / f"{robot_id}.json")


def calib_limits(bus, margin_deg):
    """Soft limits in LeRobot units from the bus calibration: {joint: (lo, hi)}.

    LeRobot DEGREES mode: deg = (raw - mid) * 360 / max_res, mid = (range_min + range_max) / 2,
    so a joint spans ±(range_max - range_min) / 2 * 360 / max_res. arm_lab/post_calibrate.py
    repeats this formula — keep the two in step.
    """
    out = {}
    for j in JOINTS:
        if j == "gripper":
            out[j] = (0.0, 100.0)
            continue
        c = bus.calibration[j]
        try:
            max_res = bus.model_resolution_table[bus.motors[j].model] - 1
        except (AttributeError, KeyError):
            max_res = 4095  # STS3215: 4096 counts/rev
        half = (c.range_max - c.range_min) / 2.0 * 360.0 / max_res
        out[j] = (-half + margin_deg, half - margin_deg)
    return out


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


def urdf_to_lerobot(joint, rad, sign, offset_deg):
    """Inverse of lerobot_to_urdf for one joint: URDF rad -> LeRobot degrees (gripper 0-100)."""
    if joint == "gripper":
        return (sign * rad - GRIPPER_LOWER_RAD) / (GRIPPER_UPPER_RAD - GRIPPER_LOWER_RAD) * 100.0
    return sign * (math.degrees(rad) - offset_deg)


class SO101Bridge(Node):
    def __init__(self):
        super().__init__("so101_bridge")
        log = self.get_logger()
        P = self.declare_parameter
        port = P("port", "/dev/so101_follower").value
        robot_id = P("robot_id", "zbot_follower_1").value
        rate = P("rate_hz", 30.0).value
        self.signs = list(P("signs", [1.0] * 6).value)
        self.offsets = list(P("offsets_deg", DEFAULT_OFFSETS_DEG).value)
        p_gain = P("p_gain", 32).value
        margin = P("margin_deg", 5.0).value
        user_lo = list(P("limit_min", DEFAULT_LIMIT_MIN).value)
        user_hi = list(P("limit_max", DEFAULT_LIMIT_MAX).value)
        rest = list(P("rest_pose", [NAN] * 6).value)
        stamp_param = P("calib_stamp", "").value
        self.vel = P("max_vel_deg_s", 30.0).value
        self.gvel = P("gripper_max_vel", 50.0).value
        self.track_err = P("track_err_deg", 15.0).value
        self.track_time = P("track_err_time_s", 0.3).value
        self.settle_s = P("rest_settle_s", 1.0).value

        if len(self.signs) != 6 or len(self.offsets) != 6:
            raise ValueError("signs and offsets_deg must each have 6 entries (one per joint)")
        if len(user_lo) != 6 or len(user_hi) != 6:
            raise ValueError("limit_min / limit_max need 6 entries each (nan = from calibration)")
        self.sign = dict(zip(JOINTS, self.signs))
        self.off = dict(zip(JOINTS, self.offsets))

        # Import here so a missing venv gives a clear message instead of a bare traceback.
        try:
            from lerobot.motors import MotorNormMode
            from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        except ImportError as e:
            raise SystemExit(
                f"Can't import LeRobot ({e}). Run from the venv: lr && ros && "
                "source ~/zbot_ws/install/setup.bash, then python -m zbot_arm.so101_bridge"
            )

        cfg = SO101FollowerConfig(port=port, id=robot_id, disable_torque_on_disconnect=False,
                                  position_p_coefficient=p_gain)
        self.robot = SO101Follower(cfg)
        self.robot.connect(calibrate=False)
        try:
            if not self.robot.is_calibrated:
                raise SystemExit(f"Calibration for '{robot_id}' doesn't match the motors; "
                                 "run lerobot-calibrate, then " + POST_CAL)
            bad = [j for j in BODY if self.robot.bus.motors[j].norm_mode != MotorNormMode.DEGREES]
            if bad:
                raise SystemExit(f"{bad} not in DEGREES mode; the radian conversion would be wrong.")

            # --- params file vs current calibration
            try:
                stamp_now = calib_stamp(calib_path(self.robot, robot_id))
            except OSError:
                stamp_now = None
            if not stamp_param:
                log.warn(f"No calib_stamp: offsets / rest_pose may not match this calibration. "
                         f"Run {POST_CAL} and pass its params file.")
            elif stamp_now and stamp_param != stamp_now:
                log.error(f"Calibration changed since the params file was made "
                          f"({stamp_param} -> {stamp_now}). Offsets and rest_pose are STALE: "
                          f"run {POST_CAL} before enabling.")

            # --- soft limits: calibration, optionally tightened by limit_min/max
            self.lim = {}
            for i, (j, (clo, chi)) in enumerate(calib_limits(self.robot.bus, margin).items()):
                lo = max(clo, user_lo[i]) if math.isfinite(user_lo[i]) else clo
                hi = min(chi, user_hi[i]) if math.isfinite(user_hi[i]) else chi
                if lo >= hi:
                    raise SystemExit(f"{j}: empty limit range [{lo:.1f}, {hi:.1f}]; "
                                     "check margin_deg / limit_min / limit_max.")
                self.lim[j] = (lo, hi)
            log.info("Soft limits (LeRobot units): " +
                     ", ".join(f"{j} [{lo:.0f}, {hi:.0f}]" for j, (lo, hi) in self.lim.items()))

            # --- rest pose
            if len(rest) == 6 and all(math.isfinite(v) for v in rest):
                outside = [j for j, v in zip(JOINTS, rest) if self.clamp(j, v) != v]
                if outside:
                    log.warn(f"rest_pose outside soft limits for {outside}; clamped.")
                self.rest = {j: self.clamp(j, rest[i]) for i, j in enumerate(JOINTS)}
            else:
                self.rest = None
                log.warn("rest_pose not set: enable false drops torque immediately, "
                         "Ctrl-C leaves torque ON.")

            self.robot.bus.disable_torque()  # start limp
            meas = self.read_pos()
        except BaseException:
            self.robot.disconnect()
            raise

        self.torque = False
        self.faulted = False
        self.releasing = False
        self.arrived_at = None
        self.read_fail = 0
        self.setpoint = dict(meas)
        self.target = dict(meas)
        self.err_t = {j: 0.0 for j in BODY}
        self.last = time.monotonic()

        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.create_subscription(JointState, "~/joint_command", self.on_command, 10)
        self.create_service(SetBool, "~/enable", self.on_enable)
        self.create_timer(1.0 / rate, self.tick)
        log.info(f"Connected to {robot_id} on {port}; torque OFF; {rate:g} Hz. "
                 "Call ~/enable true to accept commands.")

    # ---------- helpers
    def clamp(self, j, v):
        lo, hi = self.lim[j]
        return min(max(v, lo), hi)

    def vmax(self, j):
        return self.gvel if j == "gripper" else self.vel

    def read_pos(self):
        obs = self.robot.get_observation()
        return {j: obs[f"{j}.pos"] for j in JOINTS}

    def send(self, pose):
        self.robot.send_action({f"{j}.pos": v for j, v in pose.items()})

    # ---------- ROS callbacks
    def on_command(self, msg):
        log = self.get_logger()
        if not self.torque or self.faulted or self.releasing:
            log.warn("Command ignored: torque off, faulted, or releasing (call ~/enable true).",
                     throttle_duration_sec=2.0)
            return
        if len(msg.name) != len(msg.position):
            log.warn("Command ignored: name/position length mismatch.", throttle_duration_sec=2.0)
            return
        for name, rad in zip(msg.name, msg.position):
            if name not in self.lim:
                log.warn(f"Unknown joint '{name}' ignored.", throttle_duration_sec=5.0)
                continue
            if not math.isfinite(rad):
                log.warn(f"Non-finite target for {name} ignored.", throttle_duration_sec=2.0)
                continue
            v = urdf_to_lerobot(name, rad, self.sign[name], self.off[name])
            c = self.clamp(name, v)
            if c != v:
                log.warn(f"{name} target {v:.1f} clamped to {c:.1f}.", throttle_duration_sec=2.0)
            self.target[name] = c

    def on_enable(self, req, resp):
        if req.data:
            meas = self.read_pos()
            self.setpoint = dict(meas)
            self.target = dict(meas)
            self.err_t = {j: 0.0 for j in BODY}
            self.send(meas)  # goal = present before torque on
            self.robot.bus.enable_torque()
            self.torque, self.faulted, self.releasing = True, False, False
            resp.message = "Torque ON, holding current pose."
        elif not self.torque:
            resp.message = "Torque already off."
        elif self.rest is not None and not self.faulted:
            self.target = dict(self.rest)
            self.releasing, self.arrived_at = True, None
            resp.message = "Moving to rest; torque releases on arrival."
        else:
            self.robot.bus.disable_torque()
            self.torque, self.releasing = False, False
            resp.message = "Torque OFF (no rest pose, or faulted)."
        resp.success = True
        self.get_logger().info(resp.message)
        return resp

    def fault(self, reason, meas=None):
        self.faulted, self.releasing = True, False
        if meas is not None:
            self.setpoint = dict(meas)
            self.target = dict(meas)
            try:
                self.send(meas)  # stop pushing; hold where it actually is
            except Exception:
                pass
        self.get_logger().error(f"FAULT: {reason}. Holding. Call ~/enable true to reset.")

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now

        try:
            obs = self.robot.get_observation()
            self.read_fail = 0
        except Exception as e:  # a corrupted Feetech packet now and then shouldn't kill the node
            self.read_fail += 1
            self.get_logger().warn(f"read failed: {e}", throttle_duration_sec=2.0)
            if self.read_fail * dt > 1.0 and self.torque and not self.faulted:
                self.fault("bus reads failing")
            return
        meas = {j: obs[f"{j}.pos"] for j in JOINTS}

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = lerobot_to_urdf(obs, self.signs, self.offsets)
        self.pub.publish(msg)

        if not self.torque or self.faulted:
            return

        for j in JOINTS:  # slew setpoint toward target
            step = self.vmax(j) * dt
            self.setpoint[j] += max(-step, min(step, self.target[j] - self.setpoint[j]))

        for j in BODY:  # tracking watchdog
            err = meas[j] - self.setpoint[j]
            self.err_t[j] = self.err_t[j] + dt if abs(err) > self.track_err else 0.0
            if self.err_t[j] > self.track_time:
                self.fault(f"{j} tracking error {err:+.1f}° (setpoint {self.setpoint[j]:.1f})", meas)
                return

        try:
            self.send(self.setpoint)
        except Exception as e:
            self.get_logger().warn(f"write failed: {e}", throttle_duration_sec=2.0)

        if self.releasing and all(abs(self.setpoint[j] - self.target[j]) < 1e-6 for j in JOINTS):
            if self.arrived_at is None:
                self.arrived_at = now
            elif now - self.arrived_at >= self.settle_s:
                self.robot.bus.disable_torque()
                self.torque, self.releasing = False, False
                self.get_logger().info("At rest, torque OFF.")

    # ---------- shutdown (runs after the ROS context is gone: bus only, print not log)
    def ramp_blocking(self, goal, hz=50):
        start = self.read_pos()
        dur = max(abs(goal[j] - start[j]) / self.vmax(j) for j in JOINTS)
        n = max(1, int(dur * hz))
        for i in range(1, n + 1):
            self.send({j: start[j] + (goal[j] - start[j]) * i / n for j in JOINTS})
            time.sleep(1.0 / hz)

    def close(self):
        try:
            if self.torque:
                if self.rest is not None and not self.faulted:
                    print("Returning to rest pose...")
                    self.ramp_blocking(self.rest)
                    time.sleep(self.settle_s)
                    self.robot.bus.disable_torque()
                    print("At rest, torque OFF.")
                else:
                    print("Torque left ON, arm holding. Support it, then cut 12 V "
                          "or run step_joints.py --read to go limp.")
        except Exception as e:
            print(f"Shutdown move failed ({e}); torque state unknown, support the arm.")
        finally:
            try:
                self.robot.disconnect()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SO101Bridge()
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
