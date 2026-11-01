"""so101_teleop — leader arm drives a follower arm through the follower's bridge.

    ros2 launch zbot_arm so101_teleop.launch.py leader:=zbot_leader_1 follower:=zbot_follower_1

Copies /<leader>/joint_states to /<follower>/so101_bridge/joint_command (both URDF radians, so
it is a straight copy). Everything that keeps the follower safe stays in the follower's bridge:
soft limits, the 30°/s slew, the tracking watchdog, enable/rest. This node adds two gates:

  ENGAGE  Nothing is sent until the leader's body joints are all within engage_deg of the
          follower's. Pose the leader to match the follower (or the follower's rest pose), and
          the log says ENGAGED. So starting teleop never makes the follower lurch across.
  DEADMAN If no leader message arrives for timeout_s (leader unplugged, its node stopped, bus
          errors), sending stops and the follower holds its last target. The node goes back to
          waiting and must re-ENGAGE before it sends again.

Parameters:
    leader      leader arm ID                                     (zbot_leader_1)
    follower    follower arm ID                                   (zbot_follower_1)
    rate_hz     command rate                                      (30.0)
    timeout_s   deadman: leader silence that stops sending        (0.3)
    engage_deg  max body-joint mismatch to engage, degrees         (10.0)

Joint names must match on both arms (they do: both publish the URDF names).
No LeRobot needed: this node only talks ROS.
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
BODY = JOINTS[:-1]


class SO101Teleop(Node):
    def __init__(self):
        super().__init__("so101_teleop")
        P = self.declare_parameter
        self.leader = P("leader", "zbot_leader_1").value
        self.follower = P("follower", "zbot_follower_1").value
        rate = P("rate_hz", 30.0).value
        self.timeout = P("timeout_s", 0.3).value
        self.engage = math.radians(P("engage_deg", 10.0).value)
        if self.leader == self.follower:
            raise SystemExit("leader and follower must be different arms")

        self.lead = None        # {joint: rad}
        self.lead_t = None      # monotonic time of the last leader message
        self.foll = None
        self.engaged = False
        self.last_note = 0.0

        self.create_subscription(JointState, f"/{self.leader}/joint_states", self.on_leader, 10)
        self.create_subscription(JointState, f"/{self.follower}/joint_states", self.on_follower, 10)
        self.pub = self.create_publisher(
            JointState, f"/{self.follower}/so101_bridge/joint_command", 10)
        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"{self.leader} -> {self.follower}. Waiting: pose the leader to match the follower "
            f"(every body joint within {math.degrees(self.engage):.0f}°).")

    @staticmethod
    def pose(msg):
        if len(msg.name) != len(msg.position):
            return None
        p = dict(zip(msg.name, msg.position))
        if not all(j in p and math.isfinite(p[j]) for j in JOINTS):
            return None
        return p

    def on_leader(self, msg):
        p = self.pose(msg)
        if p is not None:
            self.lead, self.lead_t = p, time.monotonic()

    def on_follower(self, msg):
        p = self.pose(msg)
        if p is not None:
            self.foll = p

    def note(self, text, every=2.0):
        now = time.monotonic()
        if now - self.last_note >= every:
            self.last_note = now
            self.get_logger().info(text)

    def tick(self):
        log = self.get_logger()
        fresh = self.lead_t is not None and time.monotonic() - self.lead_t <= self.timeout

        if not fresh:
            if self.engaged:
                self.engaged = False
                log.warn(f"DEADMAN: no {self.leader} joint_states for {self.timeout:g} s. "
                         "Follower holds its last target. Re-match to re-engage.")
            elif self.lead_t is None:
                self.note(f"No /{self.leader}/joint_states yet: is the leader's node running?", 5.0)
            return

        if not self.engaged:
            if self.foll is None:
                self.note(f"No /{self.follower}/joint_states yet: is the follower's bridge running?", 5.0)
                return
            err = {j: self.lead[j] - self.foll[j] for j in BODY}
            worst = max(BODY, key=lambda j: abs(err[j]))
            if abs(err[worst]) > self.engage:
                self.note("Waiting to engage. Leader minus follower (deg): " +
                          ", ".join(f"{j} {math.degrees(err[j]):+.0f}" for j in BODY))
                return
            self.engaged = True
            log.info("ENGAGED: follower now tracks the leader.")

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = [self.lead[j] for j in JOINTS]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SO101Teleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
