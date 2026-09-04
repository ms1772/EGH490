#!/usr/bin/env python3
"""S6: bridge VRPN tracker poses to mocap4r2_msgs/RigidBodies.

    vrpn_mocap  --PoseStamped-->  [this node]  --RigidBodies-->  as2_state_estimator

`as2_state_estimator`'s mocap_pose plugin consumes `mocap4r2_msgs/RigidBodies`
and selects a body by `rigid_body_name`. The ROS 2 VRPN client publishes
`geometry_msgs/PoseStamped` per tracker. This node is the adapter.

THE SAFETY REQUIREMENT
----------------------
The stock mocap_pose plugin calls `process_mocap_pose()` unconditionally --
match or no match. When the named body is absent from the array it publishes a
default-constructed pose: position (0, 0, 0), identity orientation, at full
rate, with no warning. A drone that believes it is at the origin flies into the
net.

So this bridge holds one invariant above all others:

    A RigidBody appears in the published array if and only if it is CURRENTLY
    FRESH. Staleness is signalled by ABSENCE, never by a stale value, and a
    message with no fresh bodies at all is NOT PUBLISHED.

Two consequences follow, and both are deliberate:

  * There is no fixed-rate republication of unchanged data. Republishing the
    last pose is precisely the frozen-pose failure mode that makes a dead mocap
    link look healthy. This node publishes only when something actually
    arrived.
  * A message may carry fewer bodies than configured. That is correct. The
    downstream consumer for a missing body sees nothing, which is the signal.

Usage:
    ros2 run ... vrpn_to_rigidbodies.py --ros-args \\
        -p trackers:="['drone0','drone1','drone2']" \\
        -p stale_timeout_s:=0.1

    # Motive commonly streams numeric rigid-body IDs as names. Map them:
    ros2 run ... vrpn_to_rigidbodies.py --ros-args \\
        -p trackers:="['1','2','3']" \\
        -p rigid_body_names:="['drone0','drone1','drone2']"

Self-test (no ROS required):
    python3 vrpn_to_rigidbodies.py --self-test
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

DEFAULT_INPUT_TEMPLATE = "/vrpn_mocap/{tracker}/pose"
DEFAULT_OUTPUT_TOPIC = "/mocap/rigid_bodies"
DEFAULT_STALE_TIMEOUT_S = 0.1
DEFAULT_PUBLISH_RATE_HZ = 100.0
DEFAULT_FRAME_ID = "map"

# A quaternion this far from unit length is treated as corrupt.
QUAT_NORM_TOL = 1e-3


# --------------------------------------------------------------------------- #
# pure logic -- no ROS, unit-testable
# --------------------------------------------------------------------------- #

@dataclass
class TrackerState:
    """Last accepted sample for one tracker."""
    rigid_body_name: str
    pose: Optional[object] = None     # geometry_msgs/Pose, or a stand-in in tests
    stamp: Optional[float] = None     # receipt time, seconds
    updated: bool = False             # new data since the last publish
    rejected: int = 0                 # samples dropped as invalid


def quaternion_is_sane(x: float, y: float, z: float, w: float) -> bool:
    """Reject non-finite or non-unit quaternions before they reach the estimator."""
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        return False
    norm = math.sqrt(sum(v * v for v in values))
    return abs(norm - 1.0) <= QUAT_NORM_TOL


def position_is_sane(x: float, y: float, z: float) -> bool:
    return all(math.isfinite(v) for v in (x, y, z))


class BridgeCore:
    """Freshness bookkeeping, independent of ROS.

    `select_for_publish` is the whole safety argument in one function: it
    returns the bodies that are fresh right now, and an empty list means
    publish nothing.
    """

    def __init__(self, tracker_to_body: Dict[str, str],
                 stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S) -> None:
        self.stale_timeout_s = stale_timeout_s
        self.states: Dict[str, TrackerState] = {
            tracker: TrackerState(rigid_body_name=body)
            for tracker, body in tracker_to_body.items()
        }

    def accept(self, tracker: str, pose: object, now: float) -> bool:
        """Record a sample. Returns False if the tracker is unknown."""
        state = self.states.get(tracker)
        if state is None:
            return False
        state.pose = pose
        state.stamp = now
        state.updated = True
        return True

    def reject(self, tracker: str) -> None:
        """Count an invalid sample. The previous good pose is NOT reused --
        it simply ages out, which is the intended behaviour."""
        state = self.states.get(tracker)
        if state is not None:
            state.rejected += 1

    def is_fresh(self, tracker: str, now: float) -> bool:
        state = self.states.get(tracker)
        if state is None or state.stamp is None:
            return False
        return (now - state.stamp) <= self.stale_timeout_s

    def select_for_publish(self, now: float) -> List[Tuple[str, object]]:
        """Bodies to include in a message published at `now`.

        Empty means DO NOT PUBLISH. A message is warranted only when at least
        one tracker has produced new data since the previous publish; without
        that check a fixed-rate timer would resend unchanged poses, which is
        indistinguishable downstream from a live feed.
        """
        if not any(state.updated for state in self.states.values()):
            return []
        fresh: List[Tuple[str, object]] = []
        for tracker, state in self.states.items():
            if state.pose is not None and self.is_fresh(tracker, now):
                fresh.append((state.rigid_body_name, state.pose))
        return fresh

    def mark_published(self) -> None:
        for state in self.states.values():
            state.updated = False

    def health(self, now: float) -> Dict[str, Dict[str, object]]:
        return {
            tracker: {
                "rigid_body_name": state.rigid_body_name,
                "tracked": self.is_fresh(tracker, now),
                "age_s": None if state.stamp is None else round(now - state.stamp, 4),
                "rejected": state.rejected,
            }
            for tracker, state in self.states.items()
        }


def build_tracker_map(trackers: Sequence[str],
                      rigid_body_names: Sequence[str]) -> Dict[str, str]:
    """Pair VRPN tracker names with the rigid-body names the estimator expects."""
    if not trackers:
        raise ValueError("no trackers configured")
    if rigid_body_names and len(rigid_body_names) != len(trackers):
        raise ValueError(
            f"rigid_body_names has {len(rigid_body_names)} entries but "
            f"trackers has {len(trackers)}; they must correspond one-to-one")
    names = list(rigid_body_names) if rigid_body_names else list(trackers)
    return dict(zip(trackers, names))


# --------------------------------------------------------------------------- #
# ROS node
# --------------------------------------------------------------------------- #

def main_ros(argv: Optional[List[str]] = None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import PoseStamped
    from mocap4r2_msgs.msg import RigidBodies, RigidBody
    from std_msgs.msg import String

    class VrpnToRigidBodies(Node):
        def __init__(self) -> None:
            super().__init__("vrpn_to_rigidbodies")

            self.declare_parameter("trackers", [""])
            self.declare_parameter("rigid_body_names", [""])
            self.declare_parameter("input_topic_template", DEFAULT_INPUT_TEMPLATE)
            self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
            self.declare_parameter("stale_timeout_s", DEFAULT_STALE_TIMEOUT_S)
            self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
            self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
            self.declare_parameter("health_period_s", 1.0)

            trackers = [t for t in self.get_parameter("trackers")
                        .get_parameter_value().string_array_value if t]
            bodies = [b for b in self.get_parameter("rigid_body_names")
                      .get_parameter_value().string_array_value if b]
            if not trackers:
                self.get_logger().fatal("parameter 'trackers' is empty -- nothing to bridge")
                raise SystemExit(2)

            try:
                tracker_map = build_tracker_map(trackers, bodies)
            except ValueError as exc:
                self.get_logger().fatal(str(exc))
                raise SystemExit(2)

            self.template = self.get_parameter("input_topic_template").value
            self.frame_id = self.get_parameter("frame_id").value
            timeout = float(self.get_parameter("stale_timeout_s").value)
            self.core = BridgeCore(tracker_map, stale_timeout_s=timeout)

            # Mocap is high-rate best-effort data; match the VRPN client.
            sensor_qos = QoSProfile(depth=10,
                                    reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST)

            self.pub = self.create_publisher(
                RigidBodies, self.get_parameter("output_topic").value, sensor_qos)
            self.health_pub = self.create_publisher(String, "/mocap/health", 10)

            self.subs = []
            for tracker in tracker_map:
                topic = self.template.format(tracker=tracker)
                self.subs.append(self.create_subscription(
                    PoseStamped, topic,
                    lambda msg, t=tracker: self.on_pose(t, msg), sensor_qos))
                self.get_logger().info(
                    f"bridging {topic}  ->  rigid_body_name '{tracker_map[tracker]}'")

            rate = float(self.get_parameter("publish_rate_hz").value)
            self.create_timer(1.0 / rate, self.on_timer)
            self.create_timer(float(self.get_parameter("health_period_s").value),
                              self.on_health)

            self.frame_number = 0
            self._warned_empty = False
            self.get_logger().info(
                f"stale timeout {timeout * 1000:.0f} ms; a body absent from the "
                f"output means NOT TRACKED, never a stale pose")

        def now_s(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_pose(self, tracker: str, msg) -> None:
            p, q = msg.pose.position, msg.pose.orientation
            if not (position_is_sane(p.x, p.y, p.z)
                    and quaternion_is_sane(q.x, q.y, q.z, q.w)):
                self.core.reject(tracker)
                self.get_logger().warn(
                    f"rejected malformed pose for '{tracker}' -- dropped, not forwarded",
                    throttle_duration_sec=2.0)
                return
            self.core.accept(tracker, msg.pose, self.now_s())

        def on_timer(self) -> None:
            now = self.now_s()
            selected = self.core.select_for_publish(now)
            if not selected:
                # Nothing fresh. Publishing nothing IS the signal.
                return

            out = RigidBodies()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            self.frame_number += 1
            out.frame_number = self.frame_number
            for name, pose in selected:
                body = RigidBody()
                body.rigid_body_name = name
                body.pose = pose
                out.rigidbodies.append(body)
            self.pub.publish(out)
            self.core.mark_published()

        def on_health(self) -> None:
            import json
            now = self.now_s()
            health = self.core.health(now)
            self.health_pub.publish(String(data=json.dumps(health)))
            untracked = [t for t, h in health.items() if not h["tracked"]]
            if untracked:
                self.get_logger().warn(
                    f"NOT TRACKED: {', '.join(sorted(untracked))}",
                    throttle_duration_sec=5.0)

    rclpy.init(args=argv)
    try:
        node = VrpnToRigidBodies()
    except SystemExit as exc:
        rclpy.shutdown()
        return int(exc.code or 2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


# --------------------------------------------------------------------------- #
# self-test -- exercises the safety invariant without ROS
# --------------------------------------------------------------------------- #

def self_test() -> int:
    failures = []

    def check(label: str, condition: bool) -> None:
        print(f"  [{'PASS' if condition else 'FAIL'}] {label}")
        if not condition:
            failures.append(label)

    print("build_tracker_map")
    check("defaults body names to tracker names",
          build_tracker_map(["a", "b"], []) == {"a": "a", "b": "b"})
    check("maps explicit names",
          build_tracker_map(["1", "2"], ["drone0", "drone1"])
          == {"1": "drone0", "2": "drone1"})
    try:
        build_tracker_map(["a", "b"], ["only_one"])
        check("rejects mismatched lengths", False)
    except ValueError:
        check("rejects mismatched lengths", True)

    print("quaternion validation")
    check("accepts unit quaternion", quaternion_is_sane(0.0, 0.0, 0.0, 1.0))
    check("rejects zero quaternion", not quaternion_is_sane(0.0, 0.0, 0.0, 0.0))
    check("rejects NaN", not quaternion_is_sane(float("nan"), 0.0, 0.0, 1.0))
    check("rejects non-unit", not quaternion_is_sane(0.0, 0.0, 0.0, 0.5))
    check("rejects NaN position", not position_is_sane(1.0, float("nan"), 0.0))

    print("freshness invariant")
    core = BridgeCore({"a": "drone0", "b": "drone1"}, stale_timeout_s=0.1)
    check("publishes nothing before any data", core.select_for_publish(0.0) == [])

    core.accept("a", "POSE_A", 1.000)
    selected = core.select_for_publish(1.005)
    check("includes the one fresh body", selected == [("drone0", "POSE_A")])
    check("omits the never-seen body", all(n != "drone1" for n, _ in selected))
    core.mark_published()

    check("does not republish unchanged data", core.select_for_publish(1.010) == [])

    core.accept("b", "POSE_B", 1.020)
    selected = core.select_for_publish(1.025)
    check("includes both while both fresh", len(selected) == 2)
    core.mark_published()

    # 'a' now ages out; only 'b' keeps updating.
    core.accept("b", "POSE_B2", 1.200)
    selected = core.select_for_publish(1.205)
    check("drops the stale body from the array",
          selected == [("drone1", "POSE_B2")])
    check("stale body reported untracked",
          core.health(1.205)["a"]["tracked"] is False)
    core.mark_published()

    # Everything stops. This is the frozen-pose scenario.
    check("publishes NOTHING when the whole feed stops",
          core.select_for_publish(2.000) == [])
    check("all bodies untracked after feed loss",
          all(not h["tracked"] for h in core.health(2.000).values()))

    print("rejection does not resurrect old data")
    core2 = BridgeCore({"a": "drone0"}, stale_timeout_s=0.1)
    core2.accept("a", "GOOD", 1.000)
    core2.mark_published()
    core2.reject("a")                       # malformed sample arrives at 1.5
    check("rejected sample is not published", core2.select_for_publish(1.500) == [])
    check("rejection counted", core2.health(1.500)["a"]["rejected"] == 1)

    print()
    if failures:
        print(f"{len(failures)} FAILURE(S): " + "; ".join(failures))
        return 1
    print("all self-tests passed")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--self-test", action="store_true")
    known, rest = ap.parse_known_args()
    if known.self_test:
        return self_test()
    return main_ros(rest or None)


if __name__ == "__main__":
    sys.exit(main())
