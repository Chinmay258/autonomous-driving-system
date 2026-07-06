#!/usr/bin/env python3
"""Runs INSIDE the Autoware container. Headless golden-route probe:

1. Loads the lanelet2 map with the same projection Autoware uses
   (UtmProjector anchored at the map origin == projector_type LocalCartesianUTM),
2. Computes map-frame poses on the requested start/goal lanelets,
3. Publishes /initialpose and /planning/mission_planning/goal,
4. Waits for /planning/mission_planning/route and prints the lanelet id
   sequence as JSON on stdout (single line, prefixed GOLDEN_ROUTE=).

Usage (inside container, after sourcing the Autoware workspace):
  python3 golden_route_check.py --map /autoware_map/lanelet2_map.osm \
      --origin-lat 41.39 --origin-lon 2.165 --start-id 12 --goal-id 345
"""

import argparse
import json
import math
import sys
from itertools import pairwise

import lanelet2
import rclpy
from autoware_planning_msgs.msg import LaneletRoute
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def pose_on_lanelet(lanelet_obj, fraction: float):
    """(x, y, yaw) at an arclength fraction along the lanelet centerline."""
    pts = [(p.x, p.y) for p in lanelet_obj.centerline]
    seg_lengths = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in pairwise(pts)]
    total = sum(seg_lengths)
    target = max(0.0, min(total, fraction * total))
    run = 0.0
    for (a, b), seg in zip(pairwise(pts), seg_lengths, strict=True):
        if run + seg >= target and seg > 0:
            t = (target - run) / seg
            x = a[0] + t * (b[0] - a[0])
            y = a[1] + t * (b[1] - a[1])
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            return x, y, yaw
        run += seg
    a, b = pts[-2], pts[-1]
    return b[0], b[1], math.atan2(b[1] - a[1], b[0] - a[0])


def make_pose(x: float, y: float, yaw: float):
    from geometry_msgs.msg import Pose

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class GoldenProbe(Node):
    def __init__(self, args) -> None:
        super().__init__("golden_route_probe")
        projector = lanelet2.projection.UtmProjector(
            lanelet2.io.Origin(args.origin_lat, args.origin_lon)
        )
        lmap = lanelet2.io.load(args.map, projector)
        start = lmap.laneletLayer[args.start_id]
        goal = lmap.laneletLayer[args.goal_id]
        self._goal_id = args.goal_id
        self._start_pose = make_pose(*pose_on_lanelet(start, 0.5))
        self._goal_pose = make_pose(*pose_on_lanelet(goal, 0.5))
        self.route_ids = None

        self._pub_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        # Feed the simulator directly too: the rviz->adaptor->initializer
        # chain can stall headless, and /initialpose3d is what the planning
        # simulator actually consumes.
        self._pub_init3d = self.create_publisher(PoseWithCovarianceStamped, "/initialpose3d", 1)
        self._pub_goal = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 1)
        route_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            LaneletRoute, "/planning/mission_planning/route", self._on_route, route_qos
        )
        self._ticks = 0
        self.create_timer(1.0, self._tick)

    def _tick(self) -> None:
        self._ticks += 1
        if self.route_ids is not None:
            return
        # Re-publish until the route lands: node startup order is arbitrary.
        if self._ticks % 8 == 2:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.pose = self._start_pose
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.07
            self._pub_init.publish(msg)
            self._pub_init3d.publish(msg)
            self.get_logger().info("published /initialpose + /initialpose3d")
        if self._ticks % 8 == 6:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose = self._goal_pose
            self._pub_goal.publish(msg)
            self.get_logger().info("published goal")

    def _on_route(self, msg: LaneletRoute) -> None:
        if not msg.segments:
            return
        last = {prim.id for prim in msg.segments[-1].primitives}
        if self._goal_id not in last:
            return  # latched route from a previous request; keep waiting
        self.route_ids = [seg.preferred_primitive.id for seg in msg.segments]
        self.alternatives = [[prim.id for prim in seg.primitives] for seg in msg.segments]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    parser.add_argument("--start-id", type=int, required=True)
    parser.add_argument("--goal-id", type=int, required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    rclpy.init()
    node = GoldenProbe(args)
    import time

    deadline = time.time() + args.timeout
    while rclpy.ok() and time.time() < deadline and node.route_ids is None:
        rclpy.spin_once(node, timeout_sec=0.5)

    if node.route_ids is None:
        print("GOLDEN_ROUTE=TIMEOUT", flush=True)
        return 1
    print(
        "GOLDEN_ROUTE="
        + json.dumps({"preferred": node.route_ids, "alternatives": node.alternatives}),
        flush=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
