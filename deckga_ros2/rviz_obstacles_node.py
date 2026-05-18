#!/usr/bin/env python3
"""
rviz_obstacles_node.py

RViz MarkerArray publisher for QuickNav obstacle cubes.
Reads obstacle_xyz and obs_size from a deckga_quicknav_output.pkl and publishes
CUBE markers on /deckga/obstacles at a configurable rate.

Intended to run alongside rviz_paths_node.py (which handles path visualisation).
In RViz, add a second MarkerArray display subscribed to /deckga/obstacles.

Usage:
  python3 deckga_ros2/rviz_obstacles_node.py \\
      --deckga_pkl deckga_ros2/data/deckga_quicknav_output.pkl

Ported and adapted from dipraj-debnath/DECKGA_QuickNav_3D/deckga_quicknav_ros2/rviz_obstacles_node.py
Adaptations: CLI args instead of hardcoded paths/scales, obstacles-only (no path markers),
identity transform by default (no SCALE/MIN_Z), follows rviz_paths_node.py conventions.
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path
from typing import Any, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_DECKGA_PKL = "/mnt/c/Users/mitch/Multi-UAV Project/ROS2_MultiUAV_3D-main/deckga_ros2/data/deckga_quicknav_output.pkl"
DEFAULT_FRAME_ID = "earth"
DEFAULT_TOPIC = "/deckga/obstacles"


class ObstaclesVizNode(Node):
    def __init__(
        self,
        deckga_pkl: str,
        frame_id: str,
        topic: str,
        rate_hz: float,
        obs_alpha: float,
    ) -> None:
        super().__init__("deckga_rviz_obstacles")

        self.frame_id = frame_id
        self.obs_alpha = float(obs_alpha)

        # -------------------------------------------------------
        # Load obstacle data from pickle
        # -------------------------------------------------------
        pkl_path = Path(deckga_pkl).expanduser().resolve()
        if not pkl_path.exists():
            raise FileNotFoundError(f"PKL not found: {pkl_path}")

        with pkl_path.open("rb") as f:
            data: Dict[str, Any] = pickle.load(f)

        if "obstacle_xyz" not in data:
            raise KeyError(
                f"'obstacle_xyz' key missing from PKL. Available keys: {list(data.keys())}\n"
                "Run DECK_GA_QuickNav.py first to generate a deckga_quicknav_output.pkl."
            )

        self.obstacle_xyz = np.asarray(data["obstacle_xyz"], dtype=float)
        raw_size = np.asarray(data["obs_size"], dtype=float)

        # Normalise obs_size to 1-D (N,) — each entry is the half-size scalar
        if raw_size.ndim == 0:
            self.obs_size = np.full(len(self.obstacle_xyz), float(raw_size))
        elif raw_size.ndim == 1:
            self.obs_size = raw_size
        else:
            # (N, 3) — take the mean of the three dimensions as a scalar size
            self.obs_size = raw_size.mean(axis=1)

        self.get_logger().info(
            f"Loaded {len(self.obstacle_xyz)} obstacles from {pkl_path.name}"
        )
        self.get_logger().info(f"Frame: {self.frame_id}  Topic: {topic}")

        # -------------------------------------------------------
        # Publisher + timer
        # -------------------------------------------------------
        self.pub = self.create_publisher(MarkerArray, topic, 10)
        self.timer = self.create_timer(1.0 / float(rate_hz), self._publish)

    def _publish(self) -> None:
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for i, (center, half) in enumerate(zip(self.obstacle_xyz, self.obs_size)):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.ns = "deckga_obstacles"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD

            m.pose.position.x = float(center[0])
            m.pose.position.y = float(center[1])
            m.pose.position.z = float(center[2])
            m.pose.orientation.w = 1.0

            # Full cube side = 2 * half_size
            side = float(half) * 2.0
            m.scale.x = side
            m.scale.y = side
            m.scale.z = side

            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            m.color.a = float(self.obs_alpha)

            m.lifetime.sec = 0  # persist until overwritten

            ma.markers.append(m)

        self.pub.publish(ma)


def main() -> None:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--deckga_pkl", default=DEFAULT_DECKGA_PKL,
                        help="Path to deckga_quicknav_output.pkl")
    parser.add_argument("--frame_id", default=DEFAULT_FRAME_ID)
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--rate", type=float, default=2.0, help="Publish rate in Hz")
    parser.add_argument("--obs_alpha", type=float, default=0.6,
                        help="Obstacle cube transparency (0=invisible, 1=opaque)")
    args = parser.parse_args()

    rclpy.init()
    node = ObstaclesVizNode(
        deckga_pkl=str(args.deckga_pkl),
        frame_id=str(args.frame_id),
        topic=str(args.topic),
        rate_hz=float(args.rate),
        obs_alpha=float(args.obs_alpha),
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
