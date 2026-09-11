#!/usr/bin/env python3
"""Render the live ROS 2 graph as an image, without rqt_graph or a display.

Same picture rqt_graph draws -- nodes as ellipses, topics as boxes, an arrow
from publisher to topic and from topic to subscriber -- but produced from the
running graph with rclpy and rendered with graphviz, so it works headless,
inside WSL, and in a script.

What it adds over a raw dump:
  * nodes are grouped by namespace (one cluster per drone, one for the
    ground station), so a three-drone graph reads as three columns
  * the localisation chain is highlighted: anything carrying mocap, the
    state estimate, the frame tree, or the safety guard is coloured, so the
    path from "camera saw a marker" to "PX4 was told where it is" can be
    traced by eye
  * housekeeping topics (/rosout, /parameter_events, per-node parameter
    services) are dropped -- they connect every node to every other node
    and turn any real graph into a hairball

Usage (with ROS sourced and the stack running):
    python3 flight_ops/ros_graph.py --out /path/to/graph
        -> graph.dot, graph.svg, graph.png

    python3 flight_ops/ros_graph.py --out graph --title "R1, one drone"
    python3 flight_ops/ros_graph.py --out graph --keep-leaves   # show topics with one end
    python3 flight_ops/ros_graph.py --out graph --only-mocap     # just the highlighted chain
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import time
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

# Topics that every node touches and that carry no information about the
# system's structure.
NOISE_TOPICS = re.compile(r"^(/rosout|/parameter_events|/tf_static)$")
# Node names that are tooling, not the stack. transform_listener_impl_* are
# tf2's internal helper nodes -- one per TF listener, namespaced under
# whichever node created them -- and they only ever subscribe to /tf, so
# they add a spoke per listener and no information.
NOISE_NODES = re.compile(r"(^/(_ros2cli_|ros_graph_|rqt_)|transform_listener_impl)")

# Anything matching one of these is part of the localisation / safety story
# and gets colour.
MOCAP_TOPIC = re.compile(
    r"(mocap|vrpn|rigid_bod|self_localization|mocap_health|/tf$|volume_guard|"
    r"visual_odometry|fake_mocap|vehicle_odometry)", re.I)
MOCAP_NODE = re.compile(
    r"(mocap|vrpn|state_estimator|volume_guard|fake_mocap|platform)", re.I)

# Colours -- muted, print-friendly, distinct.
C_NODE = "#e8eef5"
C_NODE_EDGE = "#4a6785"
C_TOPIC = "#ffffff"
C_TOPIC_EDGE = "#8a8a8a"
C_HL_NODE = "#fde7c8"
C_HL_NODE_EDGE = "#c8741a"
C_HL_TOPIC = "#fff4e0"
C_HL_EDGE = "#c8741a"
C_EDGE = "#7a7a7a"
C_CLUSTER = "#f7f7f7"
C_CLUSTER_EDGE = "#bbbbbb"


@dataclass
class Graph:
    nodes: Set[str] = field(default_factory=set)
    topics: Dict[str, str] = field(default_factory=dict)             # topic -> type
    pubs: Set[Tuple[str, str]] = field(default_factory=set)          # (node, topic)
    subs: Set[Tuple[str, str]] = field(default_factory=set)          # (node, topic)


# --------------------------------------------------------------------------- #
# collection
# --------------------------------------------------------------------------- #

def collect(settle_s: float) -> Graph:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    probe = Node("ros_graph_probe")
    graph = Graph()
    try:
        # Discovery is asynchronous; give the participant time to hear
        # everyone before asking who is there.
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.1)

        for name, ns in probe.get_node_names_and_namespaces():
            full = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
            if NOISE_NODES.search(full) or full == "/ros_graph_probe":
                continue
            graph.nodes.add(full)
            for topic, types in probe.get_publisher_names_and_types_by_node(name, ns):
                if NOISE_TOPICS.match(topic):
                    continue
                graph.topics[topic] = types[0] if types else "?"
                graph.pubs.add((full, topic))
            for topic, types in probe.get_subscriber_names_and_types_by_node(name, ns):
                if NOISE_TOPICS.match(topic):
                    continue
                graph.topics.setdefault(topic, types[0] if types else "?")
                graph.subs.add((full, topic))
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    return graph


# --------------------------------------------------------------------------- #
# persistence -- a captured graph can be re-rendered without the stack running
# --------------------------------------------------------------------------- #

def save_json(graph: Graph, path: Path) -> None:
    import json
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "nodes": sorted(graph.nodes),
        "topics": graph.topics,
        "pubs": sorted(graph.pubs),
        "subs": sorted(graph.subs),
    }, indent=1), encoding="utf-8")


def load_json(path: Path) -> Graph:
    import json
    data = json.loads(path.read_text(encoding="utf-8"))
    return Graph(
        nodes=set(data["nodes"]),
        topics=dict(data["topics"]),
        pubs={tuple(p) for p in data["pubs"]},
        subs={tuple(s) for s in data["subs"]},
    )


# --------------------------------------------------------------------------- #
# layout helpers
# --------------------------------------------------------------------------- #

def namespace_of(name: str) -> str:
    """'/drone0/platform' -> 'drone0'; '/volume_guard' -> '' (ground)."""
    parts = name.strip("/").split("/")
    return parts[0] if len(parts) > 1 else ""


def dot_id(text: str) -> str:
    return '"' + text.replace('"', '\\"') + '"'


def short_type(msg_type: str) -> str:
    return msg_type.rsplit("/", 1)[-1]


def is_hl_topic(topic: str) -> bool:
    return bool(MOCAP_TOPIC.search(topic))


def is_hl_node(node: str) -> bool:
    return bool(MOCAP_NODE.search(node))


# --------------------------------------------------------------------------- #
# DOT generation
# --------------------------------------------------------------------------- #

def to_dot(graph: Graph, title: str, keep_leaves: bool, only_mocap: bool) -> str:
    # Filter noise nodes here as well as at collection time, so a dump taken
    # before a filter was tightened still renders cleanly.
    keep = {n for n in graph.nodes if not NOISE_NODES.search(n)}
    pubs = {(n, t) for n, t in graph.pubs if n in keep}
    subs = {(n, t) for n, t in graph.subs if n in keep}
    graph = Graph(nodes=keep, topics=dict(graph.topics), pubs=pubs, subs=subs)

    pub_by_topic: Dict[str, Set[str]] = defaultdict(set)
    sub_by_topic: Dict[str, Set[str]] = defaultdict(set)
    for node, topic in graph.pubs:
        pub_by_topic[topic].add(node)
    for node, topic in graph.subs:
        sub_by_topic[topic].add(node)

    topics = set(graph.topics)
    if not keep_leaves:
        # A topic with a publisher but no subscriber (or vice versa) is real,
        # but rqt_graph hides them by default because they double the clutter
        # for no structural information. Keep the highlighted ones regardless
        # -- an unconsumed mocap topic is exactly the kind of thing to notice.
        topics = {t for t in topics
                  if (pub_by_topic[t] and sub_by_topic[t]) or is_hl_topic(t)}
    if only_mocap:
        topics = {t for t in topics if is_hl_topic(t)}

    nodes_in_play = {n for t in topics for n in pub_by_topic[t] | sub_by_topic[t]}
    if not only_mocap:
        nodes_in_play |= graph.nodes

    by_ns: Dict[str, List[str]] = defaultdict(list)
    for node in sorted(nodes_in_play):
        by_ns[namespace_of(node)].append(node)
    # A cluster is a namespace that actually contains NODES (a drone). A topic
    # prefix like /mocap/... or /fake_mocap/... is not a namespace in that
    # sense -- nothing lives there -- so those topics belong to the shared
    # cluster, not to a box of their own.
    real_ns = {ns for ns in by_ns if ns}
    topic_ns: Dict[str, List[str]] = defaultdict(list)
    for topic in sorted(topics):
        ns = namespace_of(topic)
        topic_ns[ns if ns in real_ns else ""].append(topic)

    out: List[str] = []
    out.append("digraph ros {")
    out.append('  graph [rankdir=LR, splines=true, nodesep=0.25, ranksep=0.9, '
               'fontname="Helvetica", fontsize=11, '
               f'label={dot_id(title)}, labelloc=t, labeljust=l, pad=0.3, bgcolor="white"];')
    out.append('  node  [fontname="Helvetica", fontsize=9, style="filled", margin="0.08,0.04"];')
    out.append('  edge  [fontname="Helvetica", fontsize=7, arrowsize=0.6, penwidth=0.9];')

    def emit_node(node: str) -> str:
        hl = is_hl_node(node)
        return (f'  {dot_id(node)} [shape=ellipse, label={dot_id(node)}, '
                f'fillcolor="{C_HL_NODE if hl else C_NODE}", '
                f'color="{C_HL_NODE_EDGE if hl else C_NODE_EDGE}", penwidth={1.6 if hl else 1.0}];')

    def emit_topic(topic: str) -> str:
        hl = is_hl_topic(topic)
        label = f"{topic}\\n{short_type(graph.topics.get(topic, '?'))}"
        return (f'  {dot_id(topic)} [shape=box, label={dot_id(label)}, '
                f'fillcolor="{C_HL_TOPIC if hl else C_TOPIC}", '
                f'color="{C_HL_EDGE if hl else C_TOPIC_EDGE}", penwidth={1.4 if hl else 0.8}];')

    # One cluster per namespace. Topics live in the cluster of their
    # namespace too, which keeps a drone's private topics next to its nodes
    # and leaves the shared ones (/mocap/..., /tf) in the middle.
    all_ns = sorted(set(by_ns) | set(topic_ns), key=lambda s: (s == "", s))
    for ns in all_ns:
        members_n = by_ns.get(ns, [])
        members_t = topic_ns.get(ns, [])
        if not members_n and not members_t:
            continue
        if ns:
            out.append(f'  subgraph {dot_id("cluster_" + ns)} {{')
            out.append(f'    label={dot_id(ns)}; style="rounded,filled"; '
                       f'fillcolor="{C_CLUSTER}"; color="{C_CLUSTER_EDGE}"; fontsize=13; fontname="Helvetica-Bold";')
        else:
            out.append('  subgraph "cluster_ground" {')
            out.append(f'    label="ground station / shared"; style="rounded,filled"; '
                       f'fillcolor="{C_CLUSTER}"; color="{C_CLUSTER_EDGE}"; fontsize=13; fontname="Helvetica-Bold";')
        for node in members_n:
            out.append("  " + emit_node(node))
        for topic in members_t:
            out.append("  " + emit_topic(topic))
        out.append("  }")

    for topic in sorted(topics):
        hl = is_hl_topic(topic)
        colour = C_HL_EDGE if hl else C_EDGE
        width = 1.6 if hl else 0.9
        for node in sorted(pub_by_topic[topic]):
            if node in nodes_in_play:
                out.append(f'  {dot_id(node)} -> {dot_id(topic)} [color="{colour}", penwidth={width}];')
        for node in sorted(sub_by_topic[topic]):
            if node in nodes_in_play:
                out.append(f'  {dot_id(topic)} -> {dot_id(node)} [color="{colour}", penwidth={width}];')

    out.append("}")
    return "\n".join(out) + "\n"


# --------------------------------------------------------------------------- #
# rendering
# --------------------------------------------------------------------------- #

def render(dot_path: Path, fmt: str, dpi: Optional[int] = None) -> Path:
    out = dot_path.with_suffix("." + fmt)
    cmd = ["dot", f"-T{fmt}", str(dot_path), "-o", str(out)]
    if dpi and fmt == "png":
        cmd.insert(1, f"-Gdpi={dpi}")
    subprocess.run(cmd, check=True, timeout=120)
    return out


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--title", default="ROS 2 graph")
    ap.add_argument("--settle", type=float, default=3.0,
                    help="seconds to let discovery complete before reading the graph")
    ap.add_argument("--keep-leaves", action="store_true",
                    help="keep topics that have only a publisher or only a subscriber")
    ap.add_argument("--only-mocap", action="store_true",
                    help="draw only the highlighted localisation/safety chain")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--dump-json", type=Path, default=None,
                    help="also save the collected graph, so it can be re-rendered later")
    ap.add_argument("--from-json", type=Path, default=None,
                    help="render from a saved dump instead of a live graph (no ROS needed)")
    args = ap.parse_args(argv)

    if args.from_json:
        graph = load_json(args.from_json)
    else:
        try:
            graph = collect(args.settle)
        except ImportError as exc:
            print(f"ros_graph: ROS 2 is not available ({exc}); source the workspace first",
                  file=sys.stderr)
            return 2
        if args.dump_json:
            save_json(graph, args.dump_json)

    if not graph.nodes:
        print("ros_graph: no nodes discovered -- is the stack running on this domain?",
              file=sys.stderr)
        return 1

    out_base = Path(args.out).expanduser()
    out_base.parent.mkdir(parents=True, exist_ok=True)
    dot_path = out_base.with_suffix(".dot")
    dot_path.write_text(to_dot(graph, args.title, args.keep_leaves, args.only_mocap),
                        encoding="utf-8")

    written = [dot_path]
    for fmt in ("svg", "png"):
        try:
            written.append(render(dot_path, fmt, args.dpi))
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired,
                FileNotFoundError) as exc:
            print(f"ros_graph: {fmt} render failed: {exc}", file=sys.stderr)

    print(f"nodes: {len(graph.nodes)}   topics: {len(graph.topics)}   "
          f"pub edges: {len(graph.pubs)}   sub edges: {len(graph.subs)}")
    for path in written:
        print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
