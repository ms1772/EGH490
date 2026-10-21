#!/usr/bin/env python3
"""Export the live ROS 2 graph the way rqt_graph draws it, without opening rqt_graph.

This is rqt_graph's own code path -- rqt_graph.rosgraph2_impl.Graph to read the
DDS discovery graph, rqt_graph.dotcode.RosGraphDotcodeGenerator to turn it into
DOT, qt_dotgraph's PydotFactory to lay it out -- with the GUI replaced by a
command line. The .dot it writes is byte-for-byte what the GUI's "Save as DOT"
button writes for the same settings; the .svg/.png are graphviz renders of it.

Why this exists next to ros_graph.py: ros_graph.py is our own drawing, tuned
for the thesis (clusters per drone, mocap chain highlighted). This one is the
canonical rqt_graph picture, for when "what does rqt_graph show" is the
question -- and it can run headless, in a script, on a machine with no display.

The defaults are deliberately NOT rqt_graph's defaults. rqt_graph hides
"dead sinks" (topics with no subscriber) and "leaf topics" (topics with only
one connection) unless told otherwise, and on a desk with no flight controller
those are exactly the topics worth seeing: the platform's /uav_N/fmu/out/*
subscriptions with nobody publishing, its /uav_N/fmu/in/* publishers with
nobody listening. Here they are SHOWN unless --hide-dead-sinks /
--hide-leaf-topics is passed. Everything else matches the GUI defaults:
Nodes/Topics (all), namespaces clustered two levels deep, debug (/rosout,
rqt itself) and /parameter_events hidden, /tf grouped, unreachable nodes hidden.

Usage (with ROS sourced and the stack running):
    python3 flight_ops/rqt_export.py --out /path/to/graph
        -> graph.dot graph.svg graph.png graph_nodes.txt graph_topics.txt

    # drop tf2's per-listener helper nodes (nine per drone) for a readable figure
    python3 flight_ops/rqt_export.py --out graph --ns-filter '/,-.*transform_listener_impl.*'

    # what a stock rqt_graph window shows, for contrast
    python3 flight_ops/rqt_export.py --out graph --hide-dead-sinks --hide-leaf-topics

    # the same graph, about half the picture: labels relative to their cluster,
    # boxes sized to their text, tight node spacing (see --compact below)
    python3 flight_ops/rqt_export.py --out graph_compact --compact

    # re-render an earlier export, no ROS needed
    python3 flight_ops/rqt_export.py --from-dot graph.dot --out graph_compact --compact

    # in ros_graph.py's colours (mocap / localisation / safety chain in orange)
    python3 flight_ops/rqt_export.py --out graph --compact --colour

Filters use rqt_graph's own grammar: comma-separated, a leading '-' excludes,
anything with characters outside [A-Za-z0-9_/] is a start-anchored regex.

Exit status: 0 on success, 1 if no nodes were discovered or if a topic present
in discovery is missing from the DOT while the hide filters are off (which would
mean the export is not the "all topics" picture it claims to be), 2 if ROS 2
or rqt_graph cannot be imported.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Set

# Topics rqt_graph hides under its defaults for reasons unrelated to structure,
# so their absence from the DOT is expected and not an export failure.
ALWAYS_HIDDEN = {"/rosout", "/parameter_events"}
# With group_tf on, /tf and /tf_static are drawn as one group node, so the
# literal names may not appear.
TF_TOPICS = {"/tf", "/tf_static"}

GRAPH_MODES = {
    "nodes": "NODE_NODE_GRAPH",           # "Nodes only"
    "active": "NODE_TOPIC_GRAPH",         # "Nodes/Topics (active)"
    "all": "NODE_TOPIC_ALL_GRAPH",        # "Nodes/Topics (all)"  <- GUI index 2
}


def collect(settle_s: float):
    """Return (graph, topic_types) -- the same Graph object the GUI builds."""
    import rclpy
    from rqt_graph.rosgraph2_impl import Graph

    rclpy.init()
    # Named like rqt's own node so the "Hide: Debug" filter drops it from the
    # picture, exactly as it drops the real rqt_graph.
    node = rclpy.create_node("rqt_gui_py_node_%d" % os.getpid())
    try:
        # Discovery is asynchronous. The GUI fetches at t=0 and relies on the
        # user pressing Refresh; here we just wait.
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        graph = Graph(node)          # == RosGraph._update_rosgraph()
        graph.set_node_stale(5.0)
        graph.update()
        topic_types = {t: types for t, types in node.get_topic_names_and_types()}
        return graph, topic_types, node
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise


def generate(graph, node, args) -> str:
    from rqt_graph import dotcode as dc
    from qt_dotgraph.pydotfactory import PydotFactory

    gen = dc.RosGraphDotcodeGenerator(node)
    return gen.generate_dotcode(
        rosgraphinst=graph,
        ns_filter=args.ns_filter,
        topic_filter=args.topic_filter,
        graph_mode=getattr(dc, GRAPH_MODES[args.graph_mode]),
        dotcode_factory=PydotFactory(),
        hide_single_connection_topics=args.hide_leaf_topics,
        hide_dead_end_topics=args.hide_dead_sinks,
        cluster_namespaces_level=args.cluster_level,
        accumulate_actions=True,
        orientation="LR",
        quiet=not args.show_debug,
        unreachable=not args.show_unreachable,
        group_tf_nodes=not args.no_group_tf,
        hide_tf_nodes=args.hide_tf,
        group_image_nodes=True,
        hide_dynamic_reconfigure=not args.show_params,
    )


def edge_tables(graph):
    """pubs[topic] / subs[topic] -> node names, from the GUI's own edge list."""
    pubs: Dict[str, Set[str]] = defaultdict(set)
    subs: Dict[str, Set[str]] = defaultdict(set)
    for e in graph.nt_all_edges:
        if e.end.startswith(" "):        # node -> ' /topic'   (publisher)
            pubs[e.end[1:]].add(e.start)
        elif e.start.startswith(" "):    # ' /topic' -> node   (subscriber)
            subs[e.start[1:]].add(e.end)
    return pubs, subs


# --------------------------------------------------------------------------- #
# --compact: the same graph with the whitespace taken out
# --------------------------------------------------------------------------- #
# rqt_graph's DOT is graphviz output (dot -Tdot), so every node carries the
# width/height graphviz computed for it -- 0.5 in tall boxes for 14 pt text --
# and every label is the full topic name even when the enclosing cluster
# already says which namespace it is in. Both are honoured on re-layout and
# together they roughly double the picture. Measured on the one-drone lab
# graph: 4018 x 2362 pt as rqt draws it, 3062 x 1089 pt compact, same content.
#
# The functions below work on that machine-formatted DOT text (one attribute
# per line, tab-indented), which is what both the live path and --from-dot see.

_SIZE_LINE = re.compile(r"^(\s*)(width|height)=[\d.]+(,|\]|\];)?$")
_LABEL_LINE = re.compile(r'^(\s*)label="([^"]*)"(,?)$')


def strip_sizes(dot: str) -> str:
    """Drop the baked node width/height so nodes re-size to their labels."""
    out = []
    for line in dot.splitlines():
        m = _SIZE_LINE.match(line)
        if m:
            if m.group(3) in ("]", "];"):      # it was the last attribute
                out.append(m.group(1) + m.group(3))
            continue
        out.append(line)
    return "\n".join(out) + "\n"


def short_labels(dot: str, cluster_margin: int) -> str:
    """Strip the enclosing cluster's namespace from node labels, and give each
    cluster an explicit (small) margin.

    '/drone0/motion_reference/pose' inside cluster '/drone0/motion_reference'
    becomes 'pose'. Nodes outside any cluster keep their full name. The root
    graph's own margin is left alone: on the root it means page margin, in
    inches, and command-line -Gmargin would set both.
    """
    out = []
    stack: List[Optional[str]] = []     # cluster labels, innermost last
    in_graph_attr = False
    awaiting_label = False
    for line in dot.splitlines():
        s = line.strip()
        if s.startswith("subgraph cluster"):
            stack.append(None)
            awaiting_label = True
        elif s == "}" and stack:
            stack.pop()
        if s.startswith("graph ["):
            in_graph_attr = True
            if stack:
                line = line.replace("graph [", f"graph [margin={cluster_margin},", 1)
        m = _LABEL_LINE.match(line)
        if m and in_graph_attr:
            if awaiting_label and stack and stack[-1] is None:
                stack[-1] = m.group(2)
                awaiting_label = False
        elif m and not in_graph_attr:
            ns = next((c for c in reversed(stack) if c), None)
            label = m.group(2)
            if ns and label.startswith(ns + "/") and len(label) > len(ns) + 1:
                line = f'{m.group(1)}label="{label[len(ns) + 1:]}"{m.group(3)}'
        if s.endswith("];") or s.endswith("]"):
            in_graph_attr = False
        out.append(line)
    return "\n".join(out) + "\n"


def compact(dot: str, cluster_margin: int = 3) -> str:
    return short_labels(strip_sizes(dot), cluster_margin)


# --------------------------------------------------------------------------- #
# --colour: ros_graph.py's palette on rqt_graph's picture
# --------------------------------------------------------------------------- #
# rqt_graph draws everything black on white. This paints it the way
# ros_graph.py paints its own figures -- same constants, same regexes, so the
# two kinds of figure read as one set: ROS nodes blue-grey, topics white, and
# anything on the localisation / safety chain (mocap, VRPN, rigid bodies,
# self_localization, /tf, the platform, the guard) in orange, edges included.
# Names come from each stanza's tooltip, which rqt fills with the full name
# whatever the label says, so this works before or after --compact.

_STANZA_START = re.compile(r"^(\s*)(.+?)\t\[(.*)$")     # node or edge attr list
_TOOLTIP = re.compile(r'tooltip="(topic:)?([^"]*)"')
_EDGE_ID = re.compile(r"^(\S+) -> (\S+)$")


def colourise(dot: str) -> str:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ros_graph import (C_CLUSTER, C_CLUSTER_EDGE, C_EDGE, C_HL_EDGE, C_HL_NODE,
                           C_HL_NODE_EDGE, C_HL_TOPIC, C_NODE, C_NODE_EDGE, C_TOPIC,
                           C_TOPIC_EDGE, is_hl_node, is_hl_topic)

    lines = dot.splitlines()

    # Group the text into stanzas: a node/edge attribute list runs from a
    # line with '<id>\t[' to the line ending in '];' and every other line is
    # its own stanza. Cluster 'graph [' blocks are left as lines and handled
    # by a plain substitution below.
    stanzas: List[List[str]] = []
    cur: Optional[List[str]] = None
    for line in lines:
        if cur is not None:
            cur.append(line)
            if line.rstrip().endswith("];"):
                stanzas.append(cur)
                cur = None
            continue
        m = _STANZA_START.match(line)
        if m and not m.group(2).strip().startswith(("graph", "node", "edge")):
            cur = [line]
            if line.rstrip().endswith("];"):
                stanzas.append(cur)
                cur = None
        else:
            stanzas.append([line])
    if cur is not None:
        stanzas.append(cur)

    # Pass 1: what every id is called, and whether it is a topic.
    names: Dict[str, tuple] = {}
    for st in stanzas:
        m = _STANZA_START.match(st[0])
        if not m or _EDGE_ID.match(m.group(2)):
            continue
        t = _TOOLTIP.search("\n".join(st))
        if t:
            names[m.group(2)] = (t.group(2), t.group(1) is not None)

    out: List[str] = []
    for st in stanzas:
        m = _STANZA_START.match(st[0])
        if not m:
            out.extend(st)
            continue
        indent, ident, rest = m.groups()
        e = _EDGE_ID.match(ident)
        if e:
            topic = next((names[x][0] for x in e.groups() if x in names and names[x][1]), "")
            hl = is_hl_topic(topic)
            colour, width = (C_HL_EDGE, 1.6) if hl else (C_EDGE, 0.9)
            text = "\n".join(st)
            text, n = re.subn(r"penwidth=[\d.]+(?=[,\]])",
                              f'penwidth={width},color="{colour}"', text, count=1)
            if n == 0:
                text = f'{indent}{ident}\t[penwidth={width},color="{colour}",{rest}' \
                       + ("\n" + "\n".join(st[1:]) if len(st) > 1 else "")
            out.append(text)
        elif ident in names:
            name, is_topic = names[ident]
            if is_topic:
                hl = is_hl_topic(name)
                fill, edge, width = ((C_HL_TOPIC, C_HL_EDGE, 1.4) if hl
                                     else (C_TOPIC, C_TOPIC_EDGE, 0.8))
            else:
                hl = is_hl_node(name)
                fill, edge, width = ((C_HL_NODE, C_HL_NODE_EDGE, 1.6) if hl
                                     else (C_NODE, C_NODE_EDGE, 1.0))
            out.append(f'{indent}{ident}\t[style=filled,fillcolor="{fill}",'
                       f'color="{edge}",penwidth={width},{rest}')
            out.extend(st[1:])
        else:
            out.extend(st)

    text = "\n".join(out) + "\n"
    # Clusters: rqt's bold outline -> ros_graph.py's soft filled panel.
    text = text.replace("style=bold",
                        f'style="rounded,filled",fillcolor="{C_CLUSTER}",color="{C_CLUSTER_EDGE}"')
    return text


def render(dot_path: Path, fmt: str, title: str, dpi: Optional[int],
           compact_layout: bool = False, fontsize: int = 14, colour: bool = False) -> Path:
    out = dot_path.with_suffix("." + fmt)
    cmd = ["dot", f"-T{fmt}", f"-Glabel={title}", "-Glabelloc=t", "-Glabeljust=l",
           "-Gfontname=Helvetica"]
    if compact_layout:
        # nodesep is the gap between neighbours in a rank (vertical, in LR);
        # graphviz's default is 0.25 in. The node minimum size is the other
        # half of the saving: without it a 14 pt label still gets a 0.5 in box.
        cmd += ["-Gnodesep=0.05", "-Nmargin=0.05,0.02", "-Nheight=0.2", "-Nwidth=0.2",
                f"-Nfontsize={fontsize}", f"-Gfontsize={fontsize}"]
    else:
        cmd += [f"-Gfontsize={fontsize}"]
    if colour:
        cmd += ["-Gbgcolor=white", "-Nfontname=Helvetica", "-Efontname=Helvetica"]
    if dpi and fmt == "png":
        cmd.append(f"-Gdpi={dpi}")
    cmd += [str(dot_path), "-o", str(out)]
    subprocess.run(cmd, check=True, timeout=300)
    return out


def render_all(dot_path: Path, args) -> List[Path]:
    written = []
    for fmt in ("svg", "png"):
        try:
            written.append(render(dot_path, fmt, args.title, args.dpi,
                                  args.compact, args.fontsize, args.colour))
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            if fmt == "png" and args.dpi > 96:
                print(f"rqt_export: png at dpi {args.dpi} failed ({exc}); retrying at 96",
                      file=sys.stderr)
                try:
                    written.append(render(dot_path, fmt, args.title, 96,
                                          args.compact, args.fontsize, args.colour))
                    continue
                except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc2:
                    exc = exc2
            print(f"rqt_export: {fmt} render failed: {exc}", file=sys.stderr)
    return written


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output path without extension")
    ap.add_argument("--title", default="rqt_graph")
    ap.add_argument("--settle", type=float, default=8.0,
                    help="seconds to let discovery complete before reading the graph")
    ap.add_argument("--ns-filter", default="/", help="rqt_graph node filter (default '/')")
    ap.add_argument("--topic-filter", default="/", help="rqt_graph topic filter (default '/')")
    ap.add_argument("--graph-mode", choices=sorted(GRAPH_MODES), default="all")
    ap.add_argument("--cluster-level", type=int, default=2,
                    help="namespace cluster depth, as the GUI spin box (default 2)")
    ap.add_argument("--hide-dead-sinks", action="store_true",
                    help="GUI 'Hide: Dead sinks' -- drop topics with no subscriber")
    ap.add_argument("--hide-leaf-topics", action="store_true",
                    help="GUI 'Hide: Leaf topics' -- drop topics with one connection")
    ap.add_argument("--show-debug", action="store_true",
                    help="untick GUI 'Hide: Debug' (/rosout, rqt, _-prefixed nodes)")
    ap.add_argument("--show-params", action="store_true",
                    help="untick GUI 'Hide: Params' (/parameter_events)")
    ap.add_argument("--show-unreachable", action="store_true",
                    help="untick GUI 'Hide: Unreachable'")
    ap.add_argument("--hide-tf", action="store_true", help="GUI 'Hide: tf'")
    ap.add_argument("--no-group-tf", action="store_true", help="untick GUI 'Group: tf'")
    ap.add_argument("--dpi", type=int, default=150)
    ap.add_argument("--compact", action="store_true",
                    help="tight layout: labels relative to their cluster, boxes sized to "
                         "their text, nodesep 0.05. Same graph, roughly half the picture. "
                         "The .dot is then derived, not the GUI's byte-identical one.")
    ap.add_argument("--colour", "--color", action="store_true",
                    help="paint it with ros_graph.py's palette: nodes blue-grey, topics "
                         "white, the mocap/localisation/safety chain orange")
    ap.add_argument("--fontsize", type=int, default=None,
                    help="node/cluster font size (default 14; 16 with --compact)")
    ap.add_argument("--from-dot", type=Path, default=None,
                    help="re-render a .dot written by an earlier run instead of reading "
                         "the live graph (no ROS needed); pairs with --compact")
    args = ap.parse_args(argv)
    if args.fontsize is None:
        args.fontsize = 16 if args.compact else 14

    out_base = Path(args.out).expanduser()
    out_base.parent.mkdir(parents=True, exist_ok=True)
    dot_path = out_base.with_suffix(".dot")

    if args.from_dot:
        dotcode = args.from_dot.read_text(encoding="utf-8")
        if args.compact:
            dotcode = compact(dotcode)
        if args.colour:
            dotcode = colourise(dotcode)
        dot_path.write_text(dotcode, encoding="utf-8")
        written = [dot_path] + render_all(dot_path, args)
        print(f"re-rendered {args.from_dot}"
              + (" (compact)" if args.compact else "") + (" (colour)" if args.colour else ""))
        for path in written:
            print(f"wrote {path}")
        return 0

    try:
        graph, topic_types, node = collect(args.settle)
    except ImportError as exc:
        print(f"rqt_export: cannot import ROS 2 / rqt_graph ({exc}); source the workspace "
              "and install ros-humble-rqt-graph", file=sys.stderr)
        return 2

    try:
        dotcode = generate(graph, node, args)
        pubs, subs = edge_tables(graph)
        nodes = sorted(graph.nn_nodes)
        topics = sorted(t[1:] for t in graph.nt_nodes)
    finally:
        import rclpy
        node.destroy_node()
        rclpy.shutdown()

    if not nodes:
        print("rqt_export: no nodes discovered -- is the stack running on this domain?",
              file=sys.stderr)
        return 1

    # The cross-check below reads the DOT as rqt produced it; the compact
    # rewrite only touches labels and sizes, but keep the check on the source.
    rqt_dotcode = dotcode
    if args.compact:
        dotcode = compact(dotcode)
    if args.colour:
        dotcode = colourise(dotcode)
    dot_path.write_text(dotcode, encoding="utf-8")
    written = [dot_path] + render_all(dot_path, args)

    nodes_path = Path(f"{out_base}_nodes.txt")
    nodes_path.write_text("\n".join(nodes) + "\n", encoding="utf-8")
    one_ended = [t for t in topics if not pubs[t] or not subs[t]]
    lines = []
    for t in topics:
        types = ",".join(topic_types.get(t, ["?"]))
        flag = "  ONE-ENDED" if t in one_ended else ""
        lines.append(f"{t}\t{types}\tpubs={len(pubs[t])}\tsubs={len(subs[t])}{flag}")
    topics_path = Path(f"{out_base}_topics.txt")
    topics_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    written += [nodes_path, topics_path]

    # The claim this tool makes is "every topic discovery can see is in the
    # picture". Check it, unless the caller asked for topics to be hidden.
    rc = 0
    if not (args.hide_dead_sinks or args.hide_leaf_topics or args.hide_tf
            or args.topic_filter != "/"):
        expected = set(topic_types) - ALWAYS_HIDDEN
        if not args.no_group_tf:
            expected -= TF_TOPICS
        missing = sorted(t for t in expected if t not in rqt_dotcode)
        if missing:
            rc = 1
            print(f"rqt_export: {len(missing)} discovered topic(s) missing from the DOT:",
                  file=sys.stderr)
            for t in missing:
                print(f"    {t}", file=sys.stderr)
        else:
            print(f"rqt_export: cross-check OK -- all {len(expected)} discovered topics are in the DOT")

    print(f"nodes: {len(nodes)}   topics: {len(topics)}   one-ended: {len(one_ended)}   "
          f"pub edges: {sum(len(v) for v in pubs.values())}   "
          f"sub edges: {sum(len(v) for v in subs.values())}")
    for path in written:
        print(f"wrote {path}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
