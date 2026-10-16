#!/usr/bin/env python3
"""Export the live ROS 2 node/topic graph as JSON and Graphviz DOT.

Like rqt_graph, but headless (runs over SSH on the robot) and reproducible:
    python3 export_ros_graph.py OUT_PREFIX [--wait 4]
writes OUT_PREFIX.json (every node with its publishers and subscribers, with
types) and OUT_PREFIX.dot (nodes as ellipses, topics as boxes). The usual
noise is hidden: /rosout, /parameter_events, hidden nodes, this tool and the
ros2 CLI daemon. Render with: dot -Tpng OUT_PREFIX.dot -o OUT_PREFIX.png
"""
# References (full entries: docs/references.md at the repository root):
#   [R10] ROS 2 Jazzy documentation: rclpy graph API

import json
import sys
import time

import rclpy
from rclpy.node import Node

HIDE_TOPICS = {"/rosout", "/parameter_events"}
HIDE_NODES = ("_ros2cli", "graph_export", "launch_ros")


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    prefix = sys.argv[1]
    wait = float(sys.argv[sys.argv.index("--wait") + 1]) if "--wait" in sys.argv else 4.0
    rclpy.init()
    node = Node("graph_export")
    end = time.time() + wait                        # let discovery settle
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    graph = {}
    for name, ns in sorted(node.get_node_names_and_namespaces()):
        if name.startswith("_") or any(h in name for h in HIDE_NODES):
            continue
        full = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
        pubs = node.get_publisher_names_and_types_by_node(name, ns)
        subs = node.get_subscriber_names_and_types_by_node(name, ns)
        graph[full] = {
            "publishes": {t: ty for t, ty in pubs if t not in HIDE_TOPICS},
            "subscribes": {t: ty for t, ty in subs if t not in HIDE_TOPICS},
        }
    node.destroy_node()
    rclpy.shutdown()

    with open(prefix + ".json", "w") as f:
        json.dump(graph, f, indent=1, sort_keys=True)
    lines = ['digraph ros {', '  rankdir=LR; node [fontname="Helvetica", fontsize=9];']
    topics = set()
    for n, io in graph.items():
        lines.append(f'  "{n}" [shape=ellipse, style=filled, fillcolor="#dbe9f6"];')
        for t in io["publishes"]:
            topics.add(t)
            lines.append(f'  "{n}" -> "{t}";')
        for t in io["subscribes"]:
            topics.add(t)
            lines.append(f'  "{t}" -> "{n}";')
    for t in sorted(topics):
        lines.append(f'  "{t}" [shape=box, fontsize=8, color="#888888"];')
    lines.append("}")
    with open(prefix + ".dot", "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"{len(graph)} nodes, {len(topics)} topics -> {prefix}.json, {prefix}.dot")
    return 0


if __name__ == "__main__":
    sys.exit(main())
