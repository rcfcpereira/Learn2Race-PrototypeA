#!/usr/bin/env python3
"""Achieved rate, timing and serialized size of ROS 2 topics.

    python3 topic_stats.py OUT.csv [SECONDS] [TOPIC ...]

Subscribes to each topic as raw bytes (no deserialisation cost), best-effort
QoS (matches both reliable and best-effort publishers), for SECONDS (default
30), and writes one CSV row per topic:
  messages, achieved rate, median / 90th percentile / maximum gap between
  arrivals, serialized size (exact CDR bytes, as sent on the wire), and, for
  messages starting with a std_msgs/Header, the same gap statistics on the
  header stamps (publisher-side timing, without the receive jitter).

The header-stamp gaps test the firmware scheduling hypothesis
(publishing "last = now" instead of
"last += period" lets every late message delay all the following ones).
Default topics: the micro-ROS link and the Pi-side sensors of Prototype A.
"""
# References (full entries: docs/references.md at the repository root):
#   [R10] ROS 2 Jazzy documentation: rclpy, QoS, raw subscriptions
#   [R14] micro-ROS: the embedded link whose rates are measured

import csv
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

DEFAULT_TOPICS = ["/odom", "/imu/data_raw", "/imu/mag", "/light/visible_plus_ir_raw",
                  "/battery_state", "/imu/data", "/cmd_vel", "/autonomy/cmd_vel"]
HEADER_FIRST = {"nav_msgs/msg/Odometry", "sensor_msgs/msg/Imu", "sensor_msgs/msg/MagneticField",
                "sensor_msgs/msg/BatteryState", "sensor_msgs/msg/Image"}


def gaps(times):
    if len(times) < 3:
        return "", "", ""
    d = np.diff(np.asarray(times)) * 1000.0
    return f"{np.median(d):.1f}", f"{np.percentile(d, 90):.1f}", f"{d.max():.1f}"


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
    wanted = sys.argv[3:] or DEFAULT_TOPICS
    rclpy.init()
    node = Node("topic_stats")
    t_end = time.time() + 3.0
    while time.time() < t_end:                      # discovery
        rclpy.spin_once(node, timeout_sec=0.2)
    types = dict((t, ty[0]) for t, ty in node.get_topic_names_and_types())
    data = {}
    subs = []
    for topic in wanted:
        if topic not in types:
            print(f"skip {topic}: not published")
            continue
        rec = data[topic] = {"type": types[topic], "rx": [], "stamp": [], "size": []}

        def cb(msg, rec=rec):
            rec["rx"].append(time.monotonic())
            rec["size"].append(len(msg))
            if rec["type"] in HEADER_FIRST and len(msg) >= 12:
                sec, nsec = struct.unpack_from("<iI", msg, 4)     # after the CDR encapsulation
                rec["stamp"].append(sec + nsec * 1e-9)
        subs.append(node.create_subscription(get_message(types[topic]), topic, cb,
                                             qos_profile_sensor_data, raw=True))
    print(f"recording {len(subs)} topics for {seconds:.0f} s ...")
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["topic", "type", "messages", "rate_hz", "rx_gap_median_ms", "rx_gap_p90_ms",
                    "rx_gap_max_ms", "stamp_gap_median_ms", "stamp_gap_p90_ms", "stamp_gap_max_ms",
                    "size_min_b", "size_median_b", "size_max_b"])
        for topic, rec in data.items():
            n = len(rec["rx"])
            span = rec["rx"][-1] - rec["rx"][0] if n > 1 else 0
            rate = f"{(n - 1) / span:.2f}" if span > 0 else ""
            sizes = rec["size"] or [0]
            w.writerow([topic, rec["type"], n, rate, *gaps(rec["rx"]), *gaps(sorted(rec["stamp"])),
                        min(sizes), int(np.median(sizes)), max(sizes)])
            print(f"{topic:30s} {n:5d} msgs {rate:>7s} Hz  size {int(np.median(sizes))} B")
    print("->", out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
