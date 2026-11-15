#!/usr/bin/env python3
"""Records one navigation run and writes its metrics, then exits (the launch
file shuts the stack down when it does).

Writes to `out_dir`: traj.csv (ground truth and EKF at identical stamps),
plans.csv (every non-empty /plan), events.csv (/task_bt/events),
world.pgm (the simulator's true world incl. clutter) and summary.json.
Stops at mission_success / mission_failure, or after `timeout` seconds.
"""
import csv
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_metrics  # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class Recorder(Node):
    def __init__(self):
        super().__init__("nav_recorder")
        self.out = self.declare_parameter("out_dir", "/tmp/run").value
        self.goal = list(self.declare_parameter("goal", [10.2, 6.6]).value)
        self.timeout = self.declare_parameter("timeout", 120.0).value
        os.makedirs(self.out, exist_ok=True)
        self.t0 = time.monotonic()
        self.truth = {}
        self.rows = []
        self.plans = []
        self.events = []
        self.world = None
        self.mission_start = None
        self.done = False
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Odometry, "ground_truth", self.on_truth, 50)
        self.create_subscription(PoseWithCovarianceStamped, "ekf/pose", self.on_ekf, 50)
        self.create_subscription(Path, "plan", self.on_plan, 10)
        self.create_subscription(String, "task_bt/events", self.on_event, latched)
        self.create_subscription(OccupancyGrid, "sim/world", self.on_world,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(0.5, self.check_timeout)

    def now(self):
        return time.monotonic() - self.t0

    def on_truth(self, m):
        p = m.pose.pose
        self.truth[stamp_ns(m.header)] = (p.position.x, p.position.y, yaw_of(p.orientation))
        if len(self.truth) > 500:  # EKF lags by at most a few samples
            for k in sorted(self.truth)[:250]:
                del self.truth[k]

    def on_ekf(self, m):
        g = self.truth.get(stamp_ns(m.header))
        if g is None or self.mission_start is None:
            return
        p = m.pose.pose
        self.rows.append((self.now(),) + g + (p.position.x, p.position.y, yaw_of(p.orientation)))

    def on_plan(self, m):
        if m.poses:
            self.plans.append((self.now(), [(q.pose.position.x, q.pose.position.y)
                                            for q in m.poses]))

    def on_world(self, m):
        self.world = m

    def on_event(self, m):
        t = self.now()
        self.events.append((t, m.data))
        if m.data.startswith("mission_start"):
            self.mission_start = t
        if m.data in ("mission_success", "mission_failure"):
            self.finish()

    def check_timeout(self):
        if not self.done and self.now() > self.timeout:
            self.events.append((self.now(), "timeout"))
            self.finish()

    def finish(self):
        if self.done:
            return
        self.done = True
        with open(os.path.join(self.out, "traj.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "gt_x", "gt_y", "gt_theta", "est_x", "est_y", "est_theta"])
            w.writerows([f"{v:.4f}" for v in r] for r in self.rows)
        with open(os.path.join(self.out, "plans.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["plan", "t", "x", "y"])
            for i, (t, pts) in enumerate(self.plans):
                w.writerows([i, f"{t:.3f}", f"{x:.3f}", f"{y:.3f}"] for x, y in pts)
        with open(os.path.join(self.out, "events.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "event"])
            w.writerows([f"{t:.3f}", e] for t, e in self.events)
        if self.world is not None:
            self.write_world()
        gt = [r[1:4] for r in self.rows]
        est = [r[4:7] for r in self.rows]
        plan = self.plans[0][1] if self.plans else None
        summary = nav_metrics.summarize(gt, est, plan, self.events, self.goal,
                                        self.mission_start, self.now())
        with open(os.path.join(self.out, "summary.json"), "w") as f:
            json.dump(summary, f, indent=1)
        self.get_logger().info(f"run recorded: {summary}")

    def write_world(self):
        """PGM in map_server convention (row 0 = top), 0 = occupied."""
        info = self.world.info
        w, h = info.width, info.height
        px = bytearray(w * h)
        for y in range(h):
            row = self.world.data[y * w:(y + 1) * w]
            px[(h - 1 - y) * w:(h - y) * w] = bytes(0 if v > 50 else 254 for v in row)
        with open(os.path.join(self.out, "world.pgm"), "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode() + bytes(px))
        with open(os.path.join(self.out, "world.json"), "w") as f:
            json.dump({"resolution": info.resolution, "origin": [info.origin.position.x,
                                                                 info.origin.position.y]}, f)


def main():
    rclpy.init()
    node = Recorder()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
