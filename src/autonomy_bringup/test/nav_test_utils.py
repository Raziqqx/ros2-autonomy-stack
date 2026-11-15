"""Shared helpers for the navigation launch tests: launch the full stack and
watch the behaviour tree's events and the localisation error."""
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def include_stack(**args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "launch", "navigation.launch.py"])),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class MissionWatcher:
    """Collects /task_bt/events and EKF-vs-ground-truth position errors."""

    def __init__(self, node):
        self.node = node
        self.events = []  # (monotonic time, text)
        self.truth = {}
        self.errors = []
        latched = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(String, "task_bt/events",
                                 lambda m: self.events.append((time.monotonic(), m.data)), latched)
        node.create_subscription(Odometry, "ground_truth", self.on_truth, 50)
        node.create_subscription(PoseWithCovarianceStamped, "ekf/pose", self.on_ekf, 50)

    def on_truth(self, m):
        self.truth[stamp_ns(m.header)] = (m.pose.pose.position.x, m.pose.pose.position.y)

    def on_ekf(self, m):
        g = self.truth.pop(stamp_ns(m.header), None)
        if g is not None and self.started():
            p = m.pose.pose.position
            self.errors.append(math.hypot(p.x - g[0], p.y - g[1]))

    def names(self):
        return [e for _, e in self.events]

    def started(self):
        return any(e.startswith("mission_start") for e in self.names())

    def time_of(self, prefix):
        return next((t for t, e in self.events if e.startswith(prefix)), None)

    def spin_until_done(self, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if any(e in ("mission_success", "mission_failure") for e in self.names()):
                return True
        return False

    def ate_rmse(self):
        return math.sqrt(sum(e * e for e in self.errors) / len(self.errors))
