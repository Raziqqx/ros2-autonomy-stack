"""Scenario test: drive a fixed open-loop /cmd_vel pattern through the
warehouse, then check the error budgets.

Pass criteria (budgets, see DECISIONS.md):
  * EKF position error vs ground truth: max < MAX_POS_ERR, RMS < RMS_POS_ERR
  * EKF heading error: max < MAX_YAW_ERR
  * the mapper's /map has more than MIN_OCCUPIED occupied cells
  * the robot actually moved (ground-truth path length) and did not get stuck
Raw wheel-odometry error is reported alongside for comparison.
"""
import math
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MAX_POS_ERR = 0.10  # m
RMS_POS_ERR = 0.05  # m
MAX_YAW_ERR = 0.05  # rad
MIN_OCCUPIED = 500  # cells
MIN_PATH_LENGTH = 8.0  # m (commanded: 8.8 m)

START = (1.5, 1.5, 0.0)  # must match config/sim.yaml
QUARTER_TURN = (math.pi / 2) / 0.5  # seconds at 0.5 rad/s
# (duration s, v m/s, w rad/s): a loop around the free area near the start.
PATTERN = [
    (1.0, 0.0, 0.0),
    (7.0, 0.4, 0.0),
    (QUARTER_TURN, 0.0, 0.5),
    (5.0, 0.4, 0.0),
    (QUARTER_TURN, 0.0, 0.5),
    (6.0, 0.4, 0.0),
    (QUARTER_TURN, 0.0, 0.5),
    (4.0, 0.4, 0.0),
    (1.0, 0.0, 0.0),
]


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@pytest.mark.launch_test
def generate_test_description():
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("autonomy_bringup"), "launch", "localization_mapping.launch.py"])),
        launch_arguments={"seed": "7"}.items())
    return launch.LaunchDescription([stack, launch_testing.actions.ReadyToTest()])


class TestLocalizationMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("scenario_driver")
        self.truth = {}  # stamp (ns) -> (x, y, yaw)
        self.ekf_err = []  # (position error, yaw error)
        self.odom_err = []
        self.path_length = 0.0
        self.last_truth = None
        self.map = None
        n = self.node
        self.cmd_pub = n.create_publisher(Twist, "cmd_vel", 10)
        n.create_subscription(Odometry, "ground_truth", self.on_truth, 100)
        n.create_subscription(PoseWithCovarianceStamped, "ekf/pose", self.on_ekf, 100)
        n.create_subscription(Odometry, "odom", self.on_odom, 100)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        n.create_subscription(OccupancyGrid, "map", self.on_map, map_qos)

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def stamp_ns(header) -> int:
        return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec

    def on_truth(self, msg):
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.truth[self.stamp_ns(msg.header)] = pose
        if self.last_truth is not None:
            self.path_length += math.dist(pose[:2], self.last_truth[:2])
        self.last_truth = pose

    def error_to_truth(self, header, x, y, yaw, out):
        # The sim stamps odom and ground truth identically; the EKF republishes
        # the odom stamp, so samples pair up exactly.
        t = self.truth.get(self.stamp_ns(header))
        if t is not None:
            out.append((math.hypot(x - t[0], y - t[1]), abs(wrap(yaw - t[2]))))

    def on_ekf(self, msg):
        p = msg.pose.pose
        self.error_to_truth(msg.header, p.position.x, p.position.y, yaw_of(p.orientation),
                            self.ekf_err)

    def on_odom(self, msg):
        # Odometry starts at the odom origin; express it in the map frame.
        p = msg.pose.pose
        c, s = math.cos(START[2]), math.sin(START[2])
        x = START[0] + c * p.position.x - s * p.position.y
        y = START[1] + s * p.position.x + c * p.position.y
        self.error_to_truth(msg.header, x, y, START[2] + yaw_of(p.orientation), self.odom_err)

    def on_map(self, msg):
        self.map = msg

    def spin_for(self, seconds, cmd=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_ekf_tracks_ground_truth_and_map_is_built(self):
        # Wait for the stack to come up (first EKF pose), at most 20 s.
        deadline = time.monotonic() + 20.0
        while not self.ekf_err and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self.ekf_err, "no /ekf/pose paired with /ground_truth")
        self.ekf_err.clear()
        self.odom_err.clear()

        for duration, v, w in PATTERN:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = v, w
            self.spin_for(duration, cmd)
        self.spin_for(1.5)  # let the last map publish arrive

        pos = [e[0] for e in self.ekf_err]
        yaw = [e[1] for e in self.ekf_err]
        rms = math.sqrt(sum(e * e for e in pos) / len(pos))
        odom_max = max(e[0] for e in self.odom_err)
        occupied = sum(1 for v in self.map.data if v > 65) if self.map else 0
        print(f"\nSCENARIO samples={len(pos)} path={self.path_length:.2f}m "
              f"ekf_max={max(pos):.3f}m ekf_rms={rms:.3f}m ekf_yaw_max={max(yaw):.3f}rad "
              f"odom_max={odom_max:.3f}m occupied_cells={occupied}")

        self.assertGreater(len(pos), 500)
        self.assertGreater(self.path_length, MIN_PATH_LENGTH)
        self.assertLess(max(pos), MAX_POS_ERR)
        self.assertLess(rms, RMS_POS_ERR)
        self.assertLess(max(yaw), MAX_YAW_ERR)
        self.assertIsNotNone(self.map, "no /map received")
        self.assertGreater(occupied, MIN_OCCUPIED)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # SIGINT on shutdown is a clean exit for rclcpp nodes.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
