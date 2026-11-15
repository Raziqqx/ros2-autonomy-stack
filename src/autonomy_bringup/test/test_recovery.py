"""Scenario test: blocked path. After the robot has driven 2 m, the simulator
drops a 1.6 m wall 0.45 m in front of it (not in any map). With pure pursuit
the robot cannot avoid it locally, so the behaviour tree must detect the
failure (controller blocked or planner start-in-collision), run Recover
(back up + rotate), replan around the wall and still reach the goal.

Budgets: at least one recovery_start followed by recovery_done, then
mission_success within TIME_BUDGET s of mission_start; ATE RMSE < ATE_BUDGET.
"""
import os
import sys
import unittest

import launch
import launch_testing.actions
import pytest
import rclpy

sys.path.insert(0, os.path.dirname(__file__))
from nav_test_utils import MissionWatcher, include_stack  # noqa: E402

TIME_BUDGET = 90.0  # s
ATE_BUDGET = 0.05  # m


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        include_stack(seed=7, planner="astar", controller="pure_pursuit", tree="GoTo",
                      drop_after_m=2.0, wait_for_events=1),
        launch_testing.actions.ReadyToTest()])


class TestRecovery(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_recovers_from_blocked_path(self):
        node = rclpy.create_node("recovery_test")
        try:
            w = MissionWatcher(node)
            self.assertTrue(w.spin_until_done(TIME_BUDGET + 30.0), f"no outcome: {w.names()}")
            names = w.names()
            print(f"events {names}, ATE RMSE {w.ate_rmse():.4f} m")
            self.assertIn("recovery_start", names)
            self.assertIn("recovery_done", names)
            self.assertLess(names.index("recovery_start"), names.index("recovery_done"))
            self.assertIn("mission_success", names)
            self.assertLess(w.time_of("mission_success") - w.time_of("mission_start"), TIME_BUDGET)
            self.assertLess(w.ate_rmse(), ATE_BUDGET)
        finally:
            node.destroy_node()
