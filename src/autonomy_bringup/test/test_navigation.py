"""Scenario test: full stack (Theta* + DWA, behaviour tree GoTo mission) from
the dock to the far corner of the warehouse.

Budgets (see DECISIONS.md): mission_success within TIME_BUDGET s of
mission_start, EKF ATE RMSE < ATE_BUDGET m and max error < MAX_ERR_BUDGET m,
no recovery needed on the clear map.
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

TIME_BUDGET = 60.0  # s
ATE_BUDGET = 0.05  # m
MAX_ERR_BUDGET = 0.15  # m


@pytest.mark.launch_test
def generate_test_description():
    return launch.LaunchDescription([
        include_stack(seed=7, planner="thetastar", controller="dwa", tree="GoTo", wait_for_events=1),
        launch_testing.actions.ReadyToTest()])


class TestNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_reaches_goal_within_budgets(self):
        node = rclpy.create_node("navigation_test")
        try:
            w = MissionWatcher(node)
            self.assertTrue(w.spin_until_done(TIME_BUDGET + 30.0), f"no outcome: {w.names()}")
            self.assertIn("mission_success", w.names(), w.names())
            elapsed = w.time_of("mission_success") - w.time_of("mission_start")
            print(f"time to goal {elapsed:.1f} s, ATE RMSE {w.ate_rmse():.4f} m, "
                  f"max {max(w.errors):.4f} m, events {w.names()}")
            self.assertLess(elapsed, TIME_BUDGET)
            self.assertLess(w.ate_rmse(), ATE_BUDGET)
            self.assertLess(max(w.errors), MAX_ERR_BUDGET)
            self.assertNotIn("recovery_start", w.names())
        finally:
            node.destroy_node()
