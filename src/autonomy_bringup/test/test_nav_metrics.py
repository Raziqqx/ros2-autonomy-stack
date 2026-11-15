"""Unit tests for the evaluation metrics (no ROS needed)."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
import nav_metrics as m  # noqa: E402


def straight(n=101, length=10.0):
    x = np.linspace(0.0, length, n)
    return np.stack([x, np.zeros(n), np.zeros(n)], axis=1)


def test_ate_constant_offset():
    gt = straight()
    est = gt + [0.0, 0.1, 0.0]
    r = m.ate(gt, est)
    assert r["ate_rmse"] == pytest.approx(0.1)
    assert r["ate_max"] == pytest.approx(0.1)
    assert r["heading_rmse"] == pytest.approx(0.0)


def test_rpe_ignores_constant_offset_but_sees_scale_drift():
    gt = straight()
    # A constant offset is not local drift.
    assert m.rpe(gt, gt + [0.0, 0.1, 0.0])["rpe_trans"] == pytest.approx(0.0, abs=1e-9)
    # 2 % odometry scale error: 2 cm per metre.
    est = gt.copy()
    est[:, 0] *= 1.02
    assert m.rpe(gt, est, delta=1.0)["rpe_trans"] == pytest.approx(0.02, rel=1e-6)


def test_rpe_short_run_is_nan():
    assert math.isnan(m.rpe(straight(n=5, length=0.5), straight(n=5, length=0.5))["rpe_trans"])


def test_curvature_of_circle():
    r = 2.0
    a = np.linspace(0.0, math.pi, 2000)
    xy = np.stack([r * np.cos(a), r * np.sin(a)], axis=1)
    s = m.smoothness(xy, ds=0.05)
    assert s["length"] == pytest.approx(math.pi * r, rel=0.02)
    assert s["mean_abs_curv"] == pytest.approx(1.0 / r, rel=0.03)
    # Integral of k^2 ds over a half circle = pi / r.
    assert s["int_curv_sq"] == pytest.approx(math.pi / r, rel=0.05)
    assert s["total_turn"] == pytest.approx(math.pi, rel=0.03)


def test_straight_line_is_perfectly_smooth_and_corner_is_not():
    assert m.smoothness([[0, 0], [5, 0]])["mean_abs_curv"] == pytest.approx(0.0)
    corner = m.smoothness([[0, 0], [2, 0], [2, 2]])
    assert corner["total_turn"] == pytest.approx(math.pi / 2, rel=1e-6)


def test_summarize_counts_recoveries_and_time():
    gt = straight()
    events = [(1.0, "mission_start GoTo"), (5.0, "blocked"), (5.1, "recovery_start"),
              (9.0, "recovery_done"), (21.0, "mission_success")]
    s = m.summarize(gt, gt, [[0, 0], [10, 0]], events, goal=(10.0, 0.0),
                    mission_start_t=1.0, end_t=21.0)
    assert s["success"] == 1
    assert s["time_to_goal"] == pytest.approx(20.0)
    assert s["recoveries"] == 1 and s["blocked_events"] == 1
    assert s["final_goal_dist"] == pytest.approx(0.0)
    assert s["plan_length"] == pytest.approx(10.0, rel=0.01)
