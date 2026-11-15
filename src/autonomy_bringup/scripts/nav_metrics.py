"""Evaluation metrics for one navigation run (pure numpy, no ROS).

Trajectories are N x 3 arrays of (x, y, theta) sampled at the same stamps.
"""
import math

import numpy as np


def wrap(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def ate(gt, est):
    """Absolute trajectory error: RMSE and max of the position error (m) and
    RMSE of the heading error (rad). No alignment: the EKF starts at the
    known dock pose in the same map frame as the ground truth."""
    gt, est = np.asarray(gt, float), np.asarray(est, float)
    d = np.hypot(est[:, 0] - gt[:, 0], est[:, 1] - gt[:, 1])
    dth = wrap(est[:, 2] - gt[:, 2])
    return {"ate_rmse": float(np.sqrt(np.mean(d ** 2))), "ate_max": float(d.max()),
            "heading_rmse": float(np.sqrt(np.mean(dth ** 2)))}


def _relative(p, q):
    """Pose of q in the frame of p, for arrays of poses."""
    dx, dy = q[:, 0] - p[:, 0], q[:, 1] - p[:, 1]
    c, s = np.cos(p[:, 2]), np.sin(p[:, 2])
    return np.stack([c * dx + s * dy, -s * dx + c * dy, wrap(q[:, 2] - p[:, 2])], axis=1)


def rpe(gt, est, delta=1.0):
    """Relative pose error over segments of `delta` metres of ground-truth
    travel: RMSE of the translation error (m) and rotation error (rad) of the
    estimated relative motion. Measures local drift independently of
    accumulated error. Returns NaN if the run is shorter than delta."""
    gt, est = np.asarray(gt, float), np.asarray(est, float)
    s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(gt[:, 0]), np.diff(gt[:, 1])))])
    j = np.searchsorted(s, s + delta - 1e-9)  # first sample at least delta further on
    ok = j < len(s)
    if not ok.any():
        return {"rpe_trans": math.nan, "rpe_rot": math.nan}
    i, j = np.nonzero(ok)[0], j[ok]
    rg, re = _relative(gt[i], gt[j]), _relative(est[i], est[j])
    et = np.hypot(re[:, 0] - rg[:, 0], re[:, 1] - rg[:, 1])
    er = wrap(re[:, 2] - rg[:, 2])
    return {"rpe_trans": float(np.sqrt(np.mean(et ** 2))),
            "rpe_rot": float(np.sqrt(np.mean(er ** 2)))}


def resample(xy, ds):
    """Points every `ds` metres of arc length along a polyline."""
    xy = np.asarray(xy, float)
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] < 2 * ds:
        return xy[[0, -1]] if len(xy) > 1 else xy
    t = np.arange(0.0, s[-1], ds)
    return np.stack([np.interp(t, s, xy[:, 0]), np.interp(t, s, xy[:, 1])], axis=1)


def smoothness(xy, ds=0.05):
    """Curvature of a path resampled every ds metres (heading change per
    metre). Returns length, mean |curvature| (1/m), the integral of
    curvature squared (1/m) and the total absolute turning (rad). Turning in
    place shows up as a large heading change over one sample, so it counts
    as unsmooth, which is intended."""
    p = resample(xy, ds)
    length = float(np.sum(np.hypot(np.diff(p[:, 0]), np.diff(p[:, 1]))))
    if len(p) < 3:
        return {"length": length, "mean_abs_curv": 0.0, "int_curv_sq": 0.0, "total_turn": 0.0}
    h = np.arctan2(np.diff(p[:, 1]), np.diff(p[:, 0]))
    dh = wrap(np.diff(h))
    k = dh / ds
    return {"length": length, "mean_abs_curv": float(np.mean(np.abs(k))),
            "int_curv_sq": float(np.sum(k ** 2) * ds), "total_turn": float(np.sum(np.abs(dh)))}


def summarize(gt, est, plan_xy, events, goal, mission_start_t, end_t):
    """All metrics for one run. `events` is a list of (t, text)."""
    names = [e for _, e in events]
    success = "mission_success" in names
    succ_t = next((t for t, e in events if e == "mission_success"), None)
    gt = np.asarray(gt, float)
    out = {"success": int(success),
           "time_to_goal": float(succ_t - mission_start_t) if success and mission_start_t is not None
           else math.nan,
           "recoveries": sum(e == "recovery_start" for e in names),
           "planner_failures": sum(e.startswith("planner_failed") for e in names),
           "blocked_events": sum(e == "blocked" for e in names),
           "final_goal_dist": float(math.hypot(gt[-1, 0] - goal[0], gt[-1, 1] - goal[1]))
           if len(gt) else math.nan,
           "duration": float(end_t - mission_start_t) if mission_start_t is not None else math.nan}
    if len(gt) > 1:
        out.update(ate(gt, est))
        out.update(rpe(gt, est))
        sm = smoothness(gt[:, :2])
        out.update({"driven_length": sm["length"], "mean_abs_curv": sm["mean_abs_curv"],
                    "int_curv_sq": sm["int_curv_sq"]})
    if plan_xy is not None and len(plan_xy) > 1:
        sp = smoothness(plan_xy)
        out.update({"plan_length": sp["length"], "plan_turn": sp["total_turn"]})
    return out
