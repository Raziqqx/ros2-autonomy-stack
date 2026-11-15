#!/usr/bin/env python3
"""Headless experiment runner for the navigation stack.

Runs the full ROS stack (navigation.launch.py with the run recorder) once per
scenario, several scenarios in parallel on separate ROS_DOMAIN_IDs, then
aggregates every run's summary.json into:
  <out>/eval_runs.csv     one row per run (configuration + metrics)
  <out>/eval_summary.md   tables by planner x controller, by noise x clutter,
                          and for the blocked-path (recovery) scenario
Every number in those files comes from these runs. Wall time is roughly
(runs / jobs) x ~45 s; the default grid (60 runs, 6 jobs) takes ~8-10 min.

Usage (inside the image):  ros2 run autonomy_bringup run_eval.py --out /results
"""
import argparse
import csv
import itertools
import json
import math
import os
import queue
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

PLANNERS = ["astar", "thetastar"]
CONTROLLERS = ["pure_pursuit", "dwa"]
NOISE = {"low": 1.0, "high": 3.0}       # x the sim's nominal sensor errors
CLUTTER = {"none": 0, "cluttered": 12}  # random boxes not in the prior map
METRICS = ["success", "time_to_goal", "ate_rmse", "ate_max", "heading_rmse", "rpe_trans",
           "rpe_rot", "mean_abs_curv", "int_curv_sq", "driven_length", "plan_length",
           "plan_turn", "recoveries", "planner_failures", "blocked_events", "final_goal_dist",
           "duration"]


def scenarios(seeds):
    out = []
    for noise, clutter, planner, controller, seed in itertools.product(
            NOISE, CLUTTER, PLANNERS, CONTROLLERS, seeds):
        out.append({"scenario": "nav", "noise": noise, "clutter": clutter, "planner": planner,
                    "controller": controller, "seed": seed,
                    "args": {"noise_scale": NOISE[noise], "clutter_count": CLUTTER[clutter]}})
    # Blocked path: a wall drops 0.45 m in front of the robot after 2 m.
    for planner, controller, seed in itertools.product(PLANNERS, CONTROLLERS, seeds):
        out.append({"scenario": "blocked", "noise": "low", "clutter": "none", "planner": planner,
                    "controller": controller, "seed": seed, "args": {"drop_after_m": 2.0}})
    for s in out:
        s["name"] = (f"{s['scenario']}_{s['noise']}_{s['clutter']}_{s['planner']}_"
                     f"{s['controller']}_s{s['seed']}")
    return out


def run_one(s, out_dir, domains, timeout):
    domain = domains.get()
    try:
        run_dir = os.path.join(out_dir, "runs", s["name"])
        os.makedirs(run_dir, exist_ok=True)
        args = {"planner": s["planner"], "controller": s["controller"], "seed": s["seed"],
                "record_dir": run_dir, "mission_timeout": timeout, **s["args"]}
        cmd = ["ros2", "launch", "autonomy_bringup", "navigation.launch.py"] + [
            f"{k}:={v}" for k, v in args.items()]
        env = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                   ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                   ROS_LOG_DIR=os.path.join(run_dir, "ros_log"))
        t0 = time.monotonic()
        with open(os.path.join(run_dir, "launch.log"), "w") as log:
            try:
                subprocess.run(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                               timeout=timeout + 60)
            except subprocess.TimeoutExpired:
                log.write("runner: launch timed out\n")
        summary_path = os.path.join(run_dir, "summary.json")
        metrics = {"success": 0}
        if os.path.exists(summary_path):
            with open(summary_path) as f:
                metrics = json.load(f)
        print(f"[{time.monotonic() - t0:5.1f} s] {s['name']}: success={metrics.get('success')} "
              f"t={metrics.get('time_to_goal', math.nan):.1f} "
              f"ate={metrics.get('ate_rmse', math.nan):.3f} rec={metrics.get('recoveries', 0)}",
              flush=True)
        return {**{k: s[k] for k in ("scenario", "noise", "clutter", "planner", "controller",
                                     "seed")}, **{k: metrics.get(k, math.nan) for k in METRICS}}
    finally:
        domains.put(domain)


def mean(rows, key, only_success=False):
    v = [r[key] for r in rows if not (only_success and not r["success"])
         and r[key] is not None and not (isinstance(r[key], float) and math.isnan(r[key]))]
    return sum(v) / len(v) if v else math.nan


def fmt(v, nd=3):
    return "n/a" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


def table(rows, keys):
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in keys), []).append(r)
    head = " | ".join(keys) + (" | runs | success | time to goal (s) | ATE RMSE (m) | ATE max (m) "
                               "| RPE (m/m) | mean abs curvature (1/m) | int curv^2 (1/m) "
                               "| plan length (m) | plan turning (rad) | recoveries/run |")
    lines = ["| " + head, "|" + "---|" * (len(keys) + 11)]
    for g, rs in sorted(groups.items()):
        lines.append("| " + " | ".join(map(str, g)) + f" | {len(rs)} | "
                     f"{sum(r['success'] for r in rs)}/{len(rs)} | "
                     f"{fmt(mean(rs, 'time_to_goal', True), 1)} | {fmt(mean(rs, 'ate_rmse'))} | "
                     f"{fmt(mean(rs, 'ate_max'))} | {fmt(mean(rs, 'rpe_trans'))} | "
                     f"{fmt(mean(rs, 'mean_abs_curv'), 2)} | {fmt(mean(rs, 'int_curv_sq'), 1)} | "
                     f"{fmt(mean(rs, 'plan_length'), 2)} | {fmt(mean(rs, 'plan_turn'), 2)} | "
                     f"{fmt(mean(rs, 'recoveries'), 2)} |")
    return "\n".join(lines)


def recovery_table(rows):
    lines = ["| planner | controller | runs | runs needing recovery | recovered (goal reached) "
             "| success (all runs) |", "|---|---|---|---|---|---|"]
    for p, c in itertools.product(PLANNERS, CONTROLLERS):
        rs = [r for r in rows if r["planner"] == p and r["controller"] == c]
        need = [r for r in rs if r["recoveries"] > 0]
        lines.append(f"| {p} | {c} | {len(rs)} | {len(need)} | "
                     f"{sum(r['success'] for r in need)}/{len(need)} | "
                     f"{sum(r['success'] for r in rs)}/{len(rs)} |")
    return "\n".join(lines)


def write_report(rows, out_dir, wall_s, args):
    with open(os.path.join(out_dir, "eval_runs.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    nav = [r for r in rows if r["scenario"] == "nav"]
    blocked = [r for r in rows if r["scenario"] == "blocked"]
    rec = [r for r in rows if r["recoveries"] > 0]
    md = [
        "# Navigation evaluation",
        "",
        f"Generated by `run_eval.py` ({len(rows)} runs, seeds {args.seeds}, {args.jobs} in "
        f"parallel, {wall_s / 60:.1f} min wall time). Every number below comes from these runs; "
        "per-run values are in `eval_runs.csv`.",
        "",
        "Task: drive from the dock (1.5, 1.5) to (10.2, 6.6) in the 12 x 9 m warehouse with the "
        "behaviour tree's GoTo mission. Noise `low` is the nominal sim noise, `high` is 3x every "
        "sensor error (odometry scale/bias/white noise, IMU, LiDAR). `cluttered` adds 12 random "
        "0.3-0.6 m boxes that are not in the prior map used by the localiser and planner. "
        "ATE/RPE are EKF vs ground truth (RPE over 1 m segments). Smoothness is computed on the "
        "driven ground-truth path resampled every 5 cm; plan turning is the summed heading change "
        "of the first global plan. Time to goal is averaged over successful runs only.",
        "",
        "## Planner x controller (all noise and clutter levels)",
        "",
        table(nav, ["planner", "controller"]),
        "",
        "## Noise x clutter (all planners and controllers)",
        "",
        table(nav, ["noise", "clutter"]),
        "",
        "## Blocked path: recovery",
        "",
        "A 1.6 m wall appears 0.45 m in front of the robot after it has driven 2 m. A run "
        "'needs recovery' if the behaviour tree ran its Recover branch (planner failure or "
        "controller blocked); it is recovered if the goal was still reached.",
        "",
        recovery_table(blocked),
        "",
        table(blocked, ["planner", "controller"]),
        "",
        "## Recovery success rate (all runs)",
        "",
        f"Runs in which the behaviour tree ran a recovery: {len(rec)}; of those, reached the "
        f"goal: {sum(r['success'] for r in rec)} "
        f"({fmt(100.0 * sum(r['success'] for r in rec) / len(rec), 0) if rec else 'n/a'} %).",
        "",
    ]
    with open(os.path.join(out_dir, "eval_summary.md"), "w") as f:
        f.write("\n".join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--out", default="results")
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--seeds", default="1,2,3")
    ap.add_argument("--timeout", type=float, default=120.0, help="per-run mission timeout (s)")
    ap.add_argument("--filter", default="", help="only run scenarios whose name contains this")
    args = ap.parse_args()
    seeds = [int(s) for s in args.seeds.split(",")]
    todo = [s for s in scenarios(seeds) if args.filter in s["name"]]
    os.makedirs(args.out, exist_ok=True)
    domains = queue.Queue()
    for i in range(args.jobs):
        domains.put(30 + i)
    print(f"{len(todo)} runs, {args.jobs} in parallel", flush=True)
    t0 = time.monotonic()
    with ThreadPoolExecutor(args.jobs) as pool:
        rows = list(pool.map(lambda s: run_one(s, args.out, domains, args.timeout), todo))
    write_report(rows, args.out, time.monotonic() - t0, args)
    print(f"done in {(time.monotonic() - t0) / 60:.1f} min -> {args.out}/eval_summary.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
