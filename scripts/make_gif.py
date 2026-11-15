#!/usr/bin/env python3
"""Renders a recorded run (nav_recorder output) as an animated GIF: the true
world (prior map + clutter / dropped wall), the current global plan, the
ground-truth and EKF trajectories, and the behaviour-tree events.

  python3 scripts/make_gif.py results/demo/run docs/demo.gif
Needs numpy, matplotlib and pillow.
"""
import csv
import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402

TRUTH, EKF, PLAN = "#2a78d6", "#eb6834", "#1baf7a"  # categorical slots 1-3


def read_pgm(path):
    data = path.read_bytes()
    parts = data.split(maxsplit=4)
    w, h = int(parts[1]), int(parts[2])
    return np.frombuffer(data[-w * h:], np.uint8).reshape(h, w)


def main(run_dir, out, fps=8, step_s=0.5):
    run = Path(run_dir)
    traj = np.genfromtxt(run / "traj.csv", delimiter=",", names=True)
    plans = {}
    with open(run / "plans.csv") as f:
        for r in csv.DictReader(f):
            plans.setdefault(int(r["plan"]), (float(r["t"]), []))[1].append(
                (float(r["x"]), float(r["y"])))
    with open(run / "events.csv") as f:
        events = [(float(r["t"]), r["event"]) for r in csv.DictReader(f)]
    world = read_pgm(run / "world.pgm")
    meta = json.loads((run / "world.json").read_text())
    h, w = world.shape
    res, (ox, oy) = meta["resolution"], meta["origin"]
    extent = [ox, ox + w * res, oy, oy + h * res]

    fig, ax = plt.subplots(figsize=(6.4, 5.0), dpi=80)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.9, bottom=0.08)
    ax.imshow(world < 128, cmap="Greys", extent=extent, origin="upper", vmin=0, vmax=1.6)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.tick_params(colors="#555", labelsize=8)
    for s in ax.spines.values():
        s.set_color("#bbb")
    (plan_ln,) = ax.plot([], [], color=PLAN, lw=2, label="global plan")
    (gt_ln,) = ax.plot([], [], color=TRUTH, lw=2, label="ground truth")
    (est_ln,) = ax.plot([], [], color=EKF, lw=1.5, ls="--", label="EKF estimate")
    robot = plt.Circle((0, 0), 0.2, fc="none", ec=TRUTH, lw=2)
    ax.add_patch(robot)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.9)
    title = ax.set_title("", fontsize=10, loc="left")
    plan_times = sorted((t, i) for i, (t, _) in plans.items())

    t_end = traj["t"][-1]
    frames = list(np.arange(traj["t"][0], t_end, step_s)) + [t_end] * fps  # hold the end

    def draw(t):
        k = np.searchsorted(traj["t"], t, side="right")
        gt_ln.set_data(traj["gt_x"][:k], traj["gt_y"][:k])
        est_ln.set_data(traj["est_x"][:k], traj["est_y"][:k])
        if k:
            robot.center = (traj["gt_x"][k - 1], traj["gt_y"][k - 1])
        current = [i for pt, i in plan_times if pt <= t]
        pts = np.array(plans[current[-1]][1]) if current else np.empty((0, 2))
        plan_ln.set_data(pts[:, 0] if len(pts) else [], pts[:, 1] if len(pts) else [])
        last = [e for et, e in events if et <= t and not e.startswith("navigate")]
        title.set_text(f"t = {t - traj['t'][0]:5.1f} s   {last[-1] if last else ''}")
        return plan_ln, gt_ln, est_ln, robot, title

    FuncAnimation(fig, draw, frames=frames, blit=False).save(out, writer=PillowWriter(fps=fps))
    print(f"wrote {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
