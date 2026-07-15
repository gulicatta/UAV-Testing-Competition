#!/usr/bin/env python3
"""Generates official-style plots for saved tests: on the left X/Y/Z/Yaw over
time, on the right the top-down view with trajectory and obstacles.

Usage (searches only inside the given folder, no recursion):
    python3 plot_official.py generated_tests/mission1-.../consegna/    # delivery only
    python3 plot_official.py generated_tests/mission1-.../all_fails/   # full history
    python3 plot_official.py generated_tests/.../rank01_d00.648_obs1.yaml  # a single test

For every <name>.yaml + <name>.ulg pair, saves <name>_plot.png next to it.
Reads the trajectory with pyulog (host) or aerialist (fallback, inside Docker).
"""
import glob
import os
import sys

import numpy as np
import yaml
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


def read_trajectory(ulg_path):
    """Returns (t, x, y, z, yaw) of the trajectory, aligned to the origin like aerialist.
    t in seconds, z = altitude (positive upward)."""
    try:
        import pyulog
        ulog = pyulog.ULog(ulg_path, ["vehicle_local_position"])
        d = ulog.get_dataset("vehicle_local_position").data
        t = np.array(d["timestamp"], dtype=float)
        x = np.array(d["x"], dtype=float)
        y = np.array(d["y"], dtype=float)
        z = np.array(d["z"], dtype=float)
        yaw = np.array(d.get("heading", np.zeros_like(x)), dtype=float)
        return (t - t[0]) / 1e6, x - x[0], y - y[0], -z, yaw
    except Exception:
        # fallback: aerialist (only available inside the Docker image)
        from aerialist.px4.trajectory import Trajectory
        traj = Trajectory.extract_from_log(ulg_path)
        p = traj.positions
        t = np.array([q.timestamp for q in p], dtype=float)
        return (
            (t - t[0]) / 1e6,
            np.array([q.x for q in p]),
            np.array([q.y for q in p]),
            np.array([q.z for q in p]),
            np.array([q.r for q in p]),
        )


def read_obstacles(yaml_path):
    with open(yaml_path) as f:
        data = yaml.safe_load(f)
    return data.get("simulation", {}).get("obstacles", []) or []


def obstacle_patch(o):
    """Gray rectangle rotated around its center (like aerialist's Obstacle.plt_patch)."""
    s, p = o["size"], o["position"]
    l, w = float(s["l"]), float(s["w"])
    cx, cy, r = float(p["x"]), float(p["y"]), float(p.get("r", 0))
    return mpatches.Rectangle(
        (cx - l / 2, cy - w / 2), l, w, angle=r, rotation_point="center",
        edgecolor="dimgray", facecolor="gray", alpha=0.5,
    )


def plot_one(ulg_path, yaml_path, out_png, title=""):
    t, x, y, z, yaw = read_trajectory(ulg_path)
    obstacles = read_obstacles(yaml_path)

    fig = plt.figure(figsize=(11, 6))
    gs = fig.add_gridspec(4, 4, hspace=0.35, wspace=0.4)
    ax_x = fig.add_subplot(gs[0, :2])
    ax_y = fig.add_subplot(gs[1, :2])
    ax_z = fig.add_subplot(gs[2, :2])
    ax_r = fig.add_subplot(gs[3, :2])
    ax_xy = fig.add_subplot(gs[:, 2:])

    for ax, series, lab in (
        (ax_x, x, "X (m)"), (ax_y, y, "Y (m)"),
        (ax_z, z, "Z (m)"), (ax_r, yaw, "Yaw (rad)"),
    ):
        ax.plot(t, series, color="#1f4e8c", linewidth=1.4)
        ax.set_ylabel(lab)
    ax_r.set_xlabel("tempo (s)")
    for ax in (ax_x, ax_y, ax_z):
        ax.tick_params(labelbottom=False)

    # top-down view: obstacles below, trajectory on top
    for i, o in enumerate(obstacles):
        patch = obstacle_patch(o)
        if i == 0:
            patch.set_label("ostacolo")
        ax_xy.add_patch(patch)
    ax_xy.plot(x, y, color="#1f4e8c", linewidth=1.6, label="traiettoria")
    ax_xy.plot(x[0], y[0], "o", color="green", markersize=8, label="start")
    ax_xy.plot(x[-1], y[-1], "s", color="red", markersize=8, label="end")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.yaxis.set_label_position("right")
    ax_xy.yaxis.tick_right()
    ax_xy.set_aspect("equal", "datalim")
    ax_xy.legend(loc="upper right", fontsize=8, framealpha=0.9)

    if title:
        fig.suptitle(title, fontsize=13)
    fig.savefig(out_png, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"  {os.path.basename(out_png)}  ({len(obstacles)} obstacle(s))")


def collect_targets(args):
    targets = []
    for a in args:
        if os.path.isdir(a):
            for pattern in ("rank*.yaml", "fail_*.yaml", "test_*.yaml"):
                for yml in sorted(glob.glob(os.path.join(a, pattern))):
                    ulg = yml[:-5] + ".ulg"
                    if os.path.exists(ulg):
                        targets.append((ulg, yml))
        elif a.endswith(".yaml"):
            ulg = a[:-5] + ".ulg"
            if os.path.exists(ulg):
                targets.append((ulg, a))
    return targets


def main(argv):
    if not argv:
        print(__doc__)
        return 1
    targets = collect_targets(argv)
    if not targets:
        print("No .yaml + .ulg pair (rank*/fail_*/test_*) found in the given paths.",
              file=sys.stderr)
        return 1
    print(f"Generating {len(targets)} plot(s):")
    for ulg, yml in targets:
        out = yml[:-5] + "_plot.png"
        plot_one(ulg, yml, out, title=os.path.basename(yml)[:-5])
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
