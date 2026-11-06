"""Run the waypoint reducer on a synthetic planner path and plot every stage.

    pip install numpy matplotlib
    python python/demo.py            # saves docs/demo_stages.png
"""
import argparse
import math
import os

import matplotlib.pyplot as plt
import numpy as np

from waypoint_reducer import angle_filter, distance_filter_with_recovery, reduce_path


def synthetic_planner_path(step=0.05, jitter=0.005, seed=0):
    """Dense path like move_base/navfn output: straights, a 90 deg corner, an arc and a dog-leg."""
    rng = np.random.default_rng(seed)
    pts = []

    def straight(a, b):
        n = max(1, int(math.dist(a, b) / step))
        pts.extend(np.linspace(a, b, n, endpoint=False).tolist())

    def arc(centre, r, t0, t1):
        n = max(1, int(abs(t1 - t0) * r / step))
        for t in np.linspace(t0, t1, n, endpoint=False):
            pts.append([centre[0] + r * math.cos(t), centre[1] + r * math.sin(t)])

    straight((0, 0), (6, 0))
    straight((6, 0), (6, 4))                              # sharp 90 deg corner
    arc((9, 4), 3, math.pi, math.pi / 2)                  # smooth quarter circle
    straight((9, 7), (13, 7))
    straight((13, 7), (14, 6.6))                          # small dog-leg (tests corner recovery)
    straight((14, 6.6), (18, 6.6))
    pts.append([18, 6.6])
    pts = np.array(pts)
    pts[1:-1] += rng.normal(0, jitter, pts[1:-1].shape)   # planner/grid noise
    return [tuple(p) for p in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=5.0, help="angle threshold [deg]")
    ap.add_argument("--min-distance", type=float, default=1.0, help="[m]")
    ap.add_argument("--epsilon", type=float, default=0.3, help="[m]")
    ap.add_argument("--mode", choices=["cumulative", "local"], default="cumulative")
    ap.add_argument("--lookahead", type=float, default=0.25, help="[m], cumulative mode only")
    ap.add_argument("--out", default="docs/demo_stages.png")
    ap.add_argument("--compare-out", default="docs/local_vs_cumulative.png")
    args = ap.parse_args()

    path = synthetic_planner_path()
    a = angle_filter(path, args.angle, args.mode, args.lookahead)
    d_no_recovery = distance_filter_with_recovery(a, args.min_distance, epsilon=float("inf"))
    final = distance_filter_with_recovery(a, args.min_distance, args.epsilon)

    stages = [
        (f"1. Planner path ({len(path)} poses)", path, None),
        (f"2. Angle filter ({len(a)} points)", path, a),
        (f"3. Distance filter only ({len(d_no_recovery)})", path, d_no_recovery),
        (f"4. + corner recovery = final ({len(final)})", path, final),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13, 7), sharex=True, sharey=True)
    for ax, (title, dense, pts) in zip(axes.ravel(), stages):
        dx, dy = zip(*dense)
        ax.plot(dx, dy, "-", color="0.75", lw=3, label="planner path")
        if pts is not None:
            px, py = zip(*pts)
            ax.plot(px, py, "o-", color="tab:blue", ms=5, lw=1.2, label="waypoints")
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    axes[0, 0].legend(loc="lower right")
    fig.suptitle(
        f"angle >= {args.angle} deg ({args.mode}), min distance {args.min_distance} m, epsilon {args.epsilon} m"
    )
    fig.tight_layout()
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    fig.savefig(args.out, dpi=130)
    print(f"{len(path)} poses -> {len(final)} waypoints. Saved {args.out}")

    # Original per-step angle (2023 version) vs cumulative + lookahead, on the same noisy path
    fig, axes = plt.subplots(1, 2, figsize=(13, 3.6), sharey=True)
    for ax, mode in zip(axes, ["local", "cumulative"]):
        w = reduce_path(path, args.angle, args.min_distance, args.epsilon, mode, args.lookahead)
        ax.plot(*zip(*path), "-", color="0.75", lw=3)
        ax.plot(*zip(*w), "o-", color="tab:blue" if mode == "cumulative" else "tab:red", ms=5, lw=1.2)
        ax.set_title(f"{mode} angle mode: {len(w)} waypoints")
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.compare_out, dpi=130)
    print(f"Saved {args.compare_out}")


if __name__ == "__main__":
    main()
