"""Generator B -- the held-out validation set.

WRITTEN AFTER THE SCREENER WAS BUILT AND TUNED, and deliberately not consulted
while tuning. Both sides of this problem are ours, so a screener evaluated only
on the generator it was tuned against is measuring its author's imagination.

How B differs from A, on purpose:
  * A perturbs a known-good reference. B samples motions directly in joint space
    from random start and end configurations, so it is not organised around the
    pick-and-place corridor at all.
  * A's failures are mostly "the right motion, displaced". B produces sweeping
    transits, drags across the table surface, and wide arcs through the middle of
    the scene -- geometrically different families of failure.
  * Different durations (3-6 s vs a fixed 4 s), so the executor's velocity
    profile differs and the pose sampling the screener uses is stressed.
  * Some of B's motions never approach their declared target at all, which
    exercises the intent machinery from the other side.
"""
from __future__ import annotations

import numpy as np

from .model import HOME_QPOS
from .motions import Intent, Motion, N_ARM, ik_pose
from .scene import TABLE_TOP_Z


def generate_B(model, data, layout, rng, n: int) -> list[Motion]:
    cubes, bins_ = layout.cubes, layout.bins
    lo, hi = model.jnt_range[:N_ARM, 0], model.jnt_range[:N_ARM, 1]
    out = []
    for i in range(n):
        cube = cubes[rng.integers(len(cubes))]
        bin_ = bins_[rng.integers(len(bins_))]
        kind = rng.choice(["sweep", "arc", "drag", "random_via", "reach_only"],
                          p=[0.24, 0.22, 0.20, 0.20, 0.14])

        c = np.array(cube.center)
        b = np.array(bin_.drop_point)
        q = HOME_QPOS.copy()
        pts = []

        if kind == "sweep":            # low lateral traverse across the workspace
            y0, y1 = rng.uniform(-0.28, -0.05), rng.uniform(0.05, 0.28)
            z = rng.uniform(0.02, 0.14)
            x = rng.uniform(0.34, 0.58)
            pts = [np.array([x, y0, z + 0.10]), np.array([x, y0, z]),
                   np.array([x, y1, z]), np.array([x, y1, z + 0.10])]
        elif kind == "arc":            # wide overhead arc between two table points
            a = np.array([rng.uniform(0.32, 0.60), rng.uniform(-0.28, 0.28), 0.05])
            mid = (a + b) / 2 + np.array([0, 0, rng.uniform(0.10, 0.34)])
            pts = [a + [0, 0, 0.12], a, mid, b, b + [0, 0, 0.10]]
        elif kind == "drag":           # stay on the surface the whole way
            z = TABLE_TOP_Z + rng.uniform(0.015, 0.05)
            pts = [np.array([c[0], c[1], z + 0.12]), np.array([c[0], c[1], z]),
                   np.array([b[0], b[1], z]), np.array([b[0], b[1], z + 0.12])]
        elif kind == "reach_only":     # approach and withdraw, never delivers
            d = rng.normal(0, 0.05, 3); d[2] = abs(d[2])
            pts = [c + [0, 0, 0.20], c + d, c + [0, 0, 0.20]]

        if pts:
            ws = []
            for p in pts:
                q, _ = ik_pose(model, data, p, q)
                ws.append(q.copy())
            ws = np.array(ws)
        else:                          # random_via: pure joint-space, no IK at all
            k = int(rng.integers(3, 6))
            ws = np.clip(HOME_QPOS + rng.normal(0, 0.55, (k, N_ARM)), lo, hi)
            ws[0] = HOME_QPOS

        grip = np.full(len(ws), 255.0)
        grip[len(ws) // 2:] = 0.0
        out.append(Motion(
            mid=f"{layout.layout_id}-B{i:03d}", layout_id=layout.layout_id,
            waypoints=ws, grip=grip, duration=float(rng.uniform(3.0, 6.0)),
            intent=Intent(cube.oid, bin_.oid), gen="B", tag=str(kind)))
    return out
