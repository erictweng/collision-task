"""Run the arm-vs-arm margin screener and report what it costs.

    python run_margin.py

Loads the dual dataset that already has simulated truth attached, screens every
station-step at a range of margins, and prints the tradeoff. No physics is run
here -- the simulator's answers were computed once and cached in out/ds_dual.pkl.

The margin is the only parameter, so the honest way to present it is a sweep
rather than one number: the operating point is a decision, not a constant.
"""
from __future__ import annotations

import pickle
import time

import numpy as np

from screener.dual import is_arm_collision, screen_regime_a
from screener.fast import ArmProxy, ScreenConfig
from screener.model import ARMS, build_dual

MARGINS_MM = [0, 5, 10, 20, 30, 50]
N_POSES = 48


def main():
    ds = pickle.load(open("out/ds_dual.pkl", "rb"))
    motions = [dm for _, dm in ds["motions"]]
    collided = np.array([is_arm_collision(ds["outcomes"][m.mid]) for m in motions])
    n, n_safe, n_unsafe = len(motions), int((~collided).sum()), int(collided.sum())

    print(f"{n} station-steps · {n_unsafe} arm-arm collisions ({collided.mean():.1%})"
          f" · simulator {ds['sim_rate']:.1f} steps/s/core\n")

    model, _ = build_dual(ds["layouts"][next(iter(ds["layouts"]))])
    proxies = [ArmProxy(model, prefix=p) for p, _ in ARMS]
    cfg = ScreenConfig(n_poses=N_POSES)

    print(f"{'margin':>8} {'keeps':>8} {'of safe':>10} {'lets through':>14} {'speed':>12}")
    print(f"{'':>8} {'':>8} {'(keep rate)':>10} {'(unsafe)':>14} {'':>12}")
    print("  " + "-" * 54)
    for mm in MARGINS_MM:
        t0 = time.perf_counter()
        allow, _ = screen_regime_a(motions, proxies, cfg, margin=mm / 1000)
        dt = time.perf_counter() - t0
        kept = int((allow & ~collided).sum())
        leaked = int((allow & collided).sum())
        print(f"{mm:>6} mm {kept:>4}/{n_safe:<4} {kept/n_safe:>9.1%} "
              f"{leaked:>7}/{n_unsafe:<5} {n/dt:>8,.0f}/s")

    print(f"\nEvery verdict above reads only the two commanded joint trajectories.")
    print(f"No camera, no scene estimate, no physics -- which is why arm-vs-arm is")
    print(f"the half of the problem that does not degrade when the cameras drift.\n")
    print(f"Known limit: this tests {N_POSES} instants and nothing between them. A")
    print(f"crossing that happens entirely inside one interval is invisible to it.")
    print(f"Zero leaks below is a measurement on {n} motions, not a guarantee.")


if __name__ == "__main__":
    main()
