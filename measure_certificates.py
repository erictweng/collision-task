"""Does the two-sided certificate resolve enough candidates to be worth having?

Reports the resolve rate (fraction never needing physics), the soundness of each
certificate against simulated truth, and what the sweep inflation costs.
"""
from __future__ import annotations

import json
import pickle
import time

import numpy as np

from screener.certify import (COLLIDES, SAFE, UNKNOWN, CertifyConfig,
                              certify_regime_a, inner_proxy, inner_radii,
                              sagitta_bound)
from screener.dual import is_arm_collision, screen_regime_a
from screener.fast import ArmProxy, ScreenConfig
from screener.model import ARMS, build_dual

CHUNK = 32


def run(motions, proxies, inners, cfg):
    v, go, gi = [], [], []
    t0 = time.perf_counter()
    for i in range(0, len(motions), CHUNK):
        a, b, c = certify_regime_a(motions[i:i + CHUNK], proxies, inners, cfg)
        v.append(a); go.append(b); gi.append(c)
    dt = time.perf_counter() - t0
    return np.concatenate(v), np.concatenate(go), np.concatenate(gi), dt


def report(name, verdict, truth, dt, n):
    safe, coll = verdict == SAFE, verdict == COLLIDES
    unk = verdict == UNKNOWN
    bad_safe = int((safe & truth).sum())         # certified SAFE, actually collided
    bad_coll = int((coll & ~truth).sum())        # certified COLLIDES, actually fine
    print(f"\n  {name}")
    print(f"    SAFE     {safe.sum():4d}  ({safe.mean():6.1%})   unsound: {bad_safe}")
    print(f"    COLLIDES {coll.sum():4d}  ({coll.mean():6.1%})   unsound: {bad_coll}")
    print(f"    UNKNOWN  {unk.sum():4d}  ({unk.mean():6.1%})   -> simulator")
    print(f"    resolved without physics: {1 - unk.mean():.1%}"
          f"    {n/dt:,.0f} verdicts/s")
    return dict(safe=int(safe.sum()), collides=int(coll.sum()), unknown=int(unk.sum()),
                resolve_rate=float(1 - unk.mean()), unsound_safe=bad_safe,
                unsound_collides=bad_coll, verdicts_per_s=float(n / dt))


def main():
    ds = pickle.load(open("out/ds_dual.pkl", "rb"))
    motions = [dm for _, dm in ds["motions"]]
    truth = np.array([is_arm_collision(ds["outcomes"][m.mid]) for m in motions])
    sim_rate = ds["sim_rate"]
    print(f"dataset: {len(motions)} station-steps, {truth.sum()} arm-arm collisions "
          f"({truth.mean():.1%}), simulator {sim_rate:.1f} steps/s/core")

    model, _ = build_dual(ds["layouts"][next(iter(ds["layouts"]))])
    proxies = [ArmProxy(model, prefix=p) for p, _ in ARMS]
    t0 = time.perf_counter()
    inners = [inner_proxy(model, px, p, refine=True) for px, (p, _) in zip(proxies, ARMS)]
    raw = [inner_radii(model, px, p) for px, (p, _) in zip(proxies, ARMS)]
    fit_s = time.perf_counter() - t0

    ri, ro = inners[0].radii, proxies[0].radii
    live = ri > 0
    r_unrefined = raw[0][raw[0] > 0]
    print(f"\nproxy: {proxies[0].n_spheres} spheres/arm, inner fit {fit_s:.1f}s (once)")
    print(f"  outer radii mm: min {ro.min()*1000:.1f} med {np.median(ro)*1000:.1f} "
          f"max {ro.max()*1000:.1f}")
    print(f"  inner radii mm: {len(ri)} cores, med {np.median(ri)*1000:.1f} "
          f"max {ri.max()*1000:.1f}  (before medial-axis refinement: med "
          f"{np.median(r_unrefined)*1000:.1f})")
    print(f"  shell: outer med {np.median(ro)*1000:.1f} - inner med "
          f"{np.median(ri)*1000:.1f} mm -- the UNKNOWN band is made of this")

    sag = max(sagitta_bound(proxies[k], motions[:24], k, 48) for k in (0, 1))
    print(f"\nmeasured arc bulge between samples at 48 poses: {sag*1000:.2f} mm")

    res = {}
    n = len(motions)
    print("\ncertificates:")
    for name, cfg in [
        ("poses only (UNSOUND on the SAFE side - shown as the ceiling)",
         CertifyConfig(inflate_sweep=False)),
        ("swept balls (sound for straight-line interpolation)",
         CertifyConfig(inflate_sweep=True)),
        ("swept balls + measured arc bulge (what I would ship)",
         CertifyConfig(inflate_sweep=True, sagitta=sag)),
    ]:
        v, go, gi, dt = run(motions, proxies, inners, cfg)
        res[name.split(" (")[0]] = report(name, v, truth, dt, n)
        both = int(((v == SAFE) & (gi < 0)).sum())
        if both:
            print(f"    !! {both} motions certified by BOTH sides - a bug, not a tuning issue")

    # what the tuned margin screener does on the same data, for comparison
    t0 = time.perf_counter()
    allow, worst = screen_regime_a(motions, proxies, ScreenConfig(n_poses=48), margin=0.02)
    dt_m = time.perf_counter() - t0
    fa = int((allow & truth).sum())
    fr = int((~allow & ~truth).sum())
    keep = int((allow & ~truth).sum()) / max(int((~truth).sum()), 1)
    print(f"\n  tuned 20mm margin screener, same data, for comparison")
    print(f"    keeps {keep:.1%} of safe steps, {fa} false accepts, {fr} false rejects,"
          f" {n/dt_m:,.0f} verdicts/s")
    res["margin_20mm"] = dict(keep_rate=keep, false_accept=fa, false_reject=fr,
                              verdicts_per_s=float(n / dt_m))

    best = res["swept balls + measured arc bulge"]
    saved = best["resolve_rate"]
    print(f"\n  simulation avoided: {saved:.1%} of station-steps")
    print(f"  a pass over {n} steps: {n/sim_rate:.1f}s all-physics vs "
          f"{n*(1-saved)/sim_rate + n/best['verdicts_per_s']:.1f}s screened "
          f"({sim_rate=:.1f}/s)")
    res["meta"] = dict(n=n, arm_collisions=int(truth.sum()), sim_rate=float(sim_rate),
                       sagitta_m=float(sag), n_spheres=int(proxies[0].n_spheres),
                       inner_nonzero=int(live.sum()))
    json.dump(res, open("out/certificates.json", "w"), indent=1)
    print("\nwrote out/certificates.json")


if __name__ == "__main__":
    main()
