"""Run every experiment in the submission and write out/results.json.

    python experiments.py            # uses cached datasets if present
    python experiments.py --rebuild  # re-simulates truth from scratch
"""
from __future__ import annotations

import json
import pickle
import sys
import time
from pathlib import Path

import numpy as np

from screener.estimate import ErrorModel
from screener.evalrun import (COST_FALSE_ACCEPT, COST_FALSE_REJECT,
                              build_dataset, evaluate, fmt)
from screener.fast import ArmProxy, ScreenConfig, sample_joint_path
from screener.heldout import generate_B
from screener.model import build

OUT = Path("out")
OUT.mkdir(exist_ok=True)

# --- the fleet model the design is sized against (DESIGN.md sec.1) -----------
STATIONS = 10
SECONDS_PER_EXECUTION = 8.0
CANDIDATES_PER_EXECUTION = 100
LATENCY_BUDGET_S = 0.100
CHOSEN = ScreenConfig(margin=0.015)
N_HYP = 4


def cost_arithmetic(sim_rate: float, fast_rate: float) -> dict:
    exec_per_s = STATIONS / SECONDS_PER_EXECUTION
    verdicts_per_s = exec_per_s * CANDIDATES_PER_EXECUTION
    burst_sim_s = CANDIDATES_PER_EXECUTION / sim_rate
    return dict(
        sim_verdicts_per_s_per_core=sim_rate,
        core_hours_for_2M=2e6 / sim_rate / 3600,
        executions_per_s=exec_per_s,
        sustained_verdicts_per_s=verdicts_per_s,
        cores_if_simulated=verdicts_per_s / sim_rate,
        burst_latency_if_simulated_s=burst_sim_s,
        latency_budget_s=LATENCY_BUDGET_S,
        latency_overrun_x=burst_sim_s / LATENCY_BUDGET_S,
        burst_latency_fast_s=CANDIDATES_PER_EXECUTION * N_HYP / fast_rate,
        speedup_x=fast_rate / sim_rate,
        hypotheses=N_HYP,
    )


def main():
    rebuild = "--rebuild" in sys.argv
    dsA_p, dsB_p = OUT / "ds_A.pkl", OUT / "ds_B.pkl"
    if rebuild or not dsA_p.exists():
        dsA = build_dataset(n_layouts=8, per_layout=40, gen="A")
        pickle.dump(dsA, open(dsA_p, "wb"))
    else:
        dsA = pickle.load(open(dsA_p, "rb"))
    if rebuild or not dsB_p.exists():
        dsB = build_dataset(n_layouts=8, per_layout=40, gen="B", generator=generate_B)
        pickle.dump(dsB, open(dsB_p, "wb"))
    else:
        dsB = pickle.load(open(dsB_p, "rb"))

    model, _ = build(dsA["layouts"][0])
    px = ArmProxy(model)
    res = {}

    print("\n=== operating curve (generator A, perfect estimate) ===")
    curve = []
    for mar in [-0.005, 0.0, 0.005, 0.010, 0.015, 0.020, 0.025, 0.030, 0.040, 0.055]:
        r = evaluate(dsA, ScreenConfig(margin=mar), proxy=px)
        print(fmt(r, f"margin={mar*1000:+.0f}mm"))
        curve.append(dict(margin_mm=mar * 1000, keep_rate=r["keep_rate"],
                          false_accept=r["false_accept"], false_reject=r["false_reject"],
                          fa_rate_of_unsafe=r["fa_rate_of_unsafe"],
                          verdicts_per_s=r["verdicts_per_s"],
                          expected_cost=r["expected_cost"]))
    res["curve"] = curve

    base = evaluate(dsA, CHOSEN, proxy=px)
    res["arithmetic"] = cost_arithmetic(dsA["sim_rate"], base["verdicts_per_s"])
    print("\n=== screening-cost arithmetic ===")
    for k, v in res["arithmetic"].items():
        print(f"  {k:<32} {v:,.4g}")

    print("\n=== degradation under each scene error (chosen operating point) ===")
    modes = {
        "perfect": (ErrorModel(name="perfect"), CHOSEN, 1),
        "offset_8mm": (ErrorModel(offset=(0.008, 0.004, 0.0), name="offset"), CHOSEN, 1),
        "offset_8mm_+inflation": (ErrorModel(offset=(0.008, 0.004, 0.0), name="offset"),
                                  ScreenConfig(margin=CHOSEN.margin, offset_inflation=0.010), 1),
        "missing_15pct": (ErrorModel(p_missing=0.15, name="missing"), CHOSEN, 1),
        "missing_15pct_+4hyp": (ErrorModel(p_missing=0.15, name="missing"), CHOSEN, 4),
        "missing_15pct_+unknown": (ErrorModel(p_missing=0.15, shadows=True, name="missing"),
                                   ScreenConfig(margin=CHOSEN.margin, unknown_margin=0.004), 1),
        "phantom_2": (ErrorModel(n_phantom=2, name="phantom"), CHOSEN, 1),
        "all_three": (ErrorModel(offset=(0.008, 0.004, 0.0), p_missing=0.15, n_phantom=2,
                                 shadows=True, name="all"),
                      ScreenConfig(margin=CHOSEN.margin, offset_inflation=0.010,
                                   unknown_margin=0.004), 4),
    }
    deg = {}
    for name, (err, cfg, nh) in modes.items():
        r = evaluate(dsA, cfg, err=err, proxy=px, n_hypotheses=nh)
        print(fmt(r, name))
        deg[name] = {k: v for k, v in r.items() if not k.startswith("_")}
    res["degradation"] = deg

    print("\n=== held-out generator B (screener untouched) ===")
    rb = evaluate(dsB, CHOSEN, proxy=px)
    ra = evaluate(dsA, CHOSEN, proxy=px)
    print(fmt(ra, "generator A (tuned on)"))
    print(fmt(rb, "generator B (held out)"))
    res["heldout"] = dict(A={k: v for k, v in ra.items() if not k.startswith("_")},
                          B={k: v for k, v in rb.items() if not k.startswith("_")})

    # per-tag breakdown on B: which unfamiliar motion families break it
    recB = {x.mid: x for x in dsB["records"]}
    tags = {}
    for mid, a, u in zip(rb["_mids"], rb["_allow"], rb["_unsafe"]):
        t = recB[mid].tag
        d = tags.setdefault(t, dict(n=0, fa=0, fr=0, unsafe=0))
        d["n"] += 1; d["unsafe"] += int(u)
        d["fa"] += int(a and u); d["fr"] += int((not a) and (not u))
    res["heldout"]["by_tag"] = tags
    print("  by motion family:", json.dumps(tags))

    res["cost_model"] = dict(false_accept=COST_FALSE_ACCEPT, false_reject=COST_FALSE_REJECT)
    res["chosen"] = dict(margin_mm=CHOSEN.margin * 1000, n_poses=CHOSEN.n_poses,
                         n_hypotheses=N_HYP)
    json.dump(res, open(OUT / "results.json", "w"), indent=1, default=float)
    print(f"\nwrote {OUT/'results.json'}")


if __name__ == "__main__":
    main()
