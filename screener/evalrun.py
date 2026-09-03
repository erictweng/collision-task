"""Evaluation harness: build a labelled dataset, screen it, report the gap.

THE ANTI-LEAKAGE BOUNDARY LIVES HERE. Truth is computed by truth.run_truth from a
Layout; the screener is called with a SceneEstimate built by estimate.observe.
The two are never passed the same object, and `screen()` takes no argument that
could carry an Outcome. If you are auditing this submission for leakage, this
file is the only place the two halves are in scope at once.
"""
from __future__ import annotations

import pickle
import time
from dataclasses import dataclass, asdict

import numpy as np

from .estimate import ErrorModel, observe
from .fast import ArmProxy, ScreenConfig, screen
from .model import build
from .motions import generate_A
from .scene import random_layout
from .truth import run_truth

# Cost model. Derived, not asserted -- see DESIGN.md.
#   a crash    ~30 min of a station out of service + a human intervention
#   a discard  one training datum out of millions
COST_FALSE_ACCEPT = 1000.0
COST_FALSE_REJECT = 1.0


@dataclass
class Record:
    mid: str
    layout_seed: int
    tag: str
    gen: str
    unsafe: bool
    reasons: tuple
    culprit_true: str | None


def build_dataset(n_layouts=8, per_layout=40, seed0=0, gen="A", generator=None,
                  record_paths=False, verbose=True):
    """Simulate truth for every candidate. This is the expensive half."""
    gen_fn = generator or generate_A
    layouts, motions, records, outcomes = {}, [], [], {}
    t0 = time.perf_counter()
    sim_total = 0.0
    for s in range(seed0, seed0 + n_layouts):
        lay = random_layout(s)
        model, data = build(lay)
        layouts[s] = lay
        ms = gen_fn(model, data, lay, np.random.default_rng(1000 + s), per_layout)
        for mo in ms:
            o = run_truth(model, data, lay, mo, record_path=record_paths)
            sim_total += o.sim_seconds
            motions.append((s, mo))
            outcomes[mo.mid] = o
            records.append(Record(mo.mid, s, mo.tag, gen, o.unsafe, o.reasons,
                                  o.culprit))
    wall = time.perf_counter() - t0
    if verbose:
        n = len(records)
        print(f"  dataset gen={gen}: {n} motions over {n_layouts} layouts, "
              f"{np.mean([r.unsafe for r in records]):.1%} unsafe")
        print(f"  truth: {sim_total/n*1000:.1f} ms/motion of pure simulation "
              f"({n/sim_total:.1f} verdicts/s/core), {wall:.1f}s wall incl. generation")
    return dict(layouts=layouts, motions=motions, records=records,
                outcomes=outcomes, sim_rate=len(records) / sim_total)


def _proxy_cache(layouts):
    """One ArmProxy is valid for every layout -- the arm does not change."""
    s = next(iter(layouts))
    model, _ = build(layouts[s])
    return ArmProxy(model)


def evaluate(ds, cfg: ScreenConfig, err: ErrorModel = ErrorModel(),
             proxy=None, rng_seed=7, n_hypotheses=1):
    """Screen every motion and compare with truth.

    n_hypotheses > 1 draws that many independent SceneEstimates from the error
    model and permits a motion only if EVERY hypothesis permits it. That is the
    defence against discrete errors (a missing object present in some draws), and
    it costs exactly n_hypotheses x the screening time -- which the speedup budget
    was sized to afford.
    """
    proxy = proxy or _proxy_cache(ds["layouts"])
    by_layout = {}
    for s, mo in ds["motions"]:
        by_layout.setdefault(s, []).append(mo)

    allow, truth_unsafe, mids, culprits = [], [], [], []
    screen_time = 0.0
    for s, ms in by_layout.items():
        lay = ds["layouts"][s]
        ok = np.ones(len(ms), dtype=bool)
        cul = [None] * len(ms)
        for h in range(n_hypotheses):
            est = observe(lay, err, np.random.default_rng(rng_seed + 991 * s + h))
            t0 = time.perf_counter()
            vs = screen(ms, est, proxy, cfg)
            screen_time += time.perf_counter() - t0
            for i, v in enumerate(vs):
                if not v.allow and ok[i]:
                    cul[i] = v.culprit
                ok[i] &= v.allow
        allow += list(ok)
        culprits += cul
        mids += [m.mid for m in ms]
        truth_unsafe += [ds["outcomes"][m.mid].unsafe for m in ms]

    allow = np.array(allow); unsafe = np.array(truth_unsafe)
    fa = int(np.sum(allow & unsafe))           # permitted, and it collided
    fr = int(np.sum(~allow & ~unsafe))         # rejected, and it was fine
    tp = int(np.sum(~allow & unsafe))
    tn = int(np.sum(allow & ~unsafe))
    n_safe = int(np.sum(~unsafe)); n_unsafe = int(np.sum(unsafe))
    n = len(allow)
    return dict(
        n=n, false_accept=fa, false_reject=fr, caught=tp, kept=tn,
        keep_rate=tn / max(n_safe, 1),
        fa_rate_of_unsafe=fa / max(n_unsafe, 1),
        fa_rate_of_permitted=fa / max(tn + fa, 1),
        verdicts_per_s=n * n_hypotheses / max(screen_time, 1e-9),
        us_per_verdict=screen_time / max(n * n_hypotheses, 1) * 1e6,
        expected_cost=(fa * COST_FALSE_ACCEPT + fr * COST_FALSE_REJECT) / n,
        _allow=allow, _unsafe=unsafe, _mids=mids, _culprits=culprits)


def fmt(r, label=""):
    return (f"{label:<26} keep={r['keep_rate']:.3f}  "
            f"FA={r['false_accept']:<3d}({r['fa_rate_of_unsafe']:.3f} of unsafe)  "
            f"FR={r['false_reject']:<4d}  "
            f"{r['verdicts_per_s']:>8.0f} v/s  cost={r['expected_cost']:.1f}")
