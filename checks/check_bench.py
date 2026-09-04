"""Is the bench actually measuring anything?

A harness that reports plausible numbers for every screener is worse than none,
so this feeds it screeners whose answers are known in advance and asserts it
notices. It also checks the cost model for the pathology the brief warns about:
"a screener that rejects everything never crashes and is worthless."
"""
from __future__ import annotations

import pathlib
import pickle
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from screener.evalrun import (COST_FALSE_ACCEPT, COST_FALSE_REJECT,  # noqa: E402
                              evaluate, fmt)
from screener.fast import ArmProxy, ScreenConfig, Verdict  # noqa: E402
from screener.model import build  # noqa: E402

fail = []
ds = pickle.load(open(pathlib.Path(__file__).parent.parent / "out/ds_A.pkl", "rb"))
model, _ = build(ds["layouts"][next(iter(ds["layouts"]))])
proxy, cfg = ArmProxy(model), ScreenConfig(margin=0.0)


def reject_all(ms, est, proxy, cfg):
    return [Verdict(m.mid, False, 0.0, "policy") for m in ms]


def permit_all(ms, est, proxy, cfg):
    return [Verdict(m.mid, True, 9.9, None) for m in ms]


print("the screener is an argument — three of them, one cached truth:\n")
real = evaluate(ds, cfg, proxy=proxy)
none_ = evaluate(ds, cfg, proxy=proxy, screener=reject_all)
all_ = evaluate(ds, cfg, proxy=proxy, screener=permit_all)
for label, r in (("the real screener", real), ("reject everything", none_),
                 ("permit everything", all_)):
    print("  " + fmt(r, label))

print("\ndoes the bench discriminate?")
if none_["keep_rate"] != 0.0:
    fail.append("reject-everything did not score keep=0 — the screener argument is ignored")
if all_["keep_rate"] != 1.0:
    fail.append("permit-everything did not score keep=1 — the screener argument is ignored")
if all_["false_accept"] <= real["false_accept"]:
    fail.append("permit-everything should have the most false accepts")
print("  degenerate screeners land exactly where they must — the argument is live")

print(f"\ncost model sanity  (false accept : false reject = "
      f"{COST_FALSE_ACCEPT:.0f} : {COST_FALSE_REJECT:.0f})")
print(f"  real screener      expected cost {real['expected_cost']:>8.1f}"
      f"   keeps {real['keep_rate']:.1%}")
print(f"  reject everything  expected cost {none_['expected_cost']:>8.1f}"
      f"   keeps {none_['keep_rate']:.1%}")
if none_["expected_cost"] <= real["expected_cost"]:
    print("\n  WARNING: under this cost model, a screener that rejects EVERYTHING")
    print("  scores better than the real one. The brief names this exactly:")
    print("  'a screener that rejects everything never crashes and is worthless.'")
    print("  The ratio, not the screener, is what needs defending — see DESIGN.md")
    print("  on the cliff and the force trip. Reported, not silently averaged.")

print()
if fail:
    for f_ in fail:
        print("FAIL:", f_)
    sys.exit(1)
print("the bench measures the screener it is given")
