"""Assert the submission's numbers exist and cohere — phase by phase.

This does not re-derive anything. It reads out/results.json and checks that
every number the brief asks for is present, has both error directions beside it,
and is internally consistent. A green run here means the claims in DESIGN.md are
backed by a file rather than by memory.
"""
from __future__ import annotations

import json
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
fail, warn = [], []


def need(cond, msg):
    (fail if not cond else warn).append(msg) if not cond else None


def head(n, title):
    print(f"\n{n}  {title}\n" + "  " + "-" * (len(title) + 2))


r = json.loads((ROOT / "out" / "results.json").read_text())

head("PHASE 2", "truth, a fast path, and the gap between them")
a = r["arithmetic"]
sim = a["sim_verdicts_per_s_per_core"]
base = r["degradation"]["perfect"]
fast = base["verdicts_per_s"]
print(f"  simulation      {sim:>10,.1f} verdicts/s/core")
print(f"  fast path       {fast:>10,.1f} verdicts/s      speedup {fast/sim:>7,.0f}x")
print(f"  core-hours for 2M candidates by simulation: {a['core_hours_for_2M']:.1f}")
need(sim > 0, "no simulation rate recorded")
need(fast / sim > 10, f"fast path only {fast/sim:.1f}x simulation — not 'far faster'")

n, fa, fr = base["n"], base["false_accept"], base["false_reject"]
print(f"  disagreement with truth, both directions, on n={n}:")
print(f"    permitted and collided (false accept) : {fa:>4}"
      f"   {base['fa_rate_of_unsafe']:.1%} of unsafe")
print(f"    rejected and was fine  (false reject) : {fr:>4}"
      f"   keep rate {base['keep_rate']:.1%}")
need(n >= 200, f"only {n} motions evaluated — too few to report a rate")
need(0 < base["keep_rate"] < 1, "keep rate is degenerate — a screener that keeps all or none")

tags = r["heldout"].get("by_tag", {})
worst = sorted(tags.items(), key=lambda kv: -kv[1]["fa"])[:2]
print(f"  the ones it got wrong, by motion family:")
for t, d in worst:
    print(f"    {t:<12} {d['fa']:>2} false accepts of {d['unsafe']:>3} unsafe in {d['n']:>3}")
need(tags, "no per-family breakdown — 'any pattern in them named' is unanswered")

head("PHASE 3", "make it hold up")
deg = r["degradation"]
modes = {k: v for k, v in deg.items() if k != "perfect"}
for kind in ("offset", "missing", "phantom"):
    hits = [k for k in modes if k.startswith(kind)]
    need(hits, f"no degradation run for '{kind}' — the brief asks for each separately")
    for k in hits:
        d = deg[k]
        print(f"  {k:<26} keep {d['keep_rate']:>6.1%}   false accepts {d['false_accept']:>3}"
              f"   ({d['fa_rate_of_unsafe']:.1%} of unsafe)")
if "all_three" in deg and deg["all_three"]["keep_rate"] < 0.05:
    print(f"  NOTE: compound error keeps {deg['all_three']['keep_rate']:.1%} — "
          f"reported as a failure, not averaged away")

A, B = r["heldout"]["A"], r["heldout"]["B"]
print(f"  held-out generator:")
print(f"    tuned on   (A)  n={A['n']:<5} keep {A['keep_rate']:>6.1%}  fa {A['false_accept']}")
print(f"    held out   (B)  n={B['n']:<5} keep {B['keep_rate']:>6.1%}  fa {B['false_accept']}")
need(B["n"] > 0, "no held-out generator result")
if B["keep_rate"] > A["keep_rate"]:
    print("    B scores BETTER than A — B was not adversarial enough. Report it that way.")

view = ROOT / "out" / "view.html"
print(f"  view: {view.name} {'exists' if view.exists() else 'MISSING'}"
      f"{f' ({view.stat().st_size//1024} KB)' if view.exists() else ''}")
need(view.exists() and view.stat().st_size > 20_000, "out/view.html missing or trivially small")

head("PHASE 4", "extension")
curve = r.get("curve", [])
print(f"  operating point as a curve: {len(curve)} points, "
      f"margin {curve[0]['margin_mm']:+.0f} to {curve[-1]['margin_mm']:+.0f} mm")
print(f"    keep rate spans {min(c['keep_rate'] for c in curve):.1%}"
      f" to {max(c['keep_rate'] for c in curve):.1%}")
need(len(curve) >= 5, "curve has too few points to be a curve")
cert = ROOT / "out" / "certificates.json"
if cert.exists():
    c = json.loads(cert.read_text())
    rr = c.get("end_to_end", {}).get("resolve_rate")
    print(f"  staged screener (certificates): resolve rate {rr:.1%}"
          if rr else "  staged screener: certificates.json present")
else:
    print("  staged screener: not built (optional)")

print()
if fail:
    for f_ in fail:
        print("FAIL:", f_)
    sys.exit(1)
print("every number the brief asks for is present and coherent")
