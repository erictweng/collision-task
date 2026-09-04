"""Does the write-up answer every question the brief actually asked?

The brief names nine things DESIGN.md must contain. This walks that list and
reports which are covered, so the gap is found here rather than by the reader.
Coverage is judged on topic, not on heading text, so the document can be
restructured without this going stale.
"""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
design = (ROOT / "DESIGN.md").read_text()
low = design.lower()

REQUIRED = [
    ("Phase 1 design note",
     [r"phase 1", r"\bthesis\b"]),
    ("Screening-cost arithmetic",
     [r"core[- ]hours?", r"verdicts?[ /]", r"\bbudget\b"]),
    ("Intended contact vs unintended",
     [r"intended", r"\bintent\b", r"licence|license"]),
    ("Representing a wrong scene (offset, missing, phantom)",
     [r"systematic|offset", r"occlu|missing", r"phantom"]),
    ("Ground truth, and that the screener cannot read it",
     [r"ground truth|oracle", r"leak|cannot read|no path"]),
    ("Fast path, and its accuracy cost against truth",
     [r"false accept", r"false reject", r"keep rate"]),
    ("Degradation under each kind of scene error",
     [r"degrad", r"offset", r"phantom"]),
    ("Held-out generator result and the conclusion drawn",
     [r"held[- ]out", r"generator b"]),
    ("Where you sit on the tradeoff, and why",
     [r"operating point", r"cost|ratio"]),
    ("What you cut, and why",
     [r"not building|did not build|cut\b|out of scope"]),
]

print(f"DESIGN.md — {len(design.splitlines())} lines, "
      f"{len(re.findall(r'^#+ ', design, re.M))} headings\n")
missing = []
for name, pats in REQUIRED:
    hits = sum(bool(re.search(p, low)) for p in pats)
    ok = hits >= max(1, len(pats) - 1)
    print(f"  {'OK  ' if ok else 'GAP '} {name}")
    if not ok:
        missing.append(name)

readme = ROOT / "README.md"
print(f"\n  {'OK  ' if readme.exists() else 'GAP '} README.md"
      f"{f' ({len(readme.read_text().splitlines())} lines)' if readme.exists() else ''}")
if not readme.exists():
    missing.append("README.md")

print()
if missing:
    for m in missing:
        print("FAIL: DESIGN.md does not cover:", m)
    sys.exit(1)
print("the write-up covers every item the brief names")
