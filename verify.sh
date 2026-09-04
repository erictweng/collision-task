#!/usr/bin/env bash
# Check every claim this repo makes, in dependency order.
#
# Each stage is named for the brief bullet it answers, so a failure says which
# deliverable is broken rather than which file threw. Datasets are cached: truth
# is expensive and does not change, so it is simulated once and reused.
#   ./verify.sh            use cached truth  (~1 min)
#   ./verify.sh --rebuild  re-simulate truth (slow: minutes)
set -u
cd "$(dirname "$0")"

# Never the system python: `python3` on macOS is Apple's 3.9, which cannot
# import mujoco at all. PY= still overrides.
PY="${PY:-./.venv/bin/python}"
[ -x "$PY" ] || { echo "No venv. Run ./setup.sh first."; exit 1; }
REBUILD=""; [ "${1:-}" = "--rebuild" ] && REBUILD="--rebuild"

fail=0
run () {
  echo; echo "=============================================================="
  echo "  $1"; echo "=============================================================="
  shift
  if "$@"; then echo "  --> PASS"; else echo "  --> FAIL"; fail=1; fi
}

run "0. Environment" \
    "$PY" -c "import sys,mujoco,numpy;print(' python',sys.version.split()[0]);print(' mujoco',mujoco.__version__);print(' numpy',numpy.__version__)"

run "1. Invariants — containment, the oracle boundary, executor agreement" \
    "$PY" checks/check_invariants.py

run "2. Phase 2+3 — simulate truth, screen, compare (cached unless --rebuild)" \
    "$PY" experiments.py $REBUILD

run "3. The bench measures the screener it is given, not one it imports" \
    "$PY" checks/check_bench.py

run "4. Phase 2+3 numbers — speedup with the accuracy cost beside it" \
    "$PY" checks/check_results.py

run "5. Phase 3 — the review view" \
    bash -c "\"$PY\" export_view.py && \"$PY\" build_view.py"

run "6. Submission — does the write-up answer the brief" \
    "$PY" checks/check_submission.py

echo
if [ $fail -eq 0 ]; then
  echo "ALL CHECKS PASSED"
  echo "  out/results.json       every number quoted in DESIGN.md"
  echo "  out/view.html          open this to browse the wrong cases"
  echo "  out/certificates.json  the staged-screener extension"
else
  echo "SOMETHING FAILED (see above)"
fi
exit $fail
