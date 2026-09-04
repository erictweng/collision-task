#!/usr/bin/env bash
# One-time setup. Idempotent - safe to re-run.
#
# macOS ships Apple's Command Line Tools Python 3.9 as `python3`. mujoco
# publishes no cp39 wheel, so pip silently falls back to the source
# distribution and dies with "MUJOCO_PATH environment variable is not set" --
# an error that names the C library and says nothing about the real cause.
# Picking a real interpreter here is cheaper than diagnosing that again.
set -euo pipefail
cd "$(dirname "$0")"
MIN_PY="3.10"

pick_python() {
  # Not newest-first: a brand-new interpreter is exactly the case where a wheel
  # may not exist yet, which is the failure this script prevents.
  for c in python3.13 python3.12 python3.11 python3.10 python3.15 python3.14 \
           /opt/homebrew/bin/python3 /usr/local/bin/python3 python3; do
    command -v "$c" >/dev/null 2>&1 || continue
    "$c" -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null \
      && { command -v "$c"; return 0; }
  done
  return 1
}

echo "--> python"
if [ -n "${PYTHON:-}" ]; then
  PY="$(command -v "$PYTHON")" || { echo "PYTHON=$PYTHON not found"; exit 1; }
elif ! PY="$(pick_python)"; then
  echo
  echo "No Python >= $MIN_PY found. Default python3 is:"
  echo "    $(command -v python3 2>/dev/null || echo none)  $(python3 -V 2>&1 || true)"
  echo "Install one and re-run:   brew install python@3.12"
  exit 1
fi
echo "    using $PY ($("$PY" -V 2>&1))"

# A venv on the wrong interpreter is worse than none: `python -m venv` over a
# stale one upgrades it in place and leaves the rot behind.
if [ -d .venv ] && { [ ! -x .venv/bin/python ] || \
   ! .venv/bin/python -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; }; then
  echo "    existing venv older than $MIN_PY - rebuilding"; rm -rf .venv
fi

"$PY" -m venv .venv
.venv/bin/pip install --quiet --upgrade pip
# --only-binary=:all: turns a silent multi-minute source build into an instant
# honest failure. Always the right trade here.
if ! .venv/bin/pip install --quiet --only-binary=:all: -r requirements.txt; then
  echo; echo "Install failed on $("$PY" -V 2>&1) - usually no wheel for this Python."
  echo "Retry against an older one:   PYTHON=python3.12 ./setup.sh"; exit 1
fi

echo "    $(.venv/bin/python -c 'import mujoco,numpy;print("mujoco",mujoco.__version__,"numpy",numpy.__version__)')"
echo "--> panda model"
.venv/bin/python -c "
from screener.model import panda_dir
d = panda_dir(); assert (d/'panda.xml').exists(), d
print('   ', d)"
echo
echo "Setup complete.   ./verify.sh          checks everything, cached datasets (~1 min)"
echo "                  ./verify.sh --rebuild  re-simulates truth from scratch (slow)"
