# Deciding whether a motion is safe to run

A pre-execution screener for a two-arm data-collection station. Given a generated
candidate motion and a camera-derived estimate of the table, decide whether
executing it would touch something it was not supposed to touch — fast enough to
run inline, and honestly enough to survive the estimate being wrong.

**[DESIGN.md](DESIGN.md) is the document to read.** This file says what is here
and how to run it.

## The result in five lines

- Simulation is **12.1 verdicts/s/core**. It does not fail on cost — 10 cores
  would sustain the fleet — it fails on **latency**: 8.2 s per 100-candidate
  burst against a 100 ms budget.
- The fast path is **77× faster**, and that headroom is what pays for screening
  each candidate against four different guesses about where the objects are.
- **Occlusion is the only scene error that costs safety** (false accepts 9.4% →
  18.2%). Four sampled hypotheses restore the baseline exactly. Offset and
  phantoms cost yield only.
- **Arm-vs-arm screening has zero false accepts** at every margin tested and is
  exactly invariant under all three scene errors — it reads no camera estimate.
- Three controls, none sufficient alone. Per 1000 candidates: **32 human
  interventions → 0.9**, for 8% less yield.
- They are complementary, not redundant: the screener catches **100%** of
  arm-vs-arm collisions and a wrist force sensor catches **5%** of them (they
  happen at the elbow); for arm-vs-object collisions it is the other way round.

## Running it

```bash
python -m venv .venv && . .venv/bin/activate
pip install -r requirements.txt

python experiments.py --rebuild   # ~4 min: simulates truth, writes out/results.json
python export_view.py && python build_view.py   # writes out/view.html
```

`experiments.py` without `--rebuild` reuses the cached datasets in `out/`.

Truth simulation is the expensive part; `build_chunk.py` / `merge_chunks.py`
exist because it is convenient to build the dataset in pieces.

**Menagerie note.** `panda.xml` declares `<compiler meshdir="assets"/>` relative
to itself, so a scene living elsewhere fails on a missing mesh. The assembled
scene is written into the Panda model's own directory. Set `PANDA_DIR` to point
at a local checkout; otherwise `robot_descriptions` downloads one on first use.

## Layout

| | |
|---|---|
| `screener/scene.py` | object model, layout generation, the two-arm station |
| `screener/model.py` | MJCF assembly; `build_dual` attaches two prefixed Pandas |
| `screener/motions.py` | IK and generator A (perturbations of a scripted reference) |
| `screener/heldout.py` | generator B — written *after* the screener was tuned |
| `screener/truth.py` | the oracle and the tiered harm predicate |
| `screener/estimate.py` | **the only thing the screener may read**; the three error modes |
| `screener/geom.py` | vectorised sphere/box distance — imports no mujoco, by design |
| `screener/fast.py` | batched FK, the sphere proxy, the screener, self-collision |
| `screener/dual.py` | Regime A: arm vs arm |

| `screener/evalrun.py` | evaluation — **the only file where truth and estimate are both in scope** |
| `station/trip.py` | the runtime force trip. **Not part of the screener** — it runs on the robot during execution and reads a sensor; the screener is kinematic and runs before anything moves |

If you are auditing for leakage, `evalrun.py` is the place. `screen()` takes a
`SceneEstimate`; there is no argument through which an `Outcome` could arrive.

## What is not here

No learned screener, no perception system, no deformable or compressible objects,
no dynamics beyond the trajectory, no scheduler. Each is argued in DESIGN.md §11
rather than silently omitted.
