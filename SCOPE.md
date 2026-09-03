# Scope

Written after the check-in notes, before the rewrite of DESIGN.md. This is the
contract: what I am building, what I am not, and the argument the numbers have
to support.

---

## 0. The commitment sentence

> A pre-execution screener for a two-arm inference station that separates the
> half of the problem perception can see from the half it cannot, sits at an
> operating point justified by a runtime force trip rather than by a margin,
> and reports what that costs against simulated truth under three named kinds
> of wrong scene.

Explicitly not: a better simulator, a learned model, a perception system.

---

## 1. Scenario (mine to choose; chosen)

**Lights-out counterfactual data collection.** Ten stations, overnight, nobody
in the room. Candidate motions are generated in bulk against a recorded scene
and executed to see what the world does.

This choice is what fixes the cost model, so it is stated first and everything
downstream cites it.

- **False accept** (permitted, collided): station stranded until a human resets
  the scene. On an overnight run that is the rest of the shift on one station,
  plus a callout.
- **False reject** (rejected, would have been fine): one candidate out of
  millions, thrown away. Cheap in isolation; the aggregate is the yield of the
  whole pipeline.

Rejected alternatives: *factory deployment* (ratio so lopsided the tradeoff
stops being interesting — reject-almost-everything is genuinely correct there);
*model inference* (motions arrive singly, so the problem becomes latency, not
throughput, and the fleet arithmetic loses its point).

---

## 2. The cost ratio, and why the force trip is load-bearing

Measured on the existing curve (n=320: 230 safe, 90 unsafe), the cost-optimal
operating point as a function of the false-accept : false-reject ratio:

| ratio | best margin | keep rate | FA / 90 unsafe |
|------:|------------:|----------:|---------------:|
| 1000  | +25 mm | 0.12 | 1 |
| 100   | +25 mm | 0.12 | 1 |
| 30    | +25 mm | 0.12 | 1 |
| 10    | +25 mm | 0.12 | 1 |
| **8** | **+5 mm** | **0.57** | 13 |
| 6     |  +0 mm | 0.64 | 15 |
| 3     |  -5 mm | 0.68 | 18 |

**The finding:** the ratio is not a dial, it is a cliff. Anywhere above ~10:1
the screener is pinned at a 12% keep rate — it rejects seven of every eight
good motions, and a pipeline built on it starves. The operating point only
becomes useful below 8:1.

A raw crash on unattended hardware is not an 8:1 event. So the ratio has to be
*bought down*, and the only mechanism available is the one Mo raised:

**A runtime force/torque trip changes what a false accept costs.** A permitted
motion that collides, caught at a low force threshold within milliseconds of
first contact, is an aborted rollout and a logged event — not a knocked-over
bin and not damaged hardware. It converts the tail of the false-accept
distribution from "human drives to the site" into "station retries the next
candidate."

That is the argument, and it runs in one direction only: **the screener is
allowed to sit at 57% keep because the trip exists.** Remove the trip and the
defensible operating point is +25 mm and 12% yield. I will state the residual
ratio I assume (~6-8:1), what the trip must achieve for it to hold, and what
happens to the whole design if it does not.

Secondary, cheap, and also from the notes: **reject rollouts running abnormally
long for their task.** Not a collision detector, but a free anomaly signal on a
channel that already exists.

Mo's correction — preempt rather than detect mid-rollout — is accepted and is
why the screener is pre-execution. The trip is the backstop that bounds the
cost of the screener being wrong, not a replacement for it.

---

## 3. Two regimes, because they have different physics of *uncertainty*

The single biggest structural change from the notes: **two arms per station.**
Collisions split into two populations that a single margin should never have
been asked to cover.

**Regime A — arm vs arm, and arm vs its own station.** Both arms' link
geometry is known from the model; both trajectories are known because we
commanded them. **There is no perception in this path at all.** No systematic
offset, no occlusion, no phantoms. Screening here can run a tight margin at
near-zero false-reject cost, and its accuracy does not degrade under any of the
scene-error modes. It is nearly free and nearly exact, and saying so is the
point.

**Regime B — arm vs the objects on the table.** Everything uncertain lives
here. This is where margin, hypotheses, and the whole degradation story belong.

Splitting them is the direct answer to "accuracy vs cost of prevention": the
two halves sit at different points on that tradeoff *because they are supplied
by different information*, and pricing them identically is what produced a 12%
keep rate.

**Intended contact** stays inside Regime B and is resolved the same way as
before — per-body licence (fingers may reach the table and the target; the hand
less so; the rest of the arm not at all) plus a per-motion declared target. The
information comes from the task specification that generated the candidate, not
from geometry, and the note will say so plainly: *the screener knows which
object it is supposed to touch because whatever asked for the motion said so.*

---

## 4. Wrong scene, derived from the camera budget I was given

Assumed available: wrist cameras on each arm, one exocentric camera per
station. Assumed *not* available: an accurate simulator of full scene state
(shape, compression, contact dynamics).

Each error mode traced to that arrangement rather than invented:

- **Systematic offset** — one exo camera means one extrinsic calibration. When
  the mount shifts, every object moves together, in the same direction. Not
  averageable. Modelled as a constant world-frame offset per session.
- **Missing objects** — with a single exo viewpoint, occlusion is *structural
  and pose-dependent*, not a uniform dropout rate. The occluder is usually an
  arm. **The object most likely to be missing from the estimate is the one the
  arm is currently reaching over — which is the one it is most likely to hit.**
  Replacing the current uniform `p_missing` with occlusion computed from the
  exo camera's line of sight is what makes the degradation number mean
  anything. This is the highest-value change to the error model.
- **Phantoms** — a detection on a specular surface, a shadow, a previous
  object's stale track. Cheap to model, and the interesting question is whether
  a phantom costs keep rate (it should) or safety (it should not).

---

## 5. What I am not building

- No learned screener. No time to validate one, and the failure mode of an
  unvalidated model here is the exact one the exercise is about.
- No perception system. Object poses arrive as an estimate with a stated error
  model; producing them is out of scope. Phase 4's "what the screener needs
  from perception" is written as requirements instead.
- No compression, deformation or object-shape modelling. Boxes and spheres.
  Named as a blind spot: a soft or thin object screened as a box is wrong in a
  direction I do not control.
- No dynamics beyond the trajectory. Screening is kinematic; a motion that is
  geometrically clear but dynamically violent is not caught, and the force trip
  is the answer.
- No multi-station scheduler, no orchestration, no UI beyond the review view.

---

## 6. Sequence, with stop rules

Revised 14:55 after review. Three decisions confirmed: DESIGN.md is written
*before* the view; the dataset is enlarged before the cliff is quoted; Regime A
covers both arms moving. Previous draft totalled 5h55m against ~5h available and
placed the scored artifact last.

| # | Block | Window | Ends with |
|---|-------|--------|-----------|
| 0 | DESIGN.md thesis + section skeleton | 14:55-15:15 | The argument on paper before the numbers exist |
| 1 | Swept-segment fix, 4x dataset, curve re-cut at 1mm | 15:15-16:15 | An operating point that survives its own error bars |
| 2 | Regime A: second arm, arm-arm as its own screening path | 16:15-17:40 | Two-regime numbers |
| 3 | Line-of-sight occlusion replaces uniform `p_missing` | 17:40-18:20 | Degradation numbers that mean something |
| 4 | **DESIGN.md fill + failure walkthrough** | 18:20-19:15 | The submission |
| 5 | The view, with whatever remains | 19:15-20:00 | Something to walk Mo through |

**Why the doc moved ahead of the view.** Mo's note says the evaluation is on
thinking, prioritisation and code structure, not a finished deliverable. Putting
the view before DESIGN.md means any overrun upstream is paid for out of the one
artifact being scored. Inverted, an overrun costs polish instead. Block 4's
failure walkthrough produces static renders of two or three cases, so even if
block 5 is lost entirely there is something visual on the call.

**Block 1, in order.** The tunnelling check comes first because a margin tuned
around a sampling artefact is not a margin; the dataset enlargement is folded in
here because the cliff is this document's central claim and it currently rests
on false-accept counts of 1 to 18 out of 90 unsafe motions.

1. Swept-segment test between consecutive poses, replacing point sampling at 24
   discrete poses (~20m). Report the change in false accepts at fixed margin:
   this is the number that says whether the blind spot was sampling or geometry.
2. Rebuild truth at 4x (~1280 motions, roughly 3 minutes of simulation). Every
   downstream degradation figure re-runs against it.
3. Re-sample the curve at 1mm resolution across 0-10mm, where the cliff sits.
4. Only then fix the ratio and choose the operating point.

If the tunnelling fix does *not* move the curve, that is also a result and gets
reported: it would mean the false accepts are genuine geometric misjudgements,
the margin is doing real work, and block 2 is answering a different question.

**Hard checkpoint at 17:40.** If block 2 is not producing arm-arm numbers by
then, drop to a second arm on a fixed scripted trajectory plus written analysis,
and move on. Block 2 is the likeliest overrun: it needs a second arm in the
MJCF, the harm predicate extended to arm-arm contact, and a two-arm motion
generator. Do not borrow from block 3 or 4 to finish it.

**Cut order if short:** block 5 goes first, entirely. Then block 3 degrades to
the existing uniform `p_missing` with the line-of-sight argument written but not
measured. Then block 2 degrades as above. Blocks 1 and 4 are never cut, because
every number cites block 1 and block 4 is the deliverable.

## 7. Honest corrections to carry into the note

- The held-out result is currently **backwards**: generator B scores *better*
  than A (keep 0.45 vs 0.22, FA 2 vs 4). B is easier — 31% of its motions are
  unsafe against A's 28%, and its unsafe motions are less marginal. This is not
  evidence of generalisation and must not be reported as such. The correct
  conclusion is that B failed to be adversarial, and the fix is to write B
  against motion families A never produced, not to re-tune.
- Both of B's false accepts are `random_via`. That is a pattern and it gets
  named: unconstrained via-points produce paths whose sampled poses straddle an
  object between samples. **Pose sampling density is the blind spot**, not the
  margin — 24 discrete poses over a four-second motion leaves gaps a thin object
  fits through. A swept-segment test between consecutive poses is the fix and it
  is the *first* thing in block 1, before any operating point is chosen, because
  a margin tuned around a sampling artefact is not a margin.
- `all_three` scene errors currently yields a **0.00 keep rate**. A screener
  that permits nothing is reported as a failure, not folded into an average.
- The two-regime split is currently a **prediction, not a finding**. There are
  zero arm-arm cases in the dataset: the scene has one arm. The 12% keep rate
  comes entirely from proxy conservatism against posts in Regime B, so splitting
  the regimes cannot by itself lift Regime B's yield. State the predicted number
  before block 2 runs, then report whether it held.
- A pose-density sweep (16/24/40/64) was already run and moved almost nothing --
  but at a degenerate configuration where keep rate was ~3% and nearly everything
  was rejected, so it is uninformative. The tunnelling question is open, not
  answered.
