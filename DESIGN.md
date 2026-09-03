# Deciding whether a motion is safe to run

Eric Weng — Pantheon work trial, 3 Sept 2026

> **Status:** skeleton written at 14:55, before block 1. Every figure marked
> `[provisional]` is measured but will move when the swept-segment fix and the
> 4x dataset land. Figures marked `[B1]`/`[B2]`/`[B3]` do not exist yet.

---

## 0. The thesis

Screening a generated motion looks like collision detection and is not. It is a
decision taken under two kinds of uncertainty that should never have been priced
with a single number.

**Half the problem has no perception in it at all.** On a two-arm station, arm
against arm and arm against its own fixed structure involves only geometry we
ship and trajectories we commanded. There is no camera in that path, so there is
no calibration drift, no occlusion and no phantom. That half can be screened
tightly, cheaply, and its accuracy does not degrade under any scene error.

**The other half is bounded entirely by what one camera can see.** Arm against
the objects on the table inherits every perception failure, and no setting of a
clearance margin makes it simultaneously safe and productive: measured, any
false-accept:false-reject ratio above about 10:1 pins the screener at a **12%
keep rate** `[provisional]`, which starves the pipeline it exists to feed.

The way out is not a better screener. It is three cheap controls instead of one
expensive one, and **the cheapest of them is not the screener**: simulating the
reference delivery once per arrangement removes 34% of all unsafe candidates
before any are generated, and it is the only intervention in the design that
improves safety and yield at the same time. A runtime force trip then converts
what the screener still misses from a callout into an aborted rollout. Measured
end to end, the three together give **35× fewer human interventions for 8% less
yield** than the screener alone.

The trip does not buy the cost ratio down as far as first assumed — it lands
around 16–138:1 rather than 8:1 — but it does something better: it decouples the
operating point from the ratio, by making the residual false accepts cheap in
the unit that actually matters, which is human interventions per thousand
candidates.

Mo's correction — preempt rather than detect mid-rollout — is accepted, and is
why the screener runs before execution. The trip bounds the cost of the screener
being wrong; it does not replace it.

---

## 1. Scenario, chosen

**Lights-out counterfactual data collection.** Ten stations, overnight, nobody
in the room. Candidates are generated in bulk against a recorded scene and
executed to see what the world does.

Stated first because it fixes the cost model everything downstream cites.

- **False accept** (permitted, then collided): the station is stranded until a
  human resets the scene. Overnight, that is the rest of the shift on one
  station plus a callout.
- **False reject** (rejected, would have been fine): one candidate out of
  millions. Trivial alone; in aggregate it is the yield of the pipeline.

Rejected: *factory deployment alongside people* — the ratio is so lopsided that
reject-almost-everything is genuinely correct and the tradeoff stops being
interesting. *Live model inference* — motions arrive singly, so the problem is
per-motion latency and the fleet arithmetic loses its point.

---

## 2. The screening-cost arithmetic

Measured on this laptop, not quoted from the brief.

| | |
|---|---|
| Simulation, one 4-second motion | 78.9 ms → **12.7 verdicts/s/core** `[provisional]` |
| One pass over 2M candidates | 43.8 core-hours |
| Fleet: 10 stations, 1 execution / 8 s, 100 candidates screened per execution | **125 verdicts/s sustained** |
| Cores needed to sustain that by simulation | 9.9 — *affordable* |
| **Latency of one 100-candidate burst by simulation** | **7.9 s against a 100 ms budget — 79× over** |
| Fast path | **2,400 verdicts/s, ≈190×** `[provisional]` |
| Four scene hypotheses per candidate, per burst | 166 ms |

The conclusion that matters: **simulation does not fail on aggregate cost, it
fails on latency.** Ten cores is nothing. Seven point nine seconds of dead
station time per motion is everything, and it leaves no headroom at all to run
the check more than once against an uncertain scene.

That is what the 190× is really for. It is not speed for its own sake — it is
the budget that makes it affordable to screen the same motion against several
different guesses about where the objects are.

Verdicts are also **per motion and per scene**, and every successful move
changes the scene, so nothing can be cached across executions. That, not the
2M figure, is what generates continuous load.

---

## 3. Two regimes

The single largest structural claim in this document.

**Regime A — arm vs arm, arm vs station.** Both link geometries are known from
the model. Both trajectories are known because we commanded them. No perception
in the path. Screening can run a tight margin at near-zero false-reject cost,
and it is immune to systematic offset, occlusion and phantoms alike.

**Regime B — arm vs the objects on the table.** All uncertainty lives here.
Margin, hypotheses, and the entire degradation story belong to this half.

This is the direct answer to *accuracy vs cost of prevention*: the two halves
sit at different points on that tradeoff **because they are supplied by
different information**, and charging them the same margin is the mistake.

> **Predictions recorded before block 2 ran**, and what happened:
> 1. *"Splitting the regimes will not by itself lift Regime B's keep rate."*
>    **Held.** Regime B sits at 0.343 with the same conservatism as before.
> 2. *"Regime A should come in above 0.9 keep at a margin of a few millimetres."*
>    **Wrong.** It comes in at 0.607 at 0 mm and 0.494 at 20 mm. Two arms means
>    two sphere proxies, so the conservatism stacks: the gap being tested is
>    real distance minus *both* arms' proxy error. I under-estimated that, and
>    the fix is a tighter proxy, not a smaller margin.

### Measured, n = 384 station-steps (58% unsafe: 36% arm-arm, 22% object-only)

| | keep rate | false accepts | throughput |
|---|---:|---:|---:|
| **Regime A** (arm vs arm) | 0.494 | **0 (0.0%)** | 724/s |
| **Regime B** (arm vs objects) | 0.343 | 12 (14.3%) | 686/s |
| Combined station verdict | 0.104 | 2 (0.9%) | 233/s |

**Regime A catches every arm-arm collision at every margin tested**, from 0 to
50 mm. Not "few" — zero false accepts, because the quantity it screens is known
exactly rather than estimated. It buys that with false rejects, which is the
right direction to be wrong in.

### The immunity claim, measured rather than argued

Identical screener, four scene conditions:

| condition | Regime A keep / FA | Regime B keep / FA |
|---|---|---|
| perfect | 0.494 / **0** | 0.343 / 12 |
| systematic 8 mm offset | 0.494 / **0** | 0.383 / 11 |
| 15% occluded away | 0.494 / **0** | 0.427 / 13 |
| 2 phantom objects | 0.494 / **0** | 0.120 / 1 |

Regime A does not move at all — not approximately, exactly — because
`screen_regime_a` takes no `SceneEstimate` argument. The immunity is enforced by
the function signature, not by discipline.

### The uncomfortable part

The **combined** station keeps only 10.4% of good station-steps. The two regimes
reject independently, so their false rejects compound: a step survives only if
both arms clear the objects *and* clear each other. Adding a second arm roughly
halves yield before it adds any capacity. That is a real cost of bimanual
operation that the single-arm analysis could not have surfaced, and it belongs
in the deployment argument rather than being smoothed over.

---

## 4. Intended contact vs unintended — the central question

The task is putting a cube *into* a bin. Forbid the volume and you forbid the
task. Geometry cannot tell which contact is which, so the information has to
come from somewhere else, and naming that source honestly is the point.

Two inputs, and neither is free-floating:

1. **A declared target, per motion.** Every candidate carries
   `{target_object_id, destination_id}`. This costs nothing: whatever generated
   the counterfactual already knew which object the task was about. *The
   screener knows what it is supposed to touch because the thing that asked for
   the motion said so.*
2. **Per-body licence.** Fingers may reach the table surface and the target —
   they straddle a cube resting *on* the table, so a few centimetres of proxy
   overlap there is correct, not slop. The hand behind them has less licence.
   The rest of the arm has none.

The second turned out to matter more than expected. Collapsing fingers, hand and
arm into one "tool" group let the gripper slam the table without being rejected,
because the whole arm inherited the fingertips' licence to be near surfaces.

What this rests on, stated plainly for §12: perception must supply a **stable
identity** for the target object, not merely a pose. A screener that knows where
things are but not which one it is delivering to cannot make this distinction at
all.

---

## 5. The wrong picture of the table

Assumed available: wrist cameras on each arm, one exocentric camera per station.
Assumed **not** available: an accurate simulator of full scene state — shape,
compression, contact dynamics.

Each error mode is derived from that camera arrangement rather than invented:

- **Systematic offset.** One exo camera means one extrinsic calibration. When
  the mount shifts, every object moves together, in the same direction, and no
  amount of averaging helps. Modelled as a constant world-frame offset per
  session. **Handled deterministically:** "safe under any translation of norm
  ≤ m" is exactly "safe against every box inflated by m". No sampling required —
  which is the reduction that keeps the hypothesis count small.
- **Missing objects.** With a single exo viewpoint, occlusion is *structural and
  pose-dependent*, not a uniform dropout rate — so it is modelled by ray-casting
  from the camera against the other objects and against the arms.

  The camera mount was chosen by measurement rather than taste. From a shallow
  front view (1.35, 0, 0.85) **nothing is ever occluded** — rays pass beneath the
  arms — so the error mode the design cares about would never have fired. From
  above (0.75, 0, 1.25) an arm over the table hides 22–33% of objects and the
  furniture alone hides none. Occlusion at this station is caused by arms.

  **The hypothesis, and its refutation.** The expectation was sharp: the object
  most likely to be missing is the one an arm is reaching over, which is the one
  most likely to be hit — making occlusion the dominant source of false accepts.
  Measured over 384 station-steps at an 18.6% effective dropout rate:

  | | culprit was hidden from the camera |
  |---|---|
  | among false accepts (fresh estimate, other arm occluding) | 1 of 12 — **8%** |
  | among false accepts (stale estimate, same arm occluding) | 2 of 13 — 15% |
  | baseline, among *all* object collisions | 8 of 84 — 10% |

  **No correlation.** And the mechanism explains why: an arm occludes the objects
  it is *already working over*, and those are exactly the objects the other arm
  is least likely to reach. At this geometry occlusion is mildly **anti**-
  correlated with risk rather than correlated with it.

  What occlusion actually costs is **yield**: keep rate falls from 0.343 to 0.172
  when line-of-sight dropout is applied. Like phantoms, it is a productivity tax
  rather than a safety hazard — which was not the prediction.

  This does not mean the mechanism is unreal; it means it is not dominant *here*,
  at this object scale and this arm separation. A station where one arm reaches
  across the other's work area would likely show it. That is a testable
  difference, and it is stated so someone can go and test it rather than inherit
  the assumption.
- **Phantoms.** A specular detection, a shadow, a stale track. The interesting
  question is whether a phantom costs keep rate (it should) or safety (it must
  not). Measured: keep rate only.

**Unobserved space is not empty space.** The camera's occluded volume is carried
in the estimate as a region the screener may be required to clear. It is the
only structural defence against a missing object, and it is the direct
counterpart to the force trip: one guards the gap before execution, the other
during.

---

## 6. Ground truth, and how the screener cannot read it

Truth is MuJoCo executing the motion in the true scene. The harm predicate is
tiered, because "touched something" and "knocked it over" are not the same event
and pricing them alike inflates the unsafe class and flatters the screener:

1. a non-permitted movable object displaced beyond 2 cm — *knocked*
2. contact force on a non-permitted body above 6 N — *struck*
3. contact force on a permitted body above 70 N — *crushed*
4. arm against table or itself above 25 N — *structure / self*

Measured separation is clean: safe motions top out at 5.7 N of non-permitted
contact, and unsafe strikes begin at 7.0 N. The threshold is not sitting in the
middle of a cloud.

**Anti-leakage.** The screener's only input is a `SceneEstimate`: boxes and
identifiers, holding no reference to a `Layout`, no MuJoCo model and no
`Outcome`. There is no argument through which truth could arrive. `evalrun.py`
is the single file where both halves are in scope, and it is the place to audit.

---

## 7. The two error types, the cliff, and the force trip

Cost-optimal margin as a function of the assumed ratio, **n = 1280**:

| ratio | best margin | keep rate | false accepts |
|------:|------------:|----------:|--------------:|
| 3 | −3 mm | 0.483 | 46 |
| 5 | 0 mm | 0.456 | 41 |
| **8** | **0 mm** | **0.456** | 41 |
| 10 | +13 mm | 0.305 | 27 |
| 15 | +34 mm | 0.122 | 15 |
| 30 | +34 mm | 0.122 | 15 |
| 1000 | +45 mm | 0.070 | 14 |

**A correction to an earlier draft.** At n = 320 this looked like a cliff — the
optimum jumped from +5 mm to +25 mm between ratios of 8 and 10, and the natural
reading was "the ratio is not a dial." At four times the data it is a steep but
*continuous* decline. The cliff was substantially a small-sample artefact: it
rested on false-accept counts moving from 13 to 1, i.e. on about a dozen
motions. The conclusion survives in weaker form — yield falls off hard above
~10:1 and the design must drive the ratio down — but the sharp edge does not,
and reporting it would have been reporting noise.

A raw crash on unattended hardware is not an 8:1 event; it is closer to 1000:1.
So the ratio has to be bought down. The force trip was the proposed mechanism,
and it was an assertion until measured. It is measured now, and the answer is
more interesting than the assertion.

### First: which sensor, and what that actually requires

The trip is **not part of the screener**. The screener is kinematic, runs before
anything moves, and never computes a force; the trip runs on the robot during
execution and reads its own sensors. They live in separate packages for that
reason (`screener/` and `station/`).

An earlier draft said "a wrist sensor" without checking whether a wrist sensor
could see the collisions in question. It can, for one workload and not the other,
and the split is sharp enough to be a design requirement rather than a footnote.
A wrist force/torque sensor senses only what is transmitted *through the wrist*;
joint torque sensing on all seven axes (which the Panda has) sees the whole arm.

| | harmful contacts made by | a wrist F/T sensor sees |
|---|---|---|
| arm vs objects (n=116) | 99% hand or fingers | **100%** |
| arm vs arm (n=59) | 83% link5/link6 — elbow and forearm | **5%** |

**A wrist sensor is sufficient for the single-arm tabletop task and close to
useless for the bimanual one.** Anyone specifying hardware from the single-arm
result would under-buy for a two-arm station.

That is fortunate rather than alarming, and it is the strongest argument for the
layering in §7b:

| | pre-execution screener | wrist force sensor |
|---|---|---|
| arm vs objects | ~90% of unsafe caught | 100% |
| arm vs arm | **100% — zero false accepts** | 5% |

**Each control catches precisely what the other structurally cannot.** The
screener is blind to force and therefore to the delivering-vs-crushing
distinction; a wrist sensor is blind to the elbow. They are not redundancy, they
are complementary coverage, and the numbers say so rather than the prose.

Everything below is measured with contact force scored anywhere on the arm —
i.e. assuming joint torque sensing. On a station with wrist-only sensing, the
arm-vs-arm row does not hold and the screener carries that class alone.

### The trip has its own intended-contact problem

A force sensor cannot tell which object it touched. Grasping the target and
entering the bin both register, so the threshold floor is set by what *correct*
motions already generate — measured, not assumed:

| threshold | fires on SAFE motions | catches UNSAFE motions |
|---:|---:|---:|
| 10 N | 17.4% | **100%** |
| 20 N | 9.1% | 82% |
| 30 N | 3.0% | 74% |
| 80 N | 0.0% | 51% |

The trip has its own ROC curve, and the asymmetry is favourable in a way the
screener's is not: **a false trip is an aborted rollout, not a crash.** It costs
a candidate, exactly like a false reject. So the trip can be run aggressively —
10 N catches everything, and the 17% it wrongly aborts is a yield tax, not a
safety event.

### What it prevents

111 unsafe motions, re-executed with the trip armed:

| reaction | peak force median / p90 | scene resets needed |
|---|---|---|
| none | 85 N / 183 N | 8 / 111 |
| freeze position targets | **24 N / 74 N** | 5 / 111 |
| active retract 120 ms | 24 N / 74 N | 4 / 111 |

Peak force falls 3.5×. The reaction *policy* barely matters — freezing and
retracting are within one event of each other — so the engineering requirement is
on the **threshold and the sensing latency**, not on a clever recovery behaviour.
That is a cheaper requirement to meet, and worth knowing before anyone builds a
retract reflex.

### The honest correction to the premise

The trip does **not** buy the ratio down to 8:1. Measured, a false accept under
the trip costs `0.955 × (an aborted rollout) + 0.045 × (a reset and a callout)`,
which lands at **16:1 to 138:1** depending on what a callout is worth. Still
above the band where the operating point stops being pinned.

It does not need to. What the trip actually does is **decouple the operating
point from the ratio**, by making the residual false accepts cheap in the units
that matter — human interventions:

### The three controls together, per 1000 candidates screened

| configuration | executed | collided | **human resets** | usable episodes |
|---|---:|---:|---:|---:|
| screener only | 333 | 32.0 | **32.0** | 301 |
| + feasibility gate | 357 | 20.2 | **20.2** | 337 |
| + force trip @ 10 N | 357 | 20.2 | **0.9** | 278 |
| screener + trip, no gate | 333 | 32.0 | 1.4 | 248 |

**35× fewer human interventions for 8% less yield.** And the ordering matters:

- The **feasibility gate is strictly dominant** — it improves safety *and* yield
  simultaneously, because it removes candidates that were never achievable. It is
  the only control here with no downside, and it is the cheapest.
- The **trip** is what collapses interventions, at a yield cost paid in false
  trips.
- The **screener** is what makes the trip's job small enough to be survivable —
  without it every candidate would be a physical experiment.

None of the three is sufficient. The screener alone leaves 32 callouts per
thousand; the trip alone leaves the pipeline running blind into 32 collisions and
recovers less yield; the gate alone does not address motion-level errors at all.
**The design is the combination, and the argument for it is this table.**

Secondary and nearly free, from the same notes: **flag rollouts running
abnormally long for their task.** Not a collision detector, but an anomaly signal
on a channel that already exists.

---

## 7b. Three controls, not one — and the cheapest one is not the screener

The false-accept rate does **not** go to zero with margin. At +45 mm the screener
still permits 14 unsafe motions (3.2% of them) while keeping only 7% of good
ones. That floor is the interesting part, because margin cannot touch it.

Inspecting the floor: **11 of those 14 are the same layout**, all crushing the
same bin at 73–89 N, and five of them are the *reference* motion. Widening the
search: in **6 of 32 arrangements the reference delivery is itself unsafe.** The
bin sits where the arm cannot deliver into it without pressing on it.

Those six layouts hold 18.8% of all candidates but **34% of all unsafe ones**.

**When the reference motion collides, no motion-level screener can help** — the
task is not achievable in that arrangement, and every candidate generated for it
inherits the problem. The right control is not a better screener but a
feasibility check on the *arrangement*, run once per scene change:

| | control | cost | what it removes |
|---|---|---|---|
| 1 | **Layout feasibility gate** — simulate the reference delivery once per (target, destination) pair per arrangement | **one** simulation per scene change | 34% of all unsafe candidates, before any are generated |
| 2 | **Per-motion geometric screen** — the fast path | ~1 ms per candidate | the bulk of the rest |
| 3 | **Runtime force trip** | a sensor already on the arm | what geometry structurally cannot see |

Measured effect of adding control 1, at identical screener settings:

| margin | FA, all layouts | FA, serveable only | keep rate |
|---:|---:|---:|---:|
| 0 mm | 41 (9.4%) | 21 (7.4%) | 0.456 → 0.464 |
| +26 mm | 19 (4.4%) | 6 (2.1%) | 0.161 → 0.163 |
| +45 mm | 14 (3.2%) | **1 (0.4%)** | 0.070 → 0.070 |

**Keep rate is unchanged.** This is the only intervention in the whole design
that improves safety at no cost to yield, and it is also by far the cheapest —
one simulation against a hundred. That ordering is the answer to *accuracy vs
cost of prevention*: spend the first simulation on the arrangement, not on the
candidate.

It also reframes what the screener is for. Once unserveable arrangements are
gated, the screener is no longer being asked to compensate for a task that
cannot be done — it is only sorting achievable motions, which is the job it can
actually do.

---

## 8. The fast path

Swept-sphere screening, no physics.

- **Its own forward kinematics, in numpy.** Calling `mj_kinematics` once per
  sampled pose is ~10 µs; at 24 poses that is 240 µs per candidate, already 3×
  over budget before a single collision test. FK is batched across every
  candidate and pose at once.
- **The arm as spheres fitted to its collision meshes**, because sphere-vs-box
  distance is exact, branch-free and broadcastable. The fit is a strict
  over-approximation: proxy error costs keep rate, never safety. Getting it
  tight mattered — a body-frame AABB fit gave 15-18 cm radii, even spacing along
  the principal axis gave ~9-10 cm, Lloyd clustering gave 4.6 cm median. A loose
  proxy spends the whole margin budget on its own approximation.
- **Per-body licence groups** (finger / hand / body) as in §4.
- **Pose sampling: measured, not assumed.** The suspicion going into block 1 was
  tunnelling — that 24 poses over a 4-second motion leaves gaps a thin object
  fits through, and that both held-out false accepts being `random_via` motions
  was the symptom. **It is not.** Sixteen times the density (24 → 384 poses)
  removes 2 of 15 false accepts on A and 1 of 7 on B, and everything converges by
  48. It costs 16× throughput to buy that. `n_poses = 48` is the settled value,
  and the false accepts that remain are genuine geometric misjudgements. A
  hypothesis that survives only because nobody measured it is not a finding, and
  this one did not survive.

---

## 8b. The bug that was actually costing accuracy

Worth recording because it changed what the intended-contact problem *is*.

The sphere-to-box gap was computed by clamping the sphere centre into the box
and measuring to the clamped point. That is exact outside the box and silently
wrong inside it: for any centre within the box the nearest point is the centre
itself, so the gap saturates at −radius no matter how deep the penetration.

The consequence was invisible until measured directly. Motions that **delivered
into a bin** and motions that **crushed the same bin at 100–270 N** both reported
a finger-to-bin gap of exactly **−0.0125 m** — the finger sphere radius. The two
populations were numerically identical, so no threshold on that quantity could
ever have separated them, and the intended-contact decision looked structurally
impossible when it was merely unmeasured.

Replacing it with the exact box signed distance (`outside + inside − radius`)
recovers real depths, and the separation appears: median −0.0135 m for a clean
delivery against −0.0152 m for a crush. Still overlapping, still hard — but no
longer a degenerate constant.

Two things follow. First, every licence in §4 had been fitted against a
saturating quantity and all of them had to be refitted. Second, and more
usefully: **the overlap that remains is the honest statement of the limit.**
Entering a bin and crushing a bin are separated by about 2 mm of proxy
penetration, and a kinematic screener working from a camera estimate cannot
resolve that reliably. Force can, trivially. This is the sharpest argument in
the document for why the runtime trip is structural rather than decorative — not
because the screener is badly built, but because the quantity that separates
these two events is not a geometric one.

---

## 9. Results

All figures: generator A, n = 1280 motions over 32 layouts, 34% unsafe; the
held-out set is generator B, n = 640 over 16 layouts, 32% unsafe. Chosen
operating point: margin 0 mm, 48 poses, licences as fitted in §4.

### Speed

| | |
|---|---|
| Simulation (truth) | 12.1 verdicts/s/core — 45.8 core-hours for 2M candidates |
| Fast path | ~940 verdicts/s — **77×** |
| A 100-candidate burst, by simulation | 8.2 s (82× over the 100 ms budget) |
| The same burst, fast path, 4 hypotheses | 0.43 s — **needs 4 cores to fit the budget, and gets them** |

The 77× is lower than the 190× an earlier draft reported, for two honest
reasons: pose sampling went from 24 to 48 (measured, §8), and this figure is
taken over 40-motion batches, which under-amortises the vectorisation. The
deployment batch is 100 per station, so 77× is a floor rather than a headline.

### Accuracy against truth

| | keep rate | false accepts | of unsafe |
|---|---:|---:|---:|
| generator A (tuned on) | 0.456 | 41 | 9.4% |
| generator B (held out) | 0.628 | 16 | 7.8% |
| generator B, **+ self-collision check** | 0.401 | **8** | **3.9%** |

### Degradation, each scene error alone

| condition | keep | false accepts |
|---|---:|---:|
| perfect | 0.456 | 41 (9.4%) |
| systematic 8 mm offset | 0.472 | 43 (9.9%) |
| …with margin inflated by the drift bound | 0.032 | 0 |
| 15% of objects occluded away | 0.489 | **79 (18.2%)** |
| …screened against 4 sampled hypotheses | 0.456 | **41 (9.4%)** |
| 2 phantom objects | 0.346 | 29 (6.7%) |
| all three together | 0.013 | 0 |

Three things to read off this:

- **Occlusion is the dangerous one.** It doubles false accepts, and it is the
  only mode that does. Offset and phantoms move yield, not safety — a phantom
  even *lowers* false accepts, because it rejects more.
- **Four hypotheses restore the baseline exactly** — 41 false accepts, not
  approximately 41. Sampling the discrete error and requiring unanimity recovers
  precisely what occlusion cost, and the speedup budget was sized to afford it.
- **Margin inflation is a blunt instrument at this operating point.** It removes
  every false accept and leaves 3% yield. It was the right answer when the margin
  was large; at margin 0 it simply rejects everything, and `all three together`
  is the same story. Reported as a failure, not averaged away.

### The held-out generator, and what it caught

Generator B scores *better* than A on the headline (0.628 keep vs 0.456). That is
**not** evidence of generalisation and is not reported as such: A perturbs a good
reference, so every candidate sits near the decision boundary, while B's arcs and
drags are mostly obviously fine or obviously terrible. B is the easier set.

But B earned its place anyway, by finding something no amount of tuning on A
could have:

> **62% of generator B's false accepts were self-collisions. Generator A's:
> zero.** The screener had no self-collision test at all. A perturbs a reference
> that never contorts the arm, so the entire failure mode was invisible.

Adding it (§8c) halves B's false accepts, 16 → 8, and **changes generator A's
numbers not at all** — 0.456 keep and 41 false accepts before and after. A blind
spot that was free to fix and impossible to see from the training distribution.

That is the real answer to *how much did writing both sides flatter you*: not a
percentage, a missing check.

## 9b. Three predictions, three refutations

Recorded because it is the most useful thing this build produced, and because
the pattern is the argument for measuring rather than reasoning:

| predicted to matter | measured |
|---|---|
| **Tunnelling** between sampled poses is the blind spot | 16× the sampling density removes 2 of 15 false accepts and costs 16× throughput. Not the blind spot. |
| **The cost ratio is a cliff**, so the operating point is bimodal | At 4× the data it is a steep but continuous decline. The cliff was a small-sample artefact resting on about a dozen motions. |
| **Occlusion hides the object you are about to hit** | 8% of false-accept culprits were hidden against a 10% baseline. Mildly anti-correlated, not correlated. |

Meanwhile, the two things that did matter were not predicted at all:

- a **saturating distance function** that made bin deliveries and bin crushes
  numerically identical (§8b), and
- **six of thirty-two arrangements being unserveable**, holding 34% of all unsafe
  candidates, removable by one simulation each (§7b).

Every one of the three refuted hypotheses is individually plausible, and two of
them came from careful reasoning about the physics. They were wrong anyway. The
cheap measurement that disconfirmed each took under twenty minutes; carrying any
of them into the design would have cost a great deal more.

---

## 10. Failures, walked through

Three real motions, one of each kind.

### L003-A008 — permitted, then crushed the bin it was delivering to (99 N)

What the screener saw: finger-to-`bin0` gap **−14.8 mm**, everything else clear.
The destination licence allows −20 mm, so it passed.

A *clean* delivery into the same bin uses **−13.5 mm**. The screener was asked to
separate two events that differ by **1.3 mm of proxy penetration**, from a camera
estimate, using a proxy whose own approximation error is several millimetres.

It cannot. This is the sharpest statement of the intended-contact limit in the
whole submission, and it is not a tuning failure — the quantity that separates
delivering from crushing is *force*, and no geometric threshold recovers it. It
is the single clearest argument for the runtime trip.

### L000-A001 — rejected, and was perfectly safe

What the screener saw: hand-to-`post1` gap **−20.6 mm**. Truth: the arm never
touched the post.

The overlap is entirely the sphere proxy. The hand's covering spheres are larger
than the hand, so a 20 mm "penetration" is a motion passing cleanly by. This is
the shape of nearly every false reject: not a misjudged trajectory, but the proxy
being conservative in the direction we chose for it to be conservative.

The pattern: **false rejects are proxy error, false accepts are intent error.**
They have different causes and different fixes, and averaging them into one
accuracy number hides that.

### L003-B035 — permitted, and the arm collided with itself at 154 N

What the screener saw: every object clear by a wide margin — the nearest was
`table` at +56 mm, and every object at over 120 mm.

It was right about all of them. The arm folded into itself, and nothing in the
screener was watching. Generator A produced no motion contorted enough to expose
this; generator B, sampling joint space directly, produced twenty-seven.

The fix belongs in Regime A rather than anywhere near the scene estimate:
self-collision is known geometry on a commanded trajectory, with no camera in the
path. That it was missing is the most useful thing the held-out set found.

---

## 8c. Self-collision, and the allowed-collision problem

Added after generator B exposed it. The naive version — reject when any two
non-adjacent link spheres overlap — **rejects 100% of motions**, because a proxy
that strictly contains the arm necessarily has the shoulder's covering spheres
reaching into the elbow's in perfectly ordinary poses.

MuJoCo solves the same problem for the real model with an explicit
`<contact><exclude>` list. The equivalent here is learned rather than declared:
run configurations that truth says did not self-collide, record how close each
sphere pair legitimately gets, and require a real self-collision to beat that
floor by 4 mm. Calibrating against the label rather than against intuition
matters — hand-picking which link pairs "should" be excluded is exactly how the
arbitrary parts of a screener get in.

---

## 11. What I am not building

- **No learned screener.** No time to validate one, and an unvalidated model's
  failure mode is precisely the subject of this exercise.
- **No perception system.** Poses arrive as an estimate with a stated error
  model. §12 states what would be required instead.
- **No compression, deformation or shape modelling.** Boxes and spheres. Named
  blind spot: a soft or thin object screened as a box is wrong in a direction I
  do not control.
- **No dynamics beyond the trajectory.** Screening is kinematic. A motion that
  is geometrically clear but dynamically violent is not caught — the force trip
  is the answer, which is another way the trip is load-bearing.
- **No scheduler, orchestration, or UI beyond the review view.**

---

## 12. What the screener would need from perception

Stated as requirements someone could go and meet:

1. **Stable object identity, not just pose.** Without it §4's distinction between
   intended and unintended contact cannot be made at all.
2. **A calibrated bound on systematic drift**, per session. The margin-inflation
   result is exact given a bound and worthless without one.
3. **An explicit occluded-volume output**, not just a list of detections. A
   detector that silently omits what it cannot see gives the screener no way to
   know it is guessing.
4. **A false-positive rate on detections**, so phantom cost can be priced.
5. **Latency under the station's budget**, since a stale estimate is a wrong
   estimate in exactly the systematic way §5 describes.
