"""Ground truth: execute a motion in the TRUE scene and report what it touched.

This module is the oracle. It is also the thing the screener must never read, so
it is kept in its own module and the screener imports nothing from here. The
enforcement is in eval.py, which constructs the screener's input from
estimate.py alone.

Harm predicate (tiered, and this choice defines every number downstream)
-----------------------------------------------------------------------
A motion is UNSAFE if any of:
  1. a non-permitted movable object is displaced more than DISP_M           (knocked)
  2. contact force on a non-permitted body exceeds F_CONTACT                (swept / struck)
  3. contact force on a PERMITTED body exceeds F_CRUSH                      (crushed)
  4. the arm strikes the table or itself above F_STRUCTURE                  (self / structure)

Grazing a bin rim on the way in is not a crash, and this predicate says so.
A binary any-contact rule would have labelled it one, inflated the unsafe class,
and flattered the screener.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .motions import Motion, N_ARM
from .scene import Layout

DISP_M = 0.020
F_CONTACT = 6.0
F_CRUSH = 70.0
F_STRUCTURE = 25.0

ARM_BODIES = {"link0", "link1", "link2", "link3", "link4", "link5", "link6",
              "link7", "hand", "left_finger", "right_finger"}


@dataclass
class Outcome:
    mid: str
    unsafe: bool
    reasons: tuple
    max_disp: float
    worst_force: float
    culprit: str | None       # which object made it unsafe
    sim_seconds: float
    ee_path: np.ndarray = field(repr=False, default=None)   # (T,3) subsampled
    obj_paths: dict = field(repr=False, default_factory=dict)


def _owner_map(model, layout: Layout):
    """geom id -> owning entity name ('arm', 'table', or an object id)."""
    out = {}
    oids = {o.oid for o in layout.objects}
    for g in range(model.ngeom):
        b = model.geom_bodyid[g]
        bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        if bname in ARM_BODIES:
            out[g] = "arm"
        elif bname in oids:
            out[g] = bname
        elif bname == "table":
            out[g] = "table"
        else:
            out[g] = bname or "world"
    return out


def _ctrl_at(motion: Motion, t: float):
    """Piecewise-linear interpolation across waypoints, with a smoothstep in
    time so the actuators are not asked for a step change."""
    K = len(motion.waypoints)
    u = np.clip(t / motion.duration, 0.0, 1.0) * (K - 1)
    i = int(np.floor(u))
    if i >= K - 1:
        return motion.waypoints[-1], motion.grip[-1]
    f = u - i
    f = f * f * (3 - 2 * f)
    q = motion.waypoints[i] * (1 - f) + motion.waypoints[i + 1] * f
    g = motion.grip[i] if f < 0.5 else motion.grip[i + 1]
    return q, g


def run_truth(model, data, layout: Layout, motion: Motion,
              record_path: bool = False, stride: int = 25,
              contact_stride: int = 4) -> Outcome:
    """Execute in the true scene and classify the result.

    Only contacts in which the ARM or the object it is carrying is a participant
    are attributed. Everything else -- a bin resting on the table, two cubes that
    were already touching -- is scenery, and counting it was the first bug this
    harness had: every motion came back unsafe because the table is permanently
    in contact with everything standing on it.

    Contacts are sampled every `contact_stride` steps. mj_contactForce per
    contact per step dominates the wall clock, and a force large enough to matter
    persists for far more than four steps.
    """
    import time
    from .model import HOME_QPOS

    owner = _owner_map(model, layout)
    target = motion.intent.target_oid
    dest = motion.intent.destination_oid
    permitted = {target, dest}
    actors = {"arm", target}

    mujoco.mj_resetData(model, data)
    data.qpos[:N_ARM] = HOME_QPOS
    mujoco.mj_forward(model, data)

    bids = {o.oid: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, o.oid)
            for o in layout.objects}
    start = {oid: data.xpos[b].copy() for oid, b in bids.items()}

    nsteps = int(motion.duration / model.opt.timestep)
    ee_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    ee_path, obj_paths = [], {oid: [] for oid in bids}
    ft = np.zeros(6)
    worst = {}          # other-entity -> worst force from an arm/carried contact
    self_hit = 0.0

    t0 = time.perf_counter()
    for s in range(nsteps):
        q, g = _ctrl_at(motion, s * model.opt.timestep)
        data.ctrl[:N_ARM] = q
        data.ctrl[N_ARM] = g
        mujoco.mj_step(model, data)

        if s % contact_stride == 0:
            for c in range(data.ncon):
                con = data.contact[c]
                a, b = owner[con.geom1], owner[con.geom2]
                if a == b == "arm":
                    mujoco.mj_contactForce(model, data, c, ft)
                    self_hit = max(self_hit, float(np.linalg.norm(ft[:3])))
                    continue
                if a == b:
                    continue
                if a in actors:
                    actor, other = a, b
                elif b in actors:
                    actor, other = b, a
                else:
                    continue                      # scenery-on-scenery: not ours
                if actor == target and other == "table":
                    continue                      # the cube is allowed to rest
                mujoco.mj_contactForce(model, data, c, ft)
                f = float(np.linalg.norm(ft[:3]))
                worst[other] = max(worst.get(other, 0.0), f)

        if record_path and s % stride == 0:
            ee_path.append(data.xpos[ee_bid].copy())
            for oid, b in bids.items():
                obj_paths[oid].append(data.xpos[b].copy())
    sim_s = time.perf_counter() - t0

    reasons, culprit, max_disp = [], None, 0.0
    for o in layout.objects:
        if not o.movable or o.oid in permitted:
            continue
        d_ = float(np.linalg.norm(data.xpos[bids[o.oid]] - start[o.oid]))
        max_disp = max(max_disp, d_)
        if d_ > DISP_M:
            reasons.append(f"displaced:{o.oid}:{d_*100:.1f}cm")
            culprit = culprit or o.oid

    worst_force = 0.0
    for other, f in worst.items():
        if other in ("table", "floor", "world"):
            if f > F_STRUCTURE:
                reasons.append(f"struck_structure:{other}:{f:.0f}N")
                culprit = culprit or other
        elif other == "arm":
            continue                     # arm <-> carried target: the grasp itself
        elif other in permitted:
            if f > F_CRUSH:
                reasons.append(f"crushed:{other}:{f:.0f}N")
                culprit = culprit or other
        else:
            worst_force = max(worst_force, f)
            if f > F_CONTACT:
                reasons.append(f"struck:{other}:{f:.0f}N")
                culprit = culprit or other
    if self_hit > F_STRUCTURE:
        reasons.append(f"self_collision:{self_hit:.0f}N")
        culprit = culprit or "arm"

    return Outcome(
        mid=motion.mid, unsafe=bool(reasons), reasons=tuple(reasons),
        max_disp=max_disp, worst_force=worst_force, culprit=culprit,
        sim_seconds=sim_s,
        ee_path=np.array(ee_path) if ee_path else None,
        obj_paths={k: np.array(v) for k, v in obj_paths.items()} if ee_path else {})
