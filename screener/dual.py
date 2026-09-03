"""The two-arm station: Regime A.

The structural claim this module exists to test. On a bimanual station,
collisions split into two populations with different *sources of information*:

  Regime A -- arm against arm. Both link geometries come from the model we ship;
    both trajectories are ones we commanded. There is no camera anywhere in this
    path, so there is no calibration drift, no occlusion and no phantom. It can
    be screened tightly and it does not degrade under any scene error.

  Regime B -- arm against the objects. Every uncertainty lives here.

Charging both the same margin is the mistake. This module screens A; screener.fast
screens B, unchanged, once per arm.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

from .fast import ArmProxy, ScreenConfig, sample_joint_path
from .model import ARMS, HOME_QPOS
from .motions import (Intent, Motion, N_ARM, _arm_qpos_adr, arm_ctrl_ids,
                      reference_waypoints)
from .scene import Layout
from .truth import DISP_M, F_CONTACT, F_CRUSH, F_STRUCTURE, Outcome, _ctrl_at

ARM_BODY_NAMES = {"link0", "link1", "link2", "link3", "link4", "link5", "link6",
                  "link7", "hand", "left_finger", "right_finger"}
F_ARM_ARM = 8.0      # two arms touching at all is a fault; 8N filters numerical grazes


@dataclass(frozen=True)
class DualMotion:
    """One commanded step of the station: both arms move at once."""
    mid: str
    layout_id: str
    per_arm: tuple          # tuple[Motion, ...] indexed like model.ARMS
    duration: float
    tag: str
    gen: str = "A2"


def _owner_map_dual(model, layout: Layout):
    out = {}
    oids = {o.oid for o in layout.objects}
    prefixes = [p for p, _ in ARMS]
    for g in range(model.ngeom):
        b = model.geom_bodyid[g]
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or "world"
        owner = name
        for pre in prefixes:
            if name.startswith(pre) and name[len(pre):] in ARM_BODY_NAMES:
                owner = "arm:" + pre
                break
        else:
            if name not in oids and name != "table":
                owner = name
        out[g] = owner
    return out


# --------------------------------------------------------------------------
# generation
# --------------------------------------------------------------------------

def generate_dual_A(model, data, layout: Layout, rng, n: int) -> list[DualMotion]:
    """Both arms served from the same table and the same bin.

    `converge` is the case that matters: both arms routed to the shared bin at
    the same time, which is how two independently-safe motions become one
    collision. A generator that only produced independent tasks would make
    Regime A look trivially safe and prove nothing.
    """
    cubes, bins_ = layout.cubes, layout.bins
    out = []
    for i in range(n):
        tag = rng.choice(["independent", "converge", "jitter", "transit_low"],
                         p=[0.30, 0.30, 0.25, 0.15])
        # By default each arm works its own half of the pick area. Sending both
        # into the middle every time made 81% of motions collide, which measures
        # the station geometry, not the screener. `converge` is the deliberate
        # contention case: the two cubes closest to the centre line.
        ys = np.array([c.center[1] for c in cubes])
        side_a = np.flatnonzero(ys < 0)
        side_b = np.flatnonzero(ys >= 0)
        if tag == "converge" or len(side_a) == 0 or len(side_b) == 0:
            order = np.argsort(np.abs(ys))
            picks = [int(order[0]), int(order[1])]
        else:
            picks = [int(rng.choice(side_a)), int(rng.choice(side_b))]

        per = []
        for k, (prefix, _) in enumerate(ARMS):
            cube = cubes[picks[k % len(picks)]]
            bin_ = bins_[k % len(bins_)]        # each arm delivers to its own side
            transit_h = 0.22
            approach = (0.0, 0.0)
            if tag == "transit_low":
                transit_h = float(rng.uniform(0.05, 0.14))
            ws, grip, _ = reference_waypoints(model, data, layout, cube, bin_,
                                              transit_h=transit_h, approach=approach,
                                              prefix=prefix)
            if tag == "jitter":
                ws = ws + rng.normal(0, float(rng.uniform(0.02, 0.14)), ws.shape)
            per.append(Motion(f"{layout.layout_id}-D{i:03d}{prefix}", layout.layout_id,
                              np.asarray(ws, float), np.asarray(grip, float), 4.0,
                              Intent(cube.oid, bin_.oid), "A2", tag))
        out.append(DualMotion(f"{layout.layout_id}-D{i:03d}", layout.layout_id,
                              tuple(per), 4.0, str(tag)))
    return out


# --------------------------------------------------------------------------
# truth
# --------------------------------------------------------------------------

def run_truth_dual(model, data, layout: Layout, dm: DualMotion,
                   contact_stride: int = 4) -> Outcome:
    """Execute both arms simultaneously and classify.

    Adds one reason the single-arm predicate could not express: `arm_collision`.
    """
    import time
    owner = _owner_map_dual(model, layout)
    prefixes = [p for p, _ in ARMS]
    adrs = {p: _arm_qpos_adr(model, p) for p in prefixes}
    ctrls = {p: arm_ctrl_ids(model, p) for p in prefixes}
    permitted = {p: {dm.per_arm[k].intent.target_oid,
                     dm.per_arm[k].intent.destination_oid}
                 for k, p in enumerate(prefixes)}

    mujoco.mj_resetData(model, data)
    for p in prefixes:
        data.qpos[adrs[p]] = HOME_QPOS
    mujoco.mj_forward(model, data)

    bids = {o.oid: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, o.oid)
            for o in layout.objects}
    start = {oid: data.xpos[b].copy() for oid, b in bids.items()}
    nsteps = int(dm.duration / model.opt.timestep)
    ft = np.zeros(6)
    worst, arm_arm = {}, 0.0

    t0 = time.perf_counter()
    for s in range(nsteps):
        for k, p in enumerate(prefixes):
            q, g = _ctrl_at(dm.per_arm[k], s * model.opt.timestep)
            data.ctrl[ctrls[p][:N_ARM]] = q
            data.ctrl[ctrls[p][N_ARM]] = g
        mujoco.mj_step(model, data)
        if s % contact_stride:
            continue
        for c in range(data.ncon):
            con = data.contact[c]
            a, b = owner[con.geom1], owner[con.geom2]
            if a == b:
                continue
            if a.startswith("arm:") and b.startswith("arm:"):
                mujoco.mj_contactForce(model, data, c, ft)
                arm_arm = max(arm_arm, float(np.linalg.norm(ft[:3])))
                continue
            if a.startswith("arm:"):
                actor, other = a, b
            elif b.startswith("arm:"):
                actor, other = b, a
            else:
                continue
            pre = actor.split(":")[1]
            mujoco.mj_contactForce(model, data, c, ft)
            f = float(np.linalg.norm(ft[:3]))
            key = (pre, other)
            worst[key] = max(worst.get(key, 0.0), f)
    sim_s = time.perf_counter() - t0

    reasons, culprit = [], None
    if arm_arm > F_ARM_ARM:
        reasons.append(f"arm_collision:{arm_arm:.0f}N")
        culprit = "arm"

    all_permitted = set().union(*permitted.values())
    max_disp = 0.0
    for o in layout.objects:
        if not o.movable or o.oid in all_permitted:
            continue
        d_ = float(np.linalg.norm(data.xpos[bids[o.oid]] - start[o.oid]))
        max_disp = max(max_disp, d_)
        if d_ > DISP_M:
            reasons.append(f"displaced:{o.oid}:{d_*100:.1f}cm")
            culprit = culprit or o.oid

    worst_force = 0.0
    for (pre, other), f in worst.items():
        if other in ("table", "floor", "world"):
            if f > F_STRUCTURE:
                reasons.append(f"struck_structure:{other}:{f:.0f}N")
                culprit = culprit or other
        elif other in permitted[pre]:
            if f > F_CRUSH:
                reasons.append(f"crushed:{other}:{f:.0f}N")
                culprit = culprit or other
        else:
            worst_force = max(worst_force, f)
            if f > F_CONTACT:
                reasons.append(f"struck:{other}:{f:.0f}N")
                culprit = culprit or other

    return Outcome(dm.mid, bool(reasons), tuple(reasons), max_disp, worst_force,
                   culprit, sim_s)


# --------------------------------------------------------------------------
# Regime A screening
# --------------------------------------------------------------------------

def screen_regime_a(motions, proxies, cfg: ScreenConfig = ScreenConfig(),
                    margin: float = 0.02):
    """Arm-vs-arm, from commanded trajectories alone. Takes no SceneEstimate --
    that absence is the claim, and it is enforced by the signature."""
    B, N = len(motions), cfg.n_poses
    cents = []
    for k, px in enumerate(proxies):
        Q = np.stack([sample_joint_path(m.per_arm[k], N) for m in motions])
        cents.append(px.fk_spheres(Q.reshape(B * N, 7)).reshape(B, N, px.n_spheres, 3))
    A, Bc = cents[0], cents[1]
    ra, rb = proxies[0].radii, proxies[1].radii
    # |a-b|^2 without materialising the (B,N,Sa,Sb,3) difference tensor
    d2 = ((A ** 2).sum(-1)[..., :, None] + (Bc ** 2).sum(-1)[..., None, :]
          - 2 * np.einsum("bnid,bnjd->bnij", A, Bc))
    gap = np.sqrt(np.maximum(d2, 0.0)) - ra[None, None, :, None] - rb[None, None, None, :]
    worst = gap.min(axis=(1, 2, 3))
    return worst >= margin, worst


# --------------------------------------------------------------------------
# evaluation
# --------------------------------------------------------------------------

def build_dual_dataset(n_layouts=8, per_layout=24, seed0=0, verbose=True):
    from .model import build_dual
    from .scene import dual_layout
    layouts, motions, outcomes = {}, [], {}
    sim_total = 0.0
    for s in range(seed0, seed0 + n_layouts):
        lay = dual_layout(s)
        model, data = build_dual(lay)
        layouts[s] = lay
        for dm in generate_dual_A(model, data, lay, np.random.default_rng(2000 + s),
                                  per_layout):
            o = run_truth_dual(model, data, lay, dm)
            sim_total += o.sim_seconds
            motions.append((s, dm))
            outcomes[dm.mid] = o
    if verbose:
        n = len(motions)
        print(f"  dual dataset: {n} station-steps over {n_layouts} layouts, "
              f"{np.mean([outcomes[m.mid].unsafe for _, m in motions]):.1%} unsafe, "
              f"{sim_total/n*1000:.0f} ms/step ({n/sim_total:.1f}/s/core)")
    return dict(layouts=layouts, motions=motions, outcomes=outcomes,
                sim_rate=len(motions) / sim_total)


def is_arm_collision(outcome) -> bool:
    return any(r.startswith("arm_collision") for r in outcome.reasons)


def evaluate_dual(ds, cfg: ScreenConfig = ScreenConfig(), err=None,
                  margin_a: float = 0.02, n_hypotheses: int = 1, rng_seed=7):
    """Score the two regimes separately and together.

    Regime A is scored against the arm-collision label only, and it is given no
    SceneEstimate at all. Regime B is scored against object collisions, per arm,
    from the estimate. The station's verdict is the conjunction.
    """
    import time
    from .estimate import ErrorModel, observe
    from .fast import screen
    from .model import build_dual
    err = err or ErrorModel()
    prefixes = [p for p, _ in ARMS]

    any_layout = next(iter(ds["layouts"]))
    model, _ = build_dual(ds["layouts"][any_layout])
    proxies = [ArmProxy(model, prefix=p) for p in prefixes]

    by = {}
    for s, dm in ds["motions"]:
        by.setdefault(s, []).append(dm)

    rec_a, rec_b, rec_all, mids = [], [], [], []
    truth_a, truth_obj, truth_all = [], [], []
    t_a = t_b = 0.0
    for s, dms in by.items():
        lay = ds["layouts"][s]
        t0 = time.perf_counter()
        allow_a, _ = screen_regime_a(dms, proxies, cfg, margin_a)
        t_a += time.perf_counter() - t0

        ok_b = np.ones(len(dms), dtype=bool)
        for h in range(n_hypotheses):
            est = observe(lay, err, np.random.default_rng(rng_seed + 991 * s + h))
            for k in range(len(prefixes)):
                subs = [dm.per_arm[k] for dm in dms]
                t0 = time.perf_counter()
                vs = screen(subs, est, proxies[k], cfg)
                t_b += time.perf_counter() - t0
                ok_b &= np.array([v.allow for v in vs])

        for i, dm in enumerate(dms):
            o = ds["outcomes"][dm.mid]
            arm = is_arm_collision(o)
            obj = o.unsafe and not arm
            rec_a.append(bool(allow_a[i])); truth_a.append(arm)
            rec_b.append(bool(ok_b[i])); truth_obj.append(obj)
            rec_all.append(bool(allow_a[i] and ok_b[i])); truth_all.append(o.unsafe)
            mids.append(dm.mid)

    def score(allow, unsafe, seconds, n_calls):
        allow, unsafe = np.array(allow), np.array(unsafe)
        ns, nu = int((~unsafe).sum()), int(unsafe.sum())
        return dict(n=len(allow), unsafe=nu,
                    keep_rate=int((allow & ~unsafe).sum()) / max(ns, 1),
                    false_accept=int((allow & unsafe).sum()),
                    fa_rate_of_unsafe=int((allow & unsafe).sum()) / max(nu, 1),
                    false_reject=int((~allow & ~unsafe).sum()),
                    verdicts_per_s=n_calls / max(seconds, 1e-9))

    n = len(rec_a)
    return dict(regime_a=score(rec_a, truth_a, t_a, n),
                regime_b=score(rec_b, truth_obj, t_b, n * len(prefixes) * n_hypotheses),
                combined=score(rec_all, truth_all, t_a + t_b, n),
                mids=mids, allow_a=rec_a, allow_b=rec_b,
                truth_a=truth_a, truth_obj=truth_obj)


def evaluate_dual_los(ds, cfg: ScreenConfig = ScreenConfig(), margin_a: float = 0.02,
                      occluder: str = "other", rng_seed=7):
    """Regime B under LINE-OF-SIGHT occlusion, with the other arm as the occluder.

    The picture of the table is taken while the station is running, so the arm
    that is not being screened is in shot. Its mid-trajectory pose is used as the
    occluder, which is the honest version of "an arm is over the table".

    If `uniform_rate` is given, a matched uniform-dropout condition is scored
    alongside at the same average dropout rate. Same rate, different identities:
    that comparison is the point.
    """
    import time
    from .estimate import ErrorModel, observe, visibility
    from .fast import screen, sample_joint_path
    from .model import build_dual

    prefixes = [p for p, _ in ARMS]
    model, _ = build_dual(ds["layouts"][next(iter(ds["layouts"]))])
    proxies = [ArmProxy(model, prefix=p) for p in prefixes]
    by = {}
    for s, dm in ds["motions"]:
        by.setdefault(s, []).append(dm)

    rows = []
    n_missing = []
    for s, dms in by.items():
        lay = ds["layouts"][s]
        for dm in dms:
            allow, missing_all = True, set()
            for k in range(len(prefixes)):
                # "other": a fresh picture, taken while the OTHER arm is working.
                # "self": a STALE picture, taken while this arm was last over its
                # own work area -- the case in which occlusion and collision
                # share a cause.
                occ = (1 - k) if occluder == "other" else k
                mid = sample_joint_path(dm.per_arm[occ], cfg.n_poses)[cfg.n_poses // 2]
                C = proxies[occ].fk_spheres(mid[None, :])[0]
                err = ErrorModel(line_of_sight=True, arm_centers=C,
                                 arm_radii=proxies[occ].radii, name="los")
                est = observe(lay, err)
                seen = {o.oid for o in est.objects}
                missing_all |= {o.oid for o in lay.objects} - seen
                allow &= screen([dm.per_arm[k]], est, proxies[k], cfg)[0].allow
            n_missing.append(len(missing_all))
            o = ds["outcomes"][dm.mid]
            rows.append(dict(mid=dm.mid, allow=bool(allow), unsafe=o.unsafe,
                             arm=is_arm_collision(o), culprit=o.culprit,
                             missing=missing_all,
                             culprit_hidden=o.culprit in missing_all))
    return rows, float(np.mean(n_missing))
