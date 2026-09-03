"""The fast path: swept-sphere screening, no physics.

Budget, from DESIGN.md: 100 candidates x ~4 scene hypotheses inside a 100 ms
station budget, i.e. ~70 us per verdict, ~1000x cheaper than simulation.

Two things follow from that budget and shape this file.

1. It implements its own forward kinematics in numpy. Calling mj_kinematics once
   per sampled pose is ~10 us; at 24 poses that is 240 us per candidate, already
   3x over budget before a single collision test. So FK is batched over every
   candidate and every pose at once.
2. The arm is approximated by spheres fitted to its collision meshes, because
   sphere-vs-box distance is exact, branch-free and broadcastable. The fit is an
   OVER-approximation: the proxy is never smaller than the real link, so proxy
   error costs keep-rate, not safety. That direction is deliberate.
"""
from __future__ import annotations

from dataclasses import dataclass, replace

import numpy as np

from .estimate import SceneEstimate, TABLE_BOX
from .geom import sphere_box_gap

CHAIN = ["link0", "link1", "link2", "link3", "link4", "link5", "link6",
         "link7", "hand", "left_finger", "right_finger"]
# link0/link1 are bolted to the table and are permanently within any sane margin
# of it; excluding them from the proxy is standard and is recorded as a blind spot.
PROXY_SKIP = {"link0", "link1"}
FINGER_BODIES = {"left_finger", "right_finger"}
HAND_BODIES = {"hand"}
# Three proxy groups, because they have three different licences to be near
# things. Fingers must reach the table surface to pick a cube off it -- they
# straddle the object, so a few cm of proxy overlap with the table plane is
# CORRECT, not slop. The hand behind them has no such need, and the rest of the
# arm has none at all. Collapsing these into one "tool" group is what let the
# gripper slam the table without being rejected.
G_BODY, G_HAND, G_FINGER = 0, 1, 2
FINGER_OPEN = 0.04


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _geom_points(model, gid):
    """Collision points of one geom, in its parent body's frame."""
    import mujoco
    t = model.geom_type[gid]
    size = model.geom_size[gid]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[gid]
        a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        v = model.mesh_vert[a:a + n]
        if n > 300:
            v = v[:: max(1, n // 300)]
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        s = size[:3]
        v = np.array([[sx, sy, sz] for sx in (-s[0], s[0])
                      for sy in (-s[1], s[1]) for sz in (-s[2], s[2])])
    elif t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        r, h = size[0], size[1]
        v = np.array([[0, 0, -h - r], [0, 0, h + r], [r, 0, 0], [-r, 0, 0],
                      [0, r, 0], [0, -r, 0]])
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        r = size[0]
        v = np.array([[r, 0, 0], [-r, 0, 0], [0, r, 0], [0, -r, 0], [0, 0, r], [0, 0, -r]])
    else:
        return np.zeros((0, 3))
    R = _quat_to_mat(model.geom_quat[gid])
    return v @ R.T + model.geom_pos[gid]


def _fit_spheres(pts, max_k=8):
    """Cover a point cloud with spheres, by Lloyd clustering along its long axis.

    Two earlier attempts were too loose to be usable. Fitting along the body-frame
    AABB gave 15-18 cm radii (the Panda's link frames are not aligned with the
    limb). Fitting evenly along the principal axis was better but still ~9-10 cm,
    because an evenly-spaced slab must cover the widest part of the link
    everywhere. Clustering lets spheres hug the shape.

    This matters more than it sounds: clearance the screener reports is real
    clearance MINUS proxy error, so a loose proxy spends the entire margin budget
    on its own approximation and the screener ends up rejecting good motions to
    protect itself from its own geometry.

    Each radius is the exact max distance from its centre to the points assigned
    to it, so the union still strictly contains the arm -- conservative, but only
    by as much as the shape demands.
    """
    c0 = pts.mean(axis=0)
    X = pts - c0
    _, _, Vt = np.linalg.svd(X, full_matrices=False)
    axis = Vt[0]
    t = X @ axis
    L = float(t.max() - t.min())
    cross = float(np.linalg.norm(X - np.outer(t, axis), axis=1).max())
    k = int(np.clip(int(np.ceil(2.0 * L / max(cross, 1e-3))), 1, max_k))
    if k == 1:
        return c0[None, :], np.array([float(np.linalg.norm(X, axis=1).max())])
    q = np.linspace(t.min(), t.max(), k)
    centers = c0 + np.outer(q, axis)
    for _ in range(15):
        assign = np.argmin(((pts[:, None, :] - centers[None]) ** 2).sum(-1), axis=1)
        for i in range(k):
            sel = assign == i
            if sel.any():
                centers[i] = pts[sel].mean(axis=0)
    assign = np.argmin(((pts[:, None, :] - centers[None]) ** 2).sum(-1), axis=1)
    keep, radii = [], []
    for i in range(k):
        sel = assign == i
        if not sel.any():
            continue
        keep.append(centers[i])
        radii.append(float(np.linalg.norm(pts[sel] - centers[i], axis=1).max()))
    return np.array(keep), np.array(radii)


class ArmProxy:
    """Batched FK plus a sphere approximation of the arm, both built once."""

    def __init__(self, model, max_k=10):
        import mujoco
        self.links = []
        for name in CHAIN:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            jadr, jnum = model.body_jntadr[bid], model.body_jntnum[bid]
            joint = None
            if jnum > 0:
                jt = model.jnt_type[jadr]
                joint = dict(kind=int(jt), axis=model.jnt_axis[jadr].copy(),
                             anchor=model.jnt_pos[jadr].copy(),
                             slide=FINGER_OPEN * (1 if "left" in name else -1))
            pts = []
            for g in range(model.body_geomadr[bid],
                           model.body_geomadr[bid] + model.body_geomnum[bid]):
                if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
                    continue
                p = _geom_points(model, g)
                if len(p):
                    pts.append(p)
            if pts and name not in PROXY_SKIP:
                c, r = _fit_spheres(np.vstack(pts), max_k)
            else:
                c, r = np.zeros((0, 3)), np.zeros(0)
            self.links.append(dict(name=name, pos=model.body_pos[bid].copy(),
                                   R=_quat_to_mat(model.body_quat[bid]),
                                   joint=joint, sc=c, sr=r))
        # Drop spheres wholly contained in another sphere of the same link.
        # Lloyd clustering leaves a few degenerate 2-3 mm spheres sitting inside
        # their neighbours. They add nothing to coverage, and they wreck the
        # sampling-density criterion, which is scaled by sphere radius: half of
        # 2 mm is a step limit no real motion can satisfy, so every motion
        # demanded the maximum pose count for a sphere that was already covered.
        for l in self.links:
            c, r = l["sc"], l["sr"]
            if len(r) < 2:
                continue
            d = np.linalg.norm(c[:, None, :] - c[None, :, :], axis=-1)
            contained = (d + r[:, None] <= r[None, :] + 1e-9)
            np.fill_diagonal(contained, False)
            keep = ~contained.any(axis=1)
            l["sc"], l["sr"] = c[keep], r[keep]
        self.radii = np.concatenate([l["sr"] for l in self.links])
        # Intended contact is localised to the TOOL. The gripper has to reach the
        # table surface to pick anything up; the elbow does not, and letting the
        # whole arm share the gripper's licence to approach surfaces is what made
        # the first version reject 91% of good motions.
        def _grp(name):
            return G_FINGER if name in FINGER_BODIES else (
                G_HAND if name in HAND_BODIES else G_BODY)
        self.group = np.concatenate([
            np.full(len(l["sr"]), _grp(l["name"])) for l in self.links]).astype(int)
        self.n_spheres = len(self.radii)

    def fk_spheres(self, Q: np.ndarray) -> np.ndarray:
        """Q (M,7) joint vectors -> sphere centres (M, S, 3), world frame."""
        import mujoco
        M = Q.shape[0]
        R = np.repeat(np.eye(3)[None], M, axis=0)
        p = np.zeros((M, 3))
        out, ji = [], 0
        for l in self.links:
            p = p + np.einsum("mij,j->mi", R, l["pos"])
            R = R @ l["R"]
            j = l["joint"]
            if j is not None:
                if j["kind"] == int(mujoco.mjtJoint.mjJNT_HINGE):
                    a = j["axis"] / np.linalg.norm(j["axis"])
                    th = Q[:, ji]; ji += 1
                    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
                    Rj = (np.eye(3)[None] + np.sin(th)[:, None, None] * K[None]
                          + (1 - np.cos(th))[:, None, None] * (K @ K)[None])
                    anc = j["anchor"]
                    p = p + np.einsum("mij,mj->mi", R, anc[None] - np.einsum("mij,j->mi", Rj, anc))
                    R = R @ Rj
                else:                                    # slide: fingers held open
                    p = p + np.einsum("mij,j->mi", R, j["axis"] * j["slide"])
            if len(l["sc"]):
                out.append(p[:, None, :] + np.einsum("mij,kj->mki", R, l["sc"]))
        return np.concatenate(out, axis=1)


# --------------------------------------------------------------------------

@dataclass
class ScreenConfig:
    margin: float = 0.010          # clearance demanded of forbidden objects (m)
    # per proxy group: (body, hand, finger)
    table_margin: tuple = (0.006, -0.004, -0.040)
    # The TARGET is grasped: fingers close around a 44 mm cube, so deep proxy
    # overlap with it is correct. The DESTINATION is only entered: the gripper
    # passes through the bin's open mouth, which is free space, and never needs
    # to overlap a wall. Giving both the same licence is what produced the
    # single largest class of false accepts -- the arm driving through the bin
    # it was delivering to, at 100-270 N, and being permitted because the bin
    # was on its allowed list.
    target_margin: tuple = (0.000, -0.006, -0.035)
    destination_margin: tuple = (0.002, -0.004, -0.020)
    unknown_margin: float = 0.0    # clearance demanded of un-observed volume
    n_poses: int = 48   # measured: false accepts converge by 48; 384 buys nothing
    offset_inflation: float = 0.0  # deterministic cover for systematic drift


@dataclass
class Verdict:
    mid: str
    allow: bool
    min_gap: float
    culprit: str | None


def sample_joint_path(motion, n: int) -> np.ndarray:
    """Same smoothstep interpolation the executor uses, sampled n times.
    If these two ever disagree, the screener is screening a motion the robot
    will not perform -- so they are deliberately the same formula."""
    W = motion.waypoints
    K = len(W)
    u = np.linspace(0.0, 1.0, n) * (K - 1)
    i = np.clip(np.floor(u).astype(int), 0, K - 2)
    f = u - i
    f = f * f * (3 - 2 * f)
    return W[i] * (1 - f)[:, None] + W[i + 1] * f[:, None]


def _parts(est: SceneEstimate, cfg: ScreenConfig):
    lo, hi, owner = [], [], []
    for o in est.objects:
        lo.append(o.lo); hi.append(o.hi)
        owner += [o.oid] * len(o.lo)
    lo.append(TABLE_BOX[0][None, :]); hi.append(TABLE_BOX[1][None, :]); owner.append("table")
    kinds = ["obj"] * (len(owner) - 1) + ["table"]
    if cfg.unknown_margin > 0 and est.unknown_lo is not None and len(est.unknown_lo):
        lo.append(est.unknown_lo); hi.append(est.unknown_hi)
        owner += ["unknown"] * len(est.unknown_lo)
        kinds += ["unknown"] * len(est.unknown_lo)
    return np.vstack(lo), np.vstack(hi), np.array(owner), np.array(kinds)


def required_poses(proxy: ArmProxy, motions, n0: int = 24,
                   coverage: float = 0.5, cap: int = 256) -> np.ndarray:
    """How densely each motion must be sampled for the spheres to actually cover
    the swept volume.

    Point-sampling a trajectory at a fixed 24 poses is not a swept-volume test.
    Between two consecutive samples the arm moves, and a thin object sitting in
    that gap is never tested against anything -- the motion tunnels through it.
    A fixed pose count hides this because the error depends on how fast the
    motion is, not on how long it is.

    The fix: require that no sphere centre travels more than `coverage` times its
    own radius between samples. Then consecutive spheres overlap and their union
    contains the sweep. Cost scales with the speed of the individual motion, so
    a slow pick-and-place still costs 24 poses and only a violent one pays more.
    """
    B = len(motions)
    Q = np.stack([sample_joint_path(m, n0) for m in motions])
    C = proxy.fk_spheres(Q.reshape(B * n0, 7)).reshape(B, n0, proxy.n_spheres, 3)
    step = np.linalg.norm(np.diff(C, axis=1), axis=-1)        # (B, n0-1, S)
    allow = coverage * proxy.radii[None, None, :]
    ratio = (step / np.maximum(allow, 1e-6)).max(axis=(1, 2))  # per motion
    need = np.ceil((n0 - 1) * ratio).astype(int) + 1
    return np.clip(need, n0, cap)


def screen(motions, est: SceneEstimate, proxy: ArmProxy,
           cfg: ScreenConfig = ScreenConfig()) -> list[Verdict]:
    """Verdicts for a batch of motions against ONE scene estimate.

    Reads: motion waypoints, motion intent, and `est`. Nothing else. There is no
    argument through which truth could arrive.
    """
    B, N = len(motions), cfg.n_poses
    Q = np.stack([sample_joint_path(m, N) for m in motions])        # (B,N,7)
    C = proxy.fk_spheres(Q.reshape(B * N, 7)).reshape(B, N * proxy.n_spheres, 3)
    radii = np.tile(proxy.radii, N)[None, :].repeat(B, axis=0)

    lo, hi, owner, kinds = _parts(est, cfg)
    m = cfg.offset_inflation
    lo, hi = lo - m, hi + m

    G = 3
    margins = np.empty((G, B, len(owner)))
    for g in range(G):
        base = np.where(kinds == "table", cfg.table_margin[g],
                        np.where(kinds == "unknown", cfg.unknown_margin, cfg.margin))
        for b, mo in enumerate(motions):
            row = base.copy()
            row[owner == mo.intent.target_oid] = cfg.target_margin[g]
            row[owner == mo.intent.destination_oid] = cfg.destination_margin[g]
            margins[g, b] = row

    grp = np.tile(proxy.group, N)
    gap = sphere_box_gap(C, radii, lo[None], hi[None])              # (B, K, P)
    slack = np.full((B, len(owner)), np.inf)
    worst_per_part = np.full((B, len(owner)), np.inf)
    for g in range(G):
        sel = grp == g
        if not sel.any():
            continue
        gg = gap[:, sel, :].min(axis=1)                             # (B, P)
        slack = np.minimum(slack, gg - margins[g])
        worst_per_part = np.minimum(worst_per_part, gg)
    allow = slack.min(axis=1) >= 0
    worst_idx = slack.argmin(axis=1)
    return [Verdict(mo.mid, bool(allow[b]), float(worst_per_part[b, worst_idx[b]]),
                    None if allow[b] else str(owner[worst_idx[b]]))
            for b, mo in enumerate(motions)]
