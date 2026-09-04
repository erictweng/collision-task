"""Two-sided certificates for Regime A: arm against arm.

`dual.screen_regime_a` answers "is the minimum gap at least `margin`". That is an
estimate with a tuned parameter. This module replaces it with two tests that are
implications rather than estimates:

    inner cores overlap   -> the arms overlap        -> COLLIDES, proven
    outer bubbles clear   -> the arms are apart      -> SAFE, proven
    neither               -> the truth is in the shell between them -> UNKNOWN

Only UNKNOWN reaches the simulator, so the number that decides whether this is
worth having is the *resolve rate*: the fraction of candidates that never need
physics. There is no margin to tune and no accuracy to trade -- what the design
spends instead is coverage.

Soundness of each side, stated exactly, because this is the whole claim:

  COLLIDES needs nothing extra. The cores are inside the links by construction,
    so an overlap at a sampled instant is a real overlap at a real instant.
    Sampling can only make this test miss, never lie.

  SAFE does not hold at sampled poses alone -- two arms can pass through each
    other between two samples and both samples look clear. Each sphere is
    therefore replaced by the ball that contains its swept segment: centred at
    the midpoint of consecutive centres, radius r + |step|/2. That ball contains
    the straight-line sweep exactly.

  The residual assumption, named rather than buried: the true path between two
    samples is an arc, not the chord, and the arc bulges outside the segment by
    the sagitta. `sagitta_bound` measures that bulge against an 8x-finer path and
    adds it, so the inflation covers the real trajectory rather than the
    piecewise-linear one. It is measured, not assumed, but it is measured on
    these motions -- a generator with far more aggressive curvature would need it
    remeasured.

Regime A is the cleanest case in the whole system for this: two arms should never
touch, so there is no intended contact, no target, no destination, no per-body
licence and no exclusion list. Nothing here is fitted to anything.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .fast import CHAIN, PROXY_SKIP, ArmProxy, _quat_to_mat, sample_joint_path

SAFE, COLLIDES, UNKNOWN = 0, 1, 2
VERDICT_NAMES = {SAFE: "SAFE", COLLIDES: "COLLIDES", UNKNOWN: "UNKNOWN"}

_BOX_TRIS = np.array([
    [0, 1, 3], [0, 3, 2], [4, 7, 5], [4, 6, 7], [0, 4, 5], [0, 5, 1],
    [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])


def _geom_triangles(model, gid) -> np.ndarray:
    """Surface triangles of one collision geom, in its parent body's frame.

    Unlike fast._geom_points this must not subsample: a missing triangle would
    let an inner core poke through a hole in the surface, and the core's whole
    value is that it is provably inside.
    """
    t = model.geom_type[gid]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = model.geom_dataid[gid]
        va, vn = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        fa, fn = model.mesh_faceadr[mid], model.mesh_facenum[mid]
        verts = model.mesh_vert[va:va + vn]
        faces = model.mesh_face[fa:fa + fn]
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        s = model.geom_size[gid][:3]
        verts = np.array([[sx, sy, sz] for sx in (-s[0], s[0])
                          for sy in (-s[1], s[1]) for sz in (-s[2], s[2])])
        faces = _BOX_TRIS
    else:
        return np.zeros((0, 3, 3))
    R = _quat_to_mat(model.geom_quat[gid])
    verts = verts @ R.T + model.geom_pos[gid]
    return verts[faces]


def _link_triangles(model, bid) -> np.ndarray:
    tris = [_geom_triangles(model, g)
            for g in range(model.body_geomadr[bid],
                           model.body_geomadr[bid] + model.body_geomnum[bid])
            if not (model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0)]
    tris = [t for t in tris if len(t)]
    return np.concatenate(tris, axis=0) if tris else np.zeros((0, 3, 3))


def _segment_distance(p, P0, P1):
    d = P1 - P0
    dd = (d * d).sum(-1)
    t = np.clip(((p - P0) * d).sum(-1) / np.maximum(dd, 1e-18), 0.0, 1.0)
    return np.linalg.norm(p - (P0 + t[:, None] * d), axis=-1)


def _distance_to_surface(p: np.ndarray, tris: np.ndarray) -> float:
    """Exact distance from a point to a triangle soup.

    Closest point on a triangle is either the barycentric projection when it
    lands inside, or a point on one of the three edges. Taking the minimum over
    both covers every case, including the degenerate slivers a mesh always has.
    """
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    ab, ac, ap = B - A, C - A, p - A
    d00 = (ab * ab).sum(-1)
    d01 = (ab * ac).sum(-1)
    d11 = (ac * ac).sum(-1)
    d20 = (ap * ab).sum(-1)
    d21 = (ap * ac).sum(-1)
    denom = d00 * d11 - d01 * d01
    ok = np.abs(denom) > 1e-18
    safe_denom = np.where(ok, denom, 1.0)
    v = (d11 * d20 - d01 * d21) / safe_denom
    w = (d00 * d21 - d01 * d20) / safe_denom
    inside = ok & (v >= 0) & (w >= 0) & (v + w <= 1)
    proj = A + v[:, None] * ab + w[:, None] * ac
    d_face = np.where(inside, np.linalg.norm(p - proj, axis=-1), np.inf)
    d_edge = np.minimum(np.minimum(_segment_distance(p, A, B),
                                   _segment_distance(p, A, C)),
                        _segment_distance(p, B, C))
    return float(np.minimum(d_face, d_edge).min())


def _is_inside(p: np.ndarray, tris: np.ndarray) -> bool:
    """Ray-parity containment test, majority vote over three directions.

    A single ray is fragile: it can graze an edge or run along a coplanar face
    and miscount. Three unrelated directions voting removes the realistic cases
    without pretending the test is exact.
    """
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    e1, e2, s = B - A, C - A, p - A
    votes = 0
    for d in (np.array([1.0, 0.017, 0.0031]), np.array([0.011, 1.0, 0.0043]),
              np.array([0.0071, 0.013, 1.0])):
        d = d / np.linalg.norm(d)
        h = np.cross(d, e2)
        a = (e1 * h).sum(-1)
        par = np.abs(a) < 1e-14
        f = 1.0 / np.where(par, 1.0, a)
        u = f * (s * h).sum(-1)
        q = np.cross(s, e1)
        vv = f * (q @ d)
        t = f * (e2 * q).sum(-1)
        hit = (~par) & (u >= 0) & (u <= 1) & (vv >= 0) & (u + vv <= 1) & (t > 1e-9)
        votes += int(hit.sum()) % 2
    return votes >= 2


def _push_to_medial_axis(c, tris, iters=60, step0=0.010):
    """Walk a point away from the surface it is closest to.

    The Lloyd centres are cluster centroids, which sit wherever the vertices
    happen to average out -- often close to one wall of the link. The inscribed
    radius there is small, and since the UNKNOWN band is exactly the gap between
    the inner and outer radii, a badly placed core costs coverage directly.
    Stepping away from the nearest surface point is crude gradient ascent on
    distance-to-surface, and it converges toward the medial axis, which is where
    the largest contained ball lives.
    """
    best_c, best_r = c.copy(), _distance_to_surface(c, tris)
    cur, step = c.copy(), step0
    for _ in range(iters):
        near = _nearest_surface_point(cur, tris)
        d = cur - near
        n = np.linalg.norm(d)
        if n < 1e-9:
            break
        cand = cur + step * d / n
        if _is_inside(cand, tris):
            r = _distance_to_surface(cand, tris)
            if r > best_r:
                best_c, best_r, cur = cand.copy(), r, cand
                continue
        step *= 0.7
        if step < 1e-4:
            break
    return best_c, best_r


def _nearest_surface_point(p, tris):
    A, B, C = tris[:, 0], tris[:, 1], tris[:, 2]
    cands = []
    for P0, P1 in ((A, B), (A, C), (B, C)):
        d = P1 - P0
        dd = (d * d).sum(-1)
        t = np.clip(((p - P0) * d).sum(-1) / np.maximum(dd, 1e-18), 0.0, 1.0)
        cands.append(P0 + t[:, None] * d)
    ab, ac, ap = B - A, C - A, p - A
    d00, d01, d11 = (ab*ab).sum(-1), (ab*ac).sum(-1), (ac*ac).sum(-1)
    d20, d21 = (ap*ab).sum(-1), (ap*ac).sum(-1)
    den = d00 * d11 - d01 * d01
    ok = np.abs(den) > 1e-18
    sd = np.where(ok, den, 1.0)
    v = (d11 * d20 - d01 * d21) / sd
    w = (d00 * d21 - d01 * d20) / sd
    proj = A + v[:, None] * ab + w[:, None] * ac
    inside = ok & (v >= 0) & (w >= 0) & (v + w <= 1)
    cands.append(np.where(inside[:, None], proj, cands[0]))
    allc = np.concatenate(cands, axis=0)
    return allc[np.argmin(np.linalg.norm(p - allc, axis=-1))]


def inner_proxy(model, proxy: ArmProxy, prefix: str = "", refine: bool = True):
    """An independent sphere set that lies INSIDE the arm.

    Separate from the outer proxy on purpose: the two answer opposite questions,
    so nothing requires them to share centres, counts or placement. Spheres whose
    centre is not inside its own link -- possible where a link has a concave
    waist -- are dropped rather than kept at radius zero; they can only add work.
    """
    import copy
    inner = copy.deepcopy(proxy)
    kept_r = []
    for link in inner.links:
        if not len(link["sr"]):
            continue
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix + link["name"])
        tris = _link_triangles(model, bid)
        cs, rs = [], []
        for c in link["sc"]:
            if len(tris) == 0:
                continue
            if refine and _is_inside(c, tris):
                c, r = _push_to_medial_axis(c, tris)
            elif _is_inside(c, tris):
                r = _distance_to_surface(c, tris)
            else:
                continue
            if r > 1e-4:
                cs.append(c); rs.append(r)
        link["sc"] = np.array(cs).reshape(-1, 3)
        link["sr"] = np.array(rs)
        kept_r.append(link["sr"])
    inner.radii = np.concatenate(kept_r) if kept_r else np.zeros(0)
    inner.group = np.zeros(len(inner.radii), dtype=int)
    inner.n_spheres = len(inner.radii)
    return inner


def inner_radii(model, proxy: ArmProxy, prefix: str = "") -> np.ndarray:
    """Largest ball at each proxy centre that lies entirely inside its link.

    A centre that falls outside its own surface -- which the Lloyd fit permits on
    a link with a concave waist -- gets radius zero. Such a sphere contributes
    nothing to the COLLIDES side and cannot make it unsound.
    """
    out = []
    for link in proxy.links:
        if not len(link["sr"]):
            continue
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix + link["name"])
        tris = _link_triangles(model, bid)
        for c in link["sc"]:
            if len(tris) == 0 or not _is_inside(c, tris):
                out.append(0.0)
            else:
                out.append(_distance_to_surface(c, tris))
    return np.array(out)


def sagitta_bound(proxy: ArmProxy, motions, arm_index: int, n_poses: int,
                  refine: int = 8) -> float:
    """How far the real path bulges outside the chords between samples.

    Measured, not assumed: resample each motion `refine` times denser and take
    the largest distance from a fine centre to the coarse segment it lies on.
    """
    worst = 0.0
    for m in motions:
        sub = m.per_arm[arm_index]
        coarse = proxy.fk_spheres(sample_joint_path(sub, n_poses))
        fine = proxy.fk_spheres(sample_joint_path(sub, (n_poses - 1) * refine + 1))
        for i in range(n_poses - 1):
            P0, P1 = coarse[i], coarse[i + 1]
            seg = P1 - P0
            dd = (seg * seg).sum(-1)
            for j in range(i * refine + 1, (i + 1) * refine):
                p = fine[j]
                t = np.clip(((p - P0) * seg).sum(-1) / np.maximum(dd, 1e-18), 0, 1)
                d = np.linalg.norm(p - (P0 + t[:, None] * seg), axis=-1).max()
                worst = max(worst, float(d))
    return worst


@dataclass
class CertifyConfig:
    n_poses: int = 48
    inflate_sweep: bool = True    # cover the motion between samples, not just the poses
    sagitta: float = 0.0          # measured arc bulge added to the sweep inflation


def _pair_min_gap(A, ra, B, rb):
    """Smallest surface gap between two sphere sets, per motion.

    Squared distances via the expansion rather than an explicit difference
    tensor: the (B, N, Sa, Sb, 3) intermediate is what makes the direct form run
    out of memory at realistic pose counts.
    """
    d2 = ((A ** 2).sum(-1)[..., :, None] + (B ** 2).sum(-1)[..., None, :]
          - 2 * np.einsum("bnid,bnjd->bnij", A, B))
    gap = np.sqrt(np.maximum(d2, 0.0)) - ra[None, None, :, None] - rb[None, None, None, :]
    return gap.min(axis=(1, 2, 3))


def certify_regime_a(motions, proxies, inner_proxies, cfg: CertifyConfig = CertifyConfig()):
    """Three-way verdict per station step, from commanded trajectories alone.

    Takes no SceneEstimate and no margin. The absence of both is the claim, and
    the signature is where it is enforced.
    """
    N = cfg.n_poses
    centres, outer = [], []
    for k, px in enumerate(proxies):
        Q = np.stack([sample_joint_path(m.per_arm[k], N) for m in motions])
        ipx = inner_proxies[k]
        centres.append(ipx.fk_spheres(Q.reshape(-1, 7))
                       .reshape(len(motions), N, ipx.n_spheres, 3))
        C = px.fk_spheres(Q.reshape(-1, 7)).reshape(len(motions), N, px.n_spheres, 3)
        if cfg.inflate_sweep:
            step = np.linalg.norm(np.diff(C, axis=1), axis=-1)          # (B,N-1,S)
            mid = 0.5 * (C[:, :-1] + C[:, 1:])
            outer.append((mid, px.radii[None, None, :] + step / 2 + cfg.sagitta))
        else:
            outer.append((C, np.broadcast_to(px.radii, C.shape[:-1]).copy()))

    # SAFE: swept balls of the two arms are disjoint everywhere.
    (Ma, Ra), (Mb, Rb) = outer
    d2 = ((Ma ** 2).sum(-1)[..., :, None] + (Mb ** 2).sum(-1)[..., None, :]
          - 2 * np.einsum("bnid,bnjd->bnij", Ma, Mb))
    gap_out = (np.sqrt(np.maximum(d2, 0.0))
               - Ra[..., :, None] - Rb[..., None, :]).min(axis=(1, 2, 3))

    # COLLIDES: inner cores overlap at some sampled pose.
    gap_in = _pair_min_gap(centres[0], inner_proxies[0].radii,
                           centres[1], inner_proxies[1].radii)

    verdict = np.full(len(motions), UNKNOWN, dtype=int)
    verdict[gap_out >= 0.0] = SAFE
    verdict[gap_in < 0.0] = COLLIDES     # a proof beats a proof-of-the-other-side
    return verdict, gap_out, gap_in
