"""What the screener is allowed to see.

A SceneEstimate holds boxes and ids. It holds no reference to a Layout, no
mujoco model and no outcome. That is the anti-leakage mechanism: the screener's
signature takes a SceneEstimate, and there is no path from one back to the truth
it was derived from.

The three error modes are modelled separately because they fail differently:

  systematic offset  every object translated by the SAME vector (a camera mount
                     shifted). Not averageable. Handled deterministically by
                     margin inflation -- "safe under any translation of norm <= m"
                     is exactly "safe against boxes grown by m".
  missing            an object occluded and absent from the estimate. The ONLY
                     mode that produces false ACCEPTS, therefore the only one
                     that can crash hardware, therefore the one worth spending on.
  phantom            an object in the estimate that is not on the table. Costs
                     keep-rate, never safety.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .scene import Layout, TABLE_TOP_Z

# The table as the screener sees it: a workspace surface starting clear of the
# arm's own base, which is bolted to it and is permanently "in contact".
TABLE_BOX = (np.array([0.18, -0.45, -0.06]), np.array([0.70, 0.45, TABLE_TOP_Z]))

# A single exocentric camera, mounted above and in front of the work area.
# Chosen by measurement, not taste: from a shallow front view (1.35, 0, 0.85)
# nothing is ever occluded -- rays pass under the arms -- so the error mode the
# design cares about would never have fired. From above, an arm reaching over an
# object hides it, which is the case worth modelling.
CAMERA_POS = np.array([0.75, 0.0, 1.25])


@dataclass(frozen=True)
class ObservedObject:
    oid: str
    kind: str
    lo: np.ndarray      # (P,3)
    hi: np.ndarray      # (P,3)
    phantom: bool = False


@dataclass(frozen=True)
class SceneEstimate:
    layout_id: str
    objects: tuple
    unknown_lo: np.ndarray | None = None   # (U,3) volume the camera cannot see
    unknown_hi: np.ndarray | None = None
    note: str = "perfect"


@dataclass(frozen=True)
class ErrorModel:
    offset: tuple = (0.0, 0.0, 0.0)   # systematic, applied to every object
    p_missing: float = 0.0            # UNIFORM dropout (the naive model)
    n_phantom: int = 0
    shadows: bool = False             # emit the occluded volume as "unknown"
    name: str = "perfect"
    # Line-of-sight occlusion: an object is dropped when the camera can see less
    # than `visible_thresh` of it. arm_centers/arm_radii let the other arm act as
    # an occluder, which is what makes the missing object the one being reached
    # over rather than a random one.
    line_of_sight: bool = False
    visible_thresh: float = 0.34
    arm_centers: object = None
    arm_radii: object = None


def _shadow_box(lo, hi):
    """Axis-aligned over-approximation of the volume hidden behind a box from the
    camera: extend its footprint away from the camera to the far table edge.

    Crude on purpose. The point is not an exact visibility computation, it is to
    give the screener a place to put the statement 'I cannot see here', which is
    the only structural defence against the missing-object mode.
    """
    corners = np.array([[lo[0], lo[1]], [lo[0], hi[1]], [hi[0], lo[1]], [hi[0], hi[1]]])
    pts = [corners]
    cam = CAMERA_POS[:2]
    for c in corners:
        d = c - cam
        n = np.linalg.norm(d)
        if n < 1e-6:
            continue
        pts.append((c + d / n * 0.28)[None, :])
    P = np.vstack(pts)
    s_lo = np.array([P[:, 0].min(), P[:, 1].min(), TABLE_TOP_Z])
    s_hi = np.array([P[:, 0].max(), P[:, 1].max(), max(hi[2], TABLE_TOP_Z + 0.06)])
    s_lo[:2] = np.clip(s_lo[:2], [0.18, -0.45], [0.70, 0.45])
    s_hi[:2] = np.clip(s_hi[:2], [0.18, -0.45], [0.70, 0.45])
    return s_lo, s_hi


def _ray_hits_boxes(origin, targets, box_lo, box_hi):
    """Slab test: does the segment origin->target[i] pass through box[j]?

    targets (T,3), box_lo/hi (Bx,3) -> (T,Bx) boolean.
    """
    d = targets - origin                                   # (T,3)
    L = np.linalg.norm(d, axis=1, keepdims=True)
    dir_ = d / np.maximum(L, 1e-9)
    inv = 1.0 / np.where(np.abs(dir_) < 1e-9, 1e-9, dir_)   # (T,3)
    t0 = (box_lo[None] - origin[None, None]) * inv[:, None, :]
    t1 = (box_hi[None] - origin[None, None]) * inv[:, None, :]
    tmin = np.minimum(t0, t1).max(axis=-1)                 # (T,Bx)
    tmax = np.maximum(t0, t1).min(axis=-1)
    # a hit that happens strictly before the target, and in front of the camera
    return (tmax >= np.maximum(tmin, 0.0)) & (tmin < L - 1e-3) & (tmax > 1e-3)


def _ray_hits_spheres(origin, targets, centers, radii):
    """Same, against spheres -- used for the arms, which are the real occluders."""
    if centers is None or len(centers) == 0:
        return np.zeros((len(targets), 0), dtype=bool)
    d = targets - origin
    L = np.linalg.norm(d, axis=1, keepdims=True)
    dir_ = d / np.maximum(L, 1e-9)                          # (T,3)
    oc = centers[None] - origin[None, None]                 # (1,S,3)
    proj = np.einsum("td,sd->ts", dir_, oc[0])              # (T,S)
    perp2 = (oc[0] ** 2).sum(-1)[None] - proj ** 2
    return (perp2 <= radii[None] ** 2) & (proj > 0) & (proj < L - 1e-3)


def visibility(layout: Layout, arm_centers=None, arm_radii=None) -> dict:
    """Fraction of each object's sample points the exocentric camera can see.

    Occlusion from a single fixed viewpoint is structural and pose-dependent, not
    a coin flip. Modelling it as a uniform dropout rate gets the *rate* roughly
    right and the *identity* of the missing object completely wrong -- and the
    identity is the whole question, because an arm reaching over an object both
    hides it and is about to hit it. On a two-arm station the other arm is the
    dominant occluder, so it is included as spheres.
    """
    lo, hi, owner = [], [], []
    for o in layout.objects:
        for pt in o.parts:
            lo.append(np.array(pt.pos) - np.array(pt.half))
            hi.append(np.array(pt.pos) + np.array(pt.half))
            owner.append(o.oid)
    lo, hi, owner = np.array(lo), np.array(hi), np.array(owner)

    out = {}
    for o in layout.objects:
        c = np.array(o.center)
        top = max(p.pos[2] + p.half[2] for p in o.parts)
        pts = np.array([[c[0], c[1], top], c,
                        [c[0] + 0.02, c[1], top], [c[0] - 0.02, c[1], top],
                        [c[0], c[1] + 0.02, top], [c[0], c[1] - 0.02, top]])
        mask = owner != o.oid                     # an object cannot occlude itself
        blocked = _ray_hits_boxes(CAMERA_POS, pts, lo[mask], hi[mask]).any(axis=1)
        blocked |= _ray_hits_spheres(CAMERA_POS, pts, arm_centers, arm_radii).any(axis=1)
        out[o.oid] = float((~blocked).mean())
    return out


def observe(layout: Layout, err: ErrorModel = ErrorModel(), rng=None) -> SceneEstimate:
    """Turn truth into the (wrong) picture the screener gets."""
    rng = rng or np.random.default_rng(0)
    off = np.asarray(err.offset, dtype=float)
    objs, shadows = [], []

    vis = (visibility(layout, err.arm_centers, err.arm_radii)
           if err.line_of_sight else None)
    for o in layout.objects:
        if vis is not None:
            if vis[o.oid] < err.visible_thresh:
                continue                               # camera cannot see it
        elif err.p_missing > 0 and rng.random() < err.p_missing:
            continue                                   # uniform dropout
        lo = np.array([np.array(p.pos) - np.array(p.half) for p in o.parts]) + off
        hi = np.array([np.array(p.pos) + np.array(p.half) for p in o.parts]) + off
        objs.append(ObservedObject(o.oid, o.kind, lo, hi))
        if err.shadows:
            shadows.append(_shadow_box(lo.min(axis=0), hi.max(axis=0)))

    for i in range(err.n_phantom):
        x = rng.uniform(0.32, 0.60)
        y = rng.uniform(-0.28, 0.28)
        h = rng.uniform(0.03, 0.09)
        lo = np.array([[x - 0.03, y - 0.03, TABLE_TOP_Z]])
        hi = np.array([[x + 0.03, y + 0.03, TABLE_TOP_Z + h]])
        objs.append(ObservedObject(f"phantom{i}", "cube", lo, hi, phantom=True))

    u_lo = np.array([s[0] for s in shadows]) if shadows else None
    u_hi = np.array([s[1] for s in shadows]) if shadows else None
    return SceneEstimate(layout.layout_id, tuple(objs), u_lo, u_hi, err.name)
