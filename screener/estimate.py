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

# A single overhead-ish camera. Used only to decide what is occluded.
CAMERA_POS = np.array([1.35, 0.0, 0.85])


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
    p_missing: float = 0.0            # per-object probability of being occluded out
    n_phantom: int = 0
    shadows: bool = False             # emit the occluded volume as "unknown"
    name: str = "perfect"


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


def observe(layout: Layout, err: ErrorModel = ErrorModel(), rng=None) -> SceneEstimate:
    """Turn truth into the (wrong) picture the screener gets."""
    rng = rng or np.random.default_rng(0)
    off = np.asarray(err.offset, dtype=float)
    objs, shadows = [], []

    for o in layout.objects:
        if err.p_missing > 0 and rng.random() < err.p_missing:
            continue                                   # occluded out of the scene
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
