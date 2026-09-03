"""Vectorised primitive geometry for the fast path.

Everything here is batched numpy. Nothing in this module imports mujoco: the fast
path must not be able to reach the simulator, by construction.

Representation choices, and why:
  * Arm  -> spheres. A capsule sampled at a few points is a set of spheres, and
    sphere-vs-box distance is exact and branch-free, which is what lets the whole
    screen run as array maths instead of a Python loop.
  * Objects -> axis-aligned boxes. Cubes and bin walls are axis-aligned in this
    scene. A rotated object would need an OBB test; that is a named limitation.
"""
from __future__ import annotations

import numpy as np


def sphere_box_gap(centers: np.ndarray, radii: np.ndarray,
                   box_lo: np.ndarray, box_hi: np.ndarray) -> np.ndarray:
    """Signed gap between spheres and axis-aligned boxes.

    Broadcasting contract:
        centers  (..., S, 3)
        radii    (..., S)
        box_lo   (..., B, 3)
        box_hi   (..., B, 3)
    Returns      (..., S, B)  gap; negative means overlap by that depth.

    Exact for points outside the box. Inside the box the clamped point is the
    centre itself, so the gap saturates at -radius rather than reporting true
    penetration depth. We only ever compare against a threshold, so saturation
    is harmless -- but it means this number is not a penetration depth and must
    not be read as one.
    """
    c = centers[..., :, None, :]              # (..., S, 1, 3)
    lo = box_lo[..., None, :, :]              # (..., 1, B, 3)
    hi = box_hi[..., None, :, :]
    nearest = np.clip(c, lo, hi)              # (..., S, B, 3)
    d = np.linalg.norm(c - nearest, axis=-1)  # (..., S, B)
    return d - radii[..., :, None]


def boxes_from_center_size(center: np.ndarray, half: np.ndarray):
    """(...,3) centre and (...,3) half-extent -> (lo, hi)."""
    return center - half, center + half


def inflate(box_lo: np.ndarray, box_hi: np.ndarray, m: float):
    """Grow boxes by m on every side. This is how a systematic scene offset of
    bounded magnitude is handled deterministically: 'safe under any translation
    of size <= m' is exactly 'safe against boxes inflated by m'."""
    return box_lo - m, box_hi + m


def segment_spheres(a: np.ndarray, b: np.ndarray, radius: np.ndarray, n: int):
    """Sample a capsule (segment a->b, given radius) as n equal spheres.

        a, b    (..., 3)
        radius  (...,)
    Returns centers (..., n, 3), radii (..., n).

    n spheres cover the capsule with a scalloped surface; the gap between
    consecutive spheres bulges inward by at most r*(1-cos(theta)). We pick n so
    that error is well under the margin we screen with, and record it as a known
    conservatism, not a bug.
    """
    t = np.linspace(0.0, 1.0, n).reshape((1,) * (a.ndim - 1) + (n, 1))
    centers = a[..., None, :] * (1.0 - t) + b[..., None, :] * t
    radii = np.repeat(radius[..., None], n, axis=-1)
    return centers, radii


def aabb_of_points(pts: np.ndarray, pad: float = 0.0):
    """Bounding box of (..., K, 3) points -> (lo, hi) each (..., 3)."""
    return pts.min(axis=-2) - pad, pts.max(axis=-2) + pad


def aabb_overlap(a_lo, a_hi, b_lo, b_hi) -> np.ndarray:
    """Broad phase. (..., 3) vs (..., B, 3) -> (..., B) boolean."""
    return np.all((a_lo[..., None, :] <= b_hi) & (a_hi[..., None, :] >= b_lo), axis=-1)
