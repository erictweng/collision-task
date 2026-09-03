"""Scene definition, layout generation and MJCF assembly.

A Scene here is the *truth*: exact object poses. The screener never sees one of
these -- see screener/estimate.py for the only thing it is allowed to read.

Object model
------------
Every object is a set of axis-aligned boxes ("parts"). A cube is one part. A bin
is five: four walls and a floor, with an open top. Representing a bin as its
walls rather than as a solid volume is deliberate: it means "reach into the bin"
requires no special case in the geometry, only in the *margin* applied to it.
That is where intent enters, and it is the whole answer to "clearance is not
the answer" -- we do not forbid the bin's volume, we relax the clearance we
demand of the bin the motion is actually delivering to.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Sequence

import numpy as np

TABLE_TOP_Z = 0.0   # panda.xml mounts link0 at the world origin;
                    # the table top IS the base plane, or the arm spawns inside it
TABLE_HALF = (0.55, 0.45, 0.02)
REACH_X = (0.30, 0.62)
REACH_Y = (-0.30, 0.30)


@dataclass(frozen=True)
class Part:
    """One axis-aligned box belonging to an object."""
    name: str
    pos: tuple           # centre, world frame
    half: tuple          # half-extents


@dataclass(frozen=True)
class Obj:
    oid: str
    kind: str            # 'cube' | 'bin' | 'post' | 'plate'
    parts: tuple         # tuple[Part, ...]
    movable: bool
    # For containers: where a delivered object should end up. Purely a target
    # for motion generation; the screener does not use it.
    drop_point: tuple | None = None

    @property
    def center(self):
        p = np.array([pt.pos for pt in self.parts])
        return tuple(p.mean(axis=0))


@dataclass(frozen=True)
class Layout:
    layout_id: str
    objects: tuple       # tuple[Obj, ...]

    def by_id(self, oid: str) -> Obj:
        for o in self.objects:
            if o.oid == oid:
                return o
        raise KeyError(oid)

    @property
    def cubes(self):
        return [o for o in self.objects if o.kind == "cube"]

    @property
    def bins(self):
        return [o for o in self.objects if o.kind == "bin"]


# --------------------------------------------------------------------------
# object constructors
# --------------------------------------------------------------------------

CUBE_HALF = 0.022


def make_cube(oid: str, x: float, y: float) -> Obj:
    z = TABLE_TOP_Z + CUBE_HALF
    return Obj(oid, "cube",
               (Part(oid, (x, y, z), (CUBE_HALF,) * 3),), movable=True)


def make_post(oid: str, x: float, y: float, h: float = 0.11) -> Obj:
    """A fixed vertical obstacle -- a clamp stand, a camera pole. Static, and
    tall enough that a lazy overhead transit clips it."""
    z = TABLE_TOP_Z + h / 2
    return Obj(oid, "post",
               (Part(oid, (x, y, z), (0.022, 0.022, h / 2)),), movable=False)


def make_bin(oid: str, x: float, y: float,
             inner=(0.070, 0.070, 0.055), wall=0.009) -> Obj:
    """Open-top bin: four walls plus a floor. The mouth is open, so entering it
    is geometrically free -- what stops a naive screener is the margin, not the
    geometry."""
    ix, iy, iz = inner
    z0 = TABLE_TOP_Z
    parts = [
        Part(f"{oid}_floor", (x, y, z0 + wall / 2), (ix + wall, iy + wall, wall / 2)),
        Part(f"{oid}_xp", (x + ix + wall / 2, y, z0 + wall + iz / 2), (wall / 2, iy + wall, iz / 2)),
        Part(f"{oid}_xn", (x - ix - wall / 2, y, z0 + wall + iz / 2), (wall / 2, iy + wall, iz / 2)),
        Part(f"{oid}_yp", (x, y + iy + wall / 2, z0 + wall + iz / 2), (ix, wall / 2, iz / 2)),
        Part(f"{oid}_yn", (x, y - iy - wall / 2, z0 + wall + iz / 2), (ix, wall / 2, iz / 2)),
    ]
    return Obj(oid, "bin", tuple(parts), movable=False,
               drop_point=(x, y, z0 + wall + iz + 0.02))


# --------------------------------------------------------------------------
# layout generation
# --------------------------------------------------------------------------

def _non_overlapping(rng, n, min_sep, existing):
    """Rejection-sample positions on the reachable part of the table."""
    pts = list(existing)
    out = []
    for _ in range(n):
        for _try in range(400):
            x = rng.uniform(*REACH_X)
            y = rng.uniform(*REACH_Y)
            if all(math.hypot(x - px, y - py) >= min_sep for px, py in pts):
                pts.append((x, y))
                out.append((x, y))
                break
        else:
            raise RuntimeError("could not place object; table too crowded")
    return out


def random_layout(seed: int, n_cubes: int = 4, n_posts: int = 2,
                  n_bins: int = 1, clutter: bool = True) -> Layout:
    """Vary the arrangement, not just the motion. A screener evaluated on one
    arrangement tells you nothing about a fleet where arrangements keep changing.

    `clutter` places obstacles deliberately on the cube->bin corridor. Without it
    the transit obstacles sit off the path, almost nothing collides, and the
    evaluation set has no hard cases in it -- which reads as a good screener and
    is really an easy test.
    """
    rng = np.random.default_rng(seed)
    objs, placed = [], []

    bin_pts = _non_overlapping(rng, n_bins, 0.26, placed)
    for i, (x, y) in enumerate(bin_pts):
        objs.append(make_bin(f"bin{i}", x, y))
        placed.append((x, y))

    cube_pts = _non_overlapping(rng, n_cubes, 0.115, placed)
    for i, (x, y) in enumerate(cube_pts):
        objs.append(make_cube(f"cube{i}", x, y))
        placed.append((x, y))

    n_placed = 0
    if clutter:
        # one post on the corridor between a cube and the bin: the case a low or
        # direct transit is supposed to fail, and which sparse layouts never make
        for k in range(min(n_posts, len(cube_pts))):
            cx, cy = cube_pts[k]
            bx, by = bin_pts[0]
            f = rng.uniform(0.40, 0.62)
            x = cx + f * (bx - cx) + rng.normal(0, 0.012)
            y = cy + f * (by - cy) + rng.normal(0, 0.012)
            if not (REACH_X[0] <= x <= REACH_X[1] and REACH_Y[0] <= y <= REACH_Y[1]):
                continue
            if any(math.hypot(x - px, y - py) < 0.085 for px, py in placed):
                continue
            objs.append(make_post(f"post{n_placed}", x, y,
                                  h=float(rng.uniform(0.09, 0.16))))
            placed.append((x, y))
            n_placed += 1

    for i, (x, y) in enumerate(_non_overlapping(rng, n_posts - n_placed, 0.115, placed)):
        objs.append(make_post(f"post{n_placed + i}", x, y,
                              h=float(rng.uniform(0.09, 0.16))))
        placed.append((x, y))

    return Layout(f"L{seed:03d}", tuple(objs))
