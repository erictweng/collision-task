"""The three claims everything else rests on. Each one, if false, makes a number
elsewhere in this repo meaningless rather than merely wrong.

    1. The sphere proxy CONTAINS the arm. If a real link pokes outside its
       spheres, "clearance" is not clearance and every safe verdict is a guess.
    2. The screener cannot reach the oracle. Not by discipline -- by imports.
    3. The screener screens the motion the executor will actually perform.
       Two different interpolations would mean the verdict describes a
       trajectory the robot never follows.
"""
from __future__ import annotations

import ast
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import mujoco  # noqa: E402

from screener.fast import CHAIN, PROXY_SKIP, ArmProxy, sample_joint_path  # noqa: E402
from screener.model import build  # noqa: E402
from screener.scene import random_layout  # noqa: E402
from screener.truth import _ctrl_at  # noqa: E402

TOL_MM = 1.0
fail = []


def _all_collision_points(model, bid):
    """Every collision vertex of a body, in its frame -- NOT subsampled.

    fast._geom_points thins meshes above 300 vertices to keep the fit cheap. The
    fit is allowed to do that; this check is not, because the vertices it skipped
    are exactly where an uncovered corner would hide.
    """
    from screener.fast import _quat_to_mat
    out = []
    for g in range(model.body_geomadr[bid], model.body_geomadr[bid] + model.body_geomnum[bid]):
        if model.geom_contype[g] == 0 and model.geom_conaffinity[g] == 0:
            continue
        t = model.geom_type[g]
        if t == mujoco.mjtGeom.mjGEOM_MESH:
            mid = model.geom_dataid[g]
            a, n = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
            v = model.mesh_vert[a:a + n]
        elif t == mujoco.mjtGeom.mjGEOM_BOX:
            s = model.geom_size[g][:3]
            v = np.array([[x, y, z] for x in (-s[0], s[0])
                          for y in (-s[1], s[1]) for z in (-s[2], s[2])])
        else:
            continue
        out.append(v @ _quat_to_mat(model.geom_quat[g]).T + model.geom_pos[g])
    return np.vstack(out) if out else np.zeros((0, 3))


print("1. proxy containment — every collision vertex inside some sphere")
model, _ = build(random_layout(0))
px = ArmProxy(model)
worst, worst_link = -1e9, None
for link in px.links:
    if not len(link["sr"]) or link["name"] in PROXY_SKIP:
        continue
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link["name"])
    pts = _all_collision_points(model, bid)
    if not len(pts):
        continue
    d = np.linalg.norm(pts[:, None, :] - link["sc"][None], axis=-1) - link["sr"][None]
    over = float(d.min(axis=1).max())          # >0 means a vertex escaped every sphere
    if over > worst:
        worst, worst_link = over, link["name"]
print(f"   worst escape: {worst*1000:+.3f} mm on {worst_link}  (tolerance {TOL_MM} mm)")
if worst > TOL_MM / 1000:
    fail.append(f"proxy does not contain {worst_link}: {worst*1000:.2f} mm outside")

print("\n2. import boundary — the screener cannot reach the oracle")
PKG = pathlib.Path(__file__).resolve().parent.parent / "screener"
ORACLE = {"truth", "dual"}          # modules that hold outcomes


def imports_of(p):
    for node in ast.walk(ast.parse(p.read_text())):
        if isinstance(node, ast.Import):
            yield from (a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            yield ("." if node.level else "") + (node.module or "")


seen, stack, reached = set(), ["fast"], []
while stack:
    mod = stack.pop()
    if mod in seen:
        continue
    seen.add(mod)
    f = PKG / f"{mod}.py"
    if not f.exists():
        continue
    for imp in imports_of(f):
        head = imp.lstrip(".").split(".")[0]
        if head in ORACLE:
            reached.append(f"{mod} -> {head}")
        elif (PKG / f"{head}.py").exists():
            stack.append(head)
print(f"   transitive closure from screener.fast: {', '.join(sorted(seen))}")
if reached:
    fail.append("screener reaches the oracle: " + "; ".join(reached))
else:
    print("   reaches neither truth.py nor dual.py — no path to an outcome")

print("\n3. screener and executor agree on the trajectory")
lay = random_layout(1)
model, data = build(lay)
from screener.motions import generate_A  # noqa: E402
m = generate_A(model, data, lay, np.random.default_rng(0), 1)[0]
N = 48
screened = sample_joint_path(m, N)
executed = np.array([_ctrl_at(m, t)[0] for t in np.linspace(0, m.duration, N)])
err = float(np.abs(screened - executed).max())
print(f"   max joint disagreement over {N} poses: {err:.2e} rad")
if err > 1e-9:
    fail.append(f"screener and executor disagree by {err:.2e} rad")

print()
if fail:
    for f_ in fail:
        print("FAIL:", f_)
    sys.exit(1)
print("all three invariants hold")
