"""MJCF assembly. This is the only module that knows about mujoco geometry.

THE MENAGERIE MESH-PATH GOTCHA
------------------------------
panda.xml declares <compiler meshdir="assets"/>, resolved relative to the
directory of the model file being loaded. A scene that lives elsewhere and does
<include file=".../panda.xml"/> therefore looks for meshes next to *itself* and
dies on a missing .obj. The fix used here is the first of the two the brief
suggests: the assembled scene is written into the Panda model's own directory,
so the relative meshdir resolves. `panda_dir()` finds that directory whether the
model came from robot_descriptions' cache or from a local sparse clone.
"""
from __future__ import annotations

import os
import pathlib

import mujoco
import numpy as np

from .scene import Layout, Obj, TABLE_HALF, TABLE_TOP_Z

_MASS = {"cube": 0.05, "post": 0.15, "bin": 0.60}
_RGBA = {"cube": "0.85 0.45 0.20 1", "post": "0.40 0.45 0.55 1",
         "bin": "0.25 0.55 0.75 1"}


def panda_dir() -> pathlib.Path:
    """Locate the Panda MJCF directory, preferring an explicit override."""
    env = os.environ.get("PANDA_DIR")
    if env:
        return pathlib.Path(env)
    local = pathlib.Path.home() / "menagerie" / "franka_emika_panda"
    if (local / "panda.xml").exists():
        return local
    from robot_descriptions import panda_mj_description as d  # lazy: it downloads
    return pathlib.Path(d.MJCF_PATH).parent


def _body_xml(o: Obj) -> str:
    """One MuJoCo body per object; one geom per part. Objects are free bodies so
    that 'knocked over' is a thing physics can actually report."""
    cx, cy, cz = o.center
    geoms = []
    for p in o.parts:
        rel = tuple(np.array(p.pos) - np.array([cx, cy, cz]))
        geoms.append(
            f'      <geom name="{p.name}" type="box" '
            f'size="{p.half[0]:.5f} {p.half[1]:.5f} {p.half[2]:.5f}" '
            f'pos="{rel[0]:.5f} {rel[1]:.5f} {rel[2]:.5f}" '
            f'rgba="{_RGBA[o.kind]}" friction="1.0 0.02 0.001"/>'
        )
    return (
        f'    <body name="{o.oid}" pos="{cx:.5f} {cy:.5f} {cz:.5f}">\n'
        f'      <freejoint name="{o.oid}_free"/>\n'
        f'      <inertial pos="0 0 0" mass="{_MASS[o.kind]}" diaginertia="1e-3 1e-3 1e-3"/>\n'
        + "\n".join(geoms) + "\n    </body>"
    )


def scene_xml(layout: Layout) -> str:
    bodies = "\n".join(_body_xml(o) for o in layout.objects)
    return f"""<mujoco model="screening_scene_{layout.layout_id}">
  <include file="panda.xml"/>
  <statistic center="0.4 0 0.2" extent="1.2"/>
  <visual><global azimuth="140" elevation="-25"/></visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.4 0.5 0.6" rgb2="0 0 0" width="32" height="128"/>
  </asset>
  <worldbody>
    <light pos="0.4 0 2.0" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" pos="0 0 -0.60" size="3 3 0.05" rgba="0.25 0.25 0.28 1"/>
    <body name="table" pos="0.15 0 {TABLE_TOP_Z - TABLE_HALF[2] - 0.001:.5f}">
      <geom name="table_top" type="box" size="{TABLE_HALF[0]} {TABLE_HALF[1]} {TABLE_HALF[2]}"
            rgba="0.72 0.68 0.60 1" friction="1.0 0.02 0.001"/>
    </body>
{bodies}
  </worldbody>
</mujoco>
"""


def build(layout: Layout):
    """Write the scene beside panda.xml (so meshdir resolves) and load it."""
    d = panda_dir()
    path = d / f"_gen_{layout.layout_id}.xml"
    path.write_text(scene_xml(layout))
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    return model, data


# The Panda is mounted at the world origin by panda.xml; we sit the table under
# it rather than moving the arm, which keeps the included model untouched.
HOME_QPOS = np.array([0.0, -0.3, 0.0, -2.0, 0.0, 1.75, 0.7853])
ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]
