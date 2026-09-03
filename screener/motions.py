"""Task definition, inverse kinematics, and generator A.

A Motion is joint-space waypoints plus a declared intent. The intent is not
decoration: it is the input that lets the screener tell the contact it wants
from the contact it must prevent, and it is free here because the counterfactual
generator already knows which object it was aiming at.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Sequence

import mujoco
import numpy as np

from .model import ARM_JOINTS, HOME_QPOS
from .scene import Layout, Obj, CUBE_HALF, TABLE_TOP_Z

N_ARM = 7
EE_BODY = "hand"
GRIP_OPEN, GRIP_CLOSED = 255.0, 0.0


@dataclass(frozen=True)
class Intent:
    """What the motion is trying to do. Supplied by the generator that produced
    the candidate -- i.e. by the task spec, not by perception."""
    target_oid: str          # the object we mean to touch / grasp
    destination_oid: str     # the container we mean to reach into (may be None)


@dataclass(frozen=True)
class Motion:
    mid: str
    layout_id: str
    waypoints: np.ndarray    # (K, 7) joint-space targets
    grip: np.ndarray         # (K,) gripper ctrl at each waypoint
    duration: float
    intent: Intent
    gen: str                 # which generator authored it
    tag: str                 # what perturbation produced it (for failure analysis)


# --------------------------------------------------------------------------
# kinematics
# --------------------------------------------------------------------------

def _arm_qpos_adr(model, prefix: str = ""):
    """Joint addresses BY NAME.

    On the two-arm model the object free-joints are declared before the arms, so
    the arm's joints are no longer at qpos[0:7]. Every positional assumption of
    that kind is a silent wrong-body bug, so addressing is name-based throughout.
    """
    return np.array([model.jnt_qposadr[mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, prefix + j)] for j in ARM_JOINTS])


def arm_ctrl_ids(model, prefix: str = ""):
    """Actuator ids for one arm: seven joints then the gripper."""
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{prefix}actuator{i}")
           for i in range(1, 9)]
    return np.array(ids)


def adr_to_jnt(model, prefix: str = ""):
    return np.array([mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefix + j)
                     for j in ARM_JOINTS])


def arm_dofs(model, prefix: str = ""):
    """Velocity-space indices of one arm's seven joints.

    mj_jac returns a column per DOF of the whole model. On the two-arm scene the
    object free joints occupy the first 36 of them, so slicing `[:, :7]` selects
    the bin's translation and rotation instead of the arm's joints -- the IK then
    reports a jacobian for the furniture and never moves. Same failure as
    assuming qpos[0:7]: index by name, never by position."""
    return model.jnt_dofadr[adr_to_jnt(model, prefix)]


TCP_OFFSET = 0.1034      # hand body origin -> between the fingertips, +z of hand


def tcp_of(model, data, prefix: str = ""):
    """World position of the tool centre point."""
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, prefix + EE_BODY)
    R = data.xmat[bid].reshape(3, 3)
    return data.xpos[bid] + R @ np.array([0.0, 0.0, TCP_OFFSET]), R, bid


def ik_pose(model, data, target_xyz, q0, iters=200, tol=2e-3, w_rot=0.35,
            prefix: str = ""):
    """Damped least-squares IK solving for the TCP position with the gripper
    pointing down.

    Two things here were learned the hard way. Solving for the *hand body* rather
    than the TCP puts the fingertips ~10 cm below where you asked, which drives
    them through the table on every grasp and labels the entire dataset unsafe.
    And leaving orientation free lets the solver find elbow-up solutions with the
    gripper sideways, which are valid IK and nonsense motions.

    Orientation is constrained only in the hand's z-axis (point down); roll about
    that axis is left free, because pinning it costs reachability and buys
    nothing here.
    """
    adr = _arm_qpos_adr(model, prefix)
    q = np.array(q0, dtype=float)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    dofs = arm_dofs(model, prefix)
    jr = model.jnt_range[adr_to_jnt(model, prefix)]
    lo, hi = jr[:, 0], jr[:, 1]
    desired_z = np.array([0.0, 0.0, -1.0])
    err_norm = 1e9
    for _ in range(iters):
        data.qpos[adr] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        tcp, R, bid = tcp_of(model, data, prefix)
        e_pos = np.asarray(target_xyz) - tcp
        e_rot = np.cross(R[:, 2], desired_z)          # 0 when z-axis points down
        err_norm = float(np.linalg.norm(e_pos))
        if err_norm < tol and np.linalg.norm(e_rot) < 0.05:
            break
        mujoco.mj_jac(model, data, jacp, jacr, tcp, bid)   # jacobian AT the TCP
        J = np.vstack([jacp[:, dofs], w_rot * jacr[:, dofs]])
        e = np.concatenate([e_pos, w_rot * e_rot])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        q = np.clip(q + 0.5 * dq, lo, hi)
    return q, err_norm


# --------------------------------------------------------------------------
# the reference task, and perturbations of it
# --------------------------------------------------------------------------

def _cube_top(o: Obj, clearance: float) -> np.ndarray:
    c = np.array(o.center)
    return np.array([c[0], c[1], TABLE_TOP_Z + 2 * CUBE_HALF + clearance])


def _cube_center(o: Obj) -> np.ndarray:
    c = np.array(o.center)
    return np.array([c[0], c[1], TABLE_TOP_Z + CUBE_HALF])


def reference_waypoints(model, data, layout: Layout, cube: Obj, bin_: Obj,
                        transit_h: float = 0.22, approach=(0.0, 0.0),
                        prefix: str = ""):
    """The scripted reference: approach above the cube, descend, lift, transit,
    descend into the bin, retreat. Six task-space points -> six joint waypoints."""
    drop = np.array(bin_.drop_point)
    pts = [
        _cube_top(cube, transit_h) + np.array([approach[0], approach[1], 0.0]),
        _cube_center(cube),
        _cube_top(cube, transit_h),
        np.array([drop[0], drop[1], TABLE_TOP_Z + transit_h + 0.06]),
        np.array([drop[0], drop[1], drop[2]]),
        np.array([drop[0], drop[1], TABLE_TOP_Z + transit_h + 0.06]),
    ]
    grip = np.array([GRIP_OPEN, GRIP_OPEN, GRIP_CLOSED, GRIP_CLOSED, GRIP_CLOSED, GRIP_OPEN])
    q = HOME_QPOS.copy()
    ws, errs = [], []
    for p in pts:
        q, e = ik_pose(model, data, p, q, prefix=prefix)
        ws.append(q.copy())
        errs.append(e)
    return np.array(ws), grip, float(max(errs))


def generate_A(model, data, layout: Layout, rng, n: int) -> list[Motion]:
    """Generator A: the reference plus structured counterfactuals.

    Perturbations are chosen to straddle the decision boundary rather than to be
    uniformly random -- a set of obviously-fine and obviously-terrible motions
    measures nothing. 'transit_low' and 'direct' exist specifically to produce
    motions that are unsafe for reasons the gripper never touches.
    """
    cubes, bins_ = layout.cubes, layout.bins
    out = []
    for i in range(n):
        cube = cubes[rng.integers(len(cubes))]
        bin_ = bins_[rng.integers(len(bins_))]
        intent = Intent(cube.oid, bin_.oid)
        kind = rng.choice(
            ["reference", "jitter", "approach", "transit_low", "direct",
             "wrong_target", "overshoot"],
            p=[0.14, 0.22, 0.14, 0.16, 0.14, 0.10, 0.10])

        transit_h, approach = 0.22, (0.0, 0.0)
        if kind == "transit_low":
            transit_h = float(rng.uniform(0.045, 0.13))
        if kind == "approach":
            a = rng.normal(0, 0.06, 2)
            approach = (float(a[0]), float(a[1]))

        aim = cube
        if kind == "wrong_target" and len(cubes) > 1:
            others = [c for c in cubes if c.oid != cube.oid]
            aim = others[rng.integers(len(others))]

        ws, grip, ikerr = reference_waypoints(model, data, layout, aim, bin_,
                                              transit_h=transit_h, approach=approach)
        if kind == "direct":
            ws = ws[[0, 1, 4, 5]]          # skip the lift: drag across the table
            grip = grip[[0, 1, 4, 5]]
        if kind == "jitter":
            s = float(rng.uniform(0.02, 0.16))
            ws = ws + rng.normal(0, s, ws.shape)
        if kind == "overshoot":
            ws = ws.copy()
            ws[4] = ws[4] + rng.normal(0, 0.10, N_ARM)

        out.append(Motion(
            mid=f"{layout.layout_id}-A{i:03d}", layout_id=layout.layout_id,
            waypoints=np.asarray(ws, dtype=float), grip=np.asarray(grip, dtype=float),
            duration=4.0, intent=intent, gen="A", tag=kind))
    return out
