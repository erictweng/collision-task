"""The runtime force trip: what it can catch, how fast, and what it costs.

NOT PART OF THE SCREENER. The screener is kinematic and runs before anything
moves; this runs on the robot during execution and reads a sensor. They are
different controls at different times, and having this file inside `screener/`
implied otherwise.

This is the premise the operating point rests on. Section 7 of DESIGN.md argues
that a permitted motion which collides is an aborted rollout rather than a
callout, and that this is what buys the false-accept:false-reject ratio down far
enough for the screener to keep a useful fraction of candidates. That argument is
worth nothing unmeasured, so this module measures it.

Two things have to be true for the trip to work, and they pull against each other:

  1. The threshold must sit ABOVE what a correct motion generates. A force sensor
     cannot tell which object it touched, so grasping the target and entering the
     bin both register. This is the same intended-vs-unintended problem the
     screener has, in a different sensor -- and it is why the floor is measured
     from safe motions rather than assumed.

WHAT SENSOR THIS ASSUMES, measured rather than asserted. `run_with_trip` scores
contact force anywhere on the arm, which is what JOINT TORQUE sensing on all
seven axes provides (the Panda has it). A cheaper wrist force/torque sensor sees
only what is transmitted through the wrist, and the difference is entirely
workload-dependent:

    arm vs objects   99% of harmful contacts are hand or fingers -> a wrist
                     sensor sees 100% of them. Measured over 116 unsafe motions.
    arm vs arm       83% are link5/link6 only -- the elbow and forearm -> a wrist
                     sensor sees 5%. Measured over 59 arm-arm collisions.

So a wrist sensor is sufficient for the single-arm tabletop task and close to
useless for the bimanual one. That is fortunate rather than alarming: arm-vs-arm
is exactly the class the pre-execution screener catches perfectly (zero false
accepts, and no camera in the path). The two controls cover complementary
failure modes, and this is the measurement that shows it rather than assuming
it.
  2. It must fire early enough that little has moved. A trip that fires after the
     bin is on the floor has converted nothing.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from screener.model import HOME_QPOS
from screener.motions import Motion, N_ARM
from screener.scene import Layout
from screener.truth import DISP_M, _ctrl_at, _owner_map


@dataclass
class TripResult:
    mid: str
    tripped: bool
    trip_t: float          # seconds from motion start
    peak_force: float      # peak arm contact force over the whole run
    max_disp: float        # displacement of any non-permitted movable object
    settled_disp: float    # the same, if we halt at the trip
    contact_to_trip_mm: float   # how far the TCP travelled between first
                                # contact and the threshold being crossed


def run_with_trip(model, data, layout: Layout, motion: Motion,
                  trip_N: float | None = None, halt: bool = True,
                  contact_stride: int = 2, settle_s: float = 0.4,
                  reaction: str = "freeze", rewind_ms: float = 120.0) -> TripResult:
    """Execute, optionally halting the arm the first time contact force exceeds
    `trip_N`. With trip_N=None this is an uninterrupted run, used as the control.

    Halting is modelled as freezing the position targets at the current
    configuration -- the cheapest possible reaction, and a lower bound on what a
    real controller could do.
    """
    owner = _owner_map(model, layout)
    permitted = {motion.intent.target_oid, motion.intent.destination_oid}

    mujoco.mj_resetData(model, data)
    data.qpos[:N_ARM] = HOME_QPOS
    mujoco.mj_forward(model, data)
    bids = {o.oid: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, o.oid)
            for o in layout.objects}
    start = {oid: data.xpos[b].copy() for oid, b in bids.items()}
    ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")

    nsteps = int(motion.duration / model.opt.timestep)
    ft = np.zeros(6)
    peak, tripped, trip_t = 0.0, False, float("nan")
    first_contact_t, first_contact_p = None, None
    travel_mm = float("nan")
    frozen = None
    history = []            # recent commanded configurations, for the retract
    rewind_steps = max(1, int(rewind_ms / 1000.0 / model.opt.timestep))

    total = nsteps + (int(settle_s / model.opt.timestep) if halt else 0)
    for s in range(total):
        t = s * model.opt.timestep
        if frozen is None:
            q, g = _ctrl_at(motion, min(t, motion.duration))
            data.ctrl[:N_ARM] = q
            data.ctrl[N_ARM] = g
            history.append(q.copy())
            if len(history) > rewind_steps + 1:
                history.pop(0)
        else:
            data.ctrl[:N_ARM] = frozen
        mujoco.mj_step(model, data)

        if s % contact_stride:
            continue
        f_now = 0.0
        for c in range(data.ncon):
            con = data.contact[c]
            a, b = owner[con.geom1], owner[con.geom2]
            if "arm" not in (a, b):
                continue
            mujoco.mj_contactForce(model, data, c, ft)
            f_now = max(f_now, float(np.linalg.norm(ft[:3])))
        peak = max(peak, f_now)
        if f_now > 0.5 and first_contact_t is None:
            first_contact_t, first_contact_p = t, data.xpos[ee].copy()
        if trip_N is not None and not tripped and f_now > trip_N:
            tripped, trip_t = True, t
            if first_contact_p is not None:
                travel_mm = float(np.linalg.norm(data.xpos[ee] - first_contact_p) * 1000)
            if halt:
                # "freeze" holds position -- the cheapest possible reaction, and a
                # position servo will keep pressing into whatever it hit.
                # "retract" commands the configuration from rewind_ms earlier,
                # which is an active back-off along the path just travelled.
                frozen = (history[0].copy() if reaction == "retract" and history
                          else data.qpos[:N_ARM].copy())
        if frozen is None and s >= nsteps:
            break

    disp = 0.0
    for o in layout.objects:
        if o.movable and o.oid not in permitted:
            disp = max(disp, float(np.linalg.norm(data.xpos[bids[o.oid]] - start[o.oid])))
    return TripResult(motion.mid, tripped, trip_t, peak, disp, disp, travel_mm)
