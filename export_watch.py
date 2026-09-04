"""Export an animated view of what the screener sees (out/watch_data.json).

The point of the view is not a render of the robot -- it is the screener's OWN
representation: the sphere proxy of the arm, the boxes it believes the objects
are, and the gap between them at every instant. Watching that is how you see why
a verdict came out the way it did, and why the ones it gets wrong are wrong.

Motions are chosen for balance, not at random: false accepts and false rejects
first, because those are the cases worth looking at.
"""
from __future__ import annotations

import json
import pickle

import numpy as np

from screener.estimate import observe
from screener.evalrun import evaluate
from screener.fast import (ArmProxy, ScreenConfig, _parts, calibrate_self_pairs,
                           sample_joint_path, screen_self)
from screener.geom import sphere_box_gap
from screener.model import build

CFG = ScreenConfig(margin=0.0)
# The animation MUST sample the trajectory at exactly the poses the screener
# used. Sampling fewer would let a chart show a gap that never dips below a
# licence on a motion the screener rejected -- which reads as a bug in the view
# and is really two different samplings of the same path.
N_POSES = CFG.n_poses
N_PER_CLASS = 7


def main():
    ds = pickle.load(open("out/ds_A.pkl", "rb"))
    rec = {r.mid: r for r in ds["records"]}
    model0, _ = build(ds["layouts"][next(iter(ds["layouts"]))])
    px = ArmProxy(model0)
    r = evaluate(ds, CFG, proxy=px)
    verdict = dict(zip(r["_mids"], r["_allow"]))
    blamed = dict(zip(r["_mids"], r["_culprits"]))

    # balanced pick: false accepts, false rejects, then agreements
    def cls(mid):
        a, u = verdict[mid], rec[mid].unsafe
        return "false_accept" if (a and u) else ("false_reject" if (not a and not u)
                                                 else ("caught" if u else "kept"))
    chosen, seen = [], {}
    for mid in r["_mids"]:
        c = cls(mid)
        if seen.get(c, 0) < N_PER_CLASS:
            seen[c] = seen.get(c, 0) + 1
            chosen.append(mid)

    by_layout = {}
    for s, mo in ds["motions"]:
        by_layout.setdefault(s, []).append(mo)
    motion_of = {mo.mid: (s, mo) for s, ms in by_layout.items() for mo in ms}

    layouts, motions = {}, []
    mm = lambda a: np.round(1000 * np.asarray(a)).astype(int).tolist()

    for mid in chosen:
        s, mo = motion_of[mid]
        lay = ds["layouts"][s]
        est = observe(lay)
        lo, hi, owner, kinds = _parts(est, CFG)
        if lay.layout_id not in layouts:
            layouts[lay.layout_id] = dict(
                parts=[dict(owner=str(owner[i]), kind=str(kinds[i]),
                            lo=mm(lo[i]), hi=mm(hi[i])) for i in range(len(owner))])

        Q = sample_joint_path(mo, N_POSES)
        C = px.fk_spheres(Q)                                   # (N, S, 3)
        g = sphere_box_gap(C, np.tile(px.radii, (N_POSES, 1)),
                           np.repeat(lo[None], N_POSES, 0),
                           np.repeat(hi[None], N_POSES, 0))    # (N, S, P)
        per_obj = {}
        for name in sorted(set(owner)):
            per_obj[str(name)] = mm(g[:, :, owner == name].min(axis=(1, 2)))

        rc = rec[mid]
        motions.append(dict(
            mid=mid, layout=lay.layout_id, tag=rc.tag, cls=cls(mid),
            target=mo.intent.target_oid, dest=mo.intent.destination_oid,
            allow=bool(verdict[mid]), unsafe=bool(rc.unsafe),
            reasons=list(rc.reasons)[:3], blamed=str(blamed.get(mid) or ""),
            spheres=[mm(C[i]) for i in range(N_POSES)],
            gaps=per_obj))

    out = dict(layouts=layouts, motions=motions,
               radii=mm(px.radii), group=px.group.tolist(),
               config=dict(n_poses=N_POSES, margin_mm=CFG.margin * 1000,
                           table_margin=[round(1000 * v) for v in CFG.table_margin],
                           target_margin=[round(1000 * v) for v in CFG.target_margin],
                           dest_margin=[round(1000 * v) for v in CFG.destination_margin]))
    json.dump(out, open("out/watch_data.json", "w"))
    n = len(motions)
    print("exported %d motions x %d poses x %d spheres  (%.0f KB)"
          % (n, N_POSES, len(px.radii),
             len(json.dumps(out)) / 1024))
    from collections import Counter
    print(" ", Counter(m["cls"] for m in motions))


if __name__ == "__main__":
    main()
