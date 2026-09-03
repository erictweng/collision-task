"""Export a slice of the evaluation for the review view (out/view_data.json)."""
import json, pickle, sys, warnings
warnings.filterwarnings("ignore")
sys.path.insert(0, ".")
import mujoco, numpy as np

from screener.estimate import observe
from screener.evalrun import evaluate
from screener.fast import (ArmProxy, ScreenConfig, _parts, calibrate_self_pairs,
                           sample_joint_path, screen_self)
from screener.geom import sphere_box_gap
from screener.model import build
from screener.motions import _arm_qpos_adr, tcp_of

CFG = ScreenConfig(margin=0.0)
N_LAYOUTS = 6


def tcp_path(model, data, motion, n=48):
    adr = _arm_qpos_adr(model)
    out = []
    for q in sample_joint_path(motion, n):
        data.qpos[adr] = q
        mujoco.mj_kinematics(model, data)
        mujoco.mj_comPos(model, data)
        out.append(tcp_of(model, data)[0].copy())
    return np.array(out)


def main():
    ds = pickle.load(open("out/ds_A.pkl", "rb"))
    rec = {r.mid: r for r in ds["records"]}
    model0, _ = build(ds["layouts"][next(iter(ds["layouts"]))])
    px = ArmProxy(model0)
    clean = [m for _, m in ds["motions"]
             if not any(y.startswith("self_collision") for y in rec[m.mid].reasons)]
    calib = calibrate_self_pairs(px, clean)

    r = evaluate(ds, CFG, proxy=px)
    verdict = dict(zip(r["_mids"], r["_allow"]))
    culprit = dict(zip(r["_mids"], r["_culprits"]))

    by = {}
    for s, mo in ds["motions"]:
        by.setdefault(s, []).append(mo)

    layouts, motions = {}, []
    for s in sorted(by)[:N_LAYOUTS]:
        lay = ds["layouts"][s]
        model, data = build(lay)
        est = observe(lay)
        lo, hi, owner, kinds = _parts(est, CFG)
        layouts[lay.layout_id] = dict(
            id=lay.layout_id,
            parts=[dict(owner=str(owner[i]), kind=str(kinds[i]),
                        lo=[round(float(v), 4) for v in lo[i]],
                        hi=[round(float(v), 4) for v in hi[i]])
                   for i in range(len(owner)) if kinds[i] != "table"])
        ms = by[s]
        ok_self, _ = screen_self(ms, px, calib, CFG)
        C = px.fk_spheres(np.stack([sample_joint_path(m, CFG.n_poses) for m in ms])
                          .reshape(-1, 7)).reshape(len(ms), -1, 3)
        radii = np.tile(px.radii, CFG.n_poses)[None].repeat(len(ms), 0)
        gaps = sphere_box_gap(C, radii, lo[None], hi[None])
        for i, mo in enumerate(ms):
            rc = rec[mo.mid]
            per = {}
            for name in sorted(set(owner)):
                per[str(name)] = round(float(gaps[i][:, owner == name].min()), 4)
            motions.append(dict(
                mid=mo.mid, layout=lay.layout_id, tag=rc.tag,
                target=mo.intent.target_oid, dest=mo.intent.destination_oid,
                allow=bool(verdict[mo.mid]) and bool(ok_self[i]),
                unsafe=bool(rc.unsafe), reasons=list(rc.reasons)[:3],
                blamed=str(culprit.get(mo.mid) or ""),
                gaps=per,
                path=[[round(float(v), 4) for v in p]
                      for p in tcp_path(model, data, mo, 40)]))
    json.dump(dict(layouts=layouts, motions=motions,
                   config=dict(margin_mm=CFG.margin * 1000, n_poses=CFG.n_poses)),
              open("out/view_data.json", "w"))
    n = len(motions)
    print(f"exported {n} motions over {len(layouts)} layouts "
          f"({sum(m['allow'] != (not m['unsafe']) for m in motions)} disagreements)")


if __name__ == "__main__":
    main()
