"""Recover the can's REST position from the gripper motor current (branch experiment).

The bags carry gripper motor current + position, not just position. When the gripper
closes on the CAN its position plateaus (the can blocks it) and current rises; when it
closes on air the position runs to fully-closed with no plateau. The can's REST position
(what we spawn in sim) is the tool_pose xy at FIRST-CONTACT: the earliest table-level
instant in the carry where the gripper stalls against the can. For drag demos this is
EARLIER than lift-onset (the human closed on the can, dragged it, then lifted) so it
recovers the start position rather than the drag endpoint.

tool_pose is base-frame; base is at world (0,0), z+0.05, so tool xy ~= world xy.

Stage 1 (rosbags venv): python recover_grasp_current.py extract   -> grasp_signals/*.npz
Stage 2 (any):          python recover_grasp_current.py recover    -> grasp_current.json + FK compare
"""
import os
import sys, json, pathlib as pl
import numpy as np

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', '/home/j/workspace/genesis_pickaplace'))
SIG = pl.Path('/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/'
              '5d60af7b-ae54-45fa-bc0c-e90077b3afaf/scratchpad/grasp_signals')
GOAL = (0.6, -0.2)
TABLE_Z = 0.05          # base-frame table height (grasp happens ~0.016-0.04 above it)
GP_CLOSE = 30.0
CURR = 0.45             # current threshold: gripper meeting resistance


def extract(uids):
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    SIG.mkdir(exist_ok=True, parents=True)
    for uid in uids:
        path = REPO / f'inthewild_trials/raw/user_{uid}/trial_data.bag'
        if not path.exists():
            continue
        ts = get_typestore(Stores.ROS1_NOETIC)
        with Reader(str(path)) as r:
            add = {}
            for c in r.connections:
                if 'kortex' in c.msgtype:
                    md = c.msgdef.data if hasattr(c.msgdef, 'data') else c.msgdef
                    add.update(get_types_from_msg(md, c.msgtype))
            ts.register(add)
            rows = []; t0 = None
            for c, t, raw in r.messages():
                if c.topic == '/my_gen3_lite/base_feedback':
                    m = ts.deserialize_ros1(raw, c.msgtype); b = m.base
                    try:
                        mot = m.interconnect.oneof_tool_feedback.gripper_feedback[0].motor[0]
                        if t0 is None: t0 = t / 1e9
                        rows.append((t/1e9 - t0, b.tool_pose_x, b.tool_pose_y, b.tool_pose_z,
                                     mot.position, mot.current_motor))
                    except Exception:
                        pass
        np.save(SIG / f'{uid}.npy', np.array(rows))
        print(f'{uid}: {len(rows)} samples', flush=True)


def intervals(mask, minlen=15):
    out = []; i = 0; n = len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]: j += 1
            if j - i >= minlen: out.append((i, j-1))
            i = j
        else: i += 1
    return out


def first_contact(a):
    """a: (n,6) t,x,y,z,gpos,gcurr. Returns (rest_idx, liftonset_idx) or (None,None)."""
    t, x, y, z, gp, gc = a.T
    closed = gp > GP_CLOSE
    ivs = intervals(closed)
    if not ivs:
        return None, None
    # the carry = closure that lifts (>5cm) and releases near the goal
    def lift(iv): a_, b_ = iv; return float(z[a_:b_+1].max() - z[a_:b_+1].min())
    def rel_goal(iv): a_, b_ = iv; return float(np.hypot(x[b_]-GOAL[0], y[b_]-GOAL[1]))
    cands = [iv for iv in ivs if lift(iv) > 0.05 and rel_goal(iv) < 0.25]
    carry = max(cands, key=lift) if cands else max(ivs, key=lift)
    a0, b0 = carry
    # lift-onset: first sustained rise above table+5cm within the carry
    lo = a0
    for i in range(a0, b0):
        if z[i] < TABLE_Z + 0.02 and z[min(i+15, b0)] > TABLE_Z + 0.05 \
           and np.all(z[i+1:i+6] >= z[i] - 0.002):
            lo = i; break
    # first-contact: earliest table-level index in [a0, lo] where the gripper is closing
    # AND meets resistance (current high) OR position plateaus (can blocks closure)
    seg = slice(a0, max(a0+1, lo+1))
    zi = z[seg]; gci = gc[seg]; gpi = gp[seg]
    idxs = np.arange(a0, max(a0+1, lo+1))
    table = zi < TABLE_Z + 0.03
    # position plateau: pos barely changes over next 5 samples (can blocking)
    plateau = np.array([abs(gpi[min(k+5, len(gpi)-1)] - gpi[k]) < 4 for k in range(len(gpi))])
    resist = gci > CURR
    caught = table & (plateau | resist) & (gpi > GP_CLOSE)
    fc = idxs[np.argmax(caught)] if caught.any() else lo
    return int(fc), int(lo)


def recover():
    fk = json.load(open(REPO/'can_pos_recovery/fk_recovered.json'))
    out = {}; diffs = []; drag = []
    for uid in sorted(int(p.stem) for p in SIG.glob('*.npy')):
        f = SIG / f'{uid}.npy'
        a = np.load(f)
        if len(a) < 20: continue
        fc, lo = first_contact(a)
        if fc is None: continue
        rest = [round(float(a[fc, 1]), 4), round(float(a[fc, 2]), 4)]
        lift_xy = [round(float(a[lo, 1]), 4), round(float(a[lo, 2]), 4)]
        drag_cm = round(float(np.hypot(rest[0]-lift_xy[0], rest[1]-lift_xy[1])*100), 1)
        entry = dict(rest_xy=rest, liftonset_xy=lift_xy, drag_cm=drag_cm,
                     rest_t=round(float(a[fc, 0]), 2), grasp_z=round(float(a[fc, 3]), 4))
        out[str(uid)] = entry
        v = fk.get(str(uid))
        if v and v['label'] == 'success' and v['conf'] in ('HIGH', 'MED'):
            d = np.hypot(rest[0]-v['can_xy'][0], rest[1]-v['can_xy'][1])*100
            diffs.append((uid, round(d, 1), drag_cm))
            if drag_cm > 3: drag.append((uid, drag_cm))
    (REPO/'can_pos_recovery/grasp_current.json').write_text(json.dumps(out, indent=1))
    arr = np.array([d[1] for d in diffs])
    print(f"recovered {len(out)} trials -> grasp_current.json")
    print(f"first-contact rest_xy vs FK can_xy (n={len(diffs)}): "
          f"median {np.median(arr):.1f}cm p90 {np.percentile(arr,90):.1f}cm "
          f"within2 {(arr<2).sum()}/{len(arr)} within3 {(arr<3).sum()}/{len(arr)}")
    print(f"drag demos (rest->lift >3cm): {len(drag)}: {sorted(drag, key=lambda x:-x[1])[:8]}")


if __name__ == '__main__':
    if sys.argv[1] == 'extract':
        uids = sys.argv[2:] or [p.name.split('_')[1] for p in
                                (REPO/'inthewild_trials/raw').glob('user_*')]
        extract(uids)
    else:
        recover()
