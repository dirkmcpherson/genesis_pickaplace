"""Relabel per-frame demonstration reward under the settled predicates (amendment x).

WHY. The tapes carry rewards from the OLD ladder: it paid its contact rung on bare
`contact`, which counts carrying the can in and parking it (84-86% of policy grants,
12 of 26 human ones), and its top rung on the `nested` TRAINING PROXY, which
over-counts the settled predicate ~2.5x. RLPD feeds these rewards straight into its
demonstration buffer, so leaving them stale hands it one objective from the demos and
a different one from the env. (DP never reads reward and is unaffected.)

NEW LADDER -- same magnitudes, corrected predicates, each granted ONCE at the first
frame it fires:
  picked      +1  env grant (unchanged)
  released    +1  the can stays put while the tool moves away  (replaces stale `placed`,
                  which is never granted, and needs no gripper term)
  can-contact +2  env `contact` AFTER release  (requiring release first excludes the
                  carry-in route without needing the tool point, whose wrist-vs-tool
                  confusion made the old geometric clause vacuous)
  can-settle  +4  first frame the can is within nested proximity after release, and
                  the episode ends settled and upright  (replaces the nested proxy)

Trajectories are NOT modified. Only the `rewards` column is rewritten, into a NEW
directory; originals are never touched.

usage: relabel_reward.py <in_dir> <out_dir>
"""
import os, sys, glob, json
import numpy as np

NESTED_M, STILL_M, TOOL_AWAY_M, HOLD_N = 0.081, 0.002, 0.010, 10
R = {"picked": 1.0, "released": 1.0, "can_contact": 2.0, "can_settle": 4.0}


def release_frame(can, ee, tipped):
    T = len(can)
    for t in range(T - HOLD_N):
        w = slice(t, t + HOLD_N)
        if np.max(np.linalg.norm(can[w] - can[t], axis=1)) > STILL_M:
            continue
        if np.max(np.linalg.norm(ee[w] - ee[t], axis=1)) < TOOL_AWAY_M:
            continue
        if tipped[t]:
            continue
        return t
    return None


def relabel(f, out_dir):
    z = np.load(f, allow_pickle=True)
    d = {k: z[k] for k in z.files}
    s = z["states"].astype(np.float64)
    can, goal = s[:, 8:11], s[:, 15:17]
    ee = z["eef_pos"].astype(np.float64)[: len(s)]
    tipped = z["tipped"].astype(bool) if "tipped" in z.files else np.zeros(len(s), bool)
    picked = z["picked"].astype(bool)
    contact = z["contact"].astype(bool)
    T = len(s)
    rew = np.zeros(T, dtype=np.float32)
    grants = {}

    idx = np.flatnonzero(picked)
    if idx.size:
        rew[idx[0]] += R["picked"]; grants["picked"] = int(idx[0])

    rf = release_frame(can, ee, tipped)
    if rf is not None:
        rew[rf] += R["released"]; grants["released"] = rf
        cidx = np.flatnonzero(contact[rf:])
        if cidx.size:
            t = rf + int(cidx[0])
            rew[t] += R["can_contact"]; grants["can_contact"] = t
        dist = np.linalg.norm(can[:, :2] - goal, axis=1)
        near = np.flatnonzero(dist[rf:] <= NESTED_M)
        settled = bool(dist[-1] <= NESTED_M and not tipped[-1])
        if near.size and settled:
            t = rf + int(near[0])
            rew[t] += R["can_settle"]; grants["can_settle"] = t

    d["rewards"] = rew
    d["reward_ladder"] = json.dumps(R)
    d["reward_grants"] = json.dumps(grants)
    d["reward_relabelled"] = "amendment_x_2026-09-09"
    out = os.path.join(out_dir, os.path.basename(f))
    np.savez_compressed(out, **d)
    return float(z["rewards"].sum()) if "rewards" in z.files else 0.0, float(rew.sum()), grants


if __name__ == "__main__":
    ind, outd = sys.argv[1], sys.argv[2]
    os.makedirs(outd, exist_ok=True)
    files = sorted(glob.glob(os.path.join(ind, "*.npz")))
    old_tot = new_tot = 0.0
    counts = {k: 0 for k in R}
    for f in files:
        o, n, g = relabel(f, outd)
        old_tot += o; new_tot += n
        for k in g:
            counts[k] += 1
    print("tapes            :", len(files))
    print("OLD reward sum   :", round(old_tot, 1))
    print("NEW reward sum   :", round(new_tot, 1))
    print("tapes granting   :", counts)
    print("wrote            :", outd)
