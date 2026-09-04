#!/usr/bin/env python3
"""PHASE PLAN 2026-09-04: DP-only pre-pick idle pruning for CONTRACT-V1 (decision-rate) full-scope tapes, layout-preserving.
Same rule as make_dp_pruned.py (never touch anything from can-interaction on): j_pick = first decision with can_z > 0.09
AND grip closed; within [0, j_pick - margin) collapse each run of consecutive idle decisions (max |a[t+1]-a[t]| < idle_eps)
to its first decision; [j_pick - margin, end] untouched. Every per-decision array is masked identically (states, actions,
actions_delta, rewards, terminated, truncated, picked, placed, contact, nested, tipped; eef_pos keeps its final row;
sim_states/sim_actions are masked per sim step x action_repeat); scalars pass through, n is updated, prune stats stamped.
usage: prune_full_v1.py --src <dir> --dst <dir> [--margin 38] [--idle-eps 1e-3]   (38 decisions ~ 5 s at 7.5 Hz)"""
import argparse, glob, json, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import pick_env
ap = argparse.ArgumentParser(); ap.add_argument('--src', required=True); ap.add_argument('--dst', required=True)
ap.add_argument('--margin', type=int, default=38); ap.add_argument('--idle-eps', type=float, default=1e-3)
a_ = ap.parse_args(); os.makedirs(a_.dst, exist_ok=True)
tot_in = tot_out = 0; stats = {}
for f in sorted(glob.glob(os.path.join(a_.src, '*.npz'))):
    z = np.load(f, allow_pickle=True); d = {k: z[k] for k in z.files}
    n = int(d['n']); s = d['states']; a = d['actions']; rep = int(d['action_repeat'])
    assert s.shape[0] == n and a.shape[0] == n, (f, s.shape, a.shape, n)
    picked_f = (s[:, 10] > 0.09) & (a[:, 6] > pick_env.GRIP_CLOSED_FRAC)
    j = int(np.argmax(picked_f)) if picked_f.any() else n
    cut = max(j - a_.margin, 0)
    keep = np.ones(n, bool)
    if cut > 1:
        idle = np.max(np.abs(a[1:cut] - a[:cut - 1]), axis=1) < a_.idle_eps      # idle[t] -> decision t+1 repeats t
        run = False
        for t in range(1, cut):
            if idle[t - 1]:
                if run: keep[t] = False
                run = True
            else:
                run = False
    m = int(keep.sum()); tot_in += n; tot_out += m
    out = {}
    for k, v in d.items():
        v = np.asarray(v)
        if v.ndim >= 1 and v.shape[0] == n: out[k] = v[keep]
        elif v.ndim >= 1 and v.shape[0] == n + 1: out[k] = np.concatenate([v[:n][keep], v[n:]])
        elif v.ndim >= 1 and v.shape[0] == n * rep: out[k] = v.reshape(n, rep, *v.shape[1:])[keep].reshape(-1, *v.shape[1:])
        else: out[k] = v
    out['n'] = np.int64(m); out['prune_rule'] = 'pre-pick idle collapse (prune_full_v1.py)'; out['prune_margin'] = np.int64(a_.margin)
    out['prune_j_pick'] = np.int64(j); out['prune_kept'] = np.int64(m); out['prune_orig_n'] = np.int64(n)
    np.savez_compressed(os.path.join(a_.dst, os.path.basename(f)), **out)
    stats[os.path.basename(f)] = dict(n=n, kept=m, j_pick=j)
print(f'[prune_full_v1] {len(stats)} tapes: {tot_in} -> {tot_out} decisions ({100*(1-tot_out/max(1,tot_in)):.1f}% dropped, all pre-pick)')
json.dump(dict(src=os.path.abspath(a_.src), margin=a_.margin, idle_eps=a_.idle_eps, tapes=stats, total_in=tot_in, total_out=tot_out),
          open(os.path.join(a_.dst, 'prune_manifest.json'), 'w'), indent=1)
