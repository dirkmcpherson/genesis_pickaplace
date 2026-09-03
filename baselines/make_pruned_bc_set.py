#!/usr/bin/env python
"""Action-density control for the BC rows: prune near-zero-action DECISIONS from contract-v1 tapes.

WHY (user 08-20, restated 08-25; PAPER_NOTES N3/N7): the 08-19 dR2D_DP > dH_DP gap was confounded
with idle-frame density -- model teachers never dither, humans do. The control asks whether making
human demos artificially dense HELPS (density was the advantage) or HURTS (the still frames carry
task signal). Measured on the corrected-world sets: dH has 20.0% of decisions with |a_arm|inf <
1e-3 vs dDP 0.0%; 55% of those dH decisions have the grip commanded CLOSED (vs 31% overall), i.e.
most "idle" is the hold that seats the grasp -- which the hardened pick predicate requires.

BC ONLY. Deleting decisions breaks the (s_t, a_t, s_t+1) chain, so the output is NOT a valid
trajectory set: never feed it to RLPD or a world model. The manifest records bc_only=True and the
sbatch native-arm gates should refuse it for anything but DP (they check contract/sha, not this --
so the operator must not point an RLPD arm at it).

Usage (cluster, after the corrected-world sets exist):
    python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dH --dst baselines/matched_w3/dHallpruned_e3 --eps 1e-3
    python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dH --dst baselines/matched_w3/dHallpruned_e2 --eps 1e-2
    # symmetry: run the IDENTICAL rule on dDP (a near no-op -- that asymmetry IS the finding)
    python baselines/make_pruned_bc_set.py --src baselines/matched_w3/dDP --dst baselines/matched_w3/dDPallpruned_e3 --eps 1e-3
    then convert_to_lerobot.py <dst> <dst>/lerobot 8 4 none image  and run sbatch_dp.sh with DEMO_ROOT/ARM pointing at it.
"""
import argparse, glob, hashlib, json, os, shutil, sys, time
import numpy as np

def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--src', required=True); ap.add_argument('--dst', required=True)
    ap.add_argument('--eps', type=float, default=1e-3, help='prune decisions with max|a_arm| < eps (normalized units; 1e-3 = 2.5e-5 rad/sim-step)')
    ap.add_argument('--grip-eps', type=float, default=5e-3, help='a decision whose grip command moved by more than this is NEVER pruned (the closure itself)')
    ap.add_argument('--keep-terminal', action='store_true', default=True, help='never prune the terminal decision (the pick)')
    ap.add_argument('--dry-run', action='store_true'); ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    files = sorted(glob.glob(os.path.join(a.src, '*.npz')))
    if not files: sys.exit(f'FATAL: no npz in {a.src}')
    if os.path.exists(a.dst) and not a.force and not a.dry_run: sys.exit(f'FATAL: {a.dst} exists (--force to rebuild)')
    rows, n_in, n_out, n_grip = 0, 0, 0, 0
    plan = []
    for f in files:
        z = dict(np.load(f, allow_pickle=True))
        ad = z['actions_delta']; n = len(ad)
        # The grip column is an ABSOLUTE command in [-1,1], not a delta: |a_grip| ~ 0 means
        # "half open", NOT "no change". A decision where the arm holds still while the gripper is
        # CLOSING is the grasp itself and must never be pruned -- so a decision is idle only if the
        # arm delta is below eps AND the grip command is unchanged from the previous decision.
        grip = ad[:, 6]
        grip_changed = np.concatenate([[True], np.abs(np.diff(grip)) > a.grip_eps])
        keep = (np.abs(ad[:, :6]).max(1) >= a.eps) | grip_changed
        keep[-1] = True                      # the terminal decision always survives
        n_in += n; n_out += int(keep.sum()); rows += 1
        n_grip += int(((~keep) & (ad[:, 6] > 0)).sum())   # pruned decisions holding a CLOSED grip
        plan.append((f, z, keep))
    print(f'[prune] {a.src} eps={a.eps}: {rows} tapes, {n_in} -> {n_out} decisions ({1-n_out/n_in:.1%} removed; '
          f'{n_grip} of the removed had grip commanded closed)')
    if a.dry_run: return
    if os.path.exists(a.dst): shutil.rmtree(a.dst)
    os.makedirs(a.dst)
    h = hashlib.sha256()
    for f, z, keep in plan:
        out = dict(z); n = len(z['actions_delta'])
        for k, v in z.items():
            v = np.asarray(v)
            if v.ndim >= 1 and v.shape[0] == n:            # per-decision arrays
                out[k] = v[keep]
            elif v.ndim >= 1 and v.shape[0] == n + 1:      # per-decision + terminal (images, eef_pos)
                out[k] = v[np.concatenate([keep, [True]])]
            elif k in ('sim_states', 'sim_actions'):
                # the per-sim-step sub-tape indexes DECISIONS x repeat; carrying it through unpruned
                # would leave a tape whose sub-tape disagrees with its decisions (a trap for any later
                # stride-1 derivation) -- drop it, this set is BC-only anyway (audit W-3)
                out.pop(k, None)
        out['n'] = np.array(int(keep.sum()))
        name = os.path.basename(f); np.savez_compressed(os.path.join(a.dst, name), **out)
        with open(os.path.join(a.dst, name), 'rb') as fh: h.update(name.encode()); h.update(fh.read())
    src_man = os.path.join(a.src, 'manifest.json')
    man = dict(set=os.path.basename(a.dst), built=time.strftime('%Y-%m-%dT%H:%M:%S'), N=rows,
               n_kept=rows, contract='v1', bc_only=True, prune_eps=a.eps,
               prune_grip_eps=a.grip_eps, decisions_in=n_in, decisions_out=n_out, removed_frac=round(1 - n_out / n_in, 4),
               removed_with_grip_closed=n_grip, source=os.path.normpath(a.src),
               sim_variant=(json.load(open(src_man)).get('sim_variant') if os.path.exists(src_man) else None),
               content_sha256=h.hexdigest(), builder='baselines/make_pruned_bc_set.py',
               WARNING='BC ONLY -- decisions deleted, (s,a,s\') chain broken; never use for RLPD/WM')
    json.dump(man, open(os.path.join(a.dst, 'manifest.json'), 'w'), indent=1)
    print(f'  wrote {a.dst}  sha {h.hexdigest()[:16]}  bc_only=True')

if __name__ == '__main__':
    main()
