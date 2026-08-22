#!/usr/bin/env python
"""Build the dDPsucc demo set (and, optionally, the tip-truncated dDP variant) from
baselines/m1all_harvest -- the DP-teacher-harvested model-demo set.

WHY (2026-08-22, paper/ROUND_ROBIN_RESULTS_2026-08-22.md "Why dDP_RLPD < dH_RLPD"):
dDP_RLPD ignited 0/6 while dH 8/16 and dR2D 10/16. The dDP stream's 30 FAIL tapes
(kept whole at the 1200-step cap = 51% of the buffer) are off-manifold: 8/30 tip the
can; ~10% of the buffer carries a tipped can (6% strictly post-termination: tilt>60°
with grip open) at which FullTaskEnv(pick) would have TERMINATED online. The RLPD critic diverges on that stream from ~25k steps
(actor_q 32 vs 0.5 for dH/dR2D) before any online pick. Two cheap discriminators:

  --mode succ      -> baselines/m1all_harvest_succ      (63 success tapes only; fails dropped)
  --mode r2dfails  -> baselines/episodes_champion_pick_plus_dpfails  (the 66 dR2D champion success
                      tapes from --src2 + the 30 DP FAIL tapes: the SUFFICIENCY arm -- if RLPD on the
                      best-ignition set stops igniting once the DP fails are added, the fails are
                      sufficient; design audit 2026-08-22 issue 6)
  --mode tiptrunc  -> baselines/m1all_harvest_tiptrunc  (successes intact; each FAIL tape cut at
                      the first frame the env's tip rule would have fired, i.e. tilt > TIP_DEG
                      while the commanded grip is open; fails that never tip are kept whole; fails whose can is
                      already lying at frame 0 are dropped -- no transition survives)

All modes are plain copies of the source npz (same keys, same uids, same teacher), so every
downstream gate (filename pattern ^1[0-9]{5}\\.npz$ for succ; ^[0-9]{6}\\.npz$ for tiptrunc;
^[15][0-9]{5}\\.npz$ for r2dfails; RUN_REGISTRY demo fingerprint) works unchanged; a manifest.json
records provenance. KNOWN LIMITATION (impl audit 2026-08-22 F5): tiptrunc removes the post-
termination chain but the encoder (delta_encode_transitions) still emits the cut tape's last
transition with done=False, i.e. one dangling bootstrap per truncated tape remains; a proper
env-terminal guard in the encoder is the real fix and is a trainer change (registered follow-up).

Usage (cluster login node, repo root; m1all_harvest is already rsynced there):
  python baselines/make_dDPsucc.py --mode succ
  python baselines/make_dDPsucc.py --mode tiptrunc
  python baselines/make_dDPsucc.py --mode r2dfails --src2 baselines/episodes_champion_pick
  python baselines/make_dDPsucc.py --mode succ --src baselines/m1all_harvest --dst /tmp/x --dry-run
Then: for S in 0 1 2 3 4 5; do ARM=dDPsucc SEED=$S sbatch cluster/sbatch_rlpd.sh; done

Stdlib + numpy only. Never modifies the source directory.
"""
import argparse, glob, hashlib, json, os, shutil, sys, time
import numpy as np

# FullTaskEnv pick-scope tip rule (baselines/rl/full_env.py): terminate when the can's
# tilt exceeds TIP_DEG while the commanded grip is below GRIP_OPEN (can lying FREE).
TIP_DEG = 60.0
GRIP_OPEN = 0.3
# 17-dim state layout used by every RLPD/DP tape in this project
# (paper/ROUND_ROBIN_RUNNING_2026-08-19.md): 0:6 joints, 6 grip, 7 grip effort,
# 8:11 can xyz, 11:15 can quat (w,x,y,z), 15:17 goal xy.
CAN_QUAT = slice(11, 15)


def tilt_deg(q):
    """Angle between the can's own z axis and world z, quat (w,x,y,z) as Genesis returns it."""
    w, x, y, z = [float(v) for v in q]
    zz = 1.0 - 2.0 * (x * x + y * y)
    zx = 2.0 * (x * z + w * y)
    zy = 2.0 * (y * z - w * x)
    return float(np.degrees(np.arccos(np.clip(zz / (np.sqrt(zx * zx + zy * zy + zz * zz) + 1e-9), -1.0, 1.0))))


def first_tip_frame(states, actions):
    """Index of the first frame the env's tip rule would fire, or None."""
    grip_cmd = actions[:, 6]
    for i in range(len(states)):
        if grip_cmd[i] < GRIP_OPEN and tilt_deg(states[i, CAN_QUAT]) > TIP_DEG:
            return i
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['succ', 'tiptrunc', 'r2dfails'], required=True)
    ap.add_argument('--src', default='baselines/m1all_harvest')
    ap.add_argument('--src2', default='baselines/episodes_champion_pick', help='r2dfails: the dR2D success set to inject the DP fails into')
    ap.add_argument('--dst', default=None, help='default: baselines/m1all_harvest_succ | _tiptrunc | episodes_champion_pick_plus_dpfails')
    ap.add_argument('--dry-run', action='store_true', help='report what would be written; touch nothing')
    ap.add_argument('--force', action='store_true', help='overwrite an existing --dst')
    args = ap.parse_args()
    dst = args.dst or {'succ': 'baselines/m1all_harvest_succ', 'tiptrunc': 'baselines/m1all_harvest_tiptrunc',
                       'r2dfails': 'baselines/episodes_champion_pick_plus_dpfails'}[args.mode]

    files = sorted(glob.glob(os.path.join(args.src, '*.npz')))
    if not files:
        sys.exit(f'FATAL: no npz in {args.src}')
    if os.path.exists(dst) and not args.force and not args.dry_run:
        sys.exit(f'FATAL: {dst} exists (use --force to rebuild)')

    plan = []  # (src_path, dst_name, action, n_in, n_out, note)
    for f in files:
        z = np.load(f, allow_pickle=True)
        label = str(z['label'].item()) if 'label' in z.files else '?'
        stage = str(z['stage'].item()) if 'stage' in z.files else '?'
        name = os.path.basename(f)
        stem_succ = name.startswith('1')  # sbatch_dp.sh convention: success stems 1xxxxx, fails 5xxxxx
        is_succ = (label == 'success') and (stage == 'picked')
        if stem_succ != is_succ:
            sys.exit(f'FATAL: {name}: stem says success={stem_succ} but label={label} stage={stage} -- '
                     f'provenance convention broken; refusing to build')
        n = int(len(z['actions']))
        if args.mode == 'succ':
            if is_succ:
                plan.append((f, name, 'copy', n, n, 'success'))
            else:
                plan.append((f, name, 'drop', n, 0, 'fail dropped'))
        elif args.mode == 'r2dfails':
            if is_succ:
                plan.append((f, name, 'drop', n, 0, 'DP success dropped (r2dfails keeps only the DP fails)'))
            else:
                plan.append((f, name, 'copy', n, n, 'DP fail injected'))
        else:  # tiptrunc
            if is_succ:
                plan.append((f, name, 'copy', n, n, 'success'))
            else:
                k = first_tip_frame(z['states'], z['actions'])
                if k is None:
                    plan.append((f, name, 'copy', n, n, 'fail, never tipped: kept whole'))
                elif k + 1 < 2:
                    # can already lying at the IC (tip rule fires at frame 0): the env would
                    # terminate at reset; a 1-frame tape has no transition -> drop it
                    plan.append((f, name, 'drop', n, 0, 'fail, can tipped at frame 0 (lying-can IC): dropped'))
                else:
                    plan.append((f, name, 'truncate', n, k + 1, f'fail, tip rule fires at frame {k}: kept [0:{k+1}]'))

    if args.mode == 'r2dfails':
        f2 = sorted(glob.glob(os.path.join(args.src2, '*.npz')))
        if not f2:
            sys.exit(f'FATAL: no npz in --src2 {args.src2} (the dR2D champion set)')
        for f in f2:
            z = np.load(f, allow_pickle=True); name = os.path.basename(f)
            if not name.startswith('1'):
                sys.exit(f'FATAL: {name} in {args.src2} does not look like a champion success stem (1xxxxx)')
            lab = str(z['label'].item()) if 'label' in z.files else '?'
            if lab != 'success':
                sys.exit(f'FATAL: {name} in {args.src2} has label={lab}; the dR2D set must be all-success')
            plan.append((f, name, 'copy', int(len(z['actions'])), int(len(z['actions'])), 'dR2D champion success'))
        names = [p[1] for p in plan if p[2] != 'drop']
        if len(names) != len(set(names)):
            sys.exit('FATAL: filename collision between the DP fails and the champion set')
    kept = [p for p in plan if p[2] != 'drop']
    print(f'[{args.mode}] src={args.src} ({len(files)} npz)' + (f' + src2={args.src2}' if args.mode == 'r2dfails' else '') + f' -> dst={dst}')
    print(f'  keep {len(kept)} tapes, {sum(p[4] for p in kept)} frames '
          f'(source {sum(p[3] for p in plan)} frames); dropped {sum(p[2]=="drop" for p in plan)}; '
          f'truncated {sum(p[2]=="truncate" for p in plan)}')
    for p in plan:
        if p[2] != 'copy':
            print(f'    {p[1]}: {p[2]:8s} {p[3]:5d} -> {p[4]:5d}  {p[5]}')
    if args.dry_run:
        print('[dry-run] nothing written')
        return

    if os.path.exists(dst):
        shutil.rmtree(dst)
    os.makedirs(dst)
    h = hashlib.sha256()
    for src_path, name, action, n_in, n_out, note in plan:
        if action == 'drop':
            continue
        out = os.path.join(dst, name)
        if action == 'copy':
            shutil.copy2(src_path, out)
        else:
            z = np.load(src_path, allow_pickle=True)
            d = {}
            for k in z.files:
                v = z[k]
                if v.ndim >= 1 and v.shape[0] == n_in:
                    v = v[:n_out]
                d[k] = v
            if 'n' in d:
                d['n'] = np.array(n_out)
            np.savez_compressed(out, **d)
        with open(out, 'rb') as fh:
            h.update(name.encode()); h.update(fh.read())
    manifest = {
        'built': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'mode': args.mode,
        'source': os.path.normpath(args.src),
        'source_name': 'm1all_harvest',
        'rule': {'succ': 'success tapes only (label==success and stage==picked; stems 1xxxxx)',
                 'tiptrunc': f'successes intact; fail tapes truncated at first frame with tilt>{TIP_DEG} and grip cmd<{GRIP_OPEN}',
                 'r2dfails': f'all tapes of {args.src2} (dR2D champion successes) + the DP FAIL tapes (stems 5xxxxx) of {args.src}'}[args.mode],
        'source2': (os.path.normpath(args.src2) if args.mode == 'r2dfails' else None),
        'n_source': len(files), 'n_kept': len(kept),
        'frames_source': int(sum(p[3] for p in plan)), 'frames_kept': int(sum(p[4] for p in kept)),
        'per_tape': [{'name': p[1], 'action': p[2], 'n_in': p[3], 'n_out': p[4], 'note': p[5]} for p in plan],
        'content_sha256': h.hexdigest(),
        'builder': 'baselines/make_dDPsucc.py',
    }
    src_manifest = os.path.join(args.src, 'manifest.json')
    if os.path.exists(src_manifest):
        manifest['source_manifest'] = json.load(open(src_manifest))
    json.dump(manifest, open(os.path.join(dst, 'manifest.json'), 'w'), indent=1)
    print(f'  wrote {dst}/manifest.json  content_sha256={h.hexdigest()[:16]}...')


if __name__ == '__main__':
    main()
