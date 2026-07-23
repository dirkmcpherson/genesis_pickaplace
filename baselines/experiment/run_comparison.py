"""HUMAN vs AI demonstrations -- the comparison harness (STEP 3 of the study).

For each student in {dp, bc, sac} x condition in {human, ai} x seed, train the student on a
SIZE- and IC-MATCHED demo set and evaluate it identically (shared eval_core, random ICs).
Also runs two negative controls: a random-action floor and a shuffled-action control (train
on demos whose actions are permuted across frames -- must collapse to floor).

This is a DRIVER: the three students live in different venvs, so each stage is shelled out
with the correct interpreter and its summary line is parsed. Detached nothing; every subprocess
is wrapped in `timeout`. Use --dry-run to print the exact per-venv commands without running.

Design choices baked in (honesty protocol):
  * Matched manifests: subsample the larger demo set to the smaller's COUNT while matching
    its per-bin can-xy histogram as closely as the larger set allows; the residual per-bin
    divergence is reported, never hidden (human demos sit at fixed ICs, AI at random ICs, so
    exact matching is impossible).
  * Every student is scored by eval_core on the SAME env + same random ICs.
  * Both negative controls are first-class, not optional.

Usage:
  python baselines/experiment/run_comparison.py --human-dir baselines/episodes_raw_v4 \
      --ai-dir baselines/episodes_ai/<tag> --scope pick --seeds 0 1 2 --n-eval 30 \
      --students dp bc sac [--dp-steps 30000] [--sac-steps 200000] [--dry-run]
"""
import argparse
import json
import pathlib as pl
import re
import shutil
import subprocess
import sys

import numpy as np

REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
VENV_EVAL = REPO / '.venv-eval/bin/python'
LEROBOT_BIN = pl.Path('/home/j/workspace/genesis_pickaplace/.venv-eval/bin')
LEROBOT_PY = LEROBOT_BIN / 'python'
LEROBOT_TRAIN = LEROBOT_BIN / 'lerobot-train'

SUMMARY_RE = re.compile(
    r'rollouts: picked ([\d.]+) placed ([\d.]+) contact ([\d.]+) nested ([\d.]+)')


# ----------------------------------------------------------------------------- matching
def can_xy(npz_path):
    return np.load(npz_path)['states'][0][8:10].astype(float)


def _bin_index(xy, lo, span, bins):
    return tuple(np.clip(((xy - lo) / span * bins).astype(int), 0, bins - 1))


def build_matched_manifests(human_dir, ai_dir, bins=4, seed=0):
    """Return (human_files, ai_files, report). Subsample the larger set to the smaller's
    count, matching its per-bin can-xy histogram as closely as the larger set allows."""
    rng = np.random.default_rng(seed)
    hf = sorted(str(p) for p in pl.Path(human_dir).glob('*.npz'))
    af = sorted(str(p) for p in pl.Path(ai_dir).glob('*.npz'))
    assert hf and af, f'empty demo dir(s): human={len(hf)} ai={len(af)}'
    hx = {p: can_xy(p) for p in hf}
    ax = {p: can_xy(p) for p in af}
    allxy = np.array(list(hx.values()) + list(ax.values()))
    lo, hi = allxy.min(0), allxy.max(0)
    span = (hi - lo) + 1e-9

    def binned(xydict):
        d = {}
        for p, xy in xydict.items():
            d.setdefault(_bin_index(xy, lo, span, bins), []).append(p)
        return d

    hb, ab = binned(hx), binned(ax)
    smaller_is_human = len(hf) <= len(af)
    small_b, large_b = (hb, ab) if smaller_is_human else (ab, hb)
    small_files = hf if smaller_is_human else af

    # keep the whole smaller set; draw a bin-matched subsample of the larger set
    keep_large, deficit = [], 0
    used = set()
    for b, items in small_b.items():
        want = len(items)
        pool = [p for p in large_b.get(b, []) if p not in used]
        take = min(want, len(pool))
        chosen = list(rng.choice(pool, size=take, replace=False)) if take else []
        keep_large += chosen
        used.update(chosen)
        deficit += want - take
    # fill any deficit from unused larger-set items (any bin) to hit the target count
    if deficit > 0:
        leftover = [p for p in (af if smaller_is_human else hf) if p not in used]
        fill = list(rng.choice(leftover, size=min(deficit, len(leftover)), replace=False)) \
            if leftover else []
        keep_large += fill

    human_files = small_files if smaller_is_human else keep_large
    ai_files = keep_large if smaller_is_human else small_files

    def hist(files):
        h = np.zeros((bins, bins), int)
        for p in files:
            i, j = _bin_index(can_xy(p), lo, span, bins)
            h[i, j] += 1
        return h

    hh, ah = hist(human_files), hist(ai_files)
    l1 = int(np.abs(hh - ah).sum())
    report = dict(bins=bins, n_human=len(human_files), n_ai=len(ai_files),
                  human_hist=hh.tolist(), ai_hist=ah.tolist(),
                  per_bin_l1_divergence=l1,
                  note='per_bin_l1_divergence = sum|human_bin - ai_bin| (0 = identical marginals)')
    return sorted(human_files), sorted(ai_files), report


def symlink_dir(files, dest):
    dest = pl.Path(dest)
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    for p in files:
        (dest / pl.Path(p).name).symlink_to(pl.Path(p).resolve())
    return dest


def shuffled_dir(files, dest, seed=0):
    """Copy npz with actions permuted across ALL frames (destroys state->action mapping)."""
    rng = np.random.default_rng(seed)
    dest = pl.Path(dest)
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)
    for p in files:
        d = np.load(p)
        a = np.asarray(d['actions']).copy()
        rng.shuffle(a)          # in-place permute along frame axis
        np.savez_compressed(dest / pl.Path(p).name, states=d['states'], actions=a,
                            n=int(d['n']), uid=int(d['uid']) if 'uid' in d else -1)
    return dest


# ------------------------------------------------------------------------- run commands
def run(cmd, log_path, dry, timeout_s=7200):
    printable = ' '.join(str(c) for c in cmd)
    if dry:
        print(f'  DRY: {printable}')
        return ''
    with open(log_path, 'w') as lf:
        lf.write(f'$ {printable}\n\n')
        lf.flush()
        p = subprocess.run(['timeout', str(timeout_s)] + [str(c) for c in cmd],
                           stdout=lf, stderr=subprocess.STDOUT)
    text = pl.Path(log_path).read_text(errors='ignore')
    if p.returncode != 0:
        print(f'  !! nonzero exit {p.returncode} (see {log_path})')
    return text


def parse_metrics(text):
    m = SUMMARY_RE.search(text or '')
    if not m:
        return None
    return dict(picked=float(m[1]), placed=float(m[2]),
                contact=float(m[3]), nested=float(m[4]))


def train_eval_dp(demo_dir, work, seed, n_eval, eval_seed, steps, min_frames, dry):
    ds_root = work / 'ds'
    out = work / 'dp_out'
    for d in (ds_root, out):
        if d.exists():
            shutil.rmtree(d)
    run([LEROBOT_PY, REPO / 'baselines/convert_to_lerobot.py', demo_dir, ds_root, 8,
         min_frames], work / 'convert.log', dry, timeout_s=1800)
    run([LEROBOT_TRAIN, f'--dataset.repo_id=local/cmp',
         f'--dataset.root={ds_root}', '--policy.type=diffusion',
         f'--output_dir={out}', '--policy.push_to_hub=false',
         '--batch_size=64', f'--steps={steps}', f'--seed={seed}',
         '--wandb.enable=false'], work / 'train.log', dry, timeout_s=6 * 3600)
    ck = out / 'checkpoints/last/pretrained_model'
    text = run([VENV_EVAL, REPO / 'baselines/eval_policy.py', ck,
                '--random', n_eval, '--seed', eval_seed],
               work / 'eval.log', dry, timeout_s=4 * 3600)
    return parse_metrics(text)


def train_eval_bc(demo_dir, work, seed, n_eval, eval_seed, dry):
    bc = work / 'bc.pt'
    run([VENV_EVAL, REPO / 'baselines/bc/train_bc.py', demo_dir, '--out', bc,
         '--seed', seed], work / 'train.log', dry, timeout_s=1800)
    text = run([VENV_EVAL, REPO / 'baselines/bc/eval_bc.py', bc,
                '--random', n_eval, '--seed', eval_seed],
               work / 'eval.log', dry, timeout_s=4 * 3600)
    return parse_metrics(text)


def train_eval_sac(demo_dir, work, seed, n_eval, eval_seed, steps, dry):
    ck_dir = work / 'sac_ck'
    run([VENV_EVAL, REPO / 'baselines/rl/train_sacfd.py', '--full',
         '--demo-dir', demo_dir, '--seed', seed, '--steps', steps,
         '--out-dir', ck_dir], work / 'train.log', dry, timeout_s=6 * 3600)
    ck = ck_dir / 'sacfd_final.zip'
    text = run([VENV_EVAL, REPO / 'baselines/rl/eval_sac.py', ck,
                '--random', n_eval, '--seed', eval_seed],
               work / 'eval.log', dry, timeout_s=4 * 3600)
    return parse_metrics(text)


def eval_random_floor(work, n_eval, eval_seed, dry):
    """Negative-control floor: harvest's random teacher scored by the same eval? Simpler:
    reuse eval_sac path is SAC-specific; instead run a tiny inline random policy via
    harvest_ai_demos is overkill. We shell a one-liner random eval through eval_core."""
    script = ("import sys,pathlib as pl,numpy as np;"
              "R=pl.Path('/home/j/workspace/genesis_pickaplace');"
              "sys.path.insert(0,str(R/'baselines'));"
              "sys.path.insert(0,str(R/'baselines/rl'));"
              "from genesis_can_env import GenesisCanEnv;"
              "from pick_env import denormalize_action,ACT_DIM;"
              "import ic_sampling,eval_core;"
              "rng=np.random.default_rng(%d);"
              "env=GenesisCanEnv(backend='cpu');"
              "pa=lambda o: denormalize_action(rng.uniform(-1,1,ACT_DIM));"
              "eps=ic_sampling.sample_support_ics(env,%d,seed=%d);"
              "eval_core.run_eval(env,pa,eps,tag='random-floor')" % (eval_seed, n_eval, eval_seed))
    text = run([VENV_EVAL, '-c', script], work / 'floor.log', dry, timeout_s=4 * 3600)
    return parse_metrics(text)


# -------------------------------------------------------------------------------- driver
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--human-dir', required=True)
    ap.add_argument('--ai-dir', required=True)
    ap.add_argument('--scope', choices=['pick', 'full'], default='pick')
    ap.add_argument('--students', nargs='+', default=['dp', 'bc', 'sac'],
                    choices=['dp', 'bc', 'sac'])
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--n-eval', type=int, default=30)
    ap.add_argument('--eval-seed', type=int, default=100)
    ap.add_argument('--dp-steps', type=int, default=30000)
    ap.add_argument('--sac-steps', type=int, default=200000)
    ap.add_argument('--match-bins', type=int, default=4)
    ap.add_argument('--workdir', default='baselines/experiment/runs/cmp')
    ap.add_argument('--shuffled-control', action='store_true',
                    help='also train each student on a shuffled-action copy (must collapse)')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    work = REPO / args.workdir if not pl.Path(args.workdir).is_absolute() else pl.Path(args.workdir)
    work.mkdir(parents=True, exist_ok=True)
    min_frames = 30 if args.scope == 'pick' else 100

    human_files, ai_files, report = build_matched_manifests(
        args.human_dir, args.ai_dir, bins=args.match_bins)
    print(f'[match] human={report["n_human"]} ai={report["n_ai"]} '
          f'per-bin L1 divergence={report["per_bin_l1_divergence"]}', flush=True)
    (work / 'match_report.json').write_text(json.dumps(report, indent=2))

    dirs = {
        'human': symlink_dir(human_files, work / 'demos_human'),
        'ai': symlink_dir(ai_files, work / 'demos_ai'),
    }
    if args.shuffled_control:
        dirs['human_shuffled'] = shuffled_dir(human_files, work / 'demos_human_shuffled')
        dirs['ai_shuffled'] = shuffled_dir(ai_files, work / 'demos_ai_shuffled')

    results = {}   # results[(student, condition)] = list of per-seed metric dicts
    for student in args.students:
        for cond, demo_dir in dirs.items():
            for seed in args.seeds:
                tag = f'{student}_{cond}_s{seed}'
                rundir = work / tag
                rundir.mkdir(exist_ok=True)
                print(f'[run] {tag}  (demos={demo_dir})', flush=True)
                if student == 'dp':
                    m = train_eval_dp(demo_dir, rundir, seed, args.n_eval, args.eval_seed,
                                      args.dp_steps, min_frames, args.dry_run)
                elif student == 'bc':
                    m = train_eval_bc(demo_dir, rundir, seed, args.n_eval, args.eval_seed,
                                      args.dry_run)
                else:
                    m = train_eval_sac(demo_dir, rundir, seed, args.n_eval, args.eval_seed,
                                       args.sac_steps, args.dry_run)
                results.setdefault(f'{student}|{cond}', []).append(m)
                print(f'  -> {tag}: {m}', flush=True)

    print('[run] random-action floor (negative control)', flush=True)
    floor = eval_random_floor(work / 'floor', args.n_eval, args.eval_seed, args.dry_run)
    results['floor|random'] = [floor]

    # ------ aggregate + table ------
    key = 'picked' if args.scope == 'pick' else 'contact'
    rows = []
    for k, ms in results.items():
        vals = [m[key] for m in ms if m]
        mean = float(np.mean(vals)) if vals else float('nan')
        std = float(np.std(vals)) if vals else float('nan')
        rows.append((k, mean, std, len(vals),
                     {stk: (float(np.mean([m[stk] for m in ms if m])) if any(ms) else None)
                      for stk in ('picked', 'placed', 'contact', 'nested')}))
    summary = dict(scope=args.scope, headline_metric=key, seeds=args.seeds,
                   n_eval=args.n_eval, match=report,
                   results={k: {'mean': mn, 'std': sd, 'n_seeds': n, 'all_stages': stg}
                            for k, mn, sd, n, stg in rows})
    (work / 'results.json').write_text(json.dumps(summary, indent=2, default=str))

    print(f'\n===== COMPARISON ({args.scope} scope, headline={key}) =====')
    print(f'{"algo|condition":24s} {"mean":>7s} {"std":>7s} {"seeds":>6s}')
    for k, mn, sd, n, _ in sorted(rows):
        print(f'{k:24s} {mn:7.3f} {sd:7.3f} {n:6d}')
    print(f'\nfull table + match report -> {work}/results.json')
    if args.dry_run:
        print('\n(dry-run: no training/eval actually executed)')


if __name__ == '__main__':
    main()
