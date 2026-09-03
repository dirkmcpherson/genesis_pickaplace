#!/usr/bin/env python3
"""RUN_REGISTRY: prevent silent re-execution of the same training computation.

Built from paper/AUDIT_run_identity_2026-08-17.md §5 ("Minimal prevention
proposal"). Every confirmed duplicate in that audit (pair-dH == nb, clean-long
== clean, n20 == nb/clean/exp) was the SAME (script, ARM, seed, semantic
knobs, demo data) executed more than once -- sometimes across a doc-only git
commit, which is why git is deliberately split out of the "semantic" identity
below (§5 item 2, "the warn-vs-refuse split is the load-bearing design
point").

Identity key = (script name, ARM, seed, git short hash, semantic knobs passed
as KEY=VAL args, DEMO FINGERPRINT = sha256 of the sorted list of
(filename, size) in the demo dir -- cheap, no content hashing, catches both
same-path-different-contents and different-path-same-contents).

  semantic_key = sha256(script, arm, seed, knobs, demo_fingerprint)   -- no git
  full_key     = sha256(script, arm, seed, knobs, demo_fingerprint, git)

Two subcommands:
  check    - refuse (exit 2) on a FULL-key match unless env DUPLICATE_OK is a
             non-empty reason string; WARN (exit 0) on a match that differs
             ONLY in git (semantic_key matches, full_key doesn't) -- that is
             exactly the nb->pair / exp->n20 pattern (real duplicates that
             crossed a doc-only commit); otherwise exit 0 silently.
  register - append one JSON line to the registry: the key fields + timestamp
             + SLURM_JOB_ID (if set) + the DUPLICATE_OK reason (if any).

stdlib only -- this runs inside the cluster's pip-only conda env, which must
never receive a conda-installed package (see cluster/verify_env.sh history:
a conda-ffmpeg install poisoned libstdc++ for the whole box).

Usage (see cluster/sbatch_rlpd.sh for the wired call):
  python cluster/run_registry.py check --script sbatch_rlpd.sh --arm dH \
      --seed 0 --demo-dir baselines/episodes_pick_phase_all \
      --registry cluster/RUN_REGISTRY.jsonl \
      steps=100000 gamma=0.998 utd=10 ensemble_size=10 subset_size=2 \
      demo_batch=128 backup_entropy=off per_member_ln=off \
      pick_hold_reward=off pick_shaping=off action_mode=delta_joint \
      delta_ref=target action_repeat=1
  python cluster/run_registry.py register --script sbatch_rlpd.sh --arm dH \
      --seed 0 --demo-dir baselines/episodes_pick_phase_all \
      --registry cluster/RUN_REGISTRY.jsonl steps=100000 ...   # same knobs

Exit codes: 0 = ok (incl. warn), 2 = refused duplicate, 1 = usage/other error.
"""
import argparse
import datetime
import hashlib
import json
import os
import pathlib as pl
import subprocess
import sys


def demo_fingerprint(demo_dir: pl.Path) -> str:
    """sha256 of the sorted (filename, size) list of the demo dir's direct
    children (files only, no recursion, no content hashing -- cheap)."""
    if not demo_dir.is_dir():
        raise SystemExit(f'FATAL: --demo-dir {demo_dir} is not a directory')
    entries = sorted(
        (p.name, p.stat().st_size) for p in demo_dir.iterdir() if p.is_file())
    if not entries:
        raise SystemExit(f'FATAL: --demo-dir {demo_dir} has no files to fingerprint')
    blob = json.dumps(entries, sort_keys=False).encode('utf-8')
    return hashlib.sha256(blob).hexdigest()


def git_short_hash(repo: pl.Path) -> str:
    try:
        out = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(repo),
                             capture_output=True, text=True, timeout=5)
        h = out.stdout.strip()
        return h if h else 'unknown'
    except Exception:
        return 'unknown'


def parse_knobs(args):
    knobs = {}
    for a in args:
        if '=' not in a:
            raise SystemExit(f'FATAL: knob arg {a!r} is not KEY=VAL')
        k, v = a.split('=', 1)
        if k in knobs:
            raise SystemExit(f'FATAL: duplicate knob key {k!r}')
        knobs[k] = v
    return knobs


def build_keys(script, arm, seed, git, knobs, dfp):
    semantic_cfg = {
        'script': script, 'arm': arm, 'seed': int(seed),
        'knobs': dict(sorted(knobs.items())), 'demo_fingerprint': dfp,
    }
    full_cfg = dict(semantic_cfg, git=git)
    semantic_key = hashlib.sha256(
        json.dumps(semantic_cfg, sort_keys=True).encode('utf-8')).hexdigest()
    full_key = hashlib.sha256(
        json.dumps(full_cfg, sort_keys=True).encode('utf-8')).hexdigest()
    return semantic_key, full_key, semantic_cfg, full_cfg


def read_registry(path: pl.Path):
    if not path.exists():
        return []
    lines = []
    for i, raw in enumerate(path.read_text().splitlines()):
        raw = raw.strip()
        if not raw:
            continue
        try:
            lines.append(json.loads(raw))
        except json.JSONDecodeError as e:
            print(f'WARN: {path}:{i+1} is not valid JSON, skipping ({e})',
                  file=sys.stderr)
    return lines


def common_args(ap):
    ap.add_argument('--script', required=True, help='launcher script name, e.g. sbatch_rlpd.sh')
    ap.add_argument('--arm', required=True)
    ap.add_argument('--seed', required=True, type=int)
    ap.add_argument('--demo-dir', required=True)
    ap.add_argument('--git', default=None,
                    help='short git hash; auto-detected via git rev-parse if omitted')
    ap.add_argument('--registry', default='cluster/RUN_REGISTRY.jsonl')
    ap.add_argument('--repo-root', default=os.environ.get('GENESIS_PICKAPLACE_ROOT', '.'))
    ap.add_argument('--stage', choices=['start', 'end'], default=None,
                    help='register only: when in the job this line was written (08-23: the '
                         'launchers register at job START so a crashed/preempted run still '
                         'leaves its identity line -- closes the check-then-register-after-'
                         'training TOCTOU window). Recorded, NOT part of the identity key.')
    ap.add_argument('knobs', nargs='*', help='semantic knobs as KEY=VAL')


def resolve(args):
    repo = pl.Path(args.repo_root).resolve()
    # 'null'/'none' = demo-free arm (e.g. touchgoal probes); fingerprint is the literal sentinel
    no_demos = str(args.demo_dir).lower() in ('null', 'none', '')
    demo_dir = pl.Path(args.demo_dir)
    if not no_demos and not demo_dir.is_absolute():
        demo_dir = repo / demo_dir
    registry = pl.Path(args.registry)
    if not registry.is_absolute():
        registry = repo / registry
    git = args.git or git_short_hash(repo)
    knobs = parse_knobs(args.knobs)
    dfp = 'no-demos' if no_demos else demo_fingerprint(demo_dir)
    semantic_key, full_key, semantic_cfg, full_cfg = build_keys(
        args.script, args.arm, args.seed, git, knobs, dfp)
    return dict(repo=repo, demo_dir=demo_dir, registry=registry, git=git,
                knobs=knobs, dfp=dfp, semantic_key=semantic_key, full_key=full_key,
                semantic_cfg=semantic_cfg, full_cfg=full_cfg)


def cmd_check(args):
    r = resolve(args)
    lines = read_registry(r['registry'])
    full_hits = [l for l in lines if l.get('full_key') == r['full_key']]
    semantic_hits = [l for l in lines
                     if l.get('semantic_key') == r['semantic_key']
                     and l.get('full_key') != r['full_key']]

    if full_hits:
        hit = full_hits[-1]
        desc = (f"script={hit.get('script')} arm={hit.get('arm')} seed={hit.get('seed')} "
                f"git={hit.get('git')} registered={hit.get('timestamp')} "
                f"job={hit.get('slurm_job_id')}")
        dup_ok = os.environ.get('DUPLICATE_OK', '').strip()
        if dup_ok:
            print(f'REGISTRY-DUPLICATE-OK key={r["full_key"][:12]} '
                  f'proceeding despite full-key match ({desc}); reason={dup_ok!r}')
            return 0
        print(f'REGISTRY-REFUSE key={r["full_key"][:12]} full-key match with an '
              f'existing run: {desc}. This exact (script, arm, seed, git, knobs, '
              f'demo_fingerprint) has already been executed -- refusing to '
              f'silently re-run it (see paper/AUDIT_run_identity_2026-08-17.md). '
              f'Set DUPLICATE_OK="<reason>" and re-run to proceed anyway.',
              file=sys.stderr)
        return 2

    if semantic_hits:
        hit = semantic_hits[-1]
        print(f'REGISTRY-WARN key={r["semantic_key"][:12]} semantic match (script, '
              f'arm, seed, knobs, demo_fingerprint identical) with a prior run at '
              f"git={hit.get('git')} (current git={r['git']}), registered="
              f"{hit.get('timestamp')} job={hit.get('slurm_job_id')}. Every "
              f'confirmed duplicate in the run-identity audit crossed exactly this '
              f'kind of doc-only commit -- verify this is a deliberate re-run '
              f'before pooling it as an independent seed.')
        return 0

    print(f'REGISTRY-OK key={r["full_key"][:12]} no prior match '
          f'(script={args.script} arm={args.arm} seed={args.seed} git={r["git"]})')
    return 0


def cmd_register(args):
    r = resolve(args)
    entry = dict(r['full_cfg'])
    entry.update({
        'semantic_key': r['semantic_key'],
        'full_key': r['full_key'],
        'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(timespec='seconds'),
        'slurm_job_id': os.environ.get('SLURM_JOB_ID'),
        'duplicate_ok_reason': os.environ.get('DUPLICATE_OK', '').strip() or None,
        'stage': getattr(args, 'stage', None),
        'node': os.environ.get('SLURM_JOB_NODELIST') or os.environ.get('HOSTNAME'),
    })
    r['registry'].parent.mkdir(parents=True, exist_ok=True)
    with open(r['registry'], 'a') as f:
        f.write(json.dumps(entry, sort_keys=True) + '\n')
    print(f'REGISTRY-REGISTERED key={r["full_key"][:12]} arm={args.arm} '
          f'seed={args.seed} git={r["git"]} job={entry["slurm_job_id"]} '
          f'-> {r["registry"]}')
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    p_check = sub.add_parser('check', help='refuse/warn on a duplicate run identity')
    common_args(p_check)
    p_reg = sub.add_parser('register', help='append this run identity to the registry')
    common_args(p_reg)
    args = ap.parse_args()
    fn = cmd_check if args.cmd == 'check' else cmd_register
    sys.exit(fn(args))


if __name__ == '__main__':
    main()
