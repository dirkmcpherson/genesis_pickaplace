"""Eval a checkpoint (SAC .zip or DP dir) on random ICs, record videos + metrics json,
optionally log to wandb. One implementation for three uses:

  1. SACfD in-train periodic eval: spawned by the trainer with --no-wandb --json;
     the TRAINING process reads the json and logs to ITS run (single wandb writer).
  2. DP post-train eval: run with wandb on -> its own run in the same project/group.
  3. Ad-hoc: eval any checkpoint with videos, wandb optional.

Usage:
  wandb_eval.py --kind sac --checkpoint ck.zip --random 10 --max-steps 400 \
      --record-dir /tmp/ev --json /tmp/ev/metrics.json --no-wandb
  wandb_eval.py --kind dp --checkpoint baselines/outputs/dp_pick_v2/checkpoints/last/pretrained_model \
      --random 15 --group dp_pick_v3 --name dp_pick_v3-eval
"""
import os
import argparse, json, pathlib as pl, sys

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser()
ap.add_argument('--kind', choices=('sac', 'dp'), required=True)
ap.add_argument('--checkpoint', required=True)
ap.add_argument('--random', type=int, default=10)
ap.add_argument('--uids', type=int, nargs='*', default=None)
ap.add_argument('--reps', type=int, default=1)
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--max-steps', type=int, default=1200)
ap.add_argument('--sim-variant', default=None, help='Genesis world variant; default: the checkpoint sidecar value, else base. Mismatch with an explicit flag refuses.')
ap.add_argument('--cartesian', action='store_true',
                help='eval a cartesian-action policy through CartesianCanEnv')
ap.add_argument('--ic-mode', choices=['random', 'demo', 'both'], default='both',
                help="demo = the demos' OWN starting placements (in-distribution). "
                     "A policy that scores well on demo ICs but zero on random ICs "
                     "is a generalization failure, not a broken pipeline -- the "
                     "positive control this project has never had.")
ap.add_argument('--action-mode', choices=['auto', 'absolute', 'delta_joint'],
                default='auto',
                help="JOINT sac checkpoints only: 'delta_joint' integrates the "
                     "policy's per-step joint deltas onto a leashed target "
                     "(FullTaskEnv delta mode, cap 0.025/leash 5x -- must match "
                     "training). 'auto' reads the <ckpt>.action_mode.json "
                     "sidecar written by train_sacfd_full and falls back to "
                     "absolute (all pre-2026-08-11 checkpoints) with a printed "
                     "note.")
ap.add_argument('--action-repeat', type=int, default=None,
                help="JOINT delta_joint eval only: hold each policy decision N env "
                     "steps (query the model once per N steps, integrate the SAME "
                     "delta each step) so eval runs the SAME MDP as a repeat-N "
                     "training run. None = read action_repeat from the "
                     "<ckpt>.action_mode.json sidecar (falls back to 1 for sidecars "
                     "predating the field); an explicit value overrides. Evaling a "
                     "repeat-N policy at stride 1 is the silent-default bug family.")
ap.add_argument('--delta-ref', choices=['auto', 'target', 'measured'], default='auto',
                help="JOINT delta_joint eval only: WHAT the policy's delta is applied "
                     "to, mirroring FullTaskEnv._step_once. 'target' = running-target "
                     "integration scaled by the cap (every pre-2026-08-14 checkpoint). "
                     "'measured' = re-referenced to the MEASURED qpos each step and "
                     "scaled by the LEASH (5*cap) -- the closed-loop/re-record space. "
                     "'auto' (default) reads delta_ref from the "
                     "<ckpt>.action_mode.json sidecar and falls back to 'target' for "
                     "sidecars predating the field. A sidecar saying 'measured' is "
                     "ALWAYS honored: asking for target-ref on a measured-ref "
                     "checkpoint ASSERTS rather than silently evaling the wrong MDP "
                     "(the documented silent-fallback trap).")
ap.add_argument('--control', choices=['vel', 'delta', 'abs', 'abs6', 'delta6'], default='vel',
                help='cartesian env control mode (must match the policy training data)')
ap.add_argument('--obs', choices=['env', 'joint', 'ee'], default='env',
                help="obs layout the POLICY sees, when it differs from the env's own "
                     "(the obs x action 2x2 mixed cells). The env must follow the "
                     "ACTION type -- it executes what the policy emits -- so "
                     "jobs_eact = '--cartesian --control abs6 --obs joint' and "
                     "eobs_jact = '--obs ee' (joint env). 'env' = no adaptation.")
ap.add_argument('--n-action-steps', type=int, default=None,
                help='override the chunk-execution length (ACT default 100 = 2.5s '
                     'open-loop; velocity-integration drift compounds within chunks)')
ap.add_argument('--record-dir', default=None, help='default: <checkpoint>_eval_videos')
ap.add_argument('--json', dest='json_out', default=None, help='write metrics dict here')
ap.add_argument('--videos', type=int, default=3, help='max videos uploaded to wandb')
ap.add_argument('--no-wandb', action='store_true')
ap.add_argument('--project', default='genesis_pickaplace')
ap.add_argument('--group', default=None)
ap.add_argument('--name', default=None)
ap.add_argument('--step', type=int, default=None, help='training step tag for the metrics')
# --- one-harness protocol (PREREG_final_round_robin_2026-08-23 §5; 08-23) -------------
ap.add_argument('--ic-file', default=None,
                help='baselines/eval_ics.json: the SHARED IC file (sel/hold/rnd). When given, '
                     '--ic-set selects the set and --ic-index k evaluates ONLY the k-th IC '
                     '(one episode per fresh process = cluster/eval_sweep.sh). --random/--uids/'
                     '--ic-mode are ignored with a printed note.')
ap.add_argument('--ic-set', choices=['sel', 'hold', 'rnd'], default=None)
ap.add_argument('--ic-index', type=int, default=None)
ap.add_argument('--arm', default=None, help='recorded into the result JSON (provenance only)')
ap.add_argument('--ckpt-step', default=None, help='recorded into the result JSON (provenance only)')
ap.add_argument('--reward', default=None, help='sparse|dense tag, recorded only')
args = ap.parse_args()
assert (args.ic_file is None) == (args.ic_set is None), '--ic-file and --ic-set go together'
assert args.ic_index is None or args.ic_file is not None, '--ic-index needs --ic-file'
print(f'[eval] max_steps={args.max_steps} (SIM steps; the sweep of record passes 1200 explicitly '
      f'-- PREREG §5; the default here is legacy)', flush=True)

import numpy as np  # noqa: E402
# Audit F4: --kind dp samples diffusion noise off the torch global; unseeded,
# every invocation was a different draw. SAC path (deterministic) unaffected.
import torch as _th  # noqa: E402
_th.manual_seed(args.seed)
np.random.seed(args.seed)
import ic_sampling, eval_core  # noqa: E402
from genesis_can_env import GenesisCanEnv  # noqa: E402

rec = args.record_dir or (str(pl.Path(args.checkpoint).with_suffix('')) + '_eval_videos')
# Decide camera needs BEFORE building the env: genesis allows ONE world per process,
# so probing the checkpoint after construction and rebuilding would double-gs.init.
_needs_rig = False
if args.kind == 'dp':
    import json as _json
    _cfgd = _json.loads((pl.Path(args.checkpoint) / 'config.json').read_text())
    _needs_rig = any(k.startswith('observation.images.')
                     for k in _cfgd.get('input_features', {}))
# sim variant (world realization): CLI > sidecar > 'base' -- must match training/recording
def _resolve_sim_variant():
    sv_side = None
    try:
        import json as _json, pathlib as _pl
        c = _pl.Path(args.checkpoint)
        cands = ([c.with_name(c.stem + '.action_mode.json')] if c.suffix == '.zip' else [])
        cands += [(c / 'dp_sidecar.json') if c.is_dir() else None, c.parent / 'dp_sidecar.json']
        for cand in cands:
            if cand and cand.exists():
                sv_side = _json.loads(cand.read_text()).get('sim_variant')
                break
    except Exception:
        pass
    sv = args.sim_variant if args.sim_variant is not None else (sv_side or 'base')
    if args.sim_variant is not None and sv_side is not None and args.sim_variant != sv_side:
        raise SystemExit(f'FATAL: --sim-variant {args.sim_variant} != sidecar {sv_side}')
    return sv
_SIM_VARIANT = _resolve_sim_variant()
import sys as _sys, pathlib as _pl2
_sys.path.insert(0, str(_pl2.Path(__file__).resolve().parent))
from sim_variant_hook import apply_pre as _sv_pre, apply_post as _sv_post
_sv_pre(_SIM_VARIANT)
if args.cartesian:
    from cartesian_env import CartesianCanEnv
    env = CartesianCanEnv(backend='cpu', render_size=(480, 640),
                          max_steps=args.max_steps, camera_rig=_needs_rig,
                          control=args.control)
else:
    env = GenesisCanEnv(backend='cpu', render_size=(480, 640), max_steps=args.max_steps,
                        camera_rig=_needs_rig)
_sv_post(env, _SIM_VARIANT)

_dref = None            # resolved delta reference frame (sac/delta_joint only)
if args.kind == 'sac':
    from stable_baselines3 import SAC
    if args.cartesian:
        from cartesian_env import CartesianCanEnv as _C
        # MUST match the env's control mode: delta actions denormalized with the
        # velocity scale (or vice versa) are a different command type entirely.
        denormalize_action = ({'delta': _C.denormalize_delta,
                               'abs': _C.denormalize_abs,
                               'abs6': _C.denormalize_abs6,
                               'delta6': _C.denormalize_delta6}.get(args.control)
                              or _C.denormalize_action)
    else:
        from pick_env import denormalize_action
    model = SAC.load(args.checkpoint, device='cpu')

    _mode = args.action_mode
    _repeat = args.action_repeat            # None => resolve from sidecar/default
    _sc = pl.Path(str(args.checkpoint)).with_suffix('.action_mode.json')
    _side = json.loads(_sc.read_text()) if _sc.exists() else {}
    if _mode == 'auto' and not args.cartesian:
        if _side:
            _mode = _side['action_mode']
            print(f'[eval] action_mode from sidecar: {_mode}')
        else:
            _mode = 'absolute'
            print('[eval] no action_mode sidecar -> absolute (legacy checkpoint)')
    if _repeat is None:
        # sidecars predating action_repeat -> stride 1 (the old behaviour); a repeat-N
        # run always writes the field, so this only defaults legacy stride-1 runs.
        _repeat = int(_side.get('action_repeat', 1))
    _repeat = max(1, int(_repeat))

    # --- delta reference frame (2026-08-14). Resolution rules, in order:
    #   1. the sidecar is AUTHORITATIVE for 'measured': a measured-ref checkpoint
    #      integrated as target-ref is a different MDP and evals ~0.00, and that zero
    #      looks like a result (the silent-fallback trap this repo has now hit four
    #      times: grip column, control mode, action_mode, action_repeat).
    #   2. 'auto' with no delta_ref in the sidecar -> 'target' = the behaviour of every
    #      pre-2026-08-14 checkpoint, so legacy evals are byte-identical.
    _side_dref = _side.get('delta_ref')
    _dref = args.delta_ref
    if _dref == 'auto':
        if _side_dref:
            _dref = str(_side_dref)
            print(f'[eval] delta_ref from sidecar: {_dref}')
        else:
            _dref = 'target'
            print('[eval] no delta_ref in sidecar -> target (legacy checkpoint)')
    else:
        assert not (_side_dref == 'measured' and _dref != 'measured'), (
            f'checkpoint sidecar says delta_ref=measured but --delta-ref {_dref} was '
            f'requested: integrating a measured-ref policy as target-ref evaluates a '
            f'DIFFERENT MDP and silently returns ~0.00. Refusing. ({_sc})')
        print(f'[eval] delta_ref from flag: {_dref} (sidecar: {_side_dref})')
    assert _dref in ('target', 'measured'), _dref
    assert not (_dref == 'measured' and (_mode != 'delta_joint' or args.cartesian)), (
        f'delta_ref=measured is a JOINT delta_joint concept; got action_mode={_mode} '
        f'cartesian={args.cartesian}')

    if _mode == 'delta_joint' and not args.cartesian:
        print(f'[eval] action_repeat={_repeat} delta_ref={_dref} '
              f'(1 policy query per {_repeat} env step(s))')
        # Stateful delta integration, mirroring FullTaskEnv(action_mode=
        # 'delta_joint') exactly: target seeded from measured q on the first
        # step of each episode, integrated by a*cap, clipped to joint limits,
        # leashed to measured q. Constants MUST match training.
        from pick_env import ARM_LO, ARM_HI
        DJ_CAP, DJ_LEASH = 0.025, 5.0 * 0.025
        # action_repeat: query the model once every _repeat env steps and hold its
        # normalized delta for that many steps, integrating it EACH step -- mirrors
        # FullTaskEnv(action_repeat=_repeat) exactly (same delta fed to _step_once N
        # times => N*a*cap target advance), which is what training and the demo
        # encoding assume. _repeat==1 is the original stride-1 integrator.
        _dj = {'target': None, 'a': None, 'k': 0}

        def policy_action(obs):
            q = np.asarray(obs['state'][:6], dtype=np.float64)
            if _dj['target'] is None:
                _dj['target'] = q.copy()
            if _dj['k'] % _repeat == 0:
                _dj['a'], _ = model.predict(obs['state'], deterministic=True)
            _dj['k'] += 1
            a = _dj['a']
            if _dref == 'measured':
                # MIRROR of FullTaskEnv._step_once's delta_ref='measured' branch --
                # do not innovate here, the two integrators have diverged once
                # already. The action is the normalized desired PD ERROR off the
                # MEASURED arm, so it scales by the LEASH (not the cap): cap-scaling
                # under-drives 5x and the arm never keeps the demo's timing. `q` is
                # the measured qpos after the previous sim step, which is exactly
                # what the env carries in self._dj_qmeas (seeded from measured qpos
                # at reset by _sync_dj_target).
                sp = np.clip(q + np.clip(a[:6], -1.0, 1.0) * DJ_LEASH,
                             ARM_LO, ARM_HI)
            else:
                sp = np.clip(_dj['target'] + np.clip(a[:6], -1.0, 1.0) * DJ_CAP,
                             ARM_LO, ARM_HI)
            _dj['target'] = q + np.clip(sp - q, -DJ_LEASH, DJ_LEASH)
            return np.concatenate(
                [_dj['target'], [(np.clip(a[6], -1.0, 1.0) + 1.0) / 2.0]])

        def policy_reset():
            _dj['target'] = None
            _dj['a'] = None
            _dj['k'] = 0
    else:
        def policy_action(obs):
            a, _ = model.predict(obs['state'], deterministic=True)
            return denormalize_action(a)
        policy_reset = None
else:
    from dp_runner import load_dp_runner
    policy_action, policy_reset, _proprio = load_dp_runner(
        args.checkpoint, rig_provider=(env.rig_obs if _needs_rig else None),
        n_action_steps=args.n_action_steps)
    # --- DP hold-N (08-23, PREREG §2/§4.1): the learners' clock is action_repeat N. A DP
    # trained on decision-rate data (lerobot fps 30/N) is queried ONCE per N sim steps; its
    # absolute joint target q* is turned into the normalized delta the learners' MDP would
    # need to reach it in one decision, a_arm = clip((q* - target)/(N*cap)), and that SAME
    # delta is integrated a*cap per sim step on the running PD target, leashed to the
    # measured q -- a MIRROR of FullTaskEnv._step_once(delta_ref='target') and of the sac
    # branch above (do not innovate: the two integrators have diverged once already).
    # Resolution: --action-repeat, else the dp_sidecar.json next to the checkpoint, else 1.
    # repeat==1 keeps the legacy absolute pass-through BYTE-IDENTICAL (every pre-08-23 DP
    # eval); any N>1 uses the integrator. The grip is DP's own (0..1), held for the window.
    _repeat = args.action_repeat
    _dp_side = pl.Path(args.checkpoint).parent / 'dp_sidecar.json'
    _dside = json.loads(_dp_side.read_text()) if _dp_side.exists() else {}
    if _repeat is None:
        if 'action_repeat' in _dside:
            _repeat = int(_dside['action_repeat'])
            print(f'[eval] dp action_repeat from sidecar: {_repeat} ({_dp_side})')
        else:
            _repeat = 1
            print('[eval] dp: no --action-repeat and no action_repeat in dp_sidecar.json -> 1 '
                  '(legacy absolute pass-through). A repeat-N DP MUST be evaluated at N.')
    _repeat = max(1, int(_repeat))
    if _dside.get('action_repeat') is not None and int(_dside['action_repeat']) != _repeat:
        raise SystemExit(f'dp sidecar says action_repeat={_dside["action_repeat"]} but '
                         f'--action-repeat {_repeat} requested: evaluating a repeat-N DP at a '
                         f'different clock is a different MDP. Refusing. ({_dp_side})')
    assert args.delta_ref in ('auto', 'target'), 'DP hold-N exists for target-ref only'
    if _repeat > 1:
        assert not args.cartesian, 'DP hold-N is the joint delta_joint MDP'
        from pick_env import ARM_LO, ARM_HI
        DJ_CAP, DJ_LEASH = 0.025, 5.0 * 0.025
        _dref = 'target'
        _dp = {'target': None, 'a': None, 'grip': None, 'k': 0}
        _dp_base_action, _dp_base_reset = policy_action, policy_reset
        print(f'[eval] dp hold-{_repeat}: 1 policy query per {_repeat} env steps; q* -> '
              f'delta clip((q*-target)/({_repeat}*{DJ_CAP})), integrated {DJ_CAP}/step, '
              f'leash {DJ_LEASH}')

        def policy_action(obs):
            q = np.asarray(obs['state'][:6], dtype=np.float64)
            if _dp['target'] is None:
                _dp['target'] = q.copy()
            if _dp['k'] % _repeat == 0:
                phys = np.asarray(_dp_base_action(obs), dtype=np.float64)   # [q*(6), grip 0..1]
                _dp['a'] = np.clip((phys[:6] - _dp['target']) / (_repeat * DJ_CAP), -1.0, 1.0)
                _dp['grip'] = float(np.clip(phys[6], 0.0, 1.0))
            _dp['k'] += 1
            sp = np.clip(_dp['target'] + _dp['a'] * DJ_CAP, ARM_LO, ARM_HI)
            _dp['target'] = q + np.clip(sp - q, -DJ_LEASH, DJ_LEASH)
            return np.concatenate([_dp['target'], [_dp['grip']]])

        def policy_reset():
            _dp['target'] = None; _dp['a'] = None; _dp['grip'] = None; _dp['k'] = 0
            if _dp_base_reset is not None:
                _dp_base_reset()
    _mode = 'jact_absolute' if _repeat == 1 else 'jact_hold_delta_target'

# --- mixed obs x action cells: swap the state the policy sees, leave execution alone.
# Both adapters replicate collect_cartesian_dataset's constructions exactly, so the
# policy sees the same layout it trained on.
if args.obs == 'joint':
    # 17-dim joint obs inside a cartesian-executing env; CartesianCanEnv already
    # carries it as obs['joint_state'].
    assert args.cartesian, '--obs joint pairs a joint-obs policy with a cartesian env'
    _base_action = policy_action

    def policy_action(obs):
        o = dict(obs); o['state'] = obs['joint_state']
        return _base_action(o)
elif args.obs == 'ee':
    # 18-dim ee-centric obs synthesized from the joint env:
    # ee_pos(3), ee_quat(4), grip+effort (s[6:8]), world (s[8:17])
    assert not args.cartesian, '--obs ee pairs an ee-obs policy with the joint env'
    _eef = env.w['eef']

    def _np(x):
        return x.detach().cpu().numpy() if hasattr(x, 'detach') else np.asarray(x)
    _base_action = policy_action

    def policy_action(obs):
        s = obs['state']
        o = dict(obs)
        o['state'] = np.concatenate([_np(_eef.get_pos()), _np(_eef.get_quat()),
                                     s[6:8], s[8:17]]).astype(np.float32)
        return _base_action(o)

_ic_sets = {}
# --uids is honored ONLY on the --random 0 path below; passing both silently
# evaluated the wrong episodes (memory-documented trap). Crash, don't guess.
assert not (args.uids and args.random), (
    '--uids requires --random 0 (with --random N the uid list is IGNORED '
    'and the first N demo ICs are evaluated instead)')
if args.ic_file:
    # the shared IC file: sel/hold = demo uids (env.reset(uid=)), rnd = explicit poses,
    # deserialized by the SAME helper the generator uses (no second reader).
    from make_eval_ics import episodes_from_file
    if args.random or args.uids:
        print('[eval] --ic-file given: --random/--uids/--ic-mode are IGNORED', flush=True)
    _ic_sets[args.ic_set] = episodes_from_file(args.ic_file, args.ic_set, args.ic_index)
    print(f'[eval] ICs from {args.ic_file} set={args.ic_set} index={args.ic_index} '
          f'-> {len(_ic_sets[args.ic_set])} episode(s)', flush=True)
elif args.random:
    if args.ic_mode in ('demo', 'both'):
        _ic_sets['indist'] = ic_sampling.demo_ics(env, reps=1)[:args.random]
    if args.ic_mode in ('random', 'both'):
        _ic_sets['random'] = ic_sampling.sample_support_ics(
            env, args.random, seed=args.seed)
else:
    _ic_sets['indist'] = ic_sampling.demo_ics(env, uids=args.uids, reps=args.reps)
episodes = list(_ic_sets.values())[0]

# Evaluate every prepared IC set in ONE env (genesis allows one world per process).
# Prefix stays 'eval/' for the single-set case so existing dashboards keep working;
# 'both' adds eval_indist/* and eval_random/* so the generalization gap is visible
# on one chart instead of across two runs.
metrics = {}
_aggs = {}
_single = len(_ic_sets) == 1
for _name, _eps in _ic_sets.items():
    _rec = rec if _single else str(pl.Path(rec) / _name)
    _a = eval_core.run_eval(env, policy_action, _eps, policy_reset=policy_reset,
                            record_dir=_rec, tag=f'{args.kind}:{_name}')
    _aggs[_name] = _a
    _n = max(_a['n'], 1)
    _pref = 'eval' if _single else f'eval_{_name}'
    metrics.update({f'{_pref}/{k}': _a[k] / _n for k in eval_core.STAGES})
    metrics[f'{_pref}/n'] = _a['n']
if not _single:
    # eval/* == the DEMO-IC (in-distribution) set: demo can positions are now the
    # standard evaluation distribution, matching how the envs reset during training.
    # eval_random/* is still logged every time, so the generalization gap stays
    # visible rather than being quietly dropped.
    _a = _aggs.get('indist') or list(_aggs.values())[0]
    _n = max(_a['n'], 1)
    metrics.update({f'eval/{k}': _a[k] / _n for k in eval_core.STAGES})
    metrics['eval/gen_gap_picked'] = (
        (_aggs['indist']['picked'] / max(_aggs['indist']['n'], 1))
        - (_aggs['random']['picked'] / max(_aggs['random']['n'], 1))
    ) if 'indist' in _aggs and 'random' in _aggs else 0.0
agg = _aggs.get('random') or list(_aggs.values())[0]
n = max(agg['n'], 1)
if args.step is not None:
    metrics['eval/train_step'] = args.step
vids = sorted(str(p) for p in pl.Path(rec).rglob('*.mp4'))
tiled = None
if len(vids) > 1:
    import subprocess
    tiled = str(pl.Path(rec) / 'tiled.mp4')
    tc = subprocess.run([sys.executable, str(REPO / 'baselines/tile_videos.py'),
                         rec, '--out', tiled], capture_output=True, text=True)
    if tc.returncode != 0:
        print('tiling failed:', tc.stderr[-200:], flush=True)
        tiled = None
    vids = [v for v in vids if not v.endswith('tiled.mp4')]
import socket as _socket, subprocess as _sp  # noqa: E402
try:
    _git = _sp.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO), capture_output=True,
                   text=True, timeout=5).stdout.strip() or 'unknown'
except Exception:
    _git = 'unknown'
_node = dict(hostname=_socket.gethostname(),
             slurm_nodelist=os.environ.get('SLURM_JOB_NODELIST'),
             slurm_partition=os.environ.get('SLURM_JOB_PARTITION'),
             slurm_job_id=os.environ.get('SLURM_JOB_ID'),
             cuda_visible=os.environ.get('CUDA_VISIBLE_DEVICES'), sim_variant=_SIM_VARIANT)
_act_sel = ('deterministic' if args.kind == 'sac' else f'sampled(seed={args.seed})')
_eps_all = [dict(ic_set=_name, **e) for _name, _a in _aggs.items() for e in _a.get('episodes', [])]
result = dict(metrics=metrics, videos=vids, tiled=tiled, checkpoint=args.checkpoint,
              seed=args.seed, max_steps=args.max_steps, delta_ref=_dref,
              # one-harness protocol fields (PREREG §5)
              kind=args.kind, arm=args.arm, ckpt_step=args.ckpt_step, reward=args.reward,
              ic_file=args.ic_file, ic_set=args.ic_set, ic_index=args.ic_index,
              action_repeat=(int(_repeat) if '_repeat' in globals() else None),
              action_mode=(_mode if '_mode' in globals() else None),
              act_selection=_act_sel, node=_node, git=_git,
              picked=int(sum(a['picked'] for a in _aggs.values())),
              n=int(sum(a['n'] for a in _aggs.values())),
              n_steps=([e['n_steps'] for e in _eps_all] if _eps_all else None),
              episodes=_eps_all)

if args.json_out:
    pl.Path(args.json_out).write_text(json.dumps(result, indent=1))
    print(f'metrics -> {args.json_out}', flush=True)

if not args.no_wandb:
    import wandb
    run = wandb.init(project=args.project, group=args.group, name=args.name,
                     job_type='eval', config=dict(checkpoint=args.checkpoint,
                                                  seed=args.seed, n=agg['n'],
                                                  max_steps=args.max_steps))
    log = dict(metrics)
    if tiled:
        log['eval/rollouts_tiled'] = wandb.Video(tiled, format='mp4',
                                                 caption='all episodes tiled')
    for i, v in enumerate(vids[:args.videos]):
        log[f'eval/video_{i}'] = wandb.Video(v, format='mp4',
                                             caption=pl.Path(v).stem)
    run.log(log, step=args.step or 0)
    run.finish()
    print(f'wandb run {run.name} logged', flush=True)
