#!/usr/bin/env python
"""PLACE-phase evaluator for RLPD (SB3 SAC .zip) and Diffusion Policy (lerobot dir) checkpoints from an ENTRY
BANK -- the r2dreamer place protocol (PHASE_PLAN_2026-09-04 §1/§4, eval_genesis.py --entry-bank), same env, same
predicates, same metrics.json layout, so `phase_table`-style readers work across learners.

Protocol: FullTaskEnv(scope='place', entry_bank=BANK, phase_sparse=True) in the corrected world (--sim-variant, also
exported as GENESIS_SIM_VARIANT so the env's shelf band follows the world's shelf), action_repeat 4 / delta_joint cap
0.025 leash 5x / delta_ref target (asserted against the checkpoint sidecar), horizon --max-steps SIM steps (600 =
150 decisions), ONE episode per bank entry IN ORDER (each entry restored exactly once; the r2dreamer evaluator draws
the bank with replacement -- disclosed difference), success = placed_v2 (grip cmd < 0.45 + shelf footprint + shelf
z-band + tilt < 20 deg, sustained 10 frames); tips terminate; an entry that does not survive the restore is counted
as a FAILURE (`restore_failed`), never skipped.

Action selection: sac --mode sample = predict(deterministic=False) (one draw per decision, torch seeded by --seed),
--mode mode = predict(deterministic=True). dp is sampled by construction (diffusion noise, seeded by --seed); --mode
mode is refused for dp. Both policies act THROUGH the env's own delta_joint integrator (one FullTaskEnv.step per
decision): sac emits normalized deltas natively; DP's absolute window-end joint target q* becomes the delta the
learners' MDP needs to reach it in one decision, a = clip((q* - target)/(repeat*cap)), the same hold-4 rule as
wandb_eval.py's DP branch, and the grip 0..1 is mapped to [-1,1].

Output: <out>/metrics.json (episodes, placed_v2 rate, tipped, timeout, restore_failed, stages, per_episode with
outcome/steps/uid/entry_frame), one mp4 per episode when --video (ep{k}_uid{uid}_{outcome}.mp4, 7.5 fps).

usage: eval_place.py --kind sac|dp --checkpoint <zip|pretrained_model dir> --entry-bank <json> --out <dir>
                     [--mode sample|mode] [--seed 0] [--max-steps 600] [--sim-variant gc_kp4_riser3_shelf6]
                     [--video] [--limit N] [--device auto|cpu|cuda]
"""
import os, sys, json, time, argparse, pathlib as pl, socket, subprocess

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--kind', choices=('sac', 'dp'), required=True)
ap.add_argument('--checkpoint', required=True)
ap.add_argument('--entry-bank', required=True)
ap.add_argument('--out', required=True)
ap.add_argument('--mode', choices=('sample', 'mode'), default='sample')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--max-steps', type=int, default=600, help='SIM steps per episode (600 = the place-scope cap, 150 decisions at repeat 4)')
ap.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
ap.add_argument('--video', action='store_true', help='one mp4 per episode (240x320, one frame per decision)')
ap.add_argument('--limit', type=int, default=None, help='first N bank entries only (smokes)')
ap.add_argument('--device', default='auto', help='policy device for dp (auto = cuda if visible); sac is always cpu')
ap.add_argument('--arm', default=None); ap.add_argument('--tag', default=None)
args = ap.parse_args()
if args.kind == 'dp' and args.mode == 'mode':
    sys.exit('FATAL: DP has no deterministic mode (diffusion sampling); run --mode sample')

import numpy as np   # noqa: E402
import torch         # noqa: E402
torch.manual_seed(args.seed); np.random.seed(args.seed)

# ---- sidecar: every action-semantics parameter comes from the checkpoint, never a default ----
ck = pl.Path(args.checkpoint)
if args.kind == 'sac':
    sc_path = ck.with_name(ck.stem + '.action_mode.json')
else:
    sc_path = (ck / 'dp_sidecar.json') if (ck / 'dp_sidecar.json').exists() else ck.parent / 'dp_sidecar.json'
if not sc_path.exists():
    sys.exit(f'FATAL: sidecar {sc_path} missing (no silent defaults for the block of record)')
side = json.loads(sc_path.read_text())
REPEAT = int(side['action_repeat'])
if args.kind == 'sac':
    assert side.get('action_mode') == 'delta_joint' and side.get('delta_ref', 'target') == 'target', side
sv_side = side.get('sim_variant')
if sv_side and sv_side != args.sim_variant:
    sys.exit(f'FATAL: sidecar sim_variant={sv_side} != --sim-variant {args.sim_variant}')
assert REPEAT == 4, f'place protocol is action_repeat 4; sidecar says {REPEAT}'
DJ_CAP, DJ_LEASH_MULT = 0.025, 5.0
# cap/leash parity with TRAINING (review 2026-09-07: the eval integrator must not hard-code a cap the run did not use)
if side.get('delta_cap') is not None:
    assert abs(float(side['delta_cap']) - DJ_CAP) < 1e-9, f'sidecar delta_cap={side["delta_cap"]} != {DJ_CAP}'
if side.get('delta_leash') is not None:
    assert abs(float(side['delta_leash']) - DJ_CAP * DJ_LEASH_MULT) < 1e-9, f'sidecar delta_leash={side["delta_leash"]} != {DJ_CAP * DJ_LEASH_MULT}'
print(f'[eval-place] kind={args.kind} mode={args.mode} seed={args.seed} repeat={REPEAT} sim_variant={args.sim_variant} '
      f'bank={args.entry_bank} max_steps={args.max_steps} sidecar={sc_path}', flush=True)

# ---- entry bank: load + normalise the schema HERE, then hand FullTaskEnv the exact list used ----
# Adversarial eval/env review 2026-09-07 (paper/ADVERSARIAL_REVIEW_eval_env_2026-09-07.md): the policy-generated banks
# (polE_*) were dumped with grip_cmd in the NORMALISED [-1,1] action space while the restore reads PHYSICAL [0,1]
# (35/148 machine-policy place entries restored with the fingers commanded open). The rebuilt banks carry a
# `bank_version` field (physical grip). Rules here: every grip_cmd/grip_obs must lie in [0,1] (else FATAL) and a
# policy-generated bank (basename starts with 'polE') must carry bank_version (else FATAL: 'use the rebuilt bank').
_raw = json.load(open(args.entry_bank))
bank_version = None
if isinstance(_raw, dict) and 'entries' in _raw:
    bank_version = _raw.get('bank_version'); _entries_raw = list(_raw['entries'])
elif isinstance(_raw, dict):
    _entries_raw = [dict(e, uid=int(u)) for u, e in _raw.items()]
else:
    _entries_raw = list(_raw)
if bank_version is None and _entries_raw and isinstance(_entries_raw[0], dict):
    bank_version = _entries_raw[0].get('bank_version')
_bad = [e for e in _entries_raw if not (0.0 <= float(e['grip_cmd']) <= 1.0 and 0.0 <= float(e['grip_obs']) <= 1.0)]
if _bad:
    sys.exit(f'FATAL: {len(_bad)}/{len(_entries_raw)} entries of {args.entry_bank} carry grip_cmd/grip_obs outside [0,1] '
             f'(normalised-grip bank, review 2026-09-07); use the rebuilt physical-grip bank (bank_version field)')
if os.path.basename(args.entry_bank).startswith('polE') and bank_version is None:
    sys.exit(f'FATAL: policy-generated bank {args.entry_bank} has no bank_version field -- evaluate only on the rebuilt bank (review 2026-09-07)')
BANK_USED = pl.Path(args.out) / 'bank_used.json'
BANK_USED.parent.mkdir(parents=True, exist_ok=True)
BANK_USED.write_text(json.dumps([dict(e, uid=int(e['uid'])) for e in _entries_raw]))
print(f'[eval-place] bank {args.entry_bank}: {len(_entries_raw)} entries, bank_version={bank_version!r}, grip in [0,1] OK -> {BANK_USED}', flush=True)

# ---- env: the private full_env place scope, corrected world ----
os.environ['GENESIS_SIM_VARIANT'] = args.sim_variant   # FullTaskEnv reads it for the shelf band (BOX_TOP_Z + shelf_dz)
from sim_variant_hook import apply_pre, apply_post   # noqa: E402
apply_pre(args.sim_variant)
from full_env import FullTaskEnv   # noqa: E402
import sim_variants as _sv          # noqa: E402
from replay_harness import BOX_TOP_Z   # noqa: E402
env = FullTaskEnv(backend='cpu', max_steps=args.max_steps, scope='place', entry_bank=str(BANK_USED), phase_sparse=True,
                  action_mode='delta_joint', delta_cap=DJ_CAP, delta_leash_mult=DJ_LEASH_MULT, action_repeat=REPEAT,
                  delta_ref='target', render_size=((240, 320) if args.video else None))
apply_post(env, args.sim_variant)
_want_top = float(BOX_TOP_Z) + float(_sv.VARIANTS[args.sim_variant].get('shelf_dz', 0.0))
assert abs(env.shelf_top_z - _want_top) < 1e-9, (env.shelf_top_z, _want_top)
assert env.phase_sparse and env.scope == 'place' and env.action_repeat == REPEAT and env.delta_ref == 'target', vars(env).keys()
entries = list(env._entries)
if args.limit:
    entries = entries[:int(args.limit)]
print(f'[eval-place] {len(entries)} bank entries; shelf_top_z {env.shelf_top_z:.3f} (band {env.shelf_top_z + 0.01:.3f}..{env.shelf_top_z + 0.07:.3f}); '
      f'delta cap {env.delta_cap} leash {env.delta_leash}', flush=True)

# ---- policy -> normalized [-1,1]^7 action for FullTaskEnv.step ----
_dev = []
if args.kind == 'sac':
    from stable_baselines3 import SAC
    model = SAC.load(str(ck), device='cpu')
    torch.manual_seed(args.seed); np.random.seed(args.seed)   # SAC.load reseeds with the TRAINING seed
    DET = (args.mode == 'mode')

    def act(state):
        a, _ = model.predict(state, deterministic=DET)
        if not DET:
            _dev.append(float(np.abs(a - model.predict(state, deterministic=True)[0]).mean()))
        return np.asarray(a, np.float32)

    def policy_reset():
        pass
    act_selection = ('deterministic' if DET else f'sampled(seed={args.seed})')
else:
    from dp_runner import load_dp_runner
    dev = ('cuda' if torch.cuda.is_available() else 'cpu') if args.device == 'auto' else args.device
    dp_action, dp_reset, _proprio = load_dp_runner(str(ck), device=dev)
    print(f'[eval-place] dp policy on {dev}; hold-{REPEAT}: q* -> delta clip((q*-target)/({REPEAT}*{DJ_CAP})), grip 0..1 -> [-1,1]', flush=True)

    def act(state):
        phys = np.asarray(dp_action({'state': state}), np.float64)          # [q*(6), grip 0..1]
        d = np.clip((phys[:6] - env._dj_target) / (REPEAT * DJ_CAP), -1.0, 1.0)
        g = 2.0 * float(np.clip(phys[6], 0.0, 1.0)) - 1.0
        return np.concatenate([d, [g]]).astype(np.float32)

    def policy_reset():
        dp_reset()
    act_selection = f'sampled(seed={args.seed})'

# ---- episodes: one per entry, in order ----
import cv2   # noqa: E402
OUT = pl.Path(args.out); OUT.mkdir(parents=True, exist_ok=True)
STAGES = ('picked', 'placed', 'placed_v2', 'contact', 'nested')
counts = dict(placed_v2=0, tipped=0, timeout=0, restore_failed=0)
stage_counts = {k: 0 for k in STAGES}
results = []
t_all = time.time()
for k, e in enumerate(entries):
    t0 = time.time()
    uid = int(e['uid']); frame = int(e.get('frame', -1))
    env._entries = [e]                                   # exactly THIS entry (the env samples its pool uniformly)
    try:
        obs, info = env.reset()
    except RuntimeError as ex:
        if 'no entry survived restore' not in str(ex):
            raise
        counts['restore_failed'] += 1
        results.append(dict(ep=k, uid=uid, entry_frame=frame, outcome='restore_failed', steps=0, reward=0.0, seconds=round(time.time() - t0, 1),
                            video=None, stages={s: False for s in STAGES}))
        print(f'ep{k}: uid{uid}@{frame} restore_failed (counted as failure)', flush=True)
        continue
    policy_reset()
    frames = []
    if args.video:
        frames.append(np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1])
    done = False; t = 0; ep_r = 0.0; info = {}
    while not done:
        a = act(np.asarray(obs, np.float32))
        obs, r, term, trunc, info = env.step(a)
        ep_r += float(r); t += 1
        if args.video:
            frames.append(np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1])
        done = bool(term or trunc)
    success = ('placed_v2' in env._granted) or bool(info.get('placed_v2'))
    tipped = bool(info.get('tipped'))
    outcome = 'placed_v2' if success else ('tipped' if tipped else 'timeout')
    counts[outcome] += 1
    gr = set(env._granted) | {s for s in STAGES if bool(info.get(s))}
    st = {s: bool(s in gr) for s in STAGES}
    for s in STAGES:
        stage_counts[s] += int(st[s])
    vid = None
    if args.video and frames:
        vid = str(OUT / f'ep{k}_uid{uid}_{outcome}.mp4')
        vw = cv2.VideoWriter(vid, cv2.VideoWriter_fourcc(*'mp4v'), 30.0 / REPEAT, (frames[0].shape[1], frames[0].shape[0]))
        for fr in frames:
            vw.write(np.ascontiguousarray(fr.astype(np.uint8)))
        vw.release()
    results.append(dict(ep=k, uid=uid, entry_frame=frame, outcome=outcome, steps=t, reward=ep_r, seconds=round(time.time() - t0, 1), video=vid, stages=st))
    print(f'ep{k}: uid{uid}@{frame} {outcome} ({t} decisions, r={ep_r:.1f}, {time.time() - t0:.1f} s)', flush=True)

n = max(len(results), 1)
try:
    git = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO), capture_output=True, text=True, timeout=5).stdout.strip() or 'unknown'
except Exception:
    git = 'unknown'
summary = dict(checkpoint=str(ck), kind=args.kind, arm=args.arm, tag=args.tag, episodes=len(results), mode=args.mode, seed=args.seed,
               max_steps=args.max_steps, ic_mode='bank', entry_bank=str(args.entry_bank), scope='place', sim_variant=args.sim_variant,
               action_repeat=REPEAT, act_selection=act_selection, bank_version=bank_version, delta_cap=env.delta_cap, delta_leash=env.delta_leash,
               placed_v2=counts['placed_v2'] / n, tipped=counts['tipped'] / n, timeout=counts['timeout'] / n, restore_failed=counts['restore_failed'] / n,
               stages={s: stage_counts[s] / n for s in STAGES},
               mean_steps=float(np.mean([r['steps'] for r in results])) if results else 0.0,
               mean_reward=float(np.mean([r['reward'] for r in results])) if results else 0.0,
               sample_dev_mean=(float(np.mean(_dev)) if _dev else None), sample_dev_max=(float(np.max(_dev)) if _dev else None),
               restore_survival=f'{env.place_survived}/{env.place_attempts}', seconds=round(time.time() - t_all, 1),
               node=dict(hostname=socket.gethostname(), slurm_job_id=os.environ.get('SLURM_JOB_ID'), slurm_nodelist=os.environ.get('SLURM_JOB_NODELIST'),
                         cuda_visible=os.environ.get('CUDA_VISIBLE_DEVICES')), git=git, sidecar=str(sc_path),
               per_episode=results)
(OUT / 'metrics.json').write_text(json.dumps(summary, indent=1))
print(f'\n[eval-place] {len(results)} episodes ({args.mode}, {os.path.basename(args.entry_bank)}): placed_v2 {counts["placed_v2"]}/{len(results)} '
      f'({summary["placed_v2"]:.3f})  tipped {counts["tipped"]}  timeout {counts["timeout"]}  restore_failed {counts["restore_failed"]}  '
      f'mean_steps {summary["mean_steps"]:.0f}' + (f'  sample_dev_mean {summary["sample_dev_mean"]:.4f}' if _dev else '') + f'  [{summary["seconds"]:.0f} s]', flush=True)
print(f'[eval-place] wrote {OUT}/metrics.json' + (f' + {len([r for r in results if r["video"]])} mp4s' if args.video else ''), flush=True)
