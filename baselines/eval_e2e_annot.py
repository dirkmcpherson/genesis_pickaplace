#!/usr/bin/env python
"""END-TO-END (full-task) evaluator for RLPD (SB3 SAC .zip) and Diffusion Policy (lerobot dir) checkpoints
(PHASE_PLAN_2026-09-04 amendment (n), 2026-09-07).

Protocol -- the world model's end-to-end cells (amendment (d), PHASE_RESULTS §5/§5.1) with the (j)/(l') columns:
  * `FullTaskEnv(scope='full')` in the corrected world (--sim-variant, also exported as GENESIS_SIM_VARIANT so the
    env's shelf band follows the world's shelf), horizon --max-steps SIM steps (1200 = 300 decisions at repeat 4),
    action_repeat / delta_joint cap 0.025 / leash 5x / delta_ref target asserted against the checkpoint sidecar.
    A UNIFIED ladder, named by --ladder and taken from the checkpoint sidecar when it records one:
    'staged' = picked 1 / placed_v2 1 / contact_push 2 / slide_success 4 (terminal), max return 8;
    'sparse' = nested_v2 1 (terminal), max return 1. That ladder's terminal stage and the tip rule
    terminate and NOTHING else does -- i.e. exactly the training MDP.
  * Starts come from an IC FILE (--ic-file/--ic-set), one episode per start, IN ORDER, each exactly once:
    `hold` (training starts -- in-distribution, NOT held out, REVIEW_GUIDE §8 item 7), `rnd` (the random box,
    out-of-distribution), `spots60` (amendment (k), the in-training-distribution test set), `rnd300`.
    The IC is applied through the env's OWN reset: `FullTaskEnv.reset` is called normally and its single call to
    `GenesisCanEnv.reset` is redirected to this episode's kwargs, so every piece of FullTaskEnv bookkeeping
    (_t, _granted, _hold_run, _pv2_run, _sync_dj_target) runs the env's code, not a copy of it. For a pose IC
    (uid null) the env still draws a placeholder success uid for its own info dict; the TRUE start is recorded in
    per_episode['ic'].
  * ONE post-episode settle per episode, after the last decision: `GenesisCanEnv.end_of_episode()` -- the landed
    (j)/(l') implementation, never re-implemented here. It returns the honest settled `nested` and the
    `slide_success` window with its route. The settle is now a REFERENCE reading only: `nested_honest` is
    what `nested_v2` is validated against, and the (l) settle route is kept as a legacy column.
    `GenesisCanEnv` never reaches its own horizon, so this call is the only settle that ever runs.

Stage columns (success-by-stage = granted at any time in the episode), LADDER_UNIFY_BRIEF D3/D4 2026-09-10:
  HEADLINE  picked, placed_v2, contact_push, slide_success, nested_v2, nested_honest
  LEGACY    placed, contact, nested_proxy, contact_push_legacy, slide_success_settle
`slide_success` is now the env's IN-EPISODE value -- under 'staged' the paid top rung and the only non-tip terminal --
not the post-episode settle route. `nested_v2` replaces `nested_proxy` everywhere; `nested_honest` stays as the
settled REFERENCE column nested_v2 is validated against. `nested_proxy` is kept only for continuity with stored
rows: its precision is 0.114 (human) / 0.029 (machine) and it REVERSES the arm ordering, so nesting must never be
reported on it. Every metrics.json carries `ladder_provenance` (D6); a table builder refuses to merge rows with
different stamps.

Action selection: sac --mode sample = predict(deterministic=False) (one draw per decision, torch seeded by
--seed), --mode mode = predict(deterministic=True). dp is sampled by construction (diffusion noise, seeded by
--seed); --mode mode is refused for dp. Both act THROUGH the env's own delta_joint integrator (one
FullTaskEnv.step per decision): sac emits normalized deltas natively; DP's absolute window-end joint target q*
becomes a = clip((q* - target)/(repeat*cap)) -- wandb_eval.py's DP hold-N rule -- and grip 0..1 -> [-1,1].

Output: <out>/metrics.json (episodes, per-stage rates, outcome counts, per_episode with ic/outcome/steps/reward/
stages/slide_route), one mp4 per episode when --video.

usage: eval_e2e.py --kind sac|dp --checkpoint <zip|pretrained_model dir> --ic-file <json> --ic-set <key> --out <dir>
                   [--mode sample|mode] [--seed 0] [--max-steps 1200] [--sim-variant gc_kp4_riser3_shelf6]
                   [--video] [--limit N] [--device auto|cpu|cuda]
"""
import os, sys, json, time, argparse, pathlib as pl, socket, subprocess

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT', pl.Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(REPO / 'baselines')); sys.path.insert(0, str(REPO / 'baselines' / 'rl'))

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--kind', choices=('sac', 'dp'), required=True)
ap.add_argument('--checkpoint', required=True)
ap.add_argument('--ic-file', required=True, help='baselines/eval_ics*.json (schema resolved by make_eval_ics.episodes_from_file)')
ap.add_argument('--ic-set', required=True, help='hold | rnd | spots60 | sel | any list key of --ic-file')
ap.add_argument('--out', required=True)
ap.add_argument('--mode', choices=('sample', 'mode'), default='sample')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--max-steps', type=int, default=1200, help='SIM steps per episode (1200 = the full-scope cap, 300 decisions at repeat 4)')
ap.add_argument('--sim-variant', default='gc_kp4_riser3_shelf6')
ap.add_argument('--ladder', choices=('staged', 'sparse'), default=None,
                help="WHICH reward ladder the evaluation env runs (FullTaskEnv(ladder=...)). It must match the "
                     "checkpoint's -- a policy trained under one objective scored under another is a different "
                     "experiment, and the stamp in metrics.json is what a table builder checks. Taken from the "
                     "checkpoint sidecar, which is the SOURCE; --ladder is an optional ASSERTION and only an "
                     "explicit disagreement is fatal (a 'staged' default made sparse checkpoints unevaluable).")
ap.add_argument('--video', action='store_true', help='one mp4 per episode (240x320, one frame per decision)')
ap.add_argument('--records-out', default=None,
                help="Per-episode PER-ENV-FRAME stage record, in the format "
                     "baselines/rl/relabel_reward.py --records-out writes for demonstration tapes (its FrameRecorder "
                     "is imported, not re-implemented). Re-score offline under any ladder with "
                     "baselines/diagnostics/pilot_rescore.py. DEFAULT OFF; off, nothing changes. Termination is NOT "
                     "suppressed, so the episode ends where the ladder it ran under ends it.")
ap.add_argument('--limit', type=int, default=None, help='first N starts only (smokes)')
ap.add_argument('--ic-index', type=int, default=None,
                help='EPISODE ISOLATION (coordinator 2026-09-07): evaluate ONLY the k-th start of the (already '
                     '--limit-truncated) list, in this process. One process per episode is the only way to make '
                     'full-scope episodes independent -- Genesis allows one world per process, so a re-init inside '
                     'the process is not available. cluster/e2e_eval_cells.sh drives the loop and merges the '
                     'single-episode metrics with baselines/merge_e2e_iso.py.')
ap.add_argument('--device', default='auto', help='policy device for dp (auto = cuda if visible); sac is always cpu')
ap.add_argument('--require-isa', default=None, choices=('avx2', 'avx512'),
                help="Fail LOUDLY unless this machine's instruction-set class matches. DIAGNOSTIC ONLY -- do not "
                     "use it as the pinning guard. The instruction-set question is UNRESOLVED, not ruled out: the "
                     "original AVX2-vs-AVX-512 claim AND its later withdrawal both leaned on Slurm's "
                     "`AvailableFeatures` CPU-family labels, which are wrong on this cluster (a node advertising "
                     "broadwell is a Cascade Lake part), so neither is usable evidence about instruction sets. What "
                     "IS established rests only on processor counts, which are reliable -- see --require-cores. The "
                     "isa/avx512f stamps are kept precisely so that a future re-check of families against "
                     "/proc/cpuinfo, rather than Slurm's labels, is possible on cells that already exist.")
ap.add_argument('--require-cores', type=int, default=None,
                help='Fail LOUDLY unless the MACHINE has exactly this many physical cores (sockets x cores-per-socket '
                     "read from /proc/cpuinfo, which reports the whole node inside a cgroup -- i.e. machine size). "
                     'THIS IS THE GUARD OF RECORD (coordinator verdict 2026-09-07: `cores`). It rests only on '
                     'processor counts, which are reliable unlike the CPU-family labels: 53 of 53 same-core-count '
                     'comparisons were bit-identical across nodes, labels and code versions, and all 19 disagreements '
                     'had a 36-core machine on exactly one side. Sufficient on every comparison on record, and '
                     'checkable before submission.')
ap.add_argument('--threads', type=int, default=None,
                help='Pin the per-task thread count deterministically BEFORE torch/genesis are imported (OMP, MKL, '
                     'OpenBLAS, NUMEXPR, Taichi, torch). Use it ALONGSIDE --require-cores, never instead of it: the '
                     '128-comparison audit that established the core-count rule never set a thread variable, so in '
                     'every one of those comparisons the thread count simply WAS the physical core count -- the '
                     'evidence is structurally incapable of showing that a thread pin alone suffices.')
ap.add_argument('--role', choices=('preview', 'record'), default='preview',
                help="'preview' (DEFAULT) = a convenience cell produced wherever the training job happened to land; "
                     "NEVER a number for a table. 'record' = a cell of the pinned evaluation pass, and it REQUIRES "
                     "--require-isa, so a cell of record cannot exist without its instruction-set class being pinned "
                     "by construction (coordinator 2026-09-07: hardware class is partially confounded with arm in "
                     "the published 8v8, amendment (t); we do not repeat that by accepting a scheduling lottery).")
ap.add_argument('--arm', default=None); ap.add_argument('--tag', default=None)
args = ap.parse_args()
if args.kind == 'dp' and args.mode == 'mode':
    sys.exit('FATAL: DP has no deterministic mode (diffusion sampling); run --mode sample')
if args.role == 'record' and not (args.require_cores or args.threads):
    sys.exit('FATAL: --role record needs --require-cores and/or --threads. A cell of record must have its hardware '
             'configuration pinned BY CONSTRUCTION, and the axis is machine size / thread count -- not the '
             'instruction set, whose attribution was withdrawn. Run the pinned pass '
             '(cluster/sbatch_e2e_rescore.sh) or use --role preview.')


def cpu_probe():
    """Machine description read from /proc/cpuinfo -- NEVER from Slurm's feature labels, which mislabel at least one
    Cascade Lake node as `broadwell` and which are the reason both the original AVX attribution and its withdrawal
    are unusable as evidence about instruction sets. Inside a cgroup /proc/cpuinfo still reports the whole NODE, so
    the core counts below are MACHINE SIZE -- the one variable the divergence is established to track (53/53
    same-core-count comparisons bit-identical; all 19 disagreements with a 36-core machine on exactly one side). The
    task's own allocation is read from the affinity mask."""
    model, flags, sockets, cores_per_socket, logical = 'unknown', set(), set(), None, 0
    try:
        with open('/proc/cpuinfo') as fh:
            for line in fh:
                if line.startswith('processor'):
                    logical += 1
                elif line.startswith('model name') and model == 'unknown':
                    model = line.split(':', 1)[1].strip()
                elif line.startswith('flags') and not flags:
                    flags = set(line.split(':', 1)[1].split())
                elif line.startswith('physical id'):
                    sockets.add(line.split(':', 1)[1].strip())
                elif line.startswith('cpu cores') and cores_per_socket is None:
                    cores_per_socket = int(line.split(':', 1)[1].strip())
    except Exception as e:      # never fail an evaluation over a stamp
        print(f'[eval-e2e] WARNING: could not read /proc/cpuinfo ({e}); cpu stamps will say unknown', flush=True)
    n_sockets = max(len(sockets), 1)
    cores = (cores_per_socket * n_sockets) if cores_per_socket else (logical or 0)
    avx512f = 'avx512f' in flags
    isa = 'avx512' if any(f.startswith('avx512') for f in flags) else ('avx2' if 'avx2' in flags else 'pre-avx2')
    try:
        affinity = len(os.sched_getaffinity(0))
    except Exception:
        affinity = None
    return dict(cpu_model=model, isa=isa, avx512f=avx512f, cpu_cores_physical=cores, cpu_sockets=n_sockets,
                cpu_logical=logical, cpu_affinity=affinity)


HW = cpu_probe()
CPU_MODEL, CPU_ISA, CPU_AVX512F = HW['cpu_model'], HW['isa'], HW['avx512f']
if args.require_isa and CPU_ISA != args.require_isa:
    sys.exit(f'FATAL: --require-isa {args.require_isa} but this machine is {CPU_ISA} ({CPU_MODEL} on '
             f'{socket.gethostname()}). NOTE: --require-isa is a DIAGNOSTIC; the instruction-set question is '
             f'unresolved (the family labels behind it are unreliable). --require-cores is the guard of record.')
if args.require_cores is not None and int(HW['cpu_cores_physical']) != int(args.require_cores):
    sys.exit(f'FATAL: --require-cores {args.require_cores} but this machine has {HW["cpu_cores_physical"]} physical '
             f'cores ({HW["cpu_sockets"]} socket(s), {CPU_MODEL} on {socket.gethostname()}). Full-scope outcomes '
             f'track MACHINE SIZE (coordinator verdict 2026-09-07: `cores`); refusing to produce a cell on the '
             f'wrong size.')
if args.threads is not None:
    # must happen BEFORE torch / genesis / taichi are imported: they read these at import time, and the thread count
    # changes reduction order, which is the whole point of pinning it.
    assert args.threads >= 1, args.threads
    for _v in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS', 'TI_NUM_THREADS'):
        os.environ[_v] = str(args.threads)
HW['threads_requested'] = args.threads
HW['omp_num_threads'] = os.environ.get('OMP_NUM_THREADS')

import numpy as np   # noqa: E402
import torch         # noqa: E402
if args.threads is not None:
    torch.set_num_threads(int(args.threads))
HW['torch_num_threads'] = int(torch.get_num_threads())
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
    assert side.get('scope') == 'full', f'end-to-end evaluator on a scope={side.get("scope")!r} checkpoint'
sv_side = side.get('sim_variant')
if sv_side and sv_side != args.sim_variant:
    sys.exit(f'FATAL: sidecar sim_variant={sv_side} != --sim-variant {args.sim_variant}')
assert REPEAT == 4, f'the end-to-end protocol is action_repeat 4; sidecar says {REPEAT}'
DJ_CAP, DJ_LEASH_MULT = 0.025, 5.0
if side.get('delta_cap') is not None:
    assert abs(float(side['delta_cap']) - DJ_CAP) < 1e-9, f'sidecar delta_cap={side["delta_cap"]} != {DJ_CAP}'
if side.get('delta_leash') is not None:
    assert abs(float(side['delta_leash']) - DJ_CAP * DJ_LEASH_MULT) < 1e-9, f'sidecar delta_leash={side["delta_leash"]} != {DJ_CAP * DJ_LEASH_MULT}'
print(f'[eval-e2e] kind={args.kind} mode={args.mode} seed={args.seed} repeat={REPEAT} sim_variant={args.sim_variant} '
      f'ic={args.ic_file}:{args.ic_set} max_steps={args.max_steps} sidecar={sc_path}', flush=True)

# ---- ICs ----
from make_eval_ics import episodes_from_file   # noqa: E402
ics = episodes_from_file(str(REPO / args.ic_file) if not os.path.isabs(args.ic_file) else args.ic_file, args.ic_set)
if args.limit:
    ics = ics[:int(args.limit)]
assert ics, f'no ICs in {args.ic_file}:{args.ic_set}'
IC_OFFSET = 0
if args.ic_index is not None:
    assert 0 <= args.ic_index < len(ics), f'--ic-index {args.ic_index} out of range (n={len(ics)})'
    IC_OFFSET = int(args.ic_index); ics = [ics[IC_OFFSET]]
ISOLATION = 'fresh_process' if args.ic_index is not None else 'shared_process'
print(f'[eval-e2e] {len(ics)} start(s) from {args.ic_file}:{args.ic_set} (offset {IC_OFFSET}, isolation {ISOLATION}) '
      f'({sum(1 for e in ics if e.get("uid") is not None)} uid starts, {sum(1 for e in ics if e.get("uid") is None)} pose starts) '
      f'node={socket.gethostname()} pid={os.getpid()} cores={HW["cpu_cores_physical"]}p/{HW["cpu_logical"]}l '
      f'affinity={HW["cpu_affinity"]} threads={HW.get("torch_num_threads")} isa={CPU_ISA} cpu="{CPU_MODEL}" '
      f'role={args.role}', flush=True)

# ---- env: the full scope of the shared full_env, corrected world ----
os.environ['GENESIS_SIM_VARIANT'] = args.sim_variant
from sim_variant_hook import apply_pre, apply_post   # noqa: E402
apply_pre(args.sim_variant)
from full_env import FullTaskEnv, STAGE_REWARD, refuse_legacy_gates   # noqa: E402
refuse_legacy_gates()   # D1: this tree has no reward gates; a stale export must not pass
import sim_variants as _sv                       # noqa: E402
from replay_harness import BOX_TOP_Z             # noqa: E402
# The OVERLAY is the one Lane 5 delivered for DEMONSTRATIONS (`annotate_demos._draw_panel`,
# `_terminal_card`, `write_mp4`, and its chip sets). It is IMPORTED, never re-implemented, for
# two reasons: a policy clip then reads exactly like the demonstration clips the user has
# already reviewed (same chips, same colours, same diagnostics line, same terminal card), and
# there is ONE place where the conventions live. The overlay computes NO predicate of its own --
# every chip is `info` / `env._granted` and every diagnostic is `env.tracker`, read back.
import annotate_demos as AD                      # noqa: E402
# The ladder comes from the CHECKPOINT when its sidecar records one (runs trained before the
# `ladder` argument existed do not), and --ladder must agree with it. Scoring a policy under a
# different objective from the one it optimised is a different experiment, not a detail.
LADDER_NAME = side.get('ladder') or args.ladder or 'staged'
if side.get('ladder') and args.ladder and side['ladder'] != args.ladder:
    sys.exit(f"FATAL: checkpoint sidecar says ladder={side['ladder']!r} but --ladder is {args.ladder!r}")
env = FullTaskEnv(backend='cpu', max_steps=args.max_steps, scope='full', ladder=LADDER_NAME,
                  action_mode='delta_joint', delta_cap=DJ_CAP, delta_leash_mult=DJ_LEASH_MULT, action_repeat=REPEAT,
                  delta_ref='target',
                  # same camera the demonstration clips used, so the two sets are comparable
                  render_size=(AD.RENDER_HW if args.video else None))
apply_post(env, args.sim_variant)
_want_top = float(BOX_TOP_Z) + float(_sv.VARIANTS[args.sim_variant].get('shelf_dz', 0.0))
assert abs(env.shelf_top_z - _want_top) < 1e-9, (env.shelf_top_z, _want_top)
assert env.scope == 'full' and env.action_repeat == REPEAT and env.delta_ref == 'target' and not env.phase_sparse
assert env.max_steps == args.max_steps, (env.max_steps, args.max_steps)
print(f'[eval-e2e] shelf_top_z {env.shelf_top_z:.3f} (placed_v2 band {env.shelf_top_z + 0.01:.3f}..{env.shelf_top_z + 0.07:.3f}); '
      f'delta cap {env.delta_cap} leash {env.delta_leash}; ladder {env.ladder} {env.stage_reward} '
      f'terminal {env.terminal_stages}+tipped', flush=True)
LADDER = env.provenance()   # D6: written into metrics.json below

# ---- IC injection: run the env's OWN reset, redirect its single GenesisCanEnv.reset call to this episode's start.
# FullTaskEnv.reset(options={'uid': u}) does its own bookkeeping and then calls self.genv.reset(uid=int(u)); it has no
# pose-IC path (the pick evaluators drive GenesisCanEnv directly). Wrapping genv.reset keeps 100% of the env's reset
# logic and only substitutes WHICH start is restored -- no duplicated bookkeeping, no silent divergence.
_genv_reset = env.genv.reset
_CUR = {'ic': None}


def _reset_hook(*a, **kw):
    return _genv_reset(**_CUR['ic'])


env.genv.reset = _reset_hook
_placeholder_uid = int(sorted(env.success_uids)[0])

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
    print(f'[eval-e2e] dp policy on {dev}; hold-{REPEAT}: q* -> delta clip((q*-target)/({REPEAT}*{DJ_CAP})), grip 0..1 -> [-1,1]', flush=True)

    def act(state):
        phys = np.asarray(dp_action({'state': state}), np.float64)          # [q*(6), grip 0..1]
        d = np.clip((phys[:6] - env._dj_target) / (REPEAT * DJ_CAP), -1.0, 1.0)
        g = 2.0 * float(np.clip(phys[6], 0.0, 1.0)) - 1.0
        return np.concatenate([d, [g]]).astype(np.float32)

    def policy_reset():
        dp_reset()
    act_selection = f'sampled(seed={args.seed})'

# ---- episodes: one per start, in order ----
import cv2   # noqa: E402
OUT = pl.Path(args.out); OUT.mkdir(parents=True, exist_ok=True)
CKPT_TAG = args.tag or ck.parent.name
REC_DIR = None
if args.records_out:
    from relabel_reward import FrameRecorder, grip_phys_from_action, _cpu_stamp   # noqa: E402
    REC_DIR = pl.Path(args.records_out); REC_DIR.mkdir(parents=True, exist_ok=True)
    print(f'[eval-e2e] stage records -> {REC_DIR}', flush=True)
rec_rows = []


def _ic_json(ic):
    return {kk: (list(vv) if isinstance(vv, (tuple, list, np.ndarray)) else vv) for kk, vv in ic.items()}

# Borderline bands -- the ONLY numbers invented in this file, and they select clips, they never
# decide a flag. Each is "just outside a clause", i.e. the episodes where a slightly different
# constant would flip `nested_v2` and therefore `slide_success`.
BORDER_GAIN = (0.005, 0.010)        # goalward gain 5-9 mm: just under PUSH_GAIN_MM
BORDER_DIST = (0.081, 0.100)        # can 81-100 mm from the goal: just outside NESTED_TOUCH_DIST


def classify_policy(r, e):
    """Stratification label for ONE policy episode. Ordered, first match wins, so every episode
    has exactly one class.

    This is BUCKETING OF RECORDED FLAGS, not a predicate: every term is a flag the env already
    wrote (`r`, from `env._granted` / `info`) or a tracker reading (`e`). It invents no physics
    and it can change no cell. The classes are the ones the smoke test is looking for -- the
    pairs where two rungs disagree are where a bad clause would show.

    Returns (class, borderline-reason or None).
    """
    g = r['grants']
    released = bool(r['placed_v2'])          # `released` IS placed_v2-granted (stage_predicates)
    if r['slide_success']:
        k = 'slide'
    elif r['nested_v2'] and not r['pushed']:
        k = 'nested_drop'                    # arrived, but no 10 mm of post-release goalward gain
    elif r['nested_v2']:
        # nested_v2 AND pushed AND released is exactly slide_success, so reaching here means the
        # STICKY nested_v2 fired on a frame before `pushed` did -- an ordering artefact worth a look.
        k = 'nested_pushed_not_slide'
    elif r['tipped'] and not released:
        k = 'tipped_before_release'
    elif r['tipped']:
        k = 'tipped_after_release'
    elif r['contact_push']:
        k = 'push_no_nest'                   # far-side contact after release, never settled/arrived
    elif r['pushed']:
        k = 'pushed_no_contact'              # goalward gain without a far-side contact frame
    elif g.get('contact_push_legacy') is not None and not released:
        k = 'held_press_timeout'             # presses the HELD can into the goal to the horizon
    elif r['placed_v2'] and not r['picked']:
        k = 'reset_artifact'                 # rnd30 start already inside the shelf footprint
    elif r['placed_v2']:
        k = 'placed_only'
    elif r['picked']:
        k = 'picked_only'
    else:
        k = 'nopick'
    why = []
    if e.get('goalward_gain_m') is not None and BORDER_GAIN[0] <= e['goalward_gain_m'] < BORDER_GAIN[1]:
        why.append('gain %.1f mm (push needs 10)' % (e['goalward_gain_m'] * 1000))
    if e.get('dist_xy_m') is not None and BORDER_DIST[0] < e['dist_xy_m'] <= BORDER_DIST[1]:
        why.append('dist %.1f mm (nest needs <= 81)' % (e['dist_xy_m'] * 1000))
    if (e.get('dist_xy_m') is not None and e['dist_xy_m'] <= 0.081
            and e.get('at_rest') is False and not r['nested_v2']):
        why.append('within 81 mm but at_rest False on the last frame')
    if (e.get('dist_xy_m') is not None and e['dist_xy_m'] <= 0.081
            and e.get('in_hand') is True and not r['nested_v2']):
        why.append('within 81 mm but still in hand (lever %.1f mm)' % ((e.get('lever_m') or 0) * 1000))
    return k, ('; '.join(why) or None)
# Stage columns (LADDER_UNIFY_BRIEF D3/D4, 2026-09-10). HEADLINE = the unified ladder's own
# rungs plus nested_v2 (which REPLACES nested_proxy in every log, table and figure) and the
# settled nested_honest, which stays as the post-hoc REFERENCE column nested_v2 is validated
# against. LEGACY = kept so old rows stay readable, never in a headline: `placed` is the
# stale base-world band, `contact` the carry-in predicate, `nested_proxy` the withdrawn
# training proxy (precision 0.114 human / 0.029 machine, and it REVERSES the arm ordering --
# audit brief §4a), `contact_push_legacy` the (g) predicate that needs no release, and
# `slide_success_settle` the (l) settle route with its withdrawn grip clause.
HEADLINE_STAGES = ('picked', 'placed_v2', 'contact_push', 'slide_success', 'nested_v2', 'nested_honest')
LEGACY_STAGES = ('placed', 'contact', 'nested_proxy', 'contact_push_legacy', 'slide_success_settle')
STAGES = HEADLINE_STAGES + LEGACY_STAGES
# The outcome taxonomy's success is THE LADDER'S OWN PAID TERMINAL (see eval_e2e.py): under
# `sparse` that is `nested_v2`, and the hardcoded staged name recorded every sparse success
# without `pushed` as a `timeout`.
TERMINAL_STAGE = next((k for k in LADDER['terminal_stages'] if k != 'tipped'), 'slide_success')
OUTCOMES = (TERMINAL_STAGE, 'tipped', 'timeout')
counts = {k: 0 for k in OUTCOMES}
stage_counts = {k: 0 for k in STAGES}
routes = {}
results = []
t_all = time.time()
for k, ic in enumerate(ics):
    t0 = time.time()
    _CUR['ic'] = dict(ic)
    uid = ic.get('uid')
    obs, info0 = env.reset(options={'uid': int(uid) if uid is not None else _placeholder_uid})
    policy_reset()

    def _render():
        return np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1].astype(np.uint8)

    # frame-aligned overlay state, exactly the schedule annotate_demos uses: frame 0 is the
    # RESET state (no decision taken), frame j>0 shows the world after decision j-1.
    frames, snaps, diags, decs, rewards = [], [], [], [], []
    DIAG0 = dict(lever_m=float('nan'), in_hand=False, at_rest=False, goalward_gain_m=0.0,
                 pushed=False, dist_xy_m=float('nan'), grip_cmd=float('nan'))
    if args.video:
        frames.append(_render()); snaps.append(frozenset()); diags.append(dict(DIAG0))
        decs.append(0); rewards.append(0.0)
    first, grants = {}, {}          # chip key -> first decision; REPORT key -> first decision
    sparse_dec = None               # decision a `sparse` episode would have paid and ENDED on
    diag = dict(DIAG0)
    done = False; t = 0; ep_r = 0.0; info = {}
    end_reason, end_dec, tipped_ever = 'truncated', 0, False
    rec = FrameRecorder(env) if REC_DIR is not None else None
    rec_arrays = None
    while not done:
        a = act(np.asarray(obs, np.float32))
        if rec is not None:
            rec.new_decision(t, grip_phys_from_action(a))
        obs, r, term, trunc, info = env.step(a)
        d = t                        # 0-based index of the decision just executed
        ep_r += float(r); t += 1
        # sticky stage set: the env's own grants ORed with this frame's info, the same rule
        # eval_e2e.py's `_g()` uses. Read, never recomputed.
        sticky = set(env._granted) | {kk for kk in AD.ALL_CHIP_KEYS + list(AD.REPORT)
                                      if info.get(kk)}
        for kk in sticky:
            if kk in AD.REPORT and kk not in grants:
                grants[kk] = d
        for kk in AD.ALL_CHIP_KEYS:
            if kk in sticky and kk not in first:
                first[kk] = d
        if info.get('nested_v2') and sparse_dec is None:
            sparse_dec = d
        tipped_ever = tipped_ever or bool(info.get('tipped'))
        # diagnostics straight off the shared StageTracker instance the env just updated
        tr = env.tracker
        diag = dict(lever_m=float(tr.lever_m), in_hand=bool(tr.in_hand), at_rest=bool(tr.at_rest),
                    goalward_gain_m=float(tr.goalward_gain_m), pushed=bool(tr.pushed),
                    dist_xy_m=float(tr.dist_xy_m),
                    grip_cmd=float((float(np.clip(a[6], -1.0, 1.0)) + 1.0) / 2.0))
        if args.video:
            frames.append(_render()); snaps.append(frozenset(sticky)); diags.append(dict(diag))
            decs.append(d + 1); rewards.append(ep_r)
        done = bool(term or trunc)
        if done:
            end_dec = d + 1
            end_reason = ('tipped' if info.get('tipped') else
                          (next((s for s in env.terminal_stages
                                 if s != 'tipped' and info.get(s)), None)
                           or ('truncated' if trunc else 'terminated')))
    # The IN-EPISODE slide_success is now the statistic (D4): it is the env's own PAID,
    # TERMINAL rung, decided by the shared stage tracker during the episode. The post-episode
    # settle still runs, for two reasons and two only: `nested_honest` is the reference
    # column nested_v2 is validated against, and the (l) settle-route slide is kept as a
    # legacy column. Neither is what the reward paid.
    if rec is not None:
        rec_arrays = rec.arrays(); rec.detach()
    end = env.genv.end_of_episode()
    gr = set(env._granted)

    def _g(k):
        return bool(k in gr or info.get(k))

    st = {
        'picked': _g('picked'),
        'placed_v2': _g('placed_v2'),
        'contact_push': _g('contact_push'),
        'slide_success': _g('slide_success'),          # in-episode, paid, terminal
        'nested_v2': _g('nested_v2'),
        'nested_honest': bool(end['nested']),          # settled reference
        # --- legacy columns, never a headline ---
        'placed': _g('placed'),
        'contact': _g('contact'),
        'nested_proxy': _g('nested'),
        'contact_push_legacy': bool(info.get('contact_push_legacy')),
        'slide_success_settle': bool(end['slide_success']),
    }
    tipped = bool(info.get('tipped')) or tipped_ever
    outcome = TERMINAL_STAGE if st[TERMINAL_STAGE] else ('tipped' if tipped else 'timeout')
    counts[outcome] += 1
    for s in STAGES:
        stage_counts[s] += int(st[s])
    route = end.get('slide_route')
    routes[str(route)] = routes.get(str(route), 0) + 1

    # ---- end-of-episode diagnostics: the tracker's LAST update, verbatim. `env._track` is the
    # flag dict the tracker returned on the final frame, so it carries the two tilts and the
    # band test that `info` does not copy. An absent tracker frame (an episode that ended before
    # any full-scope predicate ran) leaves them None -- never 0.
    _tk = getattr(env, '_track', None) or {}
    endiag = dict(
        lever_m=(float(env.tracker.lever_m) if env.tracker.t else None),
        dist_xy_m=(float(env.tracker.dist_xy_m) if env.tracker.t else None),
        in_hand=(bool(env.tracker.in_hand) if env.tracker.t else None),
        at_rest=(bool(env.tracker.at_rest) if env.tracker.t else None),
        goalward_gain_m=(float(env.tracker.goalward_gain_m) if env.tracker.t else None),
        pushed=bool(env.tracker.pushed), released=bool(env.tracker.released),
        can_tilt_deg=(float(_tk['can_tilt_deg']) if 'can_tilt_deg' in _tk else None),
        goal_tilt_deg=(float(_tk['goal_tilt_deg']) if 'goal_tilt_deg' in _tk else None),
        in_band=(bool(_tk['in_band']) if 'in_band' in _tk else None),
        # `nested_v2` in the stage table above is STICKY (`_granted`). On policy episodes the
        # sticky read has precision 0.571 against the settle and the final-frame read has 1.000
        # (NESTED_V2_PREDICATE §5.4, recommendation 2), so BOTH are recorded and the class is
        # decided on the sticky one only where the sticky/now split is itself the finding.
        nested_v2_now=bool(env.tracker.nested_v2), nested_v2_ever=bool(env.tracker.nested_v2_ever),
        tracker_frames=int(env.tracker.t))
    row = dict(
        decisions=int(t), end_decision=int(end_dec or t), end_reason=end_reason,
        ladder=str(env.ladder), reward_staged=float(ep_r), reward_recorded=None,
        reward_label='earned', reward_sparse=(1.0 if sparse_dec is not None else 0.0),
        sparse_decision=(None if sparse_dec is None else int(sparse_dec)),
        grants={kk: int(vv) for kk, vv in sorted(grants.items())},
        picked=st['picked'], placed_v2=st['placed_v2'], contact_push=st['contact_push'],
        nested_v2=st['nested_v2'], slide_success=st['slide_success'],
        pushed=bool(env.tracker.pushed), nested_proxy=st['nested_proxy'], contact=st['contact'],
        nested_honest=st['nested_honest'], tipped=bool(tipped),
        header='{RLPD} %s | %s %s | ep%d %s' % (CKPT_TAG, args.ic_set, args.mode, IC_OFFSET + k,
                                                ('uid%d' % uid) if uid is not None else 'rnd'))
    klass, borderline = classify_policy(row, endiag)

    vid = None
    if args.video and frames:
        vid = str(OUT / ('ep%d_%s_%s.mp4' % (IC_OFFSET + k,
                                             ('uid%d' % uid) if uid is not None else 'rnd', klass)))
        # ---- the SHARED overlay (annotate_demos): LADDER chips + legacy row + diagnostics
        # line + reward line + terminal card, drawn by the same functions that drew the
        # demonstration clips. Nothing here computes a predicate.
        keep = list(range(len(frames)))
        if len(frames) + AD.CARD_FRAMES > AD.MAX_FRAMES:
            # uniform body, but NEVER drop a grant frame or its predecessor: those are the
            # frames the clip exists to let the user judge.
            forced = {0, len(frames) - 1}
            for _fd in first.values():
                forced |= {max(0, _fd), min(_fd + 1, len(frames) - 1)}
            body = AD.MAX_FRAMES - AD.CARD_FRAMES - len(forced)
            keep = sorted(forced | set(np.linspace(0, len(frames) - 1, num=max(body, 2),
                                                   dtype=int).tolist()))
        stride_note = ('' if len(keep) == len(frames)
                       else 'subsampled %d/%d' % (len(keep), len(frames)))
        W = int(frames[0].shape[1]) // 2 * 2
        H = int(frames[0].shape[0]) // 2 * 2
        staged_max = float(sum(env.stage_reward.values()))
        n_dec_axis = max(int(end_dec or t), 1)
        out_frames = []
        for j in keep:
            im = cv2.resize(np.ascontiguousarray(frames[j]), (W, H), interpolation=cv2.INTER_LINEAR)
            pan = AD._draw_panel(cv2, W, j, n_dec_axis, decs[j], first, snaps[j], diags[j],
                                 rewards[j], staged_max, row['reward_sparse'], sparse_dec,
                                 stride_note)
            out_frames.append(np.vstack([im, pan]))
        card = AD._terminal_card(cv2, out_frames[-1][:H].copy(), row)
        for _ in range(AD.CARD_FRAMES):
            out_frames.append(np.vstack([card, out_frames[-1][H:]]))
        assert len(out_frames) <= AD.MAX_FRAMES, (len(out_frames), AD.MAX_FRAMES)
        # keep wall-clock: the episode runs at 30/repeat = 7.5 decisions per second
        fps = float(np.clip((30.0 / REPEAT) * len(frames) / max(len(keep), 1), 6.0, 30.0))
        row['codec'] = AD.write_mp4(cv2, vid, out_frames, fps)
        row['video_frames'] = len(out_frames)
        row['video_bytes'] = os.path.getsize(vid)
    rec_path = None
    if REC_DIR is not None:
        rec_path = str(REC_DIR / f'ep{IC_OFFSET + k}_uid{uid if uid is not None else "rnd"}.npz')
        _meta = dict(
            kind='policy_rollout', layout='policy_rollout', checkpoint=str(ck), policy_kind=args.kind,
            arm=(args.arm or ''), tag=(args.tag or ''), mode=args.mode, seed=int(args.seed),
            ic_file=str(args.ic_file), ic_set=str(args.ic_set), ep=int(IC_OFFSET + k), order=int(k),
            uid=(-1 if uid is None else int(uid)), ic=json.dumps(_ic_json(ic)),
            n_decisions=int(t), action_repeat=int(env.action_repeat), max_steps=int(env.max_steps),
            shelf_top_z=float(env.shelf_top_z), sim_variant=str(args.sim_variant),
            reward_recorded_total=float(ep_r), end_reason=str(end_reason),
            nested_honest=bool(end['nested']), slide_success_settle=bool(end['slide_success']),
            slide_route=str(route), record_ladder=str(env.ladder), never_terminate=False,
            live_stages=json.dumps({sk: bool(st[sk]) for sk in STAGES}),
            node=json.dumps(_cpu_stamp()), provenance=json.dumps(env.provenance()))
        np.savez_compressed(rec_path, **rec_arrays, **{kk: np.asarray(vv) for kk, vv in _meta.items()})
        rec_rows.append(dict(file=os.path.basename(rec_path), ep=int(IC_OFFSET + k),
                             uid=(None if uid is None else int(uid)), n_decisions=int(t),
                             frames=int(rec_arrays['dec'].shape[0]), end_reason=str(end_reason),
                             reward=float(ep_r), nested_honest=bool(end['nested'])))
    # node / process / order stamps on EVERY episode (coordinator 2026-09-07): long-horizon full-scope episodes are
    # node-sensitive (same ckpt+IC+seed flips outcome across nodes) and, in the shared-process protocol, ORDER-dependent
    # (state leaks between episodes). `order` is the position within THIS process, so it is 0 for every isolated cell.
    results.append(dict(ep=IC_OFFSET + k, order=k, node=socket.gethostname(), pid=os.getpid(),
                        **{k: HW[k] for k in ('cpu_model', 'isa', 'avx512f', 'cpu_cores_physical', 'cpu_sockets',
                                              'cpu_logical', 'cpu_affinity', 'torch_num_threads', 'omp_num_threads')},
                        ic=_ic_json(ic),
                        uid=(int(uid) if uid is not None else None), outcome=outcome, tipped=tipped, steps=t, reward=ep_r,
                        slide_route=route, seconds=round(time.time() - t0, 1), video=vid, stages=st,
                        **({'record': rec_path} if REC_DIR is not None else {}),
                        # --- the smoke-test record (Lane 9): the first-fire decision of every
                        # flag, the tracker's end-of-episode diagnostics, the end reason, and the
                        # stratification class. Everything here is read back, never recomputed.
                        klass=klass, borderline=borderline, ckpt=CKPT_TAG,
                        end_reason=end_reason, end_decision=int(end_dec or t),
                        grants=row['grants'], end_diag=endiag,
                        reward_sparse=row['reward_sparse'], sparse_decision=row['sparse_decision'],
                        codec=row.get('codec'), video_frames=row.get('video_frames'),
                        video_bytes=row.get('video_bytes')))
    print(f'ep{k}: {"uid%d" % uid if uid is not None else "rnd"} {outcome}/{klass} '
          f'slide={int(st["slide_success"])} nested_v2={int(st["nested_v2"])}'
          f'(now={int(bool(endiag["nested_v2_now"]))}) nestedH={int(st["nested_honest"])} '
          f'push={int(st["contact_push"])} pushed={int(row["pushed"])} '
          f'placed_v2={int(st["placed_v2"])} picked={int(st["picked"])} '
          f'end={end_reason}@d{end_dec} dist={endiag["dist_xy_m"]} gain={endiag["goalward_gain_m"]} '
          f'({t} decisions, r={ep_r:.1f}, {time.time() - t0:.1f} s)'
          + (f'  BORDERLINE: {borderline}' if borderline else ''), flush=True)

n = max(len(results), 1)
try:
    git = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO), capture_output=True, text=True, timeout=5).stdout.strip() or 'unknown'
except Exception:
    git = 'unknown'
summary = dict(checkpoint=str(ck), kind=args.kind, arm=args.arm, tag=args.tag, episodes=len(results), mode=args.mode, seed=args.seed,
               max_steps=args.max_steps, ic_mode='ic_file', ic_file=str(args.ic_file), ic_set=str(args.ic_set), scope='full',
               sim_variant=args.sim_variant, action_repeat=REPEAT, act_selection=act_selection,
               role=args.role, isolation=ISOLATION, ic_index=args.ic_index, ic_offset=IC_OFFSET,
               nodes=sorted({r['node'] for r in results}), pids=sorted({r['pid'] for r in results}),
               cpu_models=sorted({r['cpu_model'] for r in results}), isa_classes=sorted({r['isa'] for r in results}),
               avx512f=sorted({bool(r['avx512f']) for r in results}), require_isa=args.require_isa,
               require_cores=args.require_cores, threads_requested=args.threads,
               core_counts=sorted({int(r['cpu_cores_physical']) for r in results}),
               thread_counts=sorted({r.get('torch_num_threads') for r in results}),
               affinities=sorted({r.get('cpu_affinity') for r in results if r.get('cpu_affinity') is not None}),
               hw_axis=('machine size (physical cores), coordinator verdict `cores` 2026-09-07; established on '
                        'processor counts alone (53/53 same-core comparisons bit-identical, all 19 disagreements '
                        'with a 36-core machine on one side). The instruction-set question is UNRESOLVED, not ruled '
                        'out: the CPU-family labels behind both the original AVX claim and its withdrawal are '
                        'unreliable on this cluster. isa/avx512f are stamped for a future re-check.'),
               delta_cap=env.delta_cap, delta_leash=env.delta_leash, amendment='n+ladder-unify',
               eval_fixes='j+l-prime+ladder-unify',
               # D6: the stamp that says WHICH ladder and WHICH code produced this cell.
               # baselines/e2e_table_all.py REFUSES to merge rows whose stamps differ.
               ladder_provenance=LADDER, ladder_stamp=LADDER['stamp'],
               slide_success=stage_counts['slide_success'] / n,
               headline_stages={s: stage_counts[s] / n for s in HEADLINE_STAGES},
               stages={s: stage_counts[s] / n for s in STAGES},
               stage_counts={s: stage_counts[s] for s in STAGES},
               legacy_stages=list(LEGACY_STAGES),
               outcomes={k: counts[k] / n for k in OUTCOMES}, slide_routes=routes,
               # Lane 9 smoke test: how many episodes landed in each stratification class, and
               # which of them sit just outside a clause. Selection metadata, never a cell.
               class_counts={kk: sum(1 for r in results if r['klass'] == kk)
                             for kk in sorted({r['klass'] for r in results})},
               borderline_eps=[r['ep'] for r in results if r.get('borderline')],
               ckpt_tag=CKPT_TAG,
               stage_notes=dict(
                   slide_success='IN-EPISODE, the ladder top rung (+4) and the only non-tip terminal: '
                                 'placed_v2 granted AND pushed AND nested_v2 (stage_predicates). STATISTIC OF RECORD.',
                   nested_v2='state-only, no settle: picked AND placed_v2 granted AND dist_xy <= 0.081 AND both '
                             'cans upright AND can in the shelf band AND not in hand AND at rest. REPLACES '
                             'nested_proxy in every log, table and figure.',
                   nested_honest='settled proximity predicate from the single end-of-episode settle -- the '
                                 'post-hoc REFERENCE column nested_v2 is validated against (D4).',
                   placed='LEGACY: stale base-world z-band (0.12-0.18) -- do not read in the corrected world',
                   contact='LEGACY: carry-in credit (84-86% of policy grants were a carry, not a push)',
                   nested_proxy='LEGACY, WITHDRAWN as a statistic: the old training proxy. Precision 0.114 '
                                '(human) / 0.029 (machine) against the settled predicate, and it REVERSES the '
                                'arm ordering (audit brief §4a). Never report nesting on it.',
                   contact_push_legacy='LEGACY (g): the same geometry WITHOUT requiring a prior release, so it '
                                       'fires while the robot still holds the can.',
                   slide_success_settle="LEGACY (l): the post-episode settle route, with the grip < 0.3 clause "
                                        "that (p) withdrew (it passed 2 of 74 human demonstrations)."),
               mean_steps=float(np.mean([r['steps'] for r in results])) if results else 0.0,
               mean_reward=float(np.mean([r['reward'] for r in results])) if results else 0.0,
               sample_dev_mean=(float(np.mean(_dev)) if _dev else None), sample_dev_max=(float(np.max(_dev)) if _dev else None),
               seconds=round(time.time() - t_all, 1),
               node=dict(hostname=socket.gethostname(), slurm_job_id=os.environ.get('SLURM_JOB_ID'), slurm_nodelist=os.environ.get('SLURM_JOB_NODELIST'),
                         cuda_visible=os.environ.get('CUDA_VISIBLE_DEVICES')), git=git, sidecar=str(sc_path),
               per_episode=results)
if REC_DIR is not None:
    summary['records_out'] = str(REC_DIR)
    (REC_DIR / 'manifest.json').write_text(json.dumps(dict(
        kind='stage_records', source='policy_rollout',
        builder='baselines/eval_e2e_annot.py --records-out',
        cell=str(OUT), checkpoint=str(ck), arm=args.arm, tag=args.tag, mode=args.mode, seed=args.seed,
        ic_file=str(args.ic_file), ic_set=str(args.ic_set), sim_variant=args.sim_variant,
        record_ladder=str(env.ladder), never_terminate=False, max_steps=int(env.max_steps),
        action_repeat=REPEAT, shelf_top_z=float(env.shelf_top_z), ladder_provenance=LADDER,
        n_episodes=len(rec_rows), frames_total=int(sum(r['frames'] for r in rec_rows)),
        decisions_total=int(sum(r['n_decisions'] for r in rec_rows)),
        node=_cpu_stamp(), per_episode=rec_rows), indent=1))
(OUT / 'metrics.json').write_text(json.dumps(summary, indent=1))
print(f'\n[eval-e2e] {len(results)} episodes ({args.mode}, {args.ic_set}): '
      + '  '.join(f'{s} {stage_counts[s]}/{len(results)}' for s in HEADLINE_STAGES)
      + '  | legacy: ' + ' '.join(f'{s} {stage_counts[s]}' for s in LEGACY_STAGES)
      + f'  | tipped {counts["tipped"]} timeout {counts["timeout"]}  mean_steps {summary["mean_steps"]:.0f}'
      + (f'  sample_dev_mean {summary["sample_dev_mean"]:.4f}' if _dev else '') + f'  [{summary["seconds"]:.0f} s]', flush=True)
print(f'[eval-e2e] wrote {OUT}/metrics.json' + (f' + {len([r for r in results if r["video"]])} mp4s' if args.video else ''), flush=True)
