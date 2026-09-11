#!/usr/bin/env python
"""END-TO-END (full-task) evaluator for RLPD (SB3 SAC .zip) and Diffusion Policy (lerobot dir) checkpoints
(PHASE_PLAN_2026-09-04 amendment (n), 2026-09-07).

Protocol -- the world model's end-to-end cells (amendment (d), PHASE_RESULTS §5/§5.1) with the (j)/(l') columns:
  * `FullTaskEnv(scope='full')` in the corrected world (--sim-variant, also exported as GENESIS_SIM_VARIANT so the
    env's shelf band follows the world's shelf), horizon --max-steps SIM steps (1200 = 300 decisions at repeat 4),
    action_repeat / delta_joint cap 0.025 / leash 5x / delta_ref target asserted against the checkpoint sidecar.
    The UNIFIED ladder (picked 1 / placed_v2 1 / contact_push 2 / slide_success 4), slide_success and the tip rule
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
    `slide_success` window with its route. In scope='full' the env terminates on the nested PROXY and
    `GenesisCanEnv` never reaches its own horizon, so this call is the only settle that ever runs.

Stage columns (success-by-stage = granted at any time in the episode), LADDER_UNIFY_BRIEF D3/D4 2026-09-10:
  HEADLINE  picked, placed_v2, contact_push, slide_success, nested_v2, nested_honest
  LEGACY    placed, contact, nested_proxy, contact_push_legacy, slide_success_settle
`slide_success` is now the env's IN-EPISODE value -- the ladder's paid top rung and its only non-tip terminal --
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
ap.add_argument('--video', action='store_true', help='one mp4 per episode (240x320, one frame per decision)')
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
env = FullTaskEnv(backend='cpu', max_steps=args.max_steps, scope='full',
                  action_mode='delta_joint', delta_cap=DJ_CAP, delta_leash_mult=DJ_LEASH_MULT, action_repeat=REPEAT,
                  delta_ref='target', render_size=((240, 320) if args.video else None))
apply_post(env, args.sim_variant)
_want_top = float(BOX_TOP_Z) + float(_sv.VARIANTS[args.sim_variant].get('shelf_dz', 0.0))
assert abs(env.shelf_top_z - _want_top) < 1e-9, (env.shelf_top_z, _want_top)
assert env.scope == 'full' and env.action_repeat == REPEAT and env.delta_ref == 'target' and not env.phase_sparse
assert env.max_steps == args.max_steps, (env.max_steps, args.max_steps)
print(f'[eval-e2e] shelf_top_z {env.shelf_top_z:.3f} (placed_v2 band {env.shelf_top_z + 0.01:.3f}..{env.shelf_top_z + 0.07:.3f}); '
      f'delta cap {env.delta_cap} leash {env.delta_leash}; staged reward {STAGE_REWARD}', flush=True)
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
OUTCOMES = ('slide_success', 'tipped', 'timeout')
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
    frames = []
    snaps = [frozenset()]          # sticky stage set aligned 1:1 with frames
    if args.video:
        frames.append(np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1])
    done = False; t = 0; ep_r = 0.0; info = {}
    while not done:
        a = act(np.asarray(obs, np.float32))
        obs, r, term, trunc, info = env.step(a)
        ep_r += float(r); t += 1
        if args.video:
            frames.append(np.asarray(env.genv.w['cam'].render()[0])[:, :, ::-1])
            _acc = set(snaps[-1]) | set(env._granted)
            for _k in ('picked', 'placed', 'placed_v2', 'contact', 'contact_push', 'nested',
                       'nested_v2', 'slide_success'):
                if info.get(_k):
                    _acc.add(_k)                # evaluator ORs _granted with info; mirror it
            snaps.append(frozenset(_acc))       # read, never recompute
        done = bool(term or trunc)
    # The IN-EPISODE slide_success is now the statistic (D4): it is the env's own PAID,
    # TERMINAL rung, decided by the shared stage tracker during the episode. The post-episode
    # settle still runs, for two reasons and two only: `nested_honest` is the reference
    # column nested_v2 is validated against, and the (l) settle-route slide is kept as a
    # legacy column. Neither is what the reward paid.
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
    tipped = bool(info.get('tipped'))
    outcome = 'slide_success' if st['slide_success'] else ('tipped' if tipped else 'timeout')
    counts[outcome] += 1
    for s in STAGES:
        stage_counts[s] += int(st[s])
    route = end.get('slide_route')
    routes[str(route)] = routes.get(str(route), 0) + 1
    vid = None
    if args.video and frames:
        vid = str(OUT / f'ep{IC_OFFSET + k}_uid{uid if uid is not None else "rnd"}_{"slide" if st["slide_success"] else outcome}.mp4')
        SC = 2
        H0, W0 = frames[0].shape[:2]
        W, H = W0 * SC, H0 * SC
        PANEL = 74
        # The chips follow the LADDER (D2): PICK/PLACE/PUSH are the three non-terminal
        # rungs and NEST2 is nested_v2. `nested` (the withdrawn proxy) is deliberately no
        # longer a chip -- an overlay that lights it invites reading nesting off it.
        CHIPS = [('PICK', 'picked'), ('PLACE', 'placed_v2'), ('PUSH', 'contact_push'),
                 ('NEST2', 'nested_v2'), ('SLIDE', 'slide_success')]
        first = {}
        for i, sn in enumerate(snaps):
            for _, key in CHIPS:
                if key in sn and key not in first:
                    first[key] = i
        vw = cv2.VideoWriter(vid, cv2.VideoWriter_fourcc(*'mp4v'), 30.0 / REPEAT, (W, H + PANEL))
        F = cv2.FONT_HERSHEY_SIMPLEX
        for i, fr in enumerate(frames):
            im = cv2.resize(np.ascontiguousarray(fr.astype(np.uint8)), (W, H),
                            interpolation=cv2.INTER_NEAREST)
            pan = np.full((PANEL, W, 3), 24, np.uint8)
            sn = snaps[i] if i < len(snaps) else snaps[-1]
            x = 6
            for label, key in CHIPS:
                on = key in sn
                fresh = on and 0 <= i - first.get(key, -99) < 6
                col = (90, 240, 90) if on else (90, 90, 90)
                if fresh:
                    col = (60, 255, 255)
                w_ = 11 * len(label) + 12
                cv2.rectangle(pan, (x, 6), (x + w_, 28), col, -1 if on else 1)
                cv2.putText(pan, label, (x + 6, 23), F, 0.44,
                            (20, 20, 20) if on else (170, 170, 170), 1, cv2.LINE_AA)
                if key in first:
                    cv2.putText(pan, 'd%d' % first[key], (x + 6, 41), F, 0.33, (150, 220, 150), 1, cv2.LINE_AA)
                x += w_ + 7
            cv2.putText(pan, 'decision %d/%d' % (i, len(frames) - 1), (6, 60), F, 0.40,
                        (200, 200, 200), 1, cv2.LINE_AA)
            # slide_success now lights as a CHIP too (it is an in-episode, paid rung), but
            # nested_honest is decided by the post-episode settle and can only ever be a
            # terminal verdict -- that is inherent to the predicate, not to the overlay.
            verdict = 'slide=%s  nested_v2=%s  nested_honest=%s  tipped=%s' % (
                int(st['slide_success']), int(st['nested_v2']), int(st['nested_honest']), int(tipped))
            cv2.putText(pan, verdict, (150, 60), F, 0.40,
                        (60, 255, 255) if st['slide_success'] else (190, 190, 190), 1, cv2.LINE_AA)
            bar_y = PANEL - 5
            cv2.line(pan, (6, bar_y), (W - 6, bar_y), (70, 70, 70), 2)
            if len(frames) > 1:
                px = int(6 + (W - 12) * i / (len(frames) - 1))
                cv2.line(pan, (px, bar_y - 3), (px, bar_y + 3), (240, 240, 240), 2)
                for key, fi in first.items():
                    fx = int(6 + (W - 12) * fi / (len(frames) - 1))
                    cv2.line(pan, (fx, bar_y - 4), (fx, bar_y + 4), (90, 240, 90), 1)
            vw.write(np.ascontiguousarray(np.vstack([im, pan])))
        vw.release()
    # node / process / order stamps on EVERY episode (coordinator 2026-09-07): long-horizon full-scope episodes are
    # node-sensitive (same ckpt+IC+seed flips outcome across nodes) and, in the shared-process protocol, ORDER-dependent
    # (state leaks between episodes). `order` is the position within THIS process, so it is 0 for every isolated cell.
    results.append(dict(ep=IC_OFFSET + k, order=k, node=socket.gethostname(), pid=os.getpid(),
                        **{k: HW[k] for k in ('cpu_model', 'isa', 'avx512f', 'cpu_cores_physical', 'cpu_sockets',
                                              'cpu_logical', 'cpu_affinity', 'torch_num_threads', 'omp_num_threads')},
                        ic={kk: (list(vv) if isinstance(vv, (tuple, list, np.ndarray)) else vv) for kk, vv in ic.items()},
                        uid=(int(uid) if uid is not None else None), outcome=outcome, tipped=tipped, steps=t, reward=ep_r,
                        slide_route=route, seconds=round(time.time() - t0, 1), video=vid, stages=st))
    print(f'ep{k}: {"uid%d" % uid if uid is not None else "rnd"} {outcome} slide={int(st["slide_success"])} '
          f'nested_v2={int(st["nested_v2"])} nestedH={int(st["nested_honest"])} '
          f'push={int(st["contact_push"])} placed_v2={int(st["placed_v2"])} picked={int(st["picked"])} '
          f'({t} decisions, r={ep_r:.1f}, {time.time() - t0:.1f} s)', flush=True)

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
(OUT / 'metrics.json').write_text(json.dumps(summary, indent=1))
print(f'\n[eval-e2e] {len(results)} episodes ({args.mode}, {args.ic_set}): '
      + '  '.join(f'{s} {stage_counts[s]}/{len(results)}' for s in HEADLINE_STAGES)
      + '  | legacy: ' + ' '.join(f'{s} {stage_counts[s]}' for s in LEGACY_STAGES)
      + f'  | tipped {counts["tipped"]} timeout {counts["timeout"]}  mean_steps {summary["mean_steps"]:.0f}'
      + (f'  sample_dev_mean {summary["sample_dev_mean"]:.4f}' if _dev else '') + f'  [{summary["seconds"]:.0f} s]', flush=True)
print(f'[eval-e2e] wrote {OUT}/metrics.json' + (f' + {len([r for r in results if r["video"]])} mp4s' if args.video else ''), flush=True)
