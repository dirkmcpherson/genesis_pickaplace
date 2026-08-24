"""RLPD trainer for the JOINT pick task -- SAC + 50/50 online/demo batches +
LayerNorm ensemble critics + high UTD (Ball et al. 2023), via RLPDSAC in
baselines/rl/rlpd_sac.py.

This is an EXTENSION of the SACfD path, not a fork:
  * the env is the same FullTaskEnv (delta_joint or absolute), and
  * the demo transitions come from train_sacfd_full's OWN encoder
    (relabel_full / delta_encode_transitions) so the RLPD demo tensors are
    BIT-IDENTICAL to SACfD's -- the human-vs-model demo-source comparison then
    differs only in the algorithm, never in the data pipeline (guarded by
    sacfd_delta_gate.py).

Why RLPD over seed-once SACfD (measured pathologies, see RLPD_PLAN.md):
  * demo dilution: injecting ~O(10^5) demo transitions into a 300k FIFO buffer
    that online data overwrites makes the rewarded demo frames a vanishing
    minority late in training. Here demos are an IMMUTABLE buffer that is
    permanently HALF of every batch.
  * sparse-reward value pessimism: LayerNorm critics + high UTD (RLPD's recipe).

The stale --cartesian branch of the previous train_rlpd was removed per
RLPD_PLAN.md (the EEF arm rides train_sacfd_full --cartesian; RLPD is the joint
arm). Recover it from git history if ever needed.

Usage (200k local pilot, the plan's primary arm):
  .venv-eval/bin/python baselines/rl/train_rlpd.py \
      --steps 200000 --scope pick --action-mode delta_joint --gamma 0.998 \
      --utd 10 --demo-dir baselines/episodes_pick_phase_all \
      --out-dir baselines/rl/checkpoints/rlpd_dH --run-name dH_RLPD_s0 \
      --project genesis_paper --seed 0 --device cuda
"""
import os
import argparse
import glob
import json
import pathlib as pl
import sys
import time

REPO = pl.Path(os.environ.get('GENESIS_PICKAPLACE_ROOT',
                              pl.Path(__file__).resolve().parents[2]))
sys.path.insert(0, str(REPO / 'baselines'))
sys.path.insert(0, str(REPO / 'baselines' / 'rl'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=200_000)
    ap.add_argument('--demo-dir', default='baselines/episodes_pick_phase_all',
                    help='human demo set (91 eps, 17/7-dim). The AI-demo arm points '
                         'this at a harvested set with the SAME layout.')
    ap.add_argument('--out-dir', default='baselines/rl/checkpoints/rlpd')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--device', default='cuda')
    # --- RLPD hypers (RLPD_PLAN.md "Pinned hypers") ---
    ap.add_argument('--utd', type=int, default=10,
                    help='critic gradient steps per env step (actor/alpha still x1). '
                         'Primary arm UTD-10 @200k ~7h; UTD-20 @100k is the alt arm.')
    ap.add_argument('--ensemble-size', type=int, default=10, help='E critic ensemble')
    ap.add_argument('--subset-size', type=int, default=2,
                    help='Z: target = min over a random Z-of-E TARGET critics')
    ap.add_argument('--demo-batch', type=int, default=128,
                    help='demo half of the 256 batch (128 demo + 128 online = 50/50)')
    ap.add_argument('--backup-entropy', choices=['on', 'off'], default='off',
                    help="entropy term in the CRITIC TARGET. 'off' = RLPD's setting "
                         "for every sparse domain (audit bug 1: 'on' at gamma=0.998 "
                         "gives a 500*alpha*H zero-reward fixed point that buries "
                         "the +1 and pays 400:1 AGAINST terminating). 'on' restores "
                         "the pre-audit behavior for comparison only.")
    ap.add_argument('--per-member-ln', choices=['on', 'off'], default='off',
                    help="per-member LayerNorm affine in the critic ensemble (audit "
                         "bug 2: shared affine ties member scales, degrading the "
                         "min-of-Z pessimism; Q refluxed to 250-1600 in the nb wave "
                         "with backup off). 'off' = original shared-LN (old ckpts).")
    ap.add_argument('--pick-shaping', choices=['on', 'off'], default='off',
                    help="DENSE-REWARD lever (scope=pick only, 08-18): potential-based "
                         "approach term -SCALE*||eef-can|| (Ng-invariant, gamma-matched "
                         "0.998). TRAINING-ONLY: eval envs never see it. Demos are NOT "
                         "relabeled (potential shaping needs no demo term). Recorded in "
                         "the sidecar. Default off = byte-identical to every prior run.")
    ap.add_argument('--pick-hold-reward', choices=['on', 'off'], default='off',
                    help="REWARD DENSITY lever (scope=pick only). 'on': the env pays "
                         "+1 for EVERY step the honest hold condition holds (can above "
                         "pick_z AND grip commanded closed) and terminates after "
                         "--pick-hold-k CONSECUTIVE held steps; demos are relabeled by "
                         "hold_region_encode_transitions with the SAME predicate/K and "
                         "cut at their own K-th held frame. 'off' = the terminal-only "
                         "+1 (66 rewarded frames in 83,465 = 0.08%%), ~1000x sparser "
                         "than any published RLPD setup (Ball 2023 Adroit and ManiSkill "
                         "both pay +1 per SOLVED step) -- the top-ranked explanatory "
                         "delta in paper/rlpd_literature_comparison_2026-08-13.md RQ5. "
                         "REQUIRES a FULL-LENGTH demo set (baselines/episodes_all or "
                         "episodes_delta_rerecord): the pick-phase tapes are cut ~2 "
                         "frames past the lift and contain no hold region (asserted).")
    ap.add_argument('--pick-hold-k', type=int, default=25,
                    help='consecutive held steps that end the episode under '
                         '--pick-hold-reward on (env termination AND demo done). '
                         'Travels in the sidecar; 25 > the hardened predicate\'s '
                         'PICK_SUSTAIN=10, so a whack-fling cannot farm the terminal.')
    ap.add_argument('--gamma', type=float, default=0.998,
                    help='0.998 ~ 500-step credit window. 0.98 made the demo pick '
                         'terminal (median frame 662) worth ~1.6e-6 from start '
                         'states -- a silent cause of the uniform SACfD zeros.')
    ap.add_argument('--ent-coef', default='auto',
                    help="SAC entropy coefficient. If the Q watchdog trips (mean "
                         "actor Q > 2), the pre-registered fix is a small FIXED "
                         "value here, e.g. 0.005.")
    ap.add_argument('--target-entropy', type=float, default=None,
                    help='default -dim/2 (= -3.5 for the 7-dim joint action)')
    ap.add_argument('--train-max-steps', type=int, default=900,
                    help='training episode horizon in SIM steps (joint; position-'
                         'target SAC can outrun the demonstrator so 900 suffices for '
                         'pick). With --action-repeat N this is ceil(900/N) decisions.')
    ap.add_argument('--action-repeat', type=int, default=1,
                    help='hold each policy decision for N consecutive sim steps '
                         '(delta_joint: same delta N times => N*a*cap target advance). '
                         'N=4 shrinks the 900-step episode to ~225 decisions, inside '
                         'the gamma=0.998 credit horizon (~500). MUST be passed '
                         'explicitly: it travels in the checkpoint sidecar and is '
                         'mirrored by the demo encoder, the in-train eval, and '
                         'wandb_eval --action-mode auto (the silent-default rule).')
    # --- shared-with-SACfD flags (mirror train_sacfd_full) ---
    ap.add_argument('--scope', choices=['full', 'pick'], default='pick',
                    help='pick: +1 and terminate on the pick (phase-1 paper core)')
    ap.add_argument('--action-mode', choices=['absolute', 'delta_joint'],
                    default='delta_joint',
                    help='delta_joint: env actions are per-step joint-target deltas '
                         '(cap 0.025, leash 5x) and demo actions are delta-encoded '
                         'to match, IMPORTED from train_sacfd_full so tensors are '
                         'bit-identical to SACfD. A sidecar records the mode for '
                         'wandb_eval --action-mode auto.')
    ap.add_argument('--delta-ref', choices=['target', 'measured'], default='target',
                    help="delta_joint only: WHAT a delta is applied to. 'target' "
                         "(default, every pre-2026-08-14 run) integrates onto a "
                         "running target -- open-loop, so a clipped frame leaves a "
                         "permanent offset (P1). 'measured' re-references the "
                         "MEASURED qpos each step (ManiSkill pd_delta style, leash-"
                         "scaled) and pairs with the measured-ref demo encoders + "
                         "the closed-loop re-recorded tapes "
                         "(baselines/episodes_delta_rerecord). Passed EXPLICITLY "
                         "(silent-default rule); it travels in the checkpoint "
                         "sidecar so wandb_eval --delta-ref auto integrates the "
                         "policy in the SAME space it trained in.")
    # --- demo-set protections (PREREG_final_round_robin_2026-08-23 §4) ---
    ap.add_argument('--sim-variant', default='base', help='Genesis world variant (baselines/sim_variants.py); MUST match the demo tapes\' stamp; sidecar + registry carry it')
    ap.add_argument('--demo-terminal-guard', choices=['on', 'off'], default='on',
                    help="LEGACY tapes only (see --demo-format). on (DEFAULT since "
                         "2026-08-23): every demo tape ends where the env would have "
                         "terminated -- tip rule (grip commanded open & tilt>60) -> "
                         "done=True, r=0, later frames dropped; scope=pick -> done=True at "
                         "the pick and later frames dropped; cap -> bootstrap. One "
                         "definition: full_env.terminal_from_tape. off = the pre-08-23 "
                         "tensors (fail tapes whole with done=False -- the unanchored "
                         "bootstrap chains behind dDP_RLPD 0/6, AUDIT_impl F1).")
    ap.add_argument('--demo-format', choices=['legacy', 'native'], default='legacy',
                    help="legacy (default): stride-1 state/command tapes re-encoded by "
                         "train_sacfd_full's delta encoders. native: contract-v1 tapes "
                         "from baselines/record_demos.py (one row per decision recorded "
                         "through THIS env: actions_delta/rewards/terminated verbatim, no "
                         "re-encoding, no relabel predicate); the tape's action_repeat / "
                         "delta_cap / delta_leash / delta_ref stamps MUST equal the run's.")
    ap.add_argument('--demo-shaping', choices=['auto', 'on', 'off'], default='auto',
                    help="dense arms: relabel the DEMO half with the SAME potential the "
                         "env pays online (full_env.pick_shaping_phi on the recorded "
                         "eef_pos, gamma = --gamma, phi(terminal)=0). Needs --demo-format "
                         "native (legacy tapes carry no eef). auto (default) = on for "
                         "native tapes when --pick-shaping on, off otherwise; 'on' with "
                         "legacy tapes is refused (no silent half-shaped buffer).")
    ap.add_argument('--pick-shaping-terminal-zero', choices=['on', 'off'], default='on',
                    help="phi(terminal)=0 in the env's shaping (Ng et al. episodic form; "
                         "PREREG §2). off reproduces the 08-19 dense runs' phi(s_T)!=0.")
    ap.add_argument('--ckpt-fracs', default='0.2,0.4,0.6,0.8,1.0',
                    help='archive model+sidecar at these fractions of --steps into '
                         '<out-dir>/ckpt_<pct>/ (PREREG §5: K=5 archived checkpoints per '
                         'run; selection on the sel ICs, confirmation on hold+rnd). '
                         'rlpd_final.zip is still written unchanged.')
    ap.add_argument('--no-wandb', action='store_true')
    ap.add_argument('--run-name', default=None)
    ap.add_argument('--project', default='genesis_paper', help='wandb project')
    ap.add_argument('--eval-freq', type=int, default=25_000,
                    help='video-eval subprocess cadence (0 disables)')
    ap.add_argument('--eval-max-steps', type=int, default=400,
                    help='eval rollout horizon (400 for pick-only curves, #21 lever)')
    args = ap.parse_args()

    # ---- env (joint only; the cartesian arm rides train_sacfd_full) ----
    t0 = time.time()
    from full_env import FullTaskEnv, STAGE_REWARD
    import pick_env
    from train_sacfd_full import (relabel_full, delta_encode_transitions,
                                  delta_encode_transitions_repeat,
                                  delta_encode_transitions_measured,
                                  delta_encode_transitions_measured_repeat,
                                  hold_region_encode_transitions, print_hold_census,
                                  native_demo_transitions, print_native_census,
                                  demo_dir_sha256)
    assert args.action_repeat >= 1, args.action_repeat
    guard = (args.demo_terminal_guard == 'on')
    native = (args.demo_format == 'native')
    if native:
        assert args.action_mode == 'delta_joint' and args.delta_ref == 'target', (
            'contract-v1 tapes are recorded through FullTaskEnv(delta_joint, '
            f'delta_ref=target); got action_mode={args.action_mode} delta_ref={args.delta_ref}')
        assert args.pick_hold_reward == 'off', 'no hold-reward relabel for native tapes (not built)'
        assert args.scope == 'pick', 'contract-v1 tapes are pick-scope recordings'
    if args.demo_shaping == 'auto':
        demo_shaping = native and args.pick_shaping == 'on'
    else:
        demo_shaping = (args.demo_shaping == 'on')
    if demo_shaping:
        assert native, ('--demo-shaping on needs --demo-format native (legacy tapes carry no '
                        'eef_pos; a half-shaped buffer is the F10 asymmetry, refused)')
        assert args.pick_shaping == 'on', '--demo-shaping on without --pick-shaping on makes no sense'
    elif native and args.pick_shaping == 'on':
        print('[demos] WARNING: --pick-shaping on with --demo-shaping off: the demo half '
              'stays sparse while online transitions are shaped (F10 asymmetry).', flush=True)
    hold_reward = args.pick_hold_reward == 'on'
    if hold_reward:
        # every precondition stated up front, none silently defaulted
        assert args.scope == 'pick', (
            'pick_hold_reward is a scope=pick lever; got scope=' + args.scope)
        assert args.action_mode == 'delta_joint', (
            'the hold-region encoder emits delta actions; got action_mode='
            + args.action_mode)
        assert args.action_repeat == 1, (
            'no decision-level hold encoder yet -- run action_repeat=1 rather than '
            f'silently striding the hold region; got {args.action_repeat}')
        assert args.pick_hold_k >= 1, args.pick_hold_k
    if args.action_repeat > 1:
        assert args.action_mode == 'delta_joint', (
            'action_repeat is only wired for delta_joint (the repeat-encoder + eval '
            f'repeat assume it); got action_mode={args.action_mode}')
    if args.delta_ref == 'measured':
        assert args.action_mode == 'delta_joint', (
            'delta_ref=measured is a delta_joint concept (it changes what the delta '
            f'is applied to); got action_mode={args.action_mode}')
    from sim_variant_hook import apply_pre, apply_post
    apply_pre(args.sim_variant)
    env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                      scope=args.scope, action_mode=args.action_mode,
                      action_repeat=args.action_repeat, delta_ref=args.delta_ref,
                      pick_hold_reward=hold_reward, pick_hold_k=args.pick_hold_k,
                      pick_shaping=(args.pick_shaping == 'on'),
                      # shaping gamma = the AGENT's discount (Ng invariance needs them
                      # equal; before 08-23 the env silently used 0.998 whatever --gamma was)
                      pick_shaping_gamma=args.gamma,
                      pick_shaping_terminal_zero=(args.pick_shaping_terminal_zero == 'on'))
    # asserts: no silent defaults -- the env must be running the mode we asked for
    assert env.scope == args.scope, (env.scope, args.scope)
    assert env.action_mode == args.action_mode, (env.action_mode, args.action_mode)
    assert env.action_repeat == args.action_repeat, (env.action_repeat, args.action_repeat)
    assert env.delta_ref == args.delta_ref, (env.delta_ref, args.delta_ref)
    assert env.pick_hold_reward is hold_reward, (env.pick_hold_reward, hold_reward)
    assert env.pick_hold_k == args.pick_hold_k, (env.pick_hold_k, args.pick_hold_k)
    apply_post(env, args.sim_variant)
    assert abs(env._pick_gamma - args.gamma) < 1e-12, (env._pick_gamma, args.gamma)
    assert env.pick_shaping_terminal_zero == (args.pick_shaping_terminal_zero == 'on')
    print(f'[env] {type(env).__name__} built in {time.time() - t0:.1f}s | '
          f'pick_z={env.pick_z:.4f} scope={env.scope} action_mode={env.action_mode} '
          f'delta_cap={env.delta_cap} delta_leash={env.delta_leash} '
          f'delta_ref={env.delta_ref} action_repeat={env.action_repeat} '
          f'pick_hold_reward={args.pick_hold_reward} '
          f'pick_hold_k={env.pick_hold_k if hold_reward else "-"} '
          f'(~{-(-args.train_max_steps // args.action_repeat)} decisions/ep)', flush=True)

    # ---- model (RLPDSAC: LN ensemble critics built at construction) ----
    from rlpd_sac import make_rlpd, DemoData
    # Q watchdog: its 2.0 default is "2x the max task return" under the terminal-only
    # +1. The hold reward raises the max discounted return to sum_{i<K} gamma^i (~24.4
    # at K=25/gamma=0.998), so leaving the threshold at 2.0 would make it scream on a
    # perfectly healthy critic -- and a watchdog that cries wolf is how the entropy-
    # backup explosion got waved off in the first place (audit §12). Scale it with the
    # reward semantics, same 2x slack. hold off -> exactly 2.0 (unchanged).
    if hold_reward:
        _max_ret = (1.0 - args.gamma ** args.pick_hold_k) / (1.0 - args.gamma)
        q_watch = 2.0 * _max_ret
    else:
        q_watch = 2.0
    model = make_rlpd(env, args.seed, args.device, q_watchdog=q_watch,
                      backup_entropy=(args.backup_entropy == 'on'),
                      per_member_ln=(args.per_member_ln == 'on'),
                      ensemble_size=args.ensemble_size, subset_size=args.subset_size,
                      utd=args.utd, gamma=args.gamma, ent_coef=args.ent_coef,
                      target_entropy=args.target_entropy, demo_batch=args.demo_batch)
    print(f'[cfg] RLPD | E={args.ensemble_size} Z={args.subset_size} UTD={args.utd} '
          f'gamma={args.gamma} ent_coef={args.ent_coef} '
          f'target_entropy={model.target_entropy} demo_batch={args.demo_batch}/256 '
          f'backup_entropy={args.backup_entropy} '
          f'per_member_ln={args.per_member_ln} '
          f'pick_hold_reward={args.pick_hold_reward} pick_hold_k={args.pick_hold_k} pick_shaping={args.pick_shaping} '
          f'q_watchdog={q_watch:.2f} '
          f'scope={args.scope} action_mode={args.action_mode} '
          f'delta_ref={args.delta_ref} '
          f'action_repeat={args.action_repeat} demo_dir={args.demo_dir} '
          f'demo_format={args.demo_format} demo_terminal_guard={args.demo_terminal_guard} '
          f'demo_shaping={"on" if demo_shaping else "off"} '
          f'pick_shaping_terminal_zero={args.pick_shaping_terminal_zero} '
          f'train_max_steps={args.train_max_steps} ckpt_fracs={args.ckpt_fracs}', flush=True)
    print(model.critic, flush=True)

    # ---- demos: SAME encoder as train_sacfd_full (bit-identical tensors) ----
    paths = sorted(glob.glob(str(REPO / args.demo_dir / '*.npz')))
    assert paths, f'no npz in {args.demo_dir}'
    demo_sha = demo_dir_sha256(paths)
    print(f'[demos] {len(paths)} npz in {args.demo_dir} content_sha256={demo_sha[:16]}... '
          f'format={args.demo_format} terminal_guard={args.demo_terminal_guard} '
          f'demo_shaping={"on" if demo_shaping else "off"}', flush=True)
    if native:
        transitions, census = native_demo_transitions(
            paths, expect=dict(sim_variant=args.sim_variant, action_repeat=args.action_repeat, delta_cap=env.delta_cap,
                               delta_leash=env.delta_leash, delta_ref=args.delta_ref),
            gamma=args.gamma, shaping=demo_shaping,
            phi_scale=FullTaskEnv.PICK_SHAPING_SCALE)
        print_native_census(census, tag=args.demo_dir)
        assert census['n_transitions'] > 0, 'empty native demo set'
        norm = None
    elif hold_reward:
        # REWARD-DENSITY path: per-frame hold reward, tape cut at the demo's own K-th
        # consecutive held frame. Same pick_z INSTANCE the env runs on and the same
        # full_env.pick_hold_held predicate the env calls -- the demo reward stream and
        # the env reward stream are one definition, not two implementations.
        transitions, census = hold_region_encode_transitions(
            paths, env.pick_z, env.delta_cap, args.pick_hold_k, args.delta_ref,
            terminal_guard=guard)
        print_hold_census(census, tag=f'{args.demo_dir} ref={args.delta_ref}')
        assert census['n_terminal'] > 0, (
            f'no demo in {args.demo_dir} shows {args.pick_hold_k} consecutive held '
            'frames -- this is a PICK-TRUNCATED set (episodes_pick_phase_all is cut '
            '~2 frames past the lift). The hold-reward arm needs FULL-LENGTH tapes '
            '(baselines/episodes_all or baselines/episodes_delta_rerecord).')
        print(f'[demos] encoder=hold_region_encode_transitions '
              f'delta_ref={args.delta_ref} cap={env.delta_cap} K={args.pick_hold_k}',
              flush=True)
        norm = None
    elif args.action_mode == 'delta_joint':
        # EXPLICIT encoder selection (no silent stride-1 fallback): action_repeat>1
        # decision-level demos vs the stride-1 encoder. repeat==1 keeps the exact
        # stride-1 tensors SACfD/the gate assert bit-equality against.
        # delta_ref picks the REFERENCE the encoder differences against, in lockstep
        # with the env: 'target' = previous COMMAND (open-loop), 'measured' = the
        # demo's RECORDED measured qpos, leash-scaled (mirrors _step_once's measured
        # branch). Mixing the two is the P1 failure mode, so both are explicit here.
        if args.delta_ref == 'measured':
            _enc = (delta_encode_transitions_measured_repeat
                    if args.action_repeat > 1 else delta_encode_transitions_measured)
        else:
            _enc = (delta_encode_transitions_repeat
                    if args.action_repeat > 1 else delta_encode_transitions)
        if args.action_repeat > 1:
            transitions = _enc(paths, env.pick_z, args.scope, env.delta_cap,
                               args.action_repeat, terminal_guard=guard)
        else:
            transitions = _enc(paths, env.pick_z, args.scope, env.delta_cap,
                               terminal_guard=guard)
        print(f'[demos] encoder={_enc.__name__} delta_ref={args.delta_ref} '
              f'cap={env.delta_cap} leash={env.delta_leash} terminal_guard={guard}', flush=True)
        norm = None                    # actions already in normalized delta space
    else:
        transitions, _ = relabel_full(paths, env.pick_z, scope=args.scope,
                                      terminal_guard=guard)
        if args.scope == 'pick':
            transitions = [(o, a, r, o2, True) if r >= STAGE_REWARD['picked'] else
                           (o, a, r, o2, d) for (o, a, r, o2, d) in transitions]
        norm = pick_env.normalize_action
    import torch as th
    demo = DemoData(transitions, norm, th.device(args.device), seed=args.seed)
    model.set_demo_data(demo)
    n_done = sum(1 for t in transitions if t[4])
    n_done_r0 = sum(1 for t in transitions if t[4] and t[2] <= 0.0)
    print(f'[demos] {len(paths)} eps -> {demo.n} transitions in the IMMUTABLE demo '
          f'buffer (50% of every batch), {demo.n_rewarded} rewarded, {n_done} terminal '
          f'({n_done_r0} zero-reward terminals = tip-guarded fails)', flush=True)

    # ---- output + action_mode sidecars (the silent-default-bug rule: control mode
    # travels WITH the artifact so wandb_eval --action-mode auto reads it) ----
    out = pl.Path(args.out_dir); out.mkdir(parents=True, exist_ok=True)
    # action_repeat travels WITH the artifact so wandb_eval --action-mode auto applies
    # the SAME repeat at eval time (stateful: one policy query per N env steps). Evaling
    # a repeat-N policy at stride 1 is the exact silent-default bug family this repo keeps
    # hitting -- the sidecar closes it.
    import subprocess
    try:
        _git = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=str(REPO),
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        _git = 'unknown'
    sidecar = {'action_mode': args.action_mode, 'action_repeat': args.action_repeat,
               'delta_ref': args.delta_ref,
               'backup_entropy': args.backup_entropy,
               'per_member_ln': args.per_member_ln, 'git': _git or 'unknown',
               'demo_dir': args.demo_dir, 'scope': args.scope,
               # reward-density lever: the TRAINING reward semantics travel with the
               # artifact too, so a later reader never has to guess which reward a
               # checkpoint's return curve was earned under. (Eval is unaffected: it
               # measures the pick, not the return -- wandb_eval ignores these keys.)
               'pick_hold_reward': args.pick_hold_reward,
               'pick_hold_k': args.pick_hold_k, 'pick_shaping': args.pick_shaping,
               # 2026-08-23 demo-set protections + budget/horizon (PREREG §2, §4)
               'demo_format': args.demo_format,
               'sim_variant': args.sim_variant,
               'demo_terminal_guard': args.demo_terminal_guard,
               'demo_shaping': ('on' if demo_shaping else 'off'),
               'pick_shaping_terminal_zero': args.pick_shaping_terminal_zero,
               'pick_shaping_gamma': args.gamma, 'gamma': args.gamma,
               'train_max_steps': args.train_max_steps, 'steps': args.steps,
               'steps_unit': 'decisions', 'demo_sha256': demo_sha,
               'demo_n_eps': len(paths), 'seed': args.seed}
    # STARTUP sidecar next to the VideoEvalCallback snapshot dir, so even the first
    # in-train eval snapshot (which the callback ALSO passes --action-mode for) has a
    # readable record; and the final one next to rlpd_final.
    (out / 'wandb_eval').mkdir(parents=True, exist_ok=True)
    (out / 'wandb_eval' / 'snapshot.action_mode.json').write_text(json.dumps(sidecar))

    from stable_baselines3.common.callbacks import CheckpointCallback, CallbackList
    from wandb_utils import init_wandb, WandbScalarCallback, VideoEvalCallback

    class SidecarCheckpointCallback(CheckpointCallback):
        """Write the action-mode sidecar NEXT TO EVERY snapshot checkpoint.

        The stock callback saves rlpd_<N>_steps.zip with no sidecar, so wandb_eval
        --action-mode auto resolves <stem>.action_mode.json, finds nothing, and
        falls back to absolute@1 -- a delta policy then evals ~0.00 and the zero
        looks like a result. Third member of the silent-default family (grip
        column, control mode; caught by newbox_supp 2026-08-13 before it produced
        13 artifact zeros in the 100k post-hoc sweep)."""

        def __init__(self, sidecar_json, **kw):
            super().__init__(**kw)
            self._sidecar_json = sidecar_json

        def _on_step(self):
            ok = super()._on_step()
            if self.n_calls % self.save_freq == 0:
                zip_path = pl.Path(self._checkpoint_path(extension='zip'))
                zip_path.with_name(zip_path.stem + '.action_mode.json').write_text(
                    self._sidecar_json)
            return ok

    from stable_baselines3.common.callbacks import BaseCallback

    class ArchiveCheckpointCallback(BaseCallback):
        """PREREG 2026-08-23 §5: K archived checkpoints at fixed fractions of the
        budget, each with its sidecar (+ step/frac), in <out>/ckpt_<pct>/rlpd_ckpt.zip
        -- selection on the `sel` ICs, confirmation on `hold`+`rnd`. Fires on the
        first step at or past each threshold; the 100% one is also written by the
        final save below (kept for callers that read rlpd_final.zip)."""

        def __init__(self, fracs, total, out_dir, sidecar):
            super().__init__()
            self._thr = sorted(set(max(1, int(round(float(f) * total))) for f in fracs))
            self._done = set(); self._out = pl.Path(out_dir); self._side = dict(sidecar)

        def _on_step(self):
            for thr in self._thr:
                if thr in self._done or self.num_timesteps < thr:
                    continue
                pct = int(round(100.0 * thr / max(1, self.model._total_timesteps)))
                d = self._out / f'ckpt_{pct:03d}'; d.mkdir(parents=True, exist_ok=True)
                self.model.save(str(d / 'rlpd_ckpt'))
                side = dict(self._side, ckpt_step=int(self.num_timesteps), ckpt_frac=thr / max(1, self.model._total_timesteps))
                (d / 'rlpd_ckpt.action_mode.json').write_text(json.dumps(side))
                self._done.add(thr)
                print(f'[ckpt] archived {d}/rlpd_ckpt.zip @ {self.num_timesteps} decisions', flush=True)
            return True

    run = init_wandb(args, name=args.run_name or out.name, tags=('rlpd',),
                     project=args.project)
    cbs = [SidecarCheckpointCallback(json.dumps(sidecar), save_freq=50_000,
                                     save_path=str(out), name_prefix='rlpd'),
           ArchiveCheckpointCallback([float(f) for f in args.ckpt_fracs.split(',') if f.strip()],
                                     args.steps, out, sidecar),
           WandbScalarCallback(run)]
    if args.eval_freq:
        cbs.append(VideoEvalCallback(
            run, out, eval_freq=args.eval_freq, max_steps=args.eval_max_steps,
            seed=args.seed, cartesian=False,
            action_mode=(args.action_mode if args.action_mode != 'absolute' else None),
            action_repeat=args.action_repeat, delta_ref=args.delta_ref))
    model.learn(total_timesteps=args.steps, log_interval=10, callback=CallbackList(cbs))
    model.save(str(out / 'rlpd_final'))
    (out / 'rlpd_final.action_mode.json').write_text(json.dumps(sidecar))
    if run is not None:
        run.finish()
    print(f'[rlpd] done in {(time.time() - t0)/3600:.1f}h -> {out}/rlpd_final.zip',
          flush=True)


if __name__ == '__main__':
    main()
