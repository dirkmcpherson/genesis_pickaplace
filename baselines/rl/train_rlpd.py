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
    ap.add_argument('--scope', choices=['full', 'pick', 'place', 'contact'], default='pick',
                    help='pick: +1 and terminate on the pick (phase-1 paper core); place: PHASE_PLAN amendment (h) '
                         '(2026-09-07) -- reset restores a banked pick-grant entry (--entry-bank), +1 and terminate '
                         'on placed_v2, tips terminate without penalty (phase_sparse), demos = --demo-format segment; '
                         'contact: the SLIDE phase of amendment (m) -- reset restores a banked placed_v2 state '
                         '(released, on the shelf), +1 and terminate on the env `contact` predicate (rewards are NOT '
                         'changed by amendment (l): slide_success is the eval statistic, never a training signal)')
    ap.add_argument('--contact-grant', choices=['bare_contact', 'slide_success', 'prior_release'], default=None,
                    help="scope=contact ONLY, REQUIRED there (PHASE_PLAN (p), no default exists): which predicate pays "
                         "+1 and ends the episode. bare_contact = the (m)/world-model-of-record grant; slide_success = "
                         "the WITHDRAWN (o) reward (refused unless CONTACT_GRANT_ALLOW_WITHDRAWN=1); prior_release = "
                         "(p)'s corrected predicate, not implemented until its clause-5 threshold is calibrated.")
    ap.add_argument('--far-release', action='store_true',
                    help='Ladder N: the release that counts for farside/home must be >= 0.10 m from '
                         'the goal. Part of the ladder stamp; refused on a non-nested ladder.')
    ap.add_argument('--ladder', choices=['staged', 'sparse', 'nested_sparse', 'nested_ramp'],
                    default='staged',
                    help="scope=full ONLY: WHICH reward ladder (a constructor argument of FullTaskEnv, "
                         "never an env var -- the reward structure must not be selectable by something a "
                         "job can silently fail to receive; that is exactly how {RLPD} and {r2dreamer} "
                         "came to optimise different objectives for 32 runs each). "
                         "'staged' = picked 1 / placed_v2 1 / contact_push 2 / slide_success 4, terminal "
                         "slide_success, max return 8. 'sparse' = nested_v2 1, terminal nested_v2, max "
                         "return 1. The two differ in reward and terminal ONLY. It is carried in the "
                         "ladder provenance stamp, so a sparse row can never merge with a staged one. "
                         "The launchers REQUIRE it explicitly.")
    ap.add_argument('--tip-guard', choices=['grip', 'not_in_hand'], required=True,
                    help="REQUIRED, no default (PHASE_PLAN amendment (aa)). WHICH guard the tip "
                         "termination uses -- it decides where every episode ENDS, so it is never "
                         "a default for the same reason the ladder is not. 'grip' = the rule of "
                         "record (commanded grip < 0.3, fired on the first frame), which the "
                         "ladder pilot and every earlier run trained under. 'not_in_hand' = the "
                         "StageTracker's in_hand (|tool_xy - can_xy| >= 0.025 m, NO gripper term) "
                         "required on 4 consecutive env frames together with tilt > 60 deg. "
                         "Threshold, termination and penalty are UNCHANGED either way. It is part "
                         "of the ladder provenance stamp, so a row run under one guard can never "
                         "merge with a row run under the other. scope=full only.")
    ap.add_argument('--goalward-shaping', choices=['off', 'on'], default='off',
                    help="scope=full ONLY (LADDER_UNIFY_BRIEF D7): potential-based goalward shaping, "
                         "phi = -2 * xy-dist(can, goal), active only while placed_v2 is granted, the can "
                         "is not in hand and the can touches the goal. DEFAULT OFF and NOT an env var -- a "
                         "fourth silent lever is what produced the two-ladder confound. It exists for the "
                         "registered sparsity fallback (no contact_push in any seed of an arm by half the "
                         "budget -> rerun that arm with it on, DISCLOSED). It is part of the ladder "
                         "provenance stamp, so a shaped row can never merge with an unshaped one.")
    ap.add_argument('--entry-bank', default=None,
                    help='scope=place/contact: entry-bank JSON (the human pick-grant bank of record, phase_banks/human_place.json); '
                         'REQUIRED for place (the env default bank is the OLD world)')
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
    ap.add_argument('--demo-format', choices=['legacy', 'native', 'segment'], default='legacy',
                    help="legacy (default): stride-1 state/command tapes re-encoded by "
                         "train_sacfd_full's delta encoders. native: contract-v1 tapes "
                         "from baselines/record_demos.py (one row per decision recorded "
                         "through THIS env: actions_delta/rewards/terminated verbatim, no "
                         "re-encoding, no relabel predicate); the tape's action_repeat / "
                         "delta_cap / delta_leash / delta_ref stamps MUST equal the run's. segment: "
                         "r2dreamer-native PLACE segments (to_dreamer_native.py --phase place --with-state; "
                         "demos_state/{dH_place,dDP_place_n39}), the same rows the WM trained on, via "
                         "place_demos.segment_transitions (scope=place only).")
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
    ap.add_argument('--ckpt-every', type=int, default=50_000,
                    help='DECISIONS between periodic snapshot zips (rlpd_<N>_steps.zip). 0 = none, keeping only '
                         'the --ckpt-fracs archive + rlpd_final.zip. The snapshots duplicate the archive and no '
                         'stage reads them; 0 is the setting of record for the place runs (disk incident '
                         '2026-09-07: the shared filesystem hit 0 bytes free and every queued job died). '
                         'Default 50000 keeps every pre-2026-09-07 run byte-identical.')
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
    from full_env import FullTaskEnv, STAGE_REWARD, refuse_legacy_gates
    # LADDER_UNIFY_BRIEF D1: this tree has ONE ladder and no environment-variable gates.
    # A launcher that still exports the old one is running on an assumption that is now
    # false, and the whole point of the unification is that such an assumption must not be
    # able to pass silently (it was inert in one of three trees for 32 runs).
    refuse_legacy_gates()
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
    segment = (args.demo_format == 'segment')
    if native:
        assert args.action_mode == 'delta_joint' and args.delta_ref == 'target', (
            'contract-v1 tapes are recorded through FullTaskEnv(delta_joint, '
            f'delta_ref=target); got action_mode={args.action_mode} delta_ref={args.delta_ref}')
        assert args.pick_hold_reward == 'off', 'no hold-reward relabel for native tapes (not built)'
        assert args.scope == 'pick', 'contract-v1 tapes are pick-scope recordings'
    PHASE_SCOPES = ('place', 'contact')
    # PHASE_PLAN amendment (n) 2026-09-07: END-TO-END full task. scope='full' with --demo-format segment reads the
    # r2dreamer-native FULL-scope demo sets the world model's (d) arm trained on ($W/demos_state_full/{dHfull_all,
    # dDPfull}) VERBATIM. Unlike a phase scope it resets from the pick-scope ICs (no entry bank), pays the STAGED
    # sparse ladder (picked 1 / placed 1 / contact 2 / nested 4) and terminates on the nested proxy, so phase_sparse
    # stays off and nothing about the pick/place/contact paths changes.
    e2e_segment = (args.scope == 'full' and segment)
    if args.scope in PHASE_SCOPES:
        # PHASE_PLAN amendments (h)/(m): every phase-scope precondition stated, none defaulted
        assert segment, f'scope={args.scope} trains on --demo-format segment (the r2dreamer-native phase segments)'
        assert args.entry_bank and os.path.exists(args.entry_bank), f'scope={args.scope} needs --entry-bank (got {args.entry_bank})'
        assert args.action_mode == 'delta_joint' and args.delta_ref == 'target' and args.action_repeat == 4, (
            'place protocol: delta_joint / target / action_repeat 4')
        assert args.pick_shaping == 'off' and args.pick_hold_reward == 'off', 'pick levers are not place levers'
        if args.scope == 'contact':
            assert args.contact_grant, ('scope=contact needs --contact-grant (PHASE_PLAN (p): (l)\'s grip clause is '
                                        'withdrawn, (o) was stopped, and the corrected predicate awaits calibration -- '
                                        'there is no correct default and the Slide runs are held)')
        assert args.eval_freq == 0, 'in-train VideoEvalCallback evaluates the PICK; phase runs are scored post hoc by eval_place.py (pass --eval-freq 0)'
        assert args.train_max_steps == 600, f'phase-scope horizon of record is 600 sim steps (got {args.train_max_steps})'
    elif e2e_segment:
        # amendment (n): every end-to-end precondition stated, none defaulted
        assert args.entry_bank is None, 'scope=full resets from the pick-scope ICs; --entry-bank is a phase lever'
        assert args.action_mode == 'delta_joint' and args.delta_ref == 'target' and args.action_repeat == 4, (
            'end-to-end protocol: delta_joint / target / action_repeat 4')
        assert args.pick_shaping == 'off' and args.pick_hold_reward == 'off', 'pick levers are not end-to-end levers'
        assert args.eval_freq == 0, ('in-train VideoEvalCallback evaluates the PICK; end-to-end runs are scored by '
                                     'baselines/eval_e2e.py (pass --eval-freq 0)')
        assert args.train_max_steps == 1200, f'end-to-end horizon of record is 1200 sim steps (got {args.train_max_steps})'
    else:
        assert not segment and args.entry_bank is None, '--demo-format segment / --entry-bank are scope=place/contact/full levers'
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
    # the env reads the world's shelf offset (placed_v2 band) from this variable (private full_env copy of record)
    os.environ['GENESIS_SIM_VARIANT'] = args.sim_variant
    env = FullTaskEnv(backend='cpu', max_steps=args.train_max_steps,
                      scope=args.scope, action_mode=args.action_mode,
                      action_repeat=args.action_repeat, delta_ref=args.delta_ref,
                      entry_bank=args.entry_bank, phase_sparse=(args.scope in PHASE_SCOPES),
                      contact_grant=args.contact_grant,
                      pick_hold_reward=hold_reward, pick_hold_k=args.pick_hold_k,
                      pick_shaping=(args.pick_shaping == 'on'),
                      # shaping gamma = the AGENT's discount (Ng invariance needs them
                      # equal; before 08-23 the env silently used 0.998 whatever --gamma was)
                      pick_shaping_gamma=args.gamma,
                      pick_shaping_terminal_zero=(args.pick_shaping_terminal_zero == 'on'),
                      ladder=args.ladder, far_release=args.far_release,
                      tip_guard=args.tip_guard,
                      # D7: constructor argument, gamma matched to the AGENT's discount so
                      # the potential is exactly policy-invariant (Ng et al. 1999)
                      goalward_shaping=(args.goalward_shaping == 'on'),
                      goalward_gamma=args.gamma)
    # asserts: no silent defaults -- the env must be running the mode we asked for
    assert env.scope == args.scope, (env.scope, args.scope)
    assert env.action_mode == args.action_mode, (env.action_mode, args.action_mode)
    assert env.action_repeat == args.action_repeat, (env.action_repeat, args.action_repeat)
    assert env.delta_ref == args.delta_ref, (env.delta_ref, args.delta_ref)
    assert env.pick_hold_reward is hold_reward, (env.pick_hold_reward, hold_reward)
    assert env.pick_hold_k == args.pick_hold_k, (env.pick_hold_k, args.pick_hold_k)
    assert env.goalward_shaping is (args.goalward_shaping == 'on'), env.goalward_shaping
    assert args.goalward_shaping == 'off' or args.scope == 'full', 'goalward shaping is a scope=full lever'
    assert env.ladder == args.ladder, (env.ladder, args.ladder)
    assert env.far_release == bool(args.far_release), (env.far_release, args.far_release)
    assert env.tip_guard == args.tip_guard, (env.tip_guard, args.tip_guard)
    assert args.tip_guard == 'grip' or args.scope == 'full', 'tip_guard is a scope=full lever'
    assert args.ladder == 'staged' or args.scope == 'full', 'ladder is a scope=full lever'
    apply_post(env, args.sim_variant)
    assert abs(env._pick_gamma - args.gamma) < 1e-12, (env._pick_gamma, args.gamma)
    if args.scope in PHASE_SCOPES:
        import sim_variants as _sv
        from replay_harness import BOX_TOP_Z as _BT
        _want = float(_BT) + float(_sv.VARIANTS[args.sim_variant].get('shelf_dz', 0.0)) if args.sim_variant != 'base' else float(_BT)
        assert env.phase_sparse and abs(env.shelf_top_z - _want) < 1e-9, (env.phase_sparse, env.shelf_top_z, _want)
        print(f'[env] {args.scope} scope: entry bank {args.entry_bank} ({len(env._entries)} entries), shelf_top_z '
              f'{env.shelf_top_z:.3f}, phase_sparse (tips terminate, no penalty), +1 on '
              f'{"placed_v2" if args.scope == "place" else "contact"}', flush=True)
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
    # Same argument for the STAGED ladder of scope='full' (amendment (n)): its max episode return is
    # sum(STAGE_REWARD) = 8, so a 2.0 threshold would fire on every healthy end-to-end critic. Warning threshold
    # only -- the watchdog prints, it never stops a run -- and no other scope's number changes.
    if hold_reward:
        _max_ret = (1.0 - args.gamma ** args.pick_hold_k) / (1.0 - args.gamma)
        q_watch = 2.0 * _max_ret
    elif args.scope == 'full':
        # the LADDER's max return, not the module default: a sparse run's ceiling is 1.0, so
        # a watchdog keyed on 8.0 would never fire and a staged one keyed on 1.0 would always
        q_watch = 2.0 * float(sum(env.stage_reward.values()))
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
    if e2e_segment:
        # amendment (n): FULL-scope segments -- staged rewards as recorded, terminals only where the recording
        # terminated. Same rows the world model trained on; see baselines/rl/full_demos.py.
        from full_demos import segment_transitions_full, print_segment_census as print_full_census
        transitions, census = segment_transitions_full(
            str(REPO / args.demo_dir), expect=dict(sim_variant=args.sim_variant, action_repeat=args.action_repeat,
                                                   delta_cap=env.delta_cap, scope='full'),
            # amendment (aa): the set must have been BUILT under the ladder this run TRAINS under.
            ladder=args.ladder)
        print_full_census(census, tag=args.demo_dir)
        assert census['n_transitions'] > 0, 'empty segment demo set'
        norm = None
    elif segment:
        from place_demos import segment_transitions, print_segment_census
        transitions, census = segment_transitions(
            str(REPO / args.demo_dir), expect=dict(sim_variant=args.sim_variant, action_repeat=args.action_repeat,
                                                    delta_cap=env.delta_cap, phase=args.scope, terminal_reward=1.0))
        print_segment_census(census, tag=args.demo_dir)
        assert census['n_transitions'] > 0, 'empty segment demo set'
        norm = None
    elif native:
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
               # 2026-09-07: cap/leash travel with the artifact (review: eval integrators must not hard-code them)
               'delta_cap': env.delta_cap, 'delta_leash': env.delta_leash,
               'entry_bank': (os.path.abspath(args.entry_bank) if args.entry_bank else None),
               'contact_grant': args.contact_grant,
               'phase_sparse': bool(getattr(env, 'phase_sparse', False)),
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
    sidecar['goalward_shaping'] = args.goalward_shaping
    sidecar['ladder'] = args.ladder
    sidecar['far_release'] = bool(args.far_release)
    sidecar['tip_guard'] = args.tip_guard
    sidecar['max_return'] = float(sum(env.stage_reward.values()))
    (out / 'wandb_eval').mkdir(parents=True, exist_ok=True)
    (out / 'wandb_eval' / 'snapshot.action_mode.json').write_text(json.dumps(sidecar))
    # ---- LADDER PROVENANCE (LADDER_UNIFY_BRIEF D6) -----------------------------------
    # Every trainer writes the ladder + the sha256 of the code that defines it into its own
    # logdir, at START, so a run can be traced to the objective it actually optimised. This
    # is the fix for "no run stamps the code it loaded" (audit brief §4): an env-var gate was
    # set at submission and inert inside the job, and nothing on disk could have revealed it.
    _prov = env.provenance()
    _prov.update(run=out.name, learner='rlpd', arm=args.demo_dir, seed=args.seed,
                 steps=args.steps, steps_unit='decisions', sim_variant=args.sim_variant,
                 written='start')
    (out / 'ladder_provenance.json').write_text(json.dumps(_prov, indent=1))
    print(f'[ladder] wrote {out}/ladder_provenance.json', flush=True)

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

    class EpisodeRolloutLogCallback(BaseCallback):
        """PHASE_PLAN amendment (u): one JSONL row per FINISHED online rollout episode --
        `step` (decisions) plus the episode's sticky stage flags -- i.e. the same shape of record
        r2dreamer writes as `episode/train_*` in metrics.jsonl, which is what makes a learning
        curve / ignition-step comparison possible for this learner at all.

        LOGGING ONLY, and deliberately inert: it reads `self.locals` (SB3 puts `dones`/`infos`
        there in collect_rollouts), wraps nothing, draws no random numbers, and swallows every
        exception -- a logging fault must never kill a training run. Cost measured on the e2e
        recipe: ~800 finished episodes per 250k-decision run, ~200 B per row, so ~0.16 MB per run
        and no measurable runtime."""

        # nested_v2 added 2026-09-10 (brief D3): it replaces `nested` (the proxy) in every
        # log, table and figure. `nested` stays so old and new runs remain comparable.
        FLAGS = ('picked', 'placed', 'placed_v2', 'contact', 'contact_push', 'nested',
                 'nested_v2', 'slide_success', 'tipped')

        def __init__(self, path):
            super().__init__()
            self._path = pl.Path(path); self._n = 0; self._warned = False
            self._acc = {}          # env index -> stages seen SO FAR this episode

        def _on_step(self):
            try:
                dones = self.locals.get('dones'); infos = self.locals.get('infos')
                if dones is None or infos is None:
                    return True
                rows = []
                for i, (d, inf) in enumerate(zip(dones, infos)):
                    # Accumulate on EVERY step, not only the done step. `info` reports a stage
                    # at the step it is granted, and SB3 auto-resets on done (so the env own
                    # `_granted` set is already cleared by the time a callback could read it) --
                    # a point-read at the terminal step therefore records ONLY the stage that
                    # ENDED the episode. Measured 2026-09-09 on e2e_rlpd_dDPfirst_s920: `tipped`
                    # (which terminates) fired in 4 of 17 episodes while `picked` read 0 in all
                    # 17. Accumulating is correct whether or not `info` happens to be sticky.
                    acc = self._acc.setdefault(i, set())
                    if isinstance(inf, dict):
                        for k in self.FLAGS:
                            if inf.get(k):
                                acc.add(k)
                    if not d:
                        continue
                    self._n += 1
                    row = {'step': int(self.num_timesteps), 'episode': int(self._n)}
                    for k in self.FLAGS:
                        row['episode/train_ep_' + k] = float(k in acc)   # sticky: the record
                        v = inf.get(k) if isinstance(inf, dict) else None
                        if v is not None:
                            row['episode/train_' + k] = float(bool(v))   # legacy terminal read
                    self._acc[i] = set()
                    rows.append(json.dumps(row))
                if rows:
                    with open(self._path, 'a') as fh:
                        fh.write('\n'.join(rows) + '\n')
            except Exception as e:                      # never fatal
                if not self._warned:
                    self._warned = True
                    print(f'[episode-log] disabled after error: {e}', flush=True)
            return True


    run = init_wandb(args, name=args.run_name or out.name, tags=('rlpd',),
                     project=args.project)
    cbs = [EpisodeRolloutLogCallback(out / 'episode_rollouts.jsonl')]
    if args.ckpt_every > 0:
        cbs.append(SidecarCheckpointCallback(json.dumps(sidecar), save_freq=args.ckpt_every,
                                             save_path=str(out), name_prefix='rlpd'))
    else:
        print('[ckpt] periodic snapshots disabled (--ckpt-every 0): only the '
              f'{args.ckpt_fracs} archive + rlpd_final.zip are written', flush=True)
    cbs += [ArchiveCheckpointCallback([float(f) for f in args.ckpt_fracs.split(',') if f.strip()],
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
