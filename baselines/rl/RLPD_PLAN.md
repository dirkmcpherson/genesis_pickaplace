# RLPD implementation plan (agent-audited 2026-08-12; APPROVED FOR PLANNING ONLY — do not implement without user go)

Motivation: SACfD zeros survive correct action geometry (dj wave 0.00 x14+0.067)
AND visible discount (gamma-0.998 pilot 0.00 at 200k, 08-12). RLPD (Ball et al.
2023) = SAC + symmetric 50/50 online/demo batches + LayerNorm critics + high
UTD + ensemble-10/subset-2 min targets + NO BC — the published recipe for
sparse-reward-plus-prior-data, natively satisfying the demos-as-data rule.

## Incidental findings from the audit (both already acted on)
- train_sacfd_full ran at gamma=0.98 via build_model: demo median pick frame
  662 -> 0.98^662 ~ 1.6e-6 (terminal invisible). FIXED: --gamma flag, default
  0.998, banner printed.
- VideoEvalCallback never passed --action-mode and snapshots have no sidecar ->
  ALL in-train eval curves of delta runs before 08-12 evaluated in absolute
  mode (final evals fine). FIXED: action_mode param wired.

## Architecture (path A of three costed)
- NEW baselines/rl/rlpd_sac.py (~250 lines): RLPDPolicy(SACPolicy) building
  LN+ensemble critics AT CONSTRUCTION (save/load round-trips; classes NOT in
  __main__ so eval subprocess unpickles); DemoData pinned-tensor buffer;
  RLPDSAC(SAC) overriding train() only: (i) explicit two-buffer 128/128
  sample, (ii) target = min over random 2-of-10 TARGET critics, (iii) critic
  loss = sum MSE all 10, (iv) actor+alpha once per env step (last grad step),
  actor vs ensemble mean, (v) assert sb3 2.8.x. Ensemble as vectorized
  EnsembleLinear (baddbmm), not a module loop.
- EVOLVE train_rlpd.py in place: mirror train_sacfd_full flags (--scope pick,
  --action-mode delta_joint, --gamma 0.998, --project, sidecars incl STARTUP
  snapshot sidecar); demo path IMPORTED from train_sacfd_full
  (delta_encode_transitions) so tensors are bit-identical to SACfD's; delete
  the stale cartesian branch; config banner + asserts (env.scope/action_mode
  == args). Current file's live bugs: joint branch ignores --scope AND has no
  action_mode (silent-default family); LN monkey-patch breaks SAC.load.
- NEW baselines/rl/sacfd_delta_gate.py: durable open-loop gate (uids
  232/242/243 re-earn pick; tensor-equality assert vs train_sacfd_full).
- NEW cluster/launch_rlpd_wave.sh: d{src}_RLPD_s{seed}, 12h/1GPU/8CPU jobs.

## Pinned hypers
batch 256=128+128; E=10 Z=2 (verify subset convention vs ikostrikov/rlpd and
record); LN after each hidden Linear in critic q-nets only; UTD=10 critic,
actor/alpha x1 per env step, polyak per grad step tau .005; gamma 0.998;
target_entropy -3.5 (=-dim/2, 7-dim); Q WATCHDOG: warn if mean start-state Q >
2.0 (max return 1 + entropy term; the measured +536 explosion at gamma .999 +
auto alpha) with pre-registered fallback ent_coef=0.005; nets 256x256, lr 3e-4,
buffer 300k online-only, learning_starts 1000; demos 91 eps
episodes_pick_phase_all, done=True at pick grants, post-pick rows kept
(comparability with SACfD; --truncate-at-pick as later ablation).

## Compute (measured: t_env ~20ms, t_grad ~9ms at 2 critics/b256)
UTD-20 @400k = 8M grads ~22h — RULED OUT. Primary: UTD-10 @200k ~7h.
Alternative arm: UTD-20 @100k (~6h) only if pilot needs it. Re-measure fps in
the 50k probe before any cluster submit; size STEPS so projected <= 9h.

## Validation ladder (ALL before cluster spend)
(a) 5k CPU smoke: banner, tensor-equality assert, batch composition 128/128
printed once, critic tree shows LN+10 nets, update counters (critic = UTD x
steps-past-start; actor = steps-past-start), AND load round-trip: wandb_eval
--kind sac on the smoke zip in a fresh process must print "action_mode from
sidecar: delta_joint" and complete (guards the LN-state-dict/__main__ pickle
failure).
(b) sacfd_delta_gate 3/3.
(c) 50k GPU throughput probe; abort budget math if fps < 6.
(d) 200k local pilot, PRE-REGISTERED: PASS = eval_indist/picked >= 0.2 on two
consecutive in-train evals at >=100k, OR ep_rew_mean >= 0.15 sustained across
a full 20k window never dipping below 0.05 (distinguishes consolidation from
the dj wave's 0.01-0.05 flickers). FAIL -> single-lever ablations in order:
gamma 0.999+fixed alpha; UTD-20@100k; 512-wide nets. No cluster until a pass.
If Q watchdog trips: stop, fixed alpha 0.005, restart (never let it ride).
(e) Cluster wave: 3 seeds dH first; dDP only after dH signal. All-zero is an
honest null ("strongest published model-free recipe") — pre-registered.

## Risk map -> guards
Silent defaults (banner+asserts, no build_model reliance); action-scale
(imported encoder, tensor equality, gate); eval-mode (startup sidecar +
callback action_mode + smoke assertion); checkpoint-load (construction-time
LN, fresh-process round-trip); Q explosion (watchdog+fallback); compute
(measured budget, <=9h); actor-UTD entropy collapse (actor x1; log alpha).

## Deviations / choices made during implementation (2026-08-12)
Implemented per this plan. Concrete choices where the plan left room, and the few
places the operational prompt overrode the plan:

- **Training env = the repo's `.venv-eval`** (sb3 2.8.0, genesis 0.2.1, torch 2.7,
  CUDA), NOT `~/workspace/genesis_sim2real/venv` (that venv has no
  stable_baselines3). The sb3-2.8.x assert in rlpd_sac.py holds there.
- **cluster/launch_rlpd_wave.sh NOT written.** The operational prompt said do not
  write a cluster sbatch (implementation + smoke only; the cluster launch is a
  separate user decision). RLPD rides the SAME launch path SACfD uses -- the inline
  `sbatch --wrap` in cluster/launch_paper_week.sh -- by swapping the python entry
  point; there is no standalone SACfD sbatch file to mirror.
- **train_rlpd cartesian branch deleted** (per plan). The joint arm is the point; the
  EEF arm rides train_sacfd_full --cartesian. Recover from git history if needed.
- **rlpd_sac.RLPDSAC subclasses SAC and overrides train() only** (plan path A). The
  50/50 draw is an EXPLICIT two-buffer sample inside train() (128 online + 128 demo),
  not a monkey-patch of buffer.sample; the LN+ensemble critic is built AT
  CONSTRUCTION by RLPDPolicy.make_critic so stock `SAC.load` round-trips (verified in
  a fresh process by wandb_eval printing "action_mode from sidecar: delta_joint").
- **Ensemble = vectorized EnsembleLinear (baddbmm)**, LayerNorm after each hidden
  Linear only, forward returns (E,B,1). Each member initialised exactly like nn.Linear.
- **Critic loss = sum over E of per-critic MSE** (literal "sum MSE all 10"; dropped
  SB3's 0.5 prefactor -- Adam is ~scale-invariant, immaterial).
- **Actor loss vs ensemble MEAN Q; alpha step also once per env step** (final UTD grad
  step), matching "actor+alpha once per env step". ent_coef used in the critic target
  every grad step (detached), but the alpha optimizer steps only on the last.
- **target_entropy = -dim/2 = -3.5** (7-dim joint), set explicitly (not SB3 auto -7).
- **Q watchdog is a one-shot WARN at mean actor-state Q > 2.0** (not start-state only;
  batch-mean actor Q is the cheap proxy). It fired in the smoke (Q climbed 2.8 -> 148
  over 700 steps at gamma .998 + auto alpha) -- exactly the pre-registered explosion
  signal; the fix (fixed --ent-coef 0.005) is a human decision, not automatic.
- **Gate uids = [308,325,297,326,265], pass >=4/5** (NOT the plan's guessed
  232/242/243). Empirical finding: every episodes_pick_phase_all pick demo is
  MARGINAL-TERMINAL (truncated ~2 frames past pick_z, <=7mm lift), 232 lifts only to
  0.1456 under delta replay and 259 isn't in the env placements map. Of the gentlest
  resettable pick demos 9/11 re-earn the pick under delta replay (lift ~1.0); the five
  chosen are the most robust. Tensor-equality (RLPD demos == SACfD encoder output)
  stays a HARD assert.
- **Demos = episodes_pick_phase_all (91 eps, 83,465 delta transitions, 66 rewarded)**,
  encoded by train_sacfd_full.delta_encode_transitions(scope='pick') -- bit-identical
  to SACfD (asserted by sacfd_delta_gate). Note the sparse signal: ~0.10 rewarded demo
  frames per 128-demo batch; RLPD PINS that density permanently (vs SACfD dilution).
