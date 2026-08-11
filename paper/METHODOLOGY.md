# Methodology Reference — Human vs. Model Demonstrations for Imitation Learning
### (pick-and-place on a simulated Kinova gen3-lite; full experimental methodology as of 2026-08-11)

This document is the complete methodological reference for the study. Every
claim-bearing number cites its protocol; every measurement instrument's known
failure modes and the controls that catch them are documented, because the
integrity machinery is itself a contribution. Companion documents:
`PAPER_PLAN.md` (living decision log), `paper/results_matrix_2026-08-11.md`
(current numbers), `paper/results_significance.md` (Bayesian analysis),
`r2dreamer/GENESIS_PORT_STATUS.md` (world-model arm engineering ledger).

---

## 1. Research question and pre-registered hypotheses

**Question.** Does training an imitation policy on model-generated
demonstrations differ from training on human demonstrations — and does the
answer depend on the learner class (pure imitation / RL-from-demos /
world-model)?

Hypotheses were locked before generation-1 results were read (PAPER_PLAN §2):

- **H1**: Behavior cloning on model demos underperforms human demos at matched
  N. *(Outcome: refuted — inverted in-distribution.)*
- **H2**: Model-demo action distributions have measurably lower entropy /
  higher smoothness than human demos. *(Distributional analysis pending.)*
- **H3** (contingent on an H1 gap): noise injection on model demos recovers
  part of the gap. *(Moot — no gap in the hypothesized direction.)*
- **H4**: world-model learners benefit ~equally from human and model
  demonstrations, because they consume demonstrations as *dynamics/reward
  data* rather than as action supervision. *(The cluster statistics wave now
  running is the direct test.)*
- A null outcome was pre-declared publishable.

**Standing integrity rules** (user-set, enforced throughout): every reported
number from ≥3 seeds with negative controls; every trained policy gets eval
videos; corrected metrics over flattering ones; datasets travel by rsync only
(never git); official numbers only from same-machine baselines.

---

## 2. Task, simulator, and world

- **Task**: pick a soda-can-sized cylinder from a table region and place/nest
  it against a static goal can on a shelf. Stages: **picked → placed →
  contact → nested**. The controlled study is scoped to the **pick phase**
  (Decision Log 2026-08-01): matched-N is only feasible there, and the pick
  predicate is the hack-resistant one.
- **Simulator**: Genesis 0.2.1 + 269 upstream commits + one local
  headless-render patch (commit f41427d in the local checkout) — not
  pip-reproducible; the checkout travels by rsync. CPU backend for physics
  (one world per process); 30 Hz control; substeps=8 world config with
  measured finger stiffness (kp 40, force 50).
- **Engine gate**: Genesis 1.2.1 was evaluated and rejected — it fails the
  pinch-grasp that the task depends on. All results are on the 0.2.1 lineage.
- **Determinism caveats** (measured): replay is machine-dependent (same-machine
  baselines mandatory; cross-box placed-stage differed +4/75) and load-dependent
  (heavy CPU load flips borderline demos → official numbers only from an idle
  machine, ×3 repetitions).

---

## 3. Demonstration corpus (the human source, dH)

### 3.1 Provenance
93 in-the-wild teleoperation trials on the real robot (ROS bags: commanded
joystick `vel_cmd` + `gripper_pos` streams). 2 are stubs (gripper never
closes) → 91 graded. 75 success-labeled, 16 fail-labeled. Of the successes, 9
never achieve a pick under simulator replay → **66 IL-usable demonstrations**,
the dataset-size ceiling for every matched comparison (`TARGET_KEPT=66`).

By furthest stage reached (replay-measured): 2 picked-only, 38 placed,
6 contact, 20 nested.

### 3.2 Initial-condition recovery
The bags do not record object poses, so per-trial can placements were
recovered:
- **Forward kinematics seeding** from the arm trajectory at grasp closure
  (validated against bag tool poses to ~1 cm), followed by CPU-parallel
  candidate search (`cpu_research.py`) scored by replay success — by
  construction only "ok-class" placements (solvable under commanded replay)
  are accepted.
- **Goal-can position** was recovered from placement *slide* geometry: two
  user-validated demos' final can positions must touch the goal → two-circle
  intersection, disambiguated by slide directions → goal **(0.672, −0.221)**.
  A ring-fit alternative was rejected after diagnosis (one-sided contact
  coverage biases fixed-radius circle fits).
- **Adversarial review**: an independent 4-agent panel audited this pipeline
  (2026-07-08), overturned a rigged negative control and a coverage
  overstatement; all fixes (goal-relocation ban for fail-labeled demos, picked
  precondition on nested) are baked into the current metrics.
- Fail-labeled demos never receive relocated goals (would manufacture
  success); they serve as negative-control material and RL negatives only.

### 3.3 Replay validation
Demonstrations are replayed through the training environment using the **raw
commanded** streams (raw gripper command pass-through including out-of-range
values; float64 arm targets) — bit-identical to the reference replay harness
(`trace_env_ablate.py`). Whole-corpus replay at the final goal: 61/75 picked,
47 placed, 15 contact, 16 nested; fail-demo negative control shows the nested
metric's false-positive floor (2/16).

---

## 4. Success predicates (measurement instruments)

All predicates are computed by the environment from simulator state; none are
learner-visible beyond the scalar reward.

- **Picked (hardened, 2026-08-09)**: can z > pick_z AND gripper commanded
  closed AND ‖eef − can‖ < 0.20 m, sustained **10 consecutive frames**.
  History: the original z+grip predicate was exploitable by *flinging* — RL
  policies batted the can airborne with closed fingers, collecting "picks" at
  episode step 6–11. Every reward-optimizing learner found this exploit before
  finding the task (SACfD 0.40 "teacher", dv3 +100-reward run at 30% phantom
  train rate). The hardening was **certified fair** by a frozen-policy
  control: a diffusion-policy checkpoint scores byte-identically under old and
  new predicates (0.67/0.53/0.20/0.07), so genuine grasps are unaffected.
- **Tip rule**: terminate when the can lies tipped *free* (tilt > 60° AND
  grip commanded open < 0.3). Census-backed: past 60° the can never recovers
  (31/32 demos); the grip-open guard is essential because demos legitimately
  carry the can pitched > 60° in-hand.
- **Placed_v2 (release-based)**: grip commanded open + can inside shelf
  footprint/z-band + tilt < 20°, sustained 10 frames.
- **Nested (honest, eval-only)**: picked precondition + proximity
  (center-distance ≤ 0.081 m) + both cans upright + settled — measured after a
  physics settle, never per-step.
- **Doctrine**: control/action mode and predicate versions never default
  silently — they travel with the artifact (config, sidecar JSON) or the
  call site passes them explicitly. Seven silent-default bugs this cycle
  (grip-column ×3, control-mode ×3, eval action-mode ×1) motivated the rule.

---

## 5. Demonstration sources

- **dH (human)**: the 66-demo corpus above. For the pick-phase study, tapes
  are truncated at the pick grant (same truncation rule for all sources).
  For BC only, idle-frame pruning is applied (teleop pauses collapse;
  29.6% of frames) — **both rates are reported** (pruned 0.67–0.80 vs
  unpruned-control 0.27 in-dist): a source property, since model demos
  contain no idle frames.
- **dDP (model, generation-0)**: harvested from a trained diffusion policy by
  closed-loop rollout with **verified harvesting**: (a) open-loop replay
  guard against manufactured success; (b) a random-policy teacher must yield
  ~0 (negative control); (c) `MIN_KEEP_FRAMES=100` rejects fling-signature
  episodes (this guard is what exposed the SACfD fling teacher: eval said
  0.40, harvest kept 0/320); (d) episode caps at 1200 steps (cap-600 runs are
  flagged; a 300-step cap once silently truncated closed-loop teachers).
  Harvests record per-episode stage labels and IC provenance.
- **dSACfD**: contingent on the delta-action SACfD wave producing a genuine
  picking teacher (the absolute-action teacher was fling-artifact; its
  harvest correctly yielded zero).
- **Matched-size rule**: every cross-source comparison uses N=66 episodes and
  the demo-IC distribution (the demos' own recovered can placements), so
  neither size nor IC coverage confounds source.

---

## 6. Learners

### 6.1 BC: Diffusion Policy (lerobot)
State-only observations (proprio + environment state + goal), joint-space
absolute actions, 100k steps, batch 64, seeds ×8 per source. The only learner
with a confirmed positive control (0.67 in-dist on demo ICs,
audit-replicated). Success-labeled demos only (user rule: "it's not fair to
train imitation-learning algorithms on failed data" — inverted for RL/WM
learners, which receive fails as zero-reward data).

### 6.2 RLfD: SACfD (SB3 SAC + demo replay injection)
Demo transitions relabeled with the staged reward by env-measured episode
stage, injected into the replay buffer (duplicate ×3), 200k–400k env steps.
Two action-space arms:
- **Absolute joint targets** (original): the honest-predicate result is
  uniformly zero (n=15 completed runs, both sources) — training reward flat at
  0 for the entire budget. This is retained as the *action-geometry control*.
- **Delta-joint** (2026-08-11): the same delta encoding as the world-model arm
  (§7) applied to `FullTaskEnv` and to the demo buffer (per-episode delta
  re-encoding; 66/66 pick grants preserved; open-loop replay gate 3/3).
  16-job wave (2 sources × 8 seeds) currently running.

### 6.3 World model: DreamerV3 (dreamerv3-torch)
Pixel observations (top + wrist cameras, 64×64×6), absolute joint actions,
demo episodes converted with action normalization into the actor's [−1,1]
convention (an earlier unnormalized-demo bug — 62.6% of demo action values
outside [−1,1] — silently poisoned all runs v6–v13 and motivated the
convention audits). Periodic sampled-action evals with scope-matched
environments. Result: an instrumented null through 3–4M steps (both sources)
— retained as the second action-geometry control.

### 6.4 World model: r2dreamer (the working arm)
r2dreamer (decoder-free DreamerV3 variant; Barlow-Twins-style representation
loss) ported to the task. The recipe that produced the confirmed champion
(`configs/env/genesis_pick_v5d4_delta.yaml`):

| component | value | provenance |
|---|---|---|
| action space | delta-joint, cap 0.025 rad/sim-step | §7 |
| actor distribution | `bounded_normal_clipped` | §7.3 |
| action repeat | 4 (one decision / 4 sim steps) | timescale-matched to their benchmarks; demos downsampled by the same rule, verified by paired open-loop replay |
| time limit | 400 sim steps (100 decisions) | ManiSkill-parity |
| entropy bonus | 3e-5 | entropy-ratchet diagnosis (sparse reward → advantage ≈ 0 → entropy-only gradient) |
| horizon (discount) | 333 (γ ≈ 0.997) | critic-leak diagnosis (reward-head leak × horizon inflated values) |
| reward scale | 100 (env and demos identically) | credit visibility |
| demo prefill | full set, ×4 duplication (~10% of the 450k buffer) | §6.5 |
| demo re-injection | full duplicated set every 150k online steps | FIFO-eviction diagnosis |
| actor-BC | **0, by design** | user rule: BC on demo actions confounds H4 — the world-model arm must consume demos as data only. (A BC pilot learned fastest of any variant and was aborted on this rule; retained as a diagnostic that the bottleneck is exploration/credit, not model capacity.) |

### 6.5 Why demo density is the load-bearing lever
The world-model learning mechanism is a *backward value cascade*: the reward
head learns demo terminals → the critic learns value near the pick → the
actor improves there → value propagates one link earlier per iteration. Its
fuel is rewarded demo frames per training batch (`train/data/reward_frames`).
Direct checkpoint interrogation (reward/value heads evaluated on demo
episodes) showed the reward head near-perfect (predicts ~94–97 at true-100
terminals, background leak ~0.08) and the failure localized to value
propagation: value ≈ 107 five steps before the pick but ≈ 2 at episode start.
The starved variant's batches decayed to zero rewarded frames (demos diluted
from 2.4% of buffer); ×4 duplication + 150k re-injection holds 4–9 rewarded
frames/batch for the whole run and flipped the arm from null to champion.

---

## 7. The delta-joint action space (the decisive design element)

### 7.1 Encoding
Policy actions a ∈ [−1,1]^7: arm dims are per-sim-step joint-target *deltas*
(a·cap radians) integrated onto a persistent target; grip is absolute
([−1,1] → 0..1). The target is clipped to joint limits and **leashed** to the
measured position (‖target − q‖ ≤ leash) so integration error can never run
away. On reset the target re-seeds from measured q (a stale target caused
first-step lunges until this was enforced on every reset path).

### 7.2 Calibration (from the demonstrations, not tuned)
Measured on 118,194 frames of commanded joint targets:
- **cap = 0.025 rad/sim-step ≈ the demos' p99 per-frame delta** (0.0259) —
  a saturated policy action moves at the demos' 99th-percentile speed
  (44°/s); 1.31% of demo frames clip on re-encoding. (An earlier cap of 0.04
  = 1.5× demo p99 allowed arm-strike "lunges".) Demo median |Δ| is 0.002 —
  humans are mostly nearly still.
- **leash = 5×cap = 0.125 rad ≈ the demos' PD-following lead p99** (0.126) —
  the leash is calibrated in absolute radians; cap and leash decouple.
- **Commanded (not measured) deltas**: encoding from measured joint positions
  fails the replay gate (the arm chases its own PD lag ~40 frames); commanded
  deltas integrate to the validated commanded-replay trajectory.
- **Downsample exactness**: mean-pooled per-frame deltas telescope to
  (cmd_{t+N} − cmd_t)/(N·cap), which the adapter re-integrates exactly — no
  converter-side downsampling; verified to 3e-4 rad reconstruction.
- **Grant slack**: pick-truncated tapes keep 48 raw frames past the recorded
  grant so the hardened predicate's 10-frame sustain can fire under open-loop
  replay lag.

### 7.3 Actor-distribution repair
r2dreamer's stock `bounded_normal` bounds the *mean* (tanh) but not samples;
with entropy-driven std near 1.0 the actor emitted actions at ±4. The
environment clips to [−1,1], but the replay buffer and imagination retain the
raw sample — the world model learns dynamics conditioned on action values
never executed, and the actor update consumes extreme-tail log-probs. The fix
(`bounded_normal_clipped`) projects samples — including `rsample`, the method
the collection and imagination paths actually call, which the stock `Bound`
wrapper never covered. DreamerV3-torch was audited and is unaffected
(`ContDist(absmax=1.0)`).

### 7.4 The replay gate (non-negotiable validation)
Any (demo set × action encoding × cap/leash) combination is launchable only
if held-out demos re-earn their pick terminal via **open-loop replay of the
exact training rows** through the exact training adapter (`replay_gate.py`).
The gate has caught: measured-vs-commanded encoding, grant-slack necessity,
and validates every re-encoded set (0.025 human set 3/3; harvested sets are
gated with per-episode ICs from raw states since their ICs are random draws).

### 7.5 The cross-learner geometry finding
Action-space geometry interacts with **exploration, not imitation**: BC
thrives on absolute joints (it never explores); every exploring learner
failed on them (SACfD 15×0, dv3 null, r2dreamer-absolute null — entropy-driven
sampling through absolute targets is arm thrash, and the hardened predicate
removed the fling shortcut); the same world-model recipe went from null to
0.91/1.00 on delta actions. The delta-SACfD wave tests whether the fix
generalizes to model-free RLfD.

---

## 8. Evaluation protocol

### 8.1 Rollout evaluation
- **Dual IC modes**: `eval_indist` = demo ICs (the recovered per-trial
  placements; the headline, matched to the demos' support); `eval_random` =
  support-random ICs (generalization). Never mixed. 15 episodes per eval.
- **Action selection**: sampled actions are the protocol default (matches the
  training-time collection policy); mode (deterministic) is reported
  alongside where relevant. Sampling equivalence to the trainer was verified
  bit-exactly (RNG-matched action-conditioning test, 2026-08-08).
- **Eval-environment fidelity**: every action-semantics parameter (action
  mode, cap, leash, repeat, reward scale) is read from the run's own config —
  an eval that defaulted the action mode evaluated delta policies in a
  different MDP (all pre-fix delta evals were voided and re-run).
- **Videos**: every eval renders per-episode top|wrist videos and a single
  tiled grid (outcome-labeled, real-time playback) to wandb.

### 8.2 The checkpoint lottery and the best-checkpoint protocol
World-model training on this task is **bistable**: policies oscillate between
champion-level and destroyed on a ~170k-step cycle with knife-edge (<5k-step)
collapse onsets. Verified consequences: checkpoint quality is a phase
lottery (a checkpoint written 4k steps from a 0.3-train-rate tick evaled
0/15; the same run's neighbors evaled 1.00). The measurement-side answer,
pre-registered:
1. During training, every checkpoint write is archived and evaluated
   (15 episodes, sampled, eval-seed 0) — the **selection** eval.
2. The best checkpoint by selection eval is then **confirmed** on three fresh
   eval seeds (+1 mode eval) — selection and confirmation on independent
   draws, eliminating selection bias.
3. Reported per run: the confirmation mean (headline), the full
   per-checkpoint eval series (the stability figure), and the final
   checkpoint (for completeness).
- Mechanism status (honest): λ-return targets exceed the task's known
  maximum return before every collapse (130–640 vs max 100, bootstrap
  compounding through terminals), but a clamp pilot **refuted causality** —
  clamping targets to 100 eliminates the overshoot entirely while collapse
  cycles persist unchanged. The overshoot is a correlate; the root cause
  (candidates: AMP inf-gradient events, re-injection distribution shocks)
  remains open. Stability claims therefore rest on the measurement protocol,
  not on a mechanism fix.

### 8.3 Statistics
- **Hierarchical Beta–Binomial** (seed-level success probabilities, grid
  marginalization) for source comparisons — pooled binomials are
  anti-conservative under seed variance. Current BC verdicts: in-dist
  P(model > human) = 0.994 (decisive); random-IC P = 0.446 (null).
- **Discovery-phase bimodality**: world-model seeds either discover picking
  or never do (local: seed 0 discovered at ~300k; seed 1 never in 1.9M).
  Statistics therefore report **P(discovery)** and **skill-given-discovery**
  separately; ≥5 seeds per source (cluster wave).
- Negative controls at every layer: random-teacher harvests, fail-demo demo
  replays, frozen-policy predicate certifications, matched-seed ablations.

### 8.4 Instrument-failure ledger (what the controls caught)
Documented because the pattern is a finding: *every reward-optimizing learner
found the measurement's weakness before the task's solution.*
1. Fling exploit of the pick predicate (SACfD 0.40, dv3 +100 30% train rate).
2. Z-band "placed" credited by knocked-onto-shelf cans (replaced by
   release-based placed_v2).
3. Nested metric without picked precondition (panel-caught, fixed).
4. Rigged negative control in IC recovery (panel-caught, fixed).
5. Eval action-mode default (delta policies evaluated as absolute — all
   affected evals voided and re-run).
6. Unnormalized demo actions into dv3 (runs v6–v13 voided).
7. Training-metric illusions from lagging windows and syncer state (two
   same-day false alarms retracted; monitoring now keys on primary evidence).

---

## 9. Compute and reproducibility

- **Local dev box** (Ryzen 5950X / RTX 3080Ti 12GB / 62GB): pilots, forensic
  probes, one-off ablations. Official numbers are never mixed across machines
  (measured cross-machine replay drift).
- **Cluster** (Tufts HPC, Slurm; L40S/A100/L40/H200): the statistics waves.
  One GPU per run; Genesis worlds are CPU-side spawn workers. Environments:
  a pip-only conda env for Genesis/lerobot/SB3 (conda library installs are
  banned after a conda-ffmpeg libstdc++ incident poisoned the stack), and an
  isolated py3.11 venv for r2dreamer (torch 2.8; interpreter-only conda
  fallback; install script ends with a world-build verification gate).
- **Provenance**: code in git (the r2dreamer port ships as a pinned-upstream
  tarball in-repo); datasets by rsync only; every trained artifact carries
  its config (hydra) and control-mode sidecar; wandb projects:
  `genesis_paper` (matrix), `dreamer_v3` (dv3), `r2dreamer_genesis`
  (world-model arm), with eval curves logged into the training runs.

---

## 10. Current experimental status (2026-08-11 evening)

Running: 10-job r2dreamer matrix wave (dH×5 / dDP×5, champion recipe,
best-checkpoint protocol in-job); 16-job delta-SACfD wave; local seed-2 and
clamp-ablation runs. Complete: BC n=8 both sources (+ hardened-predicate
robustness re-evals); absolute-SACfD control row; dv3 null rows; local
r2dreamer champion (0.91 sampled n=45 / 1.00 mode, single seed, demo ICs).
Pending: H4 verdict from the wave; H2 distributional analysis; random-IC
world-model evals; place-phase extension (banked entry-state machinery
exists; deferred).
