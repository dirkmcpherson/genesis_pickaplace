# ROUND ROBIN — WHAT IS RUNNING (submitted 2026-08-19 ~23:15 EDT)

The full 33-job block from ROUND_ROBIN_2026-08-20.md "FINAL THURSDAY SUBMIT
BLOCK" went up tonight (a day early). Verified against squeue: 12 rlpd +
3 dp-round + 8 r2d-trai + 10 genesis- (dv3) = 33, job IDs 2667157-2667192.
13 running immediately; the rest queue behind the per-user GPU cap
(QOSMaxGRESPerUser) and drain automatically.

## Job census

| condition | jobs | seeds | steps | demo set (files) |
|---|---|---|---|---|
| dDP_RLPD (core) | 6 | 0-5 | 100k | m1all_harvest (93 npz; fails kept as zero-reward negatives) |
| dH_RLPD-shaped (dense) | 6 | 3-8 | 100k | episodes_pick_phase_all (91 npz = 66 staged successes + 25 no-pick negatives) |
| dR2D_DP (core) | 3 | 0-2 | 100k | lerobot_dR2D_pick (66 episodes / 9,184 frames) |
| dR2D_r2dreamer (core) | 4 | 40-43 | 3M | genesis_r2dchamp_delta25 (52 pixel demos) |
| dH_r2dreamer-shaped (dense) | 4 | 50-53 | 3M | genesis_pick_pruned_delta25 (67 pixel demos) |
| dv3 core dH / dDP / dR2D | 3+2+2 | 0-2 / 0-1 / 0-1 | 300k | genesis_{pick,m1all,champion}_msr_delta25_r4 (67 / 93 / 52) |
| dv3 dH-shaped (dense) | 3 | 0-2 | 300k | genesis_pick_msr_delta25_r4 (67) |

Every job passed its launch gates (fixed-code grep, per-ARM demo provenance
pattern, stale-env guard, RUN_REGISTRY duplicate check) or it would have
refused before touching a GPU. r2d shaped jobs ran past startup = the shaped
hydra config rsynced correctly.

## Observation and action spaces per algorithm

The deep asymmetry to keep in every caption: **RLPD and DP are state-based
(they see ground-truth can pose); r2dreamer and dv3 are pixel-only.**
All four share the same simulated Kinova gen3-lite + can + shelf world
(FullTaskEnv, scope=pick: episode terminates +reward on a lift).

### RLPD (baselines/rl/train_rlpd.py — REDQ-style SAC: ensemble 10, subset 2, UTD 10)
- **Observation:** Box(-inf, inf, (17,), f32) =
  [6 arm joint pos (rad), gripper 0..1, grip effort, can xyz (3),
  can quat (4), goal xy (2)]  (pick_env.STATE_DIM)
- **Action:** Box(-1, 1, (7,), f32) = 6 arm joint DELTAS + gripper 0..1.
  action_mode=delta_joint, delta applied to the PD TARGET (--delta-ref
  target), delta_cap 0.025 rad/sim-step, action_repeat 1 (one action per
  sim step).
- **Reward:** sparse pick +1 (terminate on lift). gamma 0.998.
- **Demos:** human/model tapes re-encoded to delta_joint transitions in-job;
  demo-batch 128 per update. dH encoding caveat: 13.3% of demo frames carry
  deltas beyond the cap (unrepresentable as-encoded; normalization audit).
- **Eval of record:** in-job fresh-process sweep, 15 demo-IC + 15 random-IC,
  one process per episode → SWEEP-RESULT line + wandb summary fields
  sweep/demoIC, sweep/randomIC. Run names {ARM}_RLPD-n20_s{S}[-shaped],
  project genesis_paper.

### DP — diffusion policy (lerobot fork, 100k steps)
- **Observation:** observation.state (8,) f32 = [6 joint pos, gripper,
  grip effort] ++ observation.environment_state (9,) f32 = [can xyz,
  can quat, goal xy]. State-only (no cameras) — the honest-vision variant
  stayed wave-2.
- **Action:** (7,) f32 = 6 ABSOLUTE arm joint position targets (rad) +
  gripper 0..1 (jact encoding).
- **Demos:** dR2D = 66 success episodes from the r2dreamer champion
  (state tapes, same teacher as the WM arms' 52 pixel demos — different
  render pass and count; caption caveat). dDP row (already done) trains on
  the 63 success stems only (BC convention).
- **Eval of record:** post-train wandb_eval --kind dp --ic-mode both
  (15 demo-IC + 15 random-IC, fresh process per episode), headline =
  demo-IC. Runs {ARM}_DP_s{S} + {ARM}_DP_s{S}-eval, project genesis_paper;
  DP-RESULT line in the .out.

### r2dreamer (config genesis_pick_v5d4c_delta[_shaped], 3M env steps, 6 CPU worlds)
- **Observation:** image ONLY — (64, 64, 6) uint8 = topB overhead RGB ++
  above-wrist RGB (encoder MLP keys empty; no proprio, no can pose).
- **Action:** (7,) in [-1,1] = 6 arm joint deltas + gripper. delta_cap
  0.025 rad/SIM-step; action_repeat 4 (the PD target integrates a*cap on
  each of the 4 sim steps per agent step); leash |target-qpos| <= 0.125 rad;
  actor_dist bounded_normal_clipped (samples projected into [-1,1]).
  time_limit 400 sim steps = 100 agent steps.
- **Reward:** sparse pick +1, reward_scale 100 (demos carry the same +100),
  return_clamp 100. discount 0.999. actor_bc_lambda 0 (H4 design: demos are
  dynamics/reward data only). demo_duplicate 4, reinject every 150k.
- **Eval of record:** BEST-checkpoint protocol — the sbatch archives
  checkpoints, evals each (-eval-step runs, eval/picked, project
  r2dreamer_genesis), and the final checkpoint often reads 0 on igniting
  seeds (bistability) — never judge by the last checkpoint. R2D-RESULT line.
- Logdirs runs/pick_v5d4c_delta[_shaped]_{ARM}_s{S}.

### dv3 (dreamerv3-torch, configs genesis_pixels + genesis_pick_msrecipe [+ _shaped], 300k steps)
- **Observation:** image ONLY — (64, 64, 6) uint8, same two-camera rig and
  channel order as the demo datasets (state key omitted in pixel mode).
- **Action:** Box(-1, 1, (7,), f32) = 6 arm joint deltas + gripper;
  delta_cap 0.025 rad/sim-step; action_repeat 4; time_limit 600 sim steps
  = 150 decisions.
- **Reward:** sparse pick, reward_scale 100 (+100 terminal, MS parity;
  demos carry the same number). discount 0.997 (global default, no genesis
  override).
- **Demos:** stride-4 stamped (repeat.json), gate-checked against ACTREP.
- **Eval of record:** eval_success_rate / log_picked in the -joint runs,
  project dreamer_v3; DV3-RESULT line. Round-robin ignition bar = nonzero
  picked at eval.

## The shaped (dense) variants — 13 of the 33 jobs
Identical to their core condition in EVERYTHING except training reward:

    r_shaped = r_sparse + gamma * phi(s') - phi(s),   phi(s) = -2 * ||eef - can||

- Potential-based (Ng et al.) → optimal policy invariant when gamma matches
  the agent's discount: RLPD 0.998, dv3 0.997, r2dreamer 0.999 (each
  hard-coded at its wrapper boundary, not configurable — by design).
- TRAINING-ONLY: every eval metric stays sparse fresh picked.
- WM arms multiply the shaped reward by the same reward_scale 100 as the
  sparse term (uniform scaling preserves the invariance).
- Known asymmetry: demo transitions in the buffers keep SPARSE-only labels;
  online transitions carry shaping. RLPD passed its local gate with this
  asymmetry (s0 0/15, s1 4/15, s2 5/15; pooled 0.20); for the WM reward
  heads it is an open caveat, registered before data.
- Local live-fire (08-19 night, this box): FullTaskEnv shaped-vs-plain
  traces are dynamics-identical with reward difference equal to the
  potential term at both WM gammas; r2dreamer shaped config live-ran
  end-to-end (prefill 268 episodes @ reward_scale 100, online steps clean);
  dv3 shaped stack live-run was mid-flight at submit time — confirmatory
  only, verdicts appended below when done.

## Where results land (no box needed)
wandb entity jambotime — genesis_paper (RLPD sweep summaries + DP evals),
r2dreamer_genesis (-eval-step runs), dreamer_v3 (dv3 -joint runs). One
greppable line per .out: SWEEP-RESULT / DP-RESULT / R2D-RESULT / DV3-RESULT.
Aggregation rules: paper/REVIEW_GUIDE.md.

## RESULTS AS OF 08-20 ~08:00 EDT (from wandb; sweep/eval fields are the record)
- RLPD COMPLETE 12/12. dDP core (s0-5): demo-IC 1,0,1,0,?,0 /15 — 0 seeds
  ignited (s4 pending read; max 1/15). Contrast: dH 8/16, dR2D 10/16 ignited.
  dH-shaped (s3-8): 1,6,3,6,0,9 /15 — 4/6 ignited, pooled 25/90 = 0.278 vs
  sparse-dH pooled 0.221 — the registered dense expectation (ignition up,
  ceiling similar) holding directionally at n=6.
- DP dR2D COMPLETE 3/3: in-dist 0.93 / 0.93 / 1.00; random 0.73 / 0.80 / 0.73.
  Best DP cells recorded, and random-IC is ~3x the prior best (0.23 for both
  dHpruned and dDP). Caveat: same-machine rule (cluster row vs cluster rows
  is the clean comparison); 66 champion success tapes.
- r2d core dR2D (s40-43): early checkpoints eval 0 (expected; training
  continues to 3M). r2d SHAPED dH (s50-53): early evals already read
  0.93 / 0.07 / 1.00 / 0.87 — checkpoint step NOT yet verified; if these are
  <200k-step checkpoints, dense collapses the r2d ignition lottery. Verify
  step before any claim.
- dv3 core dH s0-2: FINISHED ~325k, log_picked 0 (task-x-recipe null again,
  now on fixed seeds). dv3 dDP s0: FINISHED with log_picked 1 at end of
  training — first dv3-genesis sign of life; fresh eval numbers pending.
  dv3 dR2D s0 crashed at step 0 and was requeued (now 140k); s1 88k.
  dv3 shaped dH s0-2 mid-run (170-200k): train_return nonzero with zero
  picks = shaping term flowing as designed.
