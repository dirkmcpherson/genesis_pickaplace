# Learner health — consolidated verdicts, 2026-08-28

USER (A14): "the top priority is ensuring RLPD and WM runs are behaving appropriately before we worry
about human vs machine demos." USER (A15): "don't do anything that would cause our implementations to
diverge significantly from the standard versions." Every human-vs-machine job is HELD until each
learner below is judged healthy. Sources: `paper/DV3_DIAGNOSIS_2026-08-28.md`, the r2dreamer health
audit (this file §2), the RLPD divergence census (SESSION_LOG 08-28 11:10, PAPER_NOTES N18).

## 0. A bug that touched every learner (fixed 08-28)
`FullTaskEnv` pick scope paid every stage that flipped in the terminate step, so a fast pick that also
crossed the shelf plane paid **+2** (8/56 dR2D demos; up to 31 % of online episodes in some WM runs;
the "reward 2.0 on the terminal row" the converters were made to accept). Max return of the pick task
was 2, not 1. Fixed: a single-stage scope pays exactly its own terminal (`full_env.py`); both native demo
loaders normalise old tapes to +1 and report the count (`to_dreamer_native.py` → `repeat.json
n_double_grant`; `train_sacfd_full.native_demo_transitions` → census). Runs before 08-28 carried the bug
identically across learners and sources; it is disclosed, not a confound between arms.

## 1. RLPD — UNHEALTHY (critic divergence), fix in flight
- 69/74 runs: critic_loss > 1 at some point (61 > 100). Q watchdog (mean Q > 2× max return) trips by
  10–30k steps in nearly every run and re-trips every 10k until the end.
- Training success reaches 0.85–0.97 before the blow-up (dDP s24: 0.97 @75k → 0.30 @98k). Every RLPD
  number in the tables is "best checkpoint before divergence"; final < selected in 47/74 runs.
- Ruled out: entropy (α → 4e-4 by 30k), LayerNorm (present), the corrected world (random-policy
  exploration identical across worlds).
- Deviation from the paper recipe: γ 0.998 (paper 0.99). Fix factorial N18 running at the front of the
  RLPD queue: {γ 0.99; γ 0.99 + UTD 5; γ 0.998 + UTD 5} × dH/dDP × 3 seeds. Verdict criterion:
  watchdog silent after 30k, max critic_loss < 1, final ≈ selected, training success survives.
- If it holds: the RLPD matrix is re-run under γ 0.99 (≈80 jobs) and all prior RLPD numbers superseded.

## 2. r2dreamer — UNSTABLE; its recipe DEVIATES from DreamerV3 in load-bearing ways
- Critic pinned at 90–99 (clamp 100) on non-picking states for millions of steps (dR2DDPfails s84:
  value 91–94 for 2.0M steps at picked 0.00; dH s101: 71–84 all run at picked ~0). The clamp masks
  overestimation; it does not prevent it. Reward head predicts 3–25× the realized per-step reward in
  no-pick phases (dense shaping optimism).
- Actor bistable between the std floor (entropy −6.18) and the uniform ceiling (+9.93); 16/23 shaped
  runs flip; every 0.00 checkpoint in a "good" run coincides with a flip or a late collapse (dH s103
  dies at 2.4M, dR2D s81 at 2.8M). Likely driver: target clamped, baseline value not → advantages
  non-positive once the critic reaches the clamp (`dreamer.py:502-505`).
- Selection: BEST = max of a bistable 30-checkpoint sequence; the in-job "×3 confirmations" are the
  same 15 ICs and identical in 17/17 runs. The hold re-score (sel 0.76 → hold 0.71, rnd 0.60) shows the
  selected checkpoints are real policies — a transient policy harvested from an unstable process.
- Sparse: 0 picks in ~7,700 online episodes per seed — an exploration null, not a learning failure.
- World model itself (KL, representation, cont head) looks healthy.
- Under A15 the non-standard pieces (return clamp, replay-critic loss `repval 0.3`) are deviations.
  Launched: **standard arm** (`env.return_clamp=0 loss_scales.repval=0`), dH/dDP × seeds 120–121, old
  world, dense. If standard DreamerV3 learns: it becomes the WM arm. If only the deviated recipe learns:
  the paper reports DreamerV3-standard as a negative and the variant with its instability disclosed.
- Cheap protocol fix regardless of recipe: re-score the LAST checkpoint on hold alongside BEST for every
  run and report both (the gap is the honest cost of instability). To be queued.

## 3. dv3 — UNHEALTHY (critic runaway), diagnostics in flight
- value_mean 415–515 vs max return 100 in every dense run; never past ~62k gradient updates; demo
  terminal 48 sim steps late in every pre-08-28 set; horizon 1200 where r2dreamer needed 400.
- Queued: baseline (ManiSkill-parity ×100) ×3, clamp-only ×3, clamp+ratio+horizon ×3, **standard
  DreamerV3** (reward scale 1, symlog/two-hot, no clamp) ×3, touch-the-goal-can ± clamp ×4. Under A15
  the standard arm is the one that can enter the matrix; the clamp arms are diagnostics.

## 4. DP — HEALTHY
Stable across seeds/worlds (rnd 0.5–0.6, hold ~0.9); no training pathology known; the hold ceiling
(uid 234) is a readout property, handled by reporting rnd.

## 5. What "behaving appropriately" will mean, per learner, before any release of held jobs
RLPD: bounded critic, final ≈ selected. r2dreamer: a recipe (preferably standard) with no
floor/ceiling flips and last ≈ best on hold. dv3: sustained training success with a bounded value
function under the standard recipe, or a documented negative. Only then does the {dH, dDP} × {succ,
+own fails} matrix resume, re-run under the healthy recipes.
