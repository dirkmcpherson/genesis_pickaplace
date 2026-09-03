# Adversarial design critique — the final round robin (human vs model demonstrations)

Reviewer stance: NeurIPS/CoRL reviewer 2 + statistician. Scope: the SETUP the team is about to
re-run (repeat-4 clock for all learners, re-harvested model demos), not the numbers already
collected. Everything below was checked in the sparse checkout (paths are absolute, lines are
current HEAD 4c025db); nothing was edited. Prior audits (paper/AUDIT_design_2026-08-22.md,
paper/AUDIT_impl_2026-08-22.md) are taken as read; I do not repeat their findings except where
the re-run plan fails to absorb them.

Bottom line: the repeat-4 standardization fixes ONE of roughly ten axes on which "demo source"
is confounded, and as proposed it introduces three unvalidated positive controls (DP at repeat
4, RLPD at repeat 4, r2dreamer at a 300-decision horizon) on the critical path. If the block is
launched as "same scripts, repeat 4, new dDP harvest", the paper will still have no cell in
which demonstration source is the only variable, and no column that can be read across
learners. The fix is cheap relative to the block: build the three demo sets ONCE, through the
learner's own MDP, with one harvester, one truncation rule, one terminal labelling, one
preprocessing; evaluate every checkpoint of every learner with one harness; pre-register one
primary statistic per learner; pick seed counts from the power table below, not from habit.

---

## 1. What the paper can and cannot claim under the design (as planned)

**Can claim (if the minimum design in §3 is run):**
- C1. *Within-learner, matched-N (episodes), same-IC, same-clock, success-only contrast*: "For
  learner L on this task in this simulator, training on the human set vs model set A vs model
  set B produced per-seed success rates x/y/z (demo-IC) and x'/y'/z' (random-IC), n seeds,
  effect size and CI." One claim per learner; three learners; no pooling across learners.
- C2. *Descriptive mechanism*: measured tape properties (idle density, representability at the
  learner's clock, over-cap rate, first-lift time, fail geometry) differ between sources, and
  where an intervention arm exists (fails-in vs fails-out; terminal-guard on/off) the effect of
  that property is measured (RLPD only, as registered in PAPER_NOTES N1).
- C3. *Self-distillation descriptive*: DP→DP and r2dreamer→r2dreamer rows are gen-0→gen-1
  self-distillation; the cross rows (DP demos→r2dreamer, r2dreamer demos→DP) are
  cross-distillation. That is a legitimate 2×2 of {teacher family}×{student family} — but only
  if stated as such, not as "human vs model".
- C4. A null in C1 is claimable only with the power statement ("no effect larger than Δ
  detectable at n seeds"), never as "no effect".

**Cannot claim (as designed), even after the re-run:**
- N1. "Model demonstrations are better/worse than human demonstrations" in general. Two
  teachers, both trained on the same human corpus (dDP's DP teacher on dHpruned; dR2D's
  r2dreamer champion pre-filled with genesis_pick_pruned_delta25 = the same human set,
  METHODOLOGY §6.4), one task, one sim, N=66.
- N2. H4 in either direction as a cross-learner quantitative statement. DP/RLPD see a 17-dim
  state incl. ground-truth can pose; r2dreamer/dv3 see 2×64×64 pixels
  (paper/ROUND_ROBIN_RUNNING_2026-08-19.md:28-30). Budgets differ ~30× in sim steps (RLPD
  100k, dv3 300k, r2dreamer 3M). Eval horizons/selection rules differ (§2 E1-E4). "BC inherits
  source differences, WMs route around them" (or its inverse) is not testable in this matrix;
  it is an interaction that requires a common measurement scale. At most: "the SIGN of the
  within-learner contrast differs across learners", with the scale caveat.
- N3. Anything about RLPD-as-a-method: no passing positive control exists
  (RESULTS_MATRIX_2026-08-15.md "Positive controls": MS_RLPD-ctl 0/3; reference fails
  identically on sparse; reference learns dense, ours does not). RLPD rows are "SAC+50% demo
  batches as implemented here".
- N4. "Dense reward collapses the r2dreamer lottery" at matched checkpoint coverage (AUDIT_design
  issue 4) unless the re-run archives the same K checkpoints for every arm.
- N5. Any claim resting on dv3 (two 6-episode transients, no archived checkpoint).
- N6. "Matched N" as matched DATA: N=66 episodes is 9k transitions for dR2D vs 40-83k for dH
  (AUDIT_design issue 1). Report frames; the unit mismatch is intrinsic (humans are slow) and is
  disclosure, not a fix, unless a frame-matched control is run.

---

## 2. Confound register (ranked by damage to the central within-learner source claim)

Format: mechanism → evidence → fixable-by-design (how) / disclosure-only.

### R1. The three "dH" sets are three different datasets, and the three "dR2D" sets are two
- RLPD dH = `baselines/episodes_pick_phase_all` (91 eps incl. 25 no-pick; unpruned) —
  cluster/sbatch_rlpd.sh:107. DP dH = `episodes_pick_phase_dppruned` (66, leading-idle pruned) —
  cluster/sbatch_dp.sh:187. WM dH = `genesis_pick_pruned_delta25` / `genesis_pick_msr_delta25_r4`
  (67, pruned, success-only, from episodes_pick_pruned_img) — cluster/sbatch_r2dreamer.sh:200,
  dreamerv3-torch-genesis/sbatch_genesis_multi.sh:96 + ROUND_ROBIN_RUNNING:17-18. So "dH" carries
  fails for RLPD and not for WMs, is pruned for DP/WM and not for RLPD.
- dR2D for RLPD/DP = 66 state tapes (`episodes_champion_pick`); for WMs = 52 pixel tapes from a
  SEPARATE re-harvest with `--images` (harvest_champion_demos.py:55-60; ROUND_ROBIN_2026-08-20
  §4.1 "same teacher, same ICs, different render pass and count"). 52≠66 from a deterministic
  teacher on the same ICs means the rig changed outcomes — the tapes are different episodes.
- **Fix (design):** ONE harvest per teacher WITH images; derive the state-only npz and the
  lerobot set from the same episodes. ONE human set (decide pruned vs unpruned once — §3) from
  which every learner's format is derived. Fingerprint (sha) the per-episode list and assert it
  in every sbatch.

### R2. Success/fail composition is unmatched across sources within the RL/WM rows
- dH 25-33% no-pick tapes; dDP 30 fails = 51% of transitions; dR2D 0 fails by construction
  (harvest_champion_demos.py:412-413 returns None for non-picked; `--full-outdir` also stores only
  successes). WM dDP = `genesis_m1all_delta25` (93 incl. the 30 fails) vs WM dH success-only.
- **Fix (design):** success-only for every learner and every source as the PRIMARY matrix (the
  BC rule already is). Fails become a separately registered factor (two RLPD arms: dDP+fails,
  dR2D+DPfails, both with the terminal guard below). Do not let the primary contrast carry it.

### R3. Demo tapes violate the training MDP: no terminal guard, harvester env ≠ training env
- Encoder emits fail tapes whole with done=False to the last frame
  (baselines/rl/train_sacfd_full.py:273-292, 295-348; relabel_full :43-92); the env terminates
  on tip (full_env.py:530-546). WM converter does the same: no-pick tapes "is_terminal all False
  so the value head BOOTSTRAPS at the cut" (convert_genesis_demos_repeat.py:198-205) — for a
  tape that ends tipped, the env would have terminated with value 0.
- The dDP harvester rolls the DP teacher in `GenesisCanEnv` (harvest_ai_demos.py:296 —
  `env = GenesisCanEnv(...)`), which has NO tip rule and NO pick termination; so DP fails run to
  the 1200 cap after tipping. The dR2D harvester uses r2dreamer's `GenesisPick` wrapper around `FullTaskEnv(scope=pick)`
  (harvest_champion_demos.py:248-251), which terminates on tip. The "DP fails are off-manifold 1200-step chains" finding is
  partly a harvester artifact.
- **Fix (design):** (a) harvest every model set through `FullTaskEnv(scope=pick, tip rule on,
  repeat 4)` so fails end where the env ends; (b) add `--demo-terminal-guard` to every encoder
  /converter: tip → done=True, r=0; pick → done=True; tape end without terminal → truncation
  (bootstrap) — and make the guard the default, with the old behaviour only under an explicit
  flag. The repeat-4 re-harvest is the moment to do it; retraining on guarded tapes costs nothing
  extra. (Also fix F5: make_dDPsucc tiptrunc must set done=True at the cut.)

### R4. Time-base / representability — the axis the team is fixing, but incompletely
- Only repeat-4-native tapes are exactly representable by repeat-4 learners (dR2D 1.0 exact;
  human 0.54; dDP 0.001 — PAPER_PLAN Decision Log 2026-08-23; analysis/characterize_demo_sets.py
  stride table). The plan re-harvests dDP from a repeat-4 (hold-4) teacher → exact. dH remains
  ~half-exact (intrinsic to 30 Hz teleop) and over-cap 4.4% vs dDP 7.9% vs dR2D 0.1%: the cap
  0.025 is calibrated from HUMAN p99 (METHODOLOGY §7.2) and clips DP-teacher tapes 2× more; the
  champion is 0% by construction because it was trained under the cap.
- **Fix (design):** record every set AS EXECUTED BY THE TARGET MDP: dH = open-loop replay of the
  human command stream through `FullTaskEnv(delta_joint, cap 0.025, repeat 4)` and record the
  executed tape (the `episodes_delta_rerecord` pattern, already built at stride 1:
  paper/demo_set_census_devbox_2026-08-22.md "dHrerec"); dDP = DP teacher closed-loop through
  the same env (hold-4, capped); dR2D = native. Then representability, over-cap, leash and tip
  termination are identical by construction; N = min over sources of surviving picks (expect
  dH to lose some — 54/72 at stride 1; gate it with replay_gate.py). What remains different is
  the behaviour — which is the thing under study.
- If the repeat-4 human re-record loses too many picks (<50), fall back to: WM/RL at repeat 4
  on hold-4 tapes, DP at stride 1 on the same episodes — per-learner clock, three sources on that
  learner's clock. That still makes every within-learner contrast clean; only cross-learner
  comparability (already lost, N2) is given up.

### R5. Teacher circularity, teacher competence, and self-vs-cross distillation
- Both model teachers descend from the same human corpus (N1 above). dDP teacher ≈0.67-0.80
  in-dist (PAPER_PLAN 08-02/08-09); dR2D champion 0.91-1.00. Harvest yield 19.7% (320 → 63)
  under `harvest_ai_demos --ic-mode demo` (cycles demo uids, stochastic DP) vs 1 deterministic
  rollout per uid for the champion (harvest_champion_demos.py:99-106, 359). Selection pressure
  on successes differs; fails are "first 30 encountered" for dDP (harvest_ai_demos.py:329-341),
  none for dR2D. Truncation tail: dH k+2 frames past proxy grant; dDP `PICK_TAIL=10` past the
  hardened pick (~20 past proxy; harvest_ai_demos.py:51,170); dR2D k+2 (harvest_champion_demos
  .py:415-423); WM converter adds 48 raw frames of grant slack where the tape has them
  (convert_genesis_demos_repeat.py:118-126) — model tapes don't.
- Within each learner row, exactly one model source is self-distillation (dDP_DP, dR2D_R2D).
- **Fix (design):** one harvester for both teachers (same env, same IC cycling over the same uid
  list, same action-selection mode — use `sample` for both so attempts>1 is meaningful — same
  attempts cap, same sim cap 1200, same `--verify`, same random-teacher negative control, same
  truncation = "where FullTaskEnv terminates", same MIN_KEEP). Record teacher success rate on the
  harvest ICs in the manifest and in the caption. Pre-register WHICH DP seed is the teacher (the
  median in-dist seed of the new dH_DP row, not the best). Label the matrix cells as self/cross
  distillation. Teacher competence itself is disclosure-only (you cannot equalize it without
  crippling the champion) — unless you add one arm: dR2D harvested from a weaker r2d checkpoint
  matched to the DP teacher's rate (optional, cheap, CPU).

### R6. Evaluation protocol heterogeneity (E1-E5) — within-learner OK, cross-learner fatal
- E1 Horizon: RLPD 400 sim steps (sbatch_rlpd.sh:225,232; train_rlpd.py:148); DP 1200
  (wandb_eval.py:30 default, no flag at sbatch_dp.sh:384-386); r2dreamer 400 in the final eval
  but UNSPECIFIED in selection/confirmation (sbatch_r2dreamer.sh:371-373, 414-418 omit
  `--max-steps`; :405-407 passes it); dv3 900 default (genesis_eval.py:46) vs time_limit 600.
- E2 Process isolation: RLPD one fresh process per episode; DP 30 episodes in one process; dv3
  6 episodes in one process; r2d 15 in one process.
- E3 IC sets: RLPD demo-IC = hard-coded 15 uids (sbatch_rlpd.sh:220); DP = `demo_ics(env)[:15]`
  (wandb_eval.py:269, same 15 by construction); dv3 demo-IC = `env.reset()` with NO uid → a
  random draw from success_uids per episode (envs/genesis.py:163; genesis_eval.py:145); r2d
  eval_genesis.py not in checkout. Random-IC: RLPD = first draw of rng(k), k=0..14
  (sbatch_rlpd.sh:228-233); DP = 15 draws of rng(0) (wandb_eval.py:271-272) — different points.
- E4 Action selection: RLPD deterministic; DP stochastic (now seeded, wandb_eval.py:91-93);
  r2d sampled (+1 mode); dv3 deterministic (`training=False`).
- E5 Checkpoint rule: RLPD final@100k; DP last; r2d best-of-every-latest.pt-write (~30) with
  3-seed confirmation on the SAME ICs; dv3 2-3 six-episode periodic draws of `latest.pt` at
  arbitrary wall-clock, overwritten. r2d's ignition indicator scales with coverage.
- Eval env is `GenesisCanEnv` (no tip termination, no pick termination; wandb_eval.py:112),
  training env is `FullTaskEnv` (tip terminates). Uniform across sources → disclosure; but the
  same choice must hold for all four learners' harnesses.
- **Fix (design):** ONE harness (`eval_core.run_eval` behind a per-learner policy adapter — the
  architecture already exists in wandb_eval.py), fresh process per episode for all, horizon 1200
  sim steps (= 300 decisions) for all, one JSON with {15 selection demo uids, 15 held-out demo
  uids, 30 random ICs} shared by all, action-selection per learner's convention recorded in the
  JSON, K=5 checkpoints archived at 20/40/60/80/100% of budget for EVERY learner and all scored
  with the harness; selection on the 15 selection uids, confirmation on the 15 held-out uids +
  30 random (a deterministic policy re-run on the same ICs is a null operation — PAPER_PLAN
  08-13/08-14). Assert denominators (F17). Log mode/repeat/horizon/ckpt-step/node/git.

### R7. Reward/shaping inconsistencies
- Dense ran only on dH (ROUND_ROBIN_RESULTS §7). Shaping γ: RLPD 0.998 (full_env.py:129 class
  const), dv3 0.997, r2dreamer hard-coded 0.999 vs config comment "horizon 333 # discount
  0.997" (AUDIT_impl F8 — still unverified). φ(terminal)≠0 (F9). Demo transitions keep sparse
  labels while online carry shaping (train_rlpd.py:74-79; F10: O(1) target error in Q units).
- **Fix (design):** pick ONE reward condition per learner as primary before launch (see §3: dense
  for RLPD/r2d because sparse ignition is a lottery that destroys power); verify γ per agent;
  zero φ at terminal; relabel demo transitions with the same potential (needs eef position for
  demo frames — compute by FK from recorded joint positions through the env once, offline) or
  disclose the asymmetry numerically in the caption. Apply identically to all three sources.

### R8. Hyperparameters tuned on dH
- cap 0.025 = human p99 (§7.2); γ=0.998 chosen from the human median pick frame 662
  (train_rlpd.py:99-102); r2d recipe (entropy 3e-5, horizon 333, demo_duplicate 4, reinject
  150k, time_limit 400) tuned on the pruned human set; DP 100k/batch 64/pruning tuned on dH;
  r2d time_limit 400 sim steps is SHORTER than the human median first-lift (~780-830 sim
  steps): the WM's human demos are trajectories it could never complete online. Model tapes
  (dR2D ~130, dDP 512) fit inside.
- **Fix:** R4's "record as executed" removes cap bias; time_limit→1200 for all removes the
  horizon-vs-demo-length mismatch (but needs the r2d pilot, §4). The rest is disclosure:
  "recipes were tuned on the human set; this biases toward dH, so a model-demo advantage is
  conservative, a human advantage is not."

### R9. Statistical design
- Seeds 3 (WM/DP) / 6 (RLPD). Simulated power (Fisher on ignition counts, α=.05, two-sided):
  n=6: 0.06 for 0.5-vs-0.1 ignition; n=8: 0.22; n=12: 0.43; n=16: 0.58. Per-seed pick-count
  permutation test (rates 0.4 ignited / 0.03 floor, 15 eps): n=6: 0.18; n=8: 0.33; n=10: 0.42;
  n=12: 0.52. Continuous DP rates (seed SD 0.15): n=3 detects Δ=0.34 with 0.55 power, Δ=0.2
  with 0.25; n=5: 0.88 / 0.46. **Lottery learners under sparse reward are unpowerable at any
  affordable n; the only design lever is to raise ignition (dense) so seeds yield rates, not
  coin flips.** (Scripts: scratchpad/power.py, power2.py.)
- Ignition threshold (≥3/15) discards information; 15 fixed random ICs shared by all seeds
  pooled as 90 independent trials (F16); dense-vs-sparse seeds 3-8 paired by index but analysed
  unpaired; ≥8 nominal p-values in 0.007-0.05 with no declared family; "confirmation" on already
  read data. Fix: §3 analysis plan.

### R10. Same-machine / nondeterminism
- Node classes `l40s|a100|l40|h200` (sbatch_rlpd.sh:85); replay is machine- and load-dependent
  (METHODOLOGY §2); eval sweeps run 5 concurrent Genesis worlds on 8 cores (sbatch_rlpd.sh:
  222,229); DP eval runs on the training GPU node in one process; the dR2D image re-harvest got
  52 vs 66 successes from a deterministic teacher. Fix: record node type per run and per eval
  process; run all evals CPU-only at ≤1 world per 2 cores; if feasible constrain the block to one
  node class; treat node as a blocking factor in the analysis. Residual = disclosure.

### R11. Observation modality (state w/ ground-truth can pose vs pixels) — disclosure-only
- Not fixable without a pixel DP/RLPD (honest-vision DP was wave-2 and never ran). State this in
  the first sentence of the results section; it alone forbids any cross-learner magnitude.

### R12. Unrecorded/silent knobs that will collide under the new clock
- `--train-max-steps 900` is not in the sidecar or REG_KNOBS (train_rlpd.py:109; sbatch_rlpd.sh:
  167-170; F7); under repeat 4, "100k steps" is 100k DECISIONS for SB3 (= 400k sim steps) but
  `steps: 3e5` for dv3 counts sim steps (`time_limit //= action_repeat`); r2dreamer "3M env
  steps" unit unspecified in the checkout. Pre-register the budget UNIT per learner (decisions
  vs sim steps) and put horizon/repeat/eval-horizon into every registry key.

---

## 3. Minimum valid design for the final round robin

### 3.1 Matrix
| learner | sources | seeds | reward (primary) | budget | secondary arms |
|---|---|---|---|---|---|
| DP | dH, dDP, dR2D (success-only, N matched) | 5 | n/a | 100k, batch 64 | — |
| RLPD | dH, dDP, dR2D (success-only) | 10 | dense (potential, γ=0.998, demos relabelled) | 100k decisions (150k if affordable) | dDP+fails(guarded) ×6, dR2D+DPfails(guarded) ×6 (N1 mechanism) |
| r2dreamer | dH, dDP, dR2D (success-only, pixels from the same episodes) | 6 | dense (γ verified = agent discount) | 3M (unit stated) | sparse dH ×4 only to tie to history (optional) |
| dv3 | — | — | — | — | appendix / drop (N5) |

Same seed indices across the three sources within a learner (paired blocks). Seeds are NEW
values never used before for that learner (AUDIT_rng: demo-RNG was hard-seeded pre-2fbed2a).

### 3.2 Held fixed (all learners, all sources)
- Clock: repeat 4 for RL/WM learners; DP at repeat 4 ONLY if the pilot passes (§4), else DP at
  stride 1 on the same episodes. Budget unit declared per learner.
- Episode horizon: training time_limit 1200 sim steps (300 decisions) for RL/WM (r2d pilot
  required); eval horizon 1200 sim steps for everyone.
- Action space: delta_joint cap 0.025 leash 5× (RL/WM), jact (DP) — as before; stated.
- Reward: terminal +1 on hardened pick (scale 100 for WMs), tip termination, dense potential as
  primary for RL/WM with demo relabel; sparse only as a disclosed secondary.
- Demo preprocessing: identical truncation (tape ends where FullTaskEnv terminates), identical
  terminal labelling (guard), identical idle handling (see 3.3), identical encoder.
- Eval: one harness, one IC JSON, fresh process per episode, K=5 archived checkpoints per run.
- Code: one commit hash for the whole block; no mid-block fixes (if a fix is needed, the block
  restarts for that learner).

### 3.3 Demo-set matching protocol (concrete)
1. **Human base set.** Start from the 66 IL-usable success demos (PAPER_PLAN §3). Decide pruning
   ONCE: recommended = leading-idle pruning (make_dp_pruned rule) for ALL learners, because (a) DP
   needs it, (b) the WMs already use it, (c) it removes the largest idle-density asymmetry vs
   model tapes; the unpruned human set becomes a disclosed secondary DP/RLPD arm ("preprocessing
   control"), not part of the primary matrix.
2. **Record-as-executed.** Replay each human command stream through `FullTaskEnv(scope=pick,
   delta_joint, cap 0.025, repeat 4, tip rule)` open-loop (hold-4 windows), record states/actions/
   images per decision; keep only episodes that re-earn the hardened pick (replay_gate). This is
   dH. Report how many of 66 survive.
3. **Teachers.** dDP teacher = the median-in-dist seed of the new dH_DP row (pre-registered rule);
   dR2D teacher = CHAMPION_1576820.pt (fixed). Both harvested by ONE harvester: same env as (2),
   `--mode sample`, attempts ≤3 per uid, IC list = the same success uids cycled, sim cap 1200,
   `--verify` on, random-teacher negative control on, MIN_KEEP_FRAMES same, ALL fails kept to a
   separate dir (for the secondary arms), tapes end at env termination. Manifest records teacher
   success rate on harvest ICs.
4. **N.** N = min(surviving dH, kept dDP successes, kept dR2D successes), capped at 66; subsample
   the larger sets uniformly at random (fixed seed) STRATIFIED by IC uid so the three sets cover
   the same uids with the same multiplicity. Publish the per-set census
   (analysis/characterize_demo_sets.py) — counts, frames, fails, first-lift, representability at
   stride 4, over-cap, idle fraction, OOD — as a pre-launch table.
5. **Derivation.** From each recorded set produce (a) npz state tapes for RLPD, (b) lerobot for DP,
   (c) dreamer-format stride-4 pixel demos for r2d/dv3 — from the SAME episodes; sha the episode
   list; every sbatch asserts it.
6. **Fails arms (secondary, RLPD only).** dDP+fails: the N successes + the kept DP fails (tip-
   terminated, guard on), fail share capped at the human set's (~30% of episodes); dR2D+DPfails:
   same fails injected into dR2D. Optionally dH+fails (the 25 no-pick, re-recorded) to complete
   the factor.

### 3.4 Eval protocol (pre-registered)
- Harness: `eval_core.run_eval` via `wandb_eval.py`-style adapter for sac/dp and a thin adapter
  for r2d/dv3 checkpoints (same GenesisCanEnv, same stage predicates). Fresh process per
  episode, CPU, ≤1 world per 2 cores. Horizon 1200 sim steps; repeat per checkpoint sidecar.
- ICs: `eval_ics.json` = {sel: 15 demo uids, hold: 15 OTHER success uids, rnd: 30 support ICs
  drawn once from rng(0)}. Same file for every learner and source.
- Checkpoints: K=5 at 20/40/60/80/100% of budget, archived. Selection eval on `sel` (15 eps,
  learner's default action mode). Reported headline per seed = CONFIRMATION of the selected
  checkpoint on `hold` (15) + `rnd` (30). Also report final-checkpoint on `hold`+`rnd`, and
  time-to-first-training-pick (coverage-independent).
- Action selection: RLPD deterministic; DP sampled (seeded); r2d sampled (+mode reported);
  recorded in the JSON. Denominators asserted. Node type recorded.

### 3.5 Hypotheses, rewritten falsifiable (pre-register before launch; one primary statistic)
- **P-DP.** Mean per-seed demo-IC (hold) success differs between dH and each model set (two
  contrasts, Holm). Prediction (from prior data, to be CONFIRMED on new seeds): dR2D > dDP > dH
  on hold; dR2D > {dDP, dH} on rnd. Falsified if the paired difference CI covers 0.
- **P-RLPD.** Mean per-seed hold success (continuous count/15, not ignition) differs by source
  under dense reward. Prediction: dR2D ≥ dH ≥ dDP with the success-only sets; the dDP deficit
  DISAPPEARS once fails are removed and the guard is on (N1). Falsified if dDP(success-only,
  guarded) remains below dH by ≥0.15 with CI excluding 0.
- **P-R2D.** Same statistic under dense reward. Prediction (registered by the team in
  ROUND_ROBIN_RESULTS §5): model-demo pixel sets ignite less; dense-on-dR2D tests basin entry.
  Falsifier: dR2D ≥ dH.
- **P-MECH (secondary).** Adding DP fails to dR2D lowers RLPD success by ≥0.15 (sufficiency);
  removing them from dDP raises it to within 0.1 of dH (necessity).
- **H4' (exploratory only).** The sign of (model − human) differs between DP and r2dreamer.
  Test as a source×learner interaction on standardized per-seed rates; label exploratory because
  of R11/N2.

### 3.6 Analysis plan
- Unit = seed. Primary statistic = per-seed success count on `hold` (15) and on `rnd` (30).
- Test: paired (same seed index) permutation test on seed means within learner; hierarchical
  Beta-Binomial P(A>B) as the reported posterior (analysis/bayes_source_effect.py already does
  this); 95% bootstrap CI on the mean difference = the effect size. Ignition counts reported as
  secondary descriptive only.
- Families: per learner, 2 primary contrasts (model vs human) → Holm; all other p-values BH at
  q=0.1 and labelled exploratory. No pooling of episodes across seeds as independent trials
  (shared IC sets).
- Missing cells (crashed eval) reported as missing, never as 0.
- Report per cell: n seeds; per-seed rates (sel/hold/rnd); mean ± seed-SD; CI; posterior;
  selected checkpoint step per seed; final-checkpoint rate; time-to-first-pick; demo-set sha,
  N episodes/frames/rewarded frames; horizon/repeat/action mode; node type; video grid.

### 3.7 Compute (cluster GPU-hours; timings from the last block: RLPD 2h/100k at repeat 1, DP
1.7h+eval, r2d 14-18h/3M, dv3 5h/300k)
| item | jobs | h each | GPU-h |
|---|---|---|---|
| Pilots (§4): DP-r4 dH ×2; RLPD-r4 dH ×3; r2d@1200-horizon dense dH ×2 | 7 | 3 / 3 / 16 | ~47 |
| DP 3×5 | 15 | 3 (incl. fresh-process eval) | 45 |
| RLPD 3×10 @ repeat 4, 100k decisions (sim ×4; est. 3h) | 30 | 3 | 90 |
| RLPD secondary 2×6 | 12 | 3 | 36 |
| r2dreamer 3×6 dense | 18 | 16 | 288 |
| dv3 (if kept) 3×4 | 12 | 5 | 60 |
| Harvests + re-records (CPU) | — | — | ~0 GPU; ~1-2 CPU-days |
| **Total** | 94 | | **~565 (505 without dv3)** |
At the ~13-GPU per-user cap: ≈2 days wall for everything but r2dreamer; r2dreamer 18 jobs × 16h
= two rounds ≈ 1.5 days → **≈2.5-3 days wall**, vs the last block's 26h. Budget cuts, in order:
(1) drop dv3 (−60); (2) RLPD 150k→100k (already assumed); (3) secondary arms 6→4 seeds (−12);
(4) r2d 6→5 seeds (−48; do NOT go below 5 on a lottery learner); (5) drop the sparse tie-back
runs. Do not cut the pilots — they are what make the block interpretable.

---

## 4. Before training starts (ordered) vs disclosure-only

**Must happen before any GPU is touched (in this order):**
1. Pull every artifact off the cluster before 08-24 (BEST_selected.pt + ckpt_scores.tsv for
   s50-53, dv3 latest.pt for rr_dH_s2/rr_dR2D_s1, all .out, RUN_REGISTRY.jsonl). Not design, but
   without it the paper cannot re-evaluate its own headline cells.
2. Commit a PRE-REGISTRATION file (matrix, seeds, budget units, reward condition per learner,
   checkpoint rule, eval harness + IC JSON, hypotheses §3.5, analysis §3.6, stopping rules) and
   fold the 08-15..08-23 decisions into PAPER_PLAN's decision log (it claims to be the single
   source of truth and currently is not).
3. Add the terminal guard to every demo encoder/converter (train_sacfd_full delta encoders,
   convert_genesis_demos_repeat.py, to_dreamer_demos) — default on; fix make_dDPsucc tiptrunc
   done=True. Unit-test: a tipped tape ends done=True r=0; a pick ends done=True; a timeout ends
   done=False.
4. Unify the harvester (one script, FullTaskEnv, repeat 4, tip rule, sample mode, verify,
   negctl, all-fails dir, manifest with teacher success rate) and run the dH hold-4 re-record +
   both harvests locally (CPU) while the cluster is down. Gate each set with replay_gate.py.
5. Build the three sets per §3.3 from the SAME episodes for all formats; run
   characterize_demo_sets.py on the final sets; publish the census table; sha the episode lists;
   wire the sha into every sbatch gate.
6. Positive-control pilots at the new clock (local GPU): DP at repeat 4 on dH ×2 seeds — pass
   bar ≥0.5 in-dist (prior 0.62 mean); RLPD at repeat 4 on dH ×3 seeds dense — pass bar ≥ the
   08-19 dense verdict (pooled ≥0.16, ≥1 seed ≥4/15); r2dreamer dense dH at time_limit 1200 ×2
   — pass bar ≥1 seed ignites by 1M with best-ckpt ≥0.5. Any failure → that learner keeps its
   validated clock/horizon and the pre-registration is amended BEFORE launch (fallback R4).
7. One eval harness + IC JSON + K=5 checkpoint archiving for all four learners; smoke each
   adapter on a known checkpoint (DP positive control 0.67; r2d champion 0.91/1.00; RLPD
   dH_s0@150k 0.40) and require the known numbers back. Fix `--max-steps` omissions in
   sbatch_r2dreamer.sh:371-373,414-418 and the DP fresh-process eval.
8. Shaping: verify r2d discount vs 0.999; zero φ at terminal; implement demo relabel with the
   potential (FK of recorded joints) or write the numeric caveat now; apply to all sources.
9. Registry: add train/eval horizon, repeat, budget unit, demo sha to REG_KNOBS; register at job
   START (TOCTOU, F23); seed ranges disjoint from history.
10. Node policy: record node class; CPU-only evals at ≤1 world/2 cores; if the scheduler allows,
    one node class for the block.

**Disclosure-only (write the paragraph now, don't spend GPU):**
- State vs pixel modality (R11); per-learner budgets and their units; human 30 Hz tapes are
  only ~half exactly representable at repeat 4 (intrinsic; quantified by the census);
  teacher circularity and teacher competence (R5); recipes tuned on dH (R8); eval env has no tip
  termination; F2 (demo pick terminal one frame early / weaker predicate — uniform across
  sources); same-machine history (old n=8 BC rows are local, new block is cluster); RLPD has no
  passing positive control (N3); dv3 = documented negative + anecdotal transients; single task,
  single simulator, N≈66; the dDP_RLPD 0/6 of the last round as motivation for the guard, not as
  a result about model demos.

---

## 5. Top 5 "a reviewer will reject on this"
1. **No cell varies only the demonstration source.** dH/dDP/dR2D differ simultaneously in fail
   share (33/51/0%), tape length (~800/512/130), transitions (83k/70k/9k), representability at the
   learner's clock (0.54/0.001/1.0), over-cap (4.4/7.9/0.1%), truncation tail, harvester env (tip
   rule on/off), terminal labelling, and — for the WM rows — even which human set "dH" means.
   Fix = §3.3 (one harvester, record-as-executed, success-only primary, guard on).
2. **The headline table mixes four evaluation protocols** (horizon 400/600/1200; one process vs
   fresh; fixed 15 uids vs random uid draws; final vs best-of-30 vs 6-ep periodic) and then
   reads across learners (H4 language). Fix = §3.4; otherwise forbid every cross-learner sentence.
3. **"Model demos" is two different things**: two teachers of different competence (0.7 vs 1.0),
   both trained on the human corpus, harvested by two different pipelines, one of which is
   self-distillation in every row. Fix = one harvester + registered teacher rule + self/cross
   labelling; competence = disclosure.
4. **Power and multiplicity**: n=3-6 seeds on lottery learners (Fisher power 0.06 at n=6 for
   0.5-vs-0.1 ignition), thresholded ignition as the statistic, >8 uncorrected p-values, paired
   seeds analysed unpaired, 90 "independent" episodes from 15 shared ICs, confirmation on
   already-read data. Fix = §3.1 seed counts, dense-as-primary, §3.6.
5. **Implementation validity of the RL/WM rows**: demo encoders bootstrap through states the env
   terminates at (no terminal guard), the demo pick terminal is a weaker predicate one frame
   early, RLPD has no positive control, r2d shaping γ is possibly mismatched, and the flagship WM
   contrast (dH 8/34 vs dDP 0/20) compares a pruned success-only human set with a fails-included
   model set — i.e. it is confounded by the exact mechanism the paper proposes as its finding.
   Fix = guard + unified sets + γ check; RLPD validity = disclosure (or the Adroit control if
   time appears).
