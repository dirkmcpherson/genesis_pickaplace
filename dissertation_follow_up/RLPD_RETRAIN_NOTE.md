# Note: retraining {RLPD} with a fairer configuration

Written 2026-09-17. Not started; needs the user's go-ahead. The pixel {RLPD} results in the current study come from a
configuration that departs from the published recipe in four ways, all of which plausibly hurt it. Any dissertation
chapter that reports {RLPD} at 0.00 on human demonstrations should either fix these first or disclose them.

## 1. What the current runs did

Our {RLPD} is an in-house reimplementation on Stable-Baselines3 2.8 SAC following Ball et al. 2023
(arXiv 2302.02948); the authors' code is `github.com/ikostrikov/rlpd` (JAX) and is used only as a reference.
Code: `baselines/rl/rlpd_sac.py` (algorithm), `baselines/rl/train_rlpd.py` (trainer),
`baselines/rl/rlpd_pixel.py` (the DrQ-v2 pixel path, added 2026-09-13).

| departure | ours | published / reference | why it likely matters |
|---|---|---|---|
| **frame stack** | none: one 64×64 top + wrist frame plus 8 proprioceptive values | pixel agents in this family stack frames | velocity is unobservable from one frame; the policy cannot tell a rising can from a falling one |
| **LayerNorm in the critic ensemble** | one shared set of scale/shift parameters across all 10 critics | per-member parameters | shared parameters couple the ensemble, weakening the pessimism that high update ratios rely on (found in the 2026-08-14 audit) |
| **updates per decision** | 10 | 20 for the paper's state-based results | fewer critic updates per environment step slows value propagation |
| **critic depth** | 2 layers | 3 layers for the paper's sparse tasks | capacity, minor next to the others |

Two further differences are properties of our task, not defects, and cannot be "fixed" without changing the study:
terminal-only reward (+10 once at `home`, where Adroit pays +1 on every solved step) and a 1200-decision horizon
(Adroit is 100–200 steps). They are the likeliest reason our ignition is slower than the paper's ~10k steps.

## 2. The evidence that a retrain is worth it

From the training records of the 17 human/machine seeds (`ep_rew_mean` / 10 = the rolling-100-episode `home` rate,
since `home` is the only reward):

- **Every seed finds `home` early**, between 13k and 27k of its 250k decisions, then loses it.
- **By 250k, all 9 human seeds are at or below 0.05**; 4 of 8 machine seeds hold 0.10–0.79.
- Scored cells: human 0.00 on every seed, machine 0.125 (deterministic) or 0.158 (sampled).
- The r2dreamer-teacher dataset, whose tapes are short and 94 % successful, reaches `home` by 4–6k decisions and
  scores 0.27 — so the learner is not simply broken; it is losing a skill it has already acquired.

Forgetting an acquired skill is exactly the failure mode the missing ingredients (frame stack for observability,
per-member LayerNorm and a higher update ratio for value stability) are meant to prevent.

## 3. Proposed pilot before any full rerun

Purpose: find which change matters, at the smallest cost, before spending seeds on all four datasets.

- **Arm:** machine demonstrations (the only human/machine cell that scores today, so a change is visible in both
  directions), 3 seeds per variant, 250k decisions, everything else identical to the current recipe.
- **Variants:** (a) baseline repeat, (b) + frame stack of 3, (c) + per-member LayerNorm, (d) + updates per decision 20,
  (e) all three together. Five variants × 3 seeds = 15 runs.
- **Read:** rolling `home` rate at 250k, plus whether the early peak is retained rather than lost, and the scored
  rnd30 cell at the 250k checkpoint. Register the prediction first: (b) and (e) retain the skill; (c) and (d) help
  less on their own.
- **Cost:** 14–21 GPU-hours per seed measured on this cluster (L40S ~14 h, A100 ~20 h), so about 250 GPU-hours, which
  is roughly a day of wall clock at the 20-GPU ceiling.

## 4. If the pilot succeeds

A full rerun is 8 seeds × 4 datasets = 32 runs, about 500 GPU-hours, two to three days of wall clock with nothing else
queued. Keep the old runs: they become the ablation showing what the missing ingredients cost.

Do not mix old and new {RLPD} numbers in one cell. If only part of the grid is rerun, report the new configuration as
its own condition.

## 5. Related defect already fixed in analysis, not in the launcher

The 13 (ag) {RLPD} jobs were submitted with the world models' `STEPS=2000000`, ran to the 30 h wall clock
(400–592k decisions), and were scored at their saved 250k checkpoint. The analysis cuts every {RLPD} record at 250k.
**Before any new {RLPD} batch, fix the launcher so `--steps` is the budget it claims**, or the same overrun repeats.

## 6. Pointers

- Current results and caveats: `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md` §3–§4.
- Implementation audit against the paper and reference code: `paper/rlpd_audit_2026-08-14.md`.
- Where our setting sits against the literature: `paper/rlpd_literature_comparison_2026-08-13.md`.
- Positive control and swap test (our trainer and the authors' JAX code fail the same ManiSkill control identically):
  `paper/swap_test_reference_rlpd_2026-08-15.md`, `paper/RESULTS_MATRIX_2026-08-15.md`.
- The derived `home` training curve for every {RLPD} seed:
  `paper/figures/px_phase_2026-09-14/fig_learning_curves_rlpd_home_derived.png`.
