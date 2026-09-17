# State of the `nested_sparse10` PIXEL study — 4 datasets × 4 learners (2026-09-16, 22:50)

This is the start-here page for the pixel human-vs-machine study as it stands tonight. Results so far are in
`paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md`. The regenerable tables and figures are in
`paper/figures/px_phase_2026-09-14/`. How to pull data and replot is in
`HRI_results/HOWTO_pull_and_plot_pixel_results_2026-09-15.md`. Every run is `nested_sparse10` (terminal `home` pays
+10), in the w3 world `gc_kp4_riser3_shelf6`, with `tip_guard not_in_hand`. Observation: top and wrist 64×64 RGB plus
8-dim proprioception, with no object or goal pose. Target: 8 seeds per cell.

## 1. The design

**Datasets.** Each has about 72 tapes, one per start geometry.

| short name | what it is | where | notes |
|---|---|---|---|
| human | every teleoperated attempt, 74 tapes, 13 reach `home` | `$W/demos_state_full/dHfull_all_rns10h_img` | DP uses 72 (two one-decision tapes dropped) |
| machine | Diffusion-Policy teacher (trained on pruned human), first attempt per start, 72 tapes, 14 `home` | `$W/demos_state_full/dDPfull_first_rns10h_img` | DP uses 70 |
| planner72 | motion-planner demonstrations, 72 non-empty tapes, 17 872 decisions, 67 geometries | `$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals` | curated replacement corpus (two empty starts refilled); the OLD 70-tape planner cohort in `planner_px_2026-09-15` is a different dataset and is excluded |
| r2teacher | first attempts of one human-trained pixel r2dreamer policy (seed 15), 72 tapes, 2 278 decisions | `$LAB/r2teacher_px_2026-09-15/data/training_v1` | tapes are ~30 decisions long; teacher n = 1 |

**Learners and statistics of record.**

| learner | budget | statistic per seed |
|---|---|---|
| {DreamerV3 losses} in the r2dreamer chassis | 2M online steps (pilots 1M) | mean rnd30 MODE `home` over the 0.5M and 1M milestone cells |
| {r2dreamer} contrastive loss | same | same |
| {RLPD}, DrQ-style shared encoder | 250k decisions | rnd30 MODE `home` of the 250k checkpoint |
| {Diffusion Policy}, lerobot, random 56 crop | 100k updates | rnd30 SAMPLED `home` of the final checkpoint |

Planner72 and r2teacher differ from human and machine in more than source: controller access, selection, tape length
and success balance all differ. They support "what these learners do with this data", not a clean source-only
causal claim.

## 2. Seeds per cell tonight

Counts are seeds **with a statistic** / trained or running / queued. Target 8.

| learner | human | machine | planner72 | r2teacher |
|---|---|---|---|---|
| {DreamerV3 losses} | **16** / 16 / 0 | **16** / 16 / 0 | 0 / 7 / 1 | 0 / 1 / 7 |
| {r2dreamer} | **16** / 16 / 0 | **16** / 16 / 0 | 0 / 7 / 1 | 0 / 2 / 6 |
| {RLPD} | 2 / 9 / 0 (7 scoring) | 2 / 8 / 0 (6 scoring) | 2 / 8 / 0 | 0 / 1 / 7 |
| {Diffusion Policy} | **4** / 4 / 4 | **4** / 4 / 4 | 0 / 2 / 6 | 0 / 0 / 8 |

The two new datasets have no statistic yet. That is because their world-model milestone cells were only submitted
tonight, not because the runs failed.

### Update 2026-09-17 02:30 (overnight batch 1)

Regenerated from `paper/figures/px_phase_2026-09-14/px_results_4x4.md`. Cells show seeds with the statistic /
design n and the mean rnd30 `home`. {RLPD} and the world models use MODE actions; {Diffusion Policy} uses SAMPLE.

| learner | human | machine | planner72 | r2teacher |
|---|---|---|---|---|
| {DreamerV3 losses} | 16/16, 0.306 | 16/16, 0.303 | **6/8, 0.000** | 1/8, 0.650 |
| {r2dreamer} | 16/16, 0.588 | 16/16, 0.530 | 6/8, 0.322 | 2/8, 0.492 |
| {RLPD} | **8/8, 0.000** | **8/8, 0.125** | 2/8, 0.000 | 0/8 |
| {Diffusion Policy} | 4/8, 0.025 | 4/8, 0.017 | 1/8, 0.567 | 0/8 |

- **{RLPD} registered readout:** human 0.000 v machine 0.125, exact p 0.20 (8 v 8). With human s8 it is p 0.08
  (9 v 8). Human 0/9 seeds reach `home`; machine 3/8.
- **Planner demonstrations do not work for the world models (interim, 6 seeds).** {DreamerV3 losses} reach `home` on
  0 of 6 planner seeds (picked 0.27). {r2dreamer} reaches 0.322 v human 0.588 (p 0.0001). By contrast, the first
  planner {Diffusion Policy} seed scores 0.567, far above DP on human/machine data (≤ 0.03).
- **No failures overnight.** Scoring submitted: 26 more planner72 milestone cells. The hold/release fix was needed
  again (26 evaluations pending beside 35 idle nodes). Disk 284 GB.

## 3. What happened today (09-16)

1. **Cluster incident.** A home-directory cache hit its quota and a GPU node drained. The other agent restarted the
   affected planner72 and r2teacher jobs (`RESUBMISSIONS_2026-09-16.jsonl` in each campaign root).
2. **Two "failed" world-model seeds finished training.** planner72 {DreamerV3 losses} s0 (3738545) and r2teacher
   {r2dreamer} s2 (3738586) exited 11. The segfault came after all four milestones were written, when an in-job
   evaluation hit the home-cache quota. They need scoring only and are included in the counts above.
3. **{RLPD} human/machine budget leak.** The 13 (ag) seeds were submitted with the world-model `STEPS=2000000`. Their
   sidecars record 2M decisions, and they ran to the 30 h wall clock (400k–592k decisions) without final cells.
   Their `rlpd_250000_steps.zip` checkpoints are the registered budget point, and the learning rate is constant, so
   nothing before 250k differs. Evaluation jobs 3773759–3773771 are running. Records past 250k are cut from every
   analysis.
4. **Jobs queued tonight.**
   - {Diffusion Policy} human/machine seeds 4–7: 3773972–3773979, moved to `gpu,preempt` / QOS preempt. The GPU
     partition's own QOS counts preempt jobs against the 10-GPU normal limit.
   - World-model milestone cells: 40 jobs (planner72) + 15 (r2teacher), `SWEEP_MODE=cpu64pre`, same sweep and
     64-core node class as human/machine. These were submitted with a per-pass limit of 40 instead of the campaign
     script's 4.
5. **Scheduler trap, recurring.** CPU evaluation jobs sit "Priority"-pending beside idle CPU nodes whenever more than
   20 of the user's higher-priority GPU jobs are pending (Slurm `bf_max_job_user=20`). The fix is to hold the
   GPU-limit-blocked jobs for about 30 s, then release them. They cannot start during that window, so they lose
   nothing.

## 4. Results so far (human v machine; details in the summary)

| learner | n | human | machine | p |
|---|---|---|---|---|
| {DreamerV3 losses} | 16 v 16 | 0.306 | 0.303 | 0.97 |
| {r2dreamer} | 16 v 16 | 0.588 | 0.530 | 0.09 |
| {Diffusion Policy} | 4 v 4 | 3/120 `home` | 2/120 `home` | 1.00 |
| {RLPD} | scoring tonight | | | |

**{RLPD} training record (post hoc, 9 v 8).** At 250k, picked is 0.44 v 0.78 (p < 0.001). The `home` rate
(`ep_rew_mean`/10) peaks early on every seed. It then collapses to ≤ 0.05 on all 9 human seeds while 4 of 8 machine
seeds keep 0.10–0.79. The user's note: RLPD should have used a frame stack. It is too late to change for this study,
so disclose it as a limitation.

**Observation control.** The same {DreamerV3 losses} recipe on 17-dim privileged state reached `home` 0 times in 2M
steps (PHASE_PLAN (ai)). Pixels ignite 64/64 world-model seeds.

## 5. Expected timeline

About 420 GPU-hours remain at a 20-GPU ceiling. That puts all training around Friday 09-18 morning, with no
preemption assumed. The r2teacher {RLPD} seeds start last and take 14–21 h each, so they set the finish.

## 6. Overnight monitoring (this session)

Every ~3 h, in batches:
1. Re-run the two milestone sweeps.
2. Apply the hold/release fix if evaluations stall.
3. Pull metrics.
4. Regenerate the 4 × 4 tables and figures.
5. Commit when new cells change a number.

No training job is resubmitted without the evidence of why it ended.
