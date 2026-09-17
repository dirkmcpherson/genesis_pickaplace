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
- **Planner demonstrations slow the world models down (interim, 6 seeds).**
  - {DreamerV3 losses}: the statistic of record (0.5M + 1M cells) is 0.000 on all 6 planner seeds, with picked 0.27.
    It is late ignition, not only failure. In training, s0 and s2 reach `home` from ~1.1–1.3M counter steps and end
    with 62 % and 47 % `home` over their last 200 episodes; s0's 2M cell reads 2/30. The other planner seeds stay at
    ≤ 3.5 %. Human and machine seeds ignite at 0.2–0.5M, so the fixed 0.5M/1M window is too early here. Report the 2M
    cell and the training record beside the statistic of record for this dataset.
  - {r2dreamer}: 0.322 v human 0.588 (p 0.0001). Training-record `home` over the last 200 episodes is 70–90 % on every
    finished planner seed, so the gap is in MODE evaluation on random starts, not in training.
  - {Diffusion Policy}: the first planner seed scores 0.567, far above DP on human/machine data (≤ 0.03).
- **No failures overnight.** Scoring submitted: 26 more planner72 milestone cells. The hold/release fix was needed
  again (26 evaluations pending beside 35 idle nodes). Disk 284 GB.

### Update 2026-09-17 05:20 (overnight batch 2)

- Planner72 world models are now 7/8 seeds. {DreamerV3 losses}: statistic 0.000 on all 7. {r2dreamer}: 0.310 v human
  0.588 and machine 0.530, exact p ≤ 0.0001 both (interim).
- No other cell changed. The planner {Diffusion Policy} seed 5 training finished and its evaluation is pending.
- Queue: 27 GPU jobs running (planner72 RLPD 6, r2teacher 8, planner72 DP/world-model 4, plus evaluations). 32 are
  waiting for GPUs, including all 8 {Diffusion Policy} human/machine s4–7.
- No failures. 12 more milestone cells were submitted. Disk 273 GB.

### Update 2026-09-17 07:40 (batch 3, user request)

Chart: `paper/figures/px_phase_2026-09-14/fig_results_4x4_home.png` (and `_picked`). New cells since batch 2:
- {r2dreamer} planner72 is complete at 8/8: 0.283.
- {r2dreamer} r2teacher is at 3/8: 0.533.
- {RLPD} r2teacher s5 is the first cell: 0.033.
- {Diffusion Policy} planner72 is at 2/8: 0.567 for both s5 and s6. These are distinct models (training seeds 5 and 6,
  different weights) whose episode outcomes on the same 30 starts differ but total the same.

### 2026-09-17 ~08:30 — queued {RLPD} moved to the back (user)

The three unstarted {RLPD} r2teacher seeds (s4 3765132, s6 3765141, s7 3765144) now have `Nice=10000` (priority
717 → 1). Every other queued job starts first. The reason is the user's plan to fix RLPD before spending more GPU
time on the current recipe. Running {RLPD} seeds (planner72 s2–s7, r2teacher s0–s3) were not touched. To undo:
`scontrol update JobId=<id> Nice=0`.

### 2026-09-17 ~09:00 — CORRECTION: planner72 result depends on the checkpoint window (user caught it)

Every 2M world-model run saves and scores checkpoints at 0.5M, 1M, 1.5M and 2M. The statistic of record averages
only 0.5M and 1M, because the (af) pilot seeds trained to 1M. On planner72 data the world models ignite later, so
that window understates them. rnd30 MODE `home` mean at each checkpoint (seeds scored):

| learner | dataset | 0.5M | 1M | 1.5M | 2M |
|---|---|---|---|---|---|
| {r2dreamer} | human | 0.573 (16) | 0.602 (16) | 0.605 (13) | 0.572 (13) |
| {r2dreamer} | machine | 0.494 (16) | 0.567 (16) | 0.582 (13) | 0.556 (13) |
| {r2dreamer} | planner72 | **0.083 (8)** | 0.483 (8) | 0.548 (7) | **0.595 (7)** |
| {DreamerV3 losses} | human (cluster) | 0.250 (12) | 0.286 (12) | 0.346 (8) | 0.263 (8) |
| {DreamerV3 losses} | machine (cluster) | 0.344 (12) | 0.314 (12) | 0.279 (8) | 0.433 (8) |
| {DreamerV3 losses} | planner72 | 0.000 (8) | 0.000 (7) | 0.000 (7) | **0.090 (7)**, one seed 16/30 |

- **{r2dreamer} on planner72 catches up by 1.5–2M.** At 2M it matches human and machine (0.595 v 0.572 / 0.556).
  The earlier line "planner demonstrations slow the world models down … 0.28 v 0.59, p 0.0001" is a statement about
  *when* it learns, not *whether*. Do not quote it as a performance gap.
- **{DreamerV3 losses} on planner72 stays poor through 2M:** 0/7 seeds above 0 until 2M, then 3 of 7 seeds score
  (16, 2, 1 of 30).
- **Proposal for the 4-dataset comparison.** Report every checkpoint. Use the 2M cell (or the 1.5M + 2M mean) as the
  cross-dataset statistic over the seeds that trained to 2M. The 0.5M + 1M statistic stays the registered
  human-v-machine number, because it is the only one every human/machine seed has.

### 2026-09-17 ~09:30 — FIRST-ROUND REPORTING RULE (user): 8 seeds per condition, world models from 2M runs

- **World models:** statistic = per-seed mean rnd30 MODE `home` of the **1.5M and 2M** cells, over the 8
  lowest-numbered seeds that trained to 2M (result-blind). Human and machine: {DreamerV3 losses} s8–15, {r2dreamer}
  s3–10. Planner72 and r2teacher: s0–7. Every checkpoint (0.5/1/1.5/2M) is tabulated beside it.
- **Other learners:** {RLPD} is the 250k rnd30 MODE cell, s0–7. {Diffusion Policy} is the rnd30 SAMPLE cell, s0–7.
- **What stays registered:** the human-v-machine 16 v 16 on the 0.5M + 1M cells, reported separately.
- **Current values:** {DreamerV3 losses} human 0.304, machine 0.356, planner 0.045 (7), r2teacher 0.433 (1).
  {r2dreamer} human 0.596, machine 0.565, planner 0.571 (7), r2teacher 0.650 (2).
- **Tests:** human v machine p 0.53 ({DreamerV3 losses}) and 0.56 ({r2dreamer}). Planner v human: {DreamerV3 losses}
  p 0.006; {r2dreamer} p 0.53. Both planner comparisons are interim.
- **Outputs:** `px_results_4x4.md` (incl. the per-checkpoint table) and `fig_results_4x4_home.png` regenerate with
  this rule.

### Update 2026-09-17 13:25 (batch)

- {RLPD} planner72 is now 8/8: eval 0.00, training 0.00.
- r2teacher world models: {DreamerV3 losses} 2 seeds (eval 0.42, training 0.88); {r2dreamer} 4 seeds (eval 0.57,
  training 0.93). {Diffusion Policy} r2teacher training starts: 3 seeds, 1.00.
- Planner {Diffusion Policy} s0 and s1 started training. No failures. Disk 249 GB.

### Update 2026-09-17 16:25 (batch)

- **Planner72 is now complete for all three reward-using learners at 8/8:** {DreamerV3 losses} eval 0.04 /
  training 0.07; {r2dreamer} eval 0.55 / training 0.90; {RLPD} 0.00 on both. Only planner {Diffusion Policy}
  (3/8) is short.
- r2teacher grows: {DreamerV3 losses} 3 seeds (eval 0.48), {r2dreamer} 4 (0.57), {Diffusion Policy} 3 eval /
  5 training (1.00 on training starts).
- All 5 planner {Diffusion Policy} seeds are training; the 8 human/machine {Diffusion Policy} seeds are still
  waiting for GPUs. No failures. Disk 239 GB.

### Update 2026-09-17 19:25 (batch)

- **{Diffusion Policy} r2teacher is the first new-dataset cell at 8/8:** eval 0.60, training 1.00 — the highest
  {Diffusion Policy} cell in the study (human/machine are 0.02–0.03).
- **All 8 human/machine {Diffusion Policy} seeds s4–s7 finally started** after ~21 h waiting; those two cells stay at
  4/8 until they finish (~6 h each).
- {RLPD} r2teacher now 2 seeds: eval 0.27, training 0.43 — the only non-machine dataset where {RLPD} scores.
- Two deprioritised r2teacher {RLPD} seeds started as GPUs freed (5 running, 1 waiting).
- No failures. 16 more world-model cells submitted for scoring. Disk 233 GB.

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
