# Handoff: pixel study, 4 demonstration datasets × 4 learners (2026-09-17, 12:30)

This page is for a reader who did not follow the work. It explains what the study is, what the numbers show today,
what is still running, and what is still undecided. Every result names its learner in braces. A missing seed or cell is
absent, never zero.

Related documents:
- Detailed log: `HRI_results/STATE_PIXEL_4x4_2026-09-16.md`, with dated updates and corrections.
- Human-v-machine summary: `paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md`.
- Pull and replot commands: `HRI_results/HOWTO_pull_and_plot_pixel_results_2026-09-15.md`.
- How every run was launched: `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`.

Branch `ladder-unify-2026-09-11`, head `a335943` plus this file.

## 1. The study in one paragraph

A simulated Kinova arm must pick a can, place it and slide it into a goal ring. The terminal stage `home` pays +10;
nothing else pays (`nested_sparse10`). The policy sees two 64×64 cameras plus 8 joint and gripper numbers, with no
object or goal pose. Four demonstration datasets are compared under four learners. Every dataset starts from the same
can positions (checked tape by tape: 56 positions within 5 mm; the human set has one extra position and repeated
attempts). The goal is identical in all four.

| dataset | tapes | tapes reaching `home` | median decisions | origin |
|---|---|---|---|---|
| human | 74 | 13 (18 %) | 389 | every teleoperated attempt, raw |
| machine | 72 | 14 (19 %) | 601 (the cap) | first attempt per start of a state-based Diffusion Policy teacher trained on pruned human data |
| planner | 72 | 68 (94 %) | 256 | motion-planner demonstrations |
| r2dreamer teacher | 72 | 68 (94 %) | 33 | first attempts of one human-trained pixel r2dreamer policy |

Human and machine are matched on success rate. Planner and teacher differ in success rate, tape length and privileged
access, as well as in source. A difference involving them is not a pure "source" effect.

| learner | budget | notes |
|---|---|---|
| {DreamerV3 losses} (r2dreamer chassis, reconstruction loss) | 2M online steps | checkpoints at 0.5, 1, 1.5 and 2M |
| {r2dreamer} (contrastive loss) | 2M online steps | same |
| {RLPD} (our SB3 reimplementation of Ball et al. 2023, DrQ-v2 pixel encoder) | 250k decisions | no frame stack; LayerNorm parameters shared across the critic ensemble |
| {Diffusion Policy} (lerobot, random 56-pixel crop) | 100k updates | imitation only; ignores reward |

## 2. Reporting rule for the first round (user, 2026-09-17)

- **Seeds.** 8 per condition. World models use only runs that trained to 2M, taking the 8 lowest seed numbers that
  reached 2M (result-blind). Human and machine: {DreamerV3 losses} s8–15, {r2dreamer} s3–10. Everything else: s0–7.
- **World-model statistic.** Per-seed mean of the 1.5M and 2M checkpoint cells. A single checkpoint swings between 0
  and 17/30 on one seed, so one cell is too noisy. Tables also list every checkpoint.
- **Two charts, both on `home`.**
  - **Eval:** 30 random starts, sampled actions for every learner. For {RLPD}, sampled was chosen after seeing that it
    scores higher than deterministic, so its eval p-value is not a clean test.
  - **Training:** the 15 demonstration starts, 14 of them used in training. World models have only deterministic cells
    there; {RLPD} and {Diffusion Policy} are sampled.
- **Registered human-v-machine readouts stay as registered.** They use the 0.5M + 1M cells with 16 v 16 seeds, and
  deterministic {RLPD} at 8 v 8. See the summary document.

## 3. Results today (mean `home` rate; seeds with a cell in brackets)

Files: `paper/figures/px_phase_2026-09-14/fig_results_4x4_sampled_home.png` (eval) and `fig_results_4x4_hold15_home.png`
(training), each with a `_picked` twin.

| learner | chart | human | machine | planner | r2dreamer teacher |
|---|---|---|---|---|---|
| {DreamerV3 losses} | eval | 0.31 (8) | 0.35 (8) | 0.04 (7) | 0.28 (1) |
| | training | 0.47 (8) | 0.50 (8) | 0.08 (7) | 0.83 (1) |
| {r2dreamer} | eval | 0.59 (8) | 0.53 (8) | 0.54 (7) | 0.56 (2) |
| | training | 0.91 (8) | 0.92 (8) | 0.89 (7) | 0.93 (2) |
| {RLPD} | eval | 0.00 (8) | 0.16 (8) | 0.00 (6) | 0.00 (1) |
| | training | 0.00 (8) | 0.18 (8) | 0.00 (7) | 0.07 (1) |
| {Diffusion Policy} | eval | 0.03 (4) | 0.02 (4) | 0.57 (3) | 0.60 (2) |
| | training | 0.03 (4) | 0.03 (4) | 0.93 (3) | 1.00 (2) |

**What the numbers support (exact permutation tests on per-seed values; cells under 8 seeds are interim):**
1. **Human v machine, world models: no difference.** Eval p 0.64 ({DreamerV3 losses}) and 0.29 ({r2dreamer}). The
   registered 16 v 16 readouts agree: 0.306 v 0.303, p 0.97; 0.588 v 0.530, p 0.09.
2. **{r2dreamer} learns equally well from all four datasets.** On planner data it learns later (0.08 at 0.5M against
   0.55 for human) but catches up by 1.5–2M (0.60 at 2M against 0.57 human). An earlier statement that planner data
   "hurts" {r2dreamer} was an artefact of scoring only the 0.5M/1M checkpoints and has been withdrawn.
3. **{DreamerV3 losses} do poorly on planner data:** 0.04 v 0.31 human, p 0.01, 7 seeds. Some seeds reach `home` in
   training only after about 1.1M steps.
4. **{RLPD}: human data gives zero on every seed.** Machine 3 of 8 seeds reach `home`. The registered deterministic
   test gives p 0.20. In training, every human and machine seed reaches `home` within 13–27k decisions, then loses it;
   only machine seeds keep some. Planner {RLPD} almost never reaches `home`. Teacher-data {RLPD} learns fastest (0.5 by
   13–16k decisions).
5. **{Diffusion Policy} tracks the success rate of its data.** It barely finishes on human or machine data, whose tapes
   are ~80 % failures, and it imitates failures too. It scores 0.57–1.00 on planner and teacher data, whose tapes are
   94 % successes. The human arm trains on raw demonstrations. Pruning (dropping no-pick attempts, trimming idle time)
   raised state-based Diffusion Policy clearly in earlier work; a pruned pixel set has not been built.
6. **Observation control ({DreamerV3 losses}, local, 1 seed).** 17-dim privileged state never reached `home` in 2M
   steps, while pixels ignite 64/64 world-model seeds.

## 4. Known defects and caveats a writer must carry

- **{RLPD} budget leak.** The 13 human/machine seeds s2–s8 were launched with the world models' 2M setting, trained
  to the 30 h wall clock (400–592k decisions) and were scored at their saved 250k checkpoint. Learning rate is constant,
  so the checkpoint is equivalent to a 250k run. Records past 250k are cut from all analysis.
- **{RLPD} recipe gaps versus the paper.** No frame stack. Shared LayerNorm parameters. 10 critic updates per decision,
  where the paper uses 20. Two-layer critics. Terminal-only reward. The user plans to fix RLPD later, so the three
  unstarted teacher {RLPD} seeds (s4, s6, s7) were pushed to the back of the queue (`Nice=10000`).
- **Two "failed" world-model seeds finished training.** planner {DreamerV3 losses} s0 and teacher {r2dreamer} s2 wrote
  all milestones, then crashed in an in-job evaluation on a full home-directory cache. They are scored normally.
- **Human {RLPD} seed 8** is an extra seed outside the design. It is drawn in figures and excluded from tables.
- **Planner {Diffusion Policy} s5 and s6** both score 17/30. They are distinct models (training seeds 5 and 6,
  different weights) with different per-episode outcomes.

## 5. Cluster state at 12:00 on 09-17

**Running (20 GPUs, the per-user limit):**
- Teacher {RLPD} s0–s3.
- Teacher world models: {DreamerV3 losses} s1–s5 and s7; {r2dreamer} s3–s6.
- Teacher {Diffusion Policy} s2–s6.
- Planner {DreamerV3 losses} s7 (nearly done).
- Plus 21 world-model scoring jobs on CPU nodes.

**Waiting for GPUs:**
- 14 {Diffusion Policy} jobs: human/machine s4–s7 (3773972–79), planner s0–s4, teacher s7. They have waited since
  yesterday; every other GPU job has now started, so they are next.
- The 3 deprioritised teacher {RLPD} seeds, last.

**Estimated finish.** Pixel {Diffusion Policy} tonight. Teacher world models later today. The deprioritised {RLPD}
seeds are open-ended (14–21 h each once started).

**Nothing failed since 09-16 22:00. Disk 260 GB free.**

## 6. How to keep it moving (anyone with cluster access)

1. **Score new world-model milestones** for the two new datasets. Idempotent; the campaign script's own limit of 4 per
   pass is too slow, so use 40:
   ```bash
   LAB=/cluster/tufts/shortlab/jstale02
   for C in planner72 r2teacher; do ROOT=$LAB/${C}_px_2026-09-15; ( export LAB W=$ROOT GP=$LAB/planner_px_2026-09-15/gp \
     R2=$LAB/planner_px_2026-09-15/r2dreamer CELLROOT=$ROOT/evaluation SBATCH_FILE=$ROOT/preparation/launch/milestone_eval.sbatch \
     SWEEP_MODE=cpu64pre RUN_FILTER="native_rns10h_img_(dreamer|r2dreamer)_s" MS_FILTER=all MAXJOBS=40; \
     python3 $ROOT/preparation/scheduling.py --root $ROOT --evaluation >/dev/null && bash $ROOT/preparation/launch/milestone_sweep.sh ); done
   ```
   {RLPD} and {Diffusion Policy} submit their own scoring when training ends.
2. **Scheduler trap.** CPU scoring jobs sit "Priority"-pending beside idle nodes whenever more than 20 of the user's
   higher-priority GPU jobs are waiting. Fix: hold every job pending on `QOSMaxGRESPerUser` for ~40 s, then release.
   Those jobs cannot start anyway, so they lose nothing. Afterwards verify that no job is left in `JobHeldUser`.
3. **Regenerate.** Pull (HOWTO §2), then:
   ```bash
   V=~/workspace/genesis_sim2real/venv/bin/python; F=paper/figures/px_phase_2026-09-14
   $V baselines/diagnostics/px_phase_analysis.py --data-root ~/data/genesis_pickaplace/px_analysis_2026-09-14 --out-dir $F --tex paper/figures/px_rise_time_by_phase.tex
   $V baselines/diagnostics/px_results_4x4_sampled.py --set rnd30 --wm-mode sample --out $F/px_results_4x4_sampled_per_seed.csv
   $V baselines/diagnostics/px_results_4x4_sampled.py --set hold15 --wm-mode mode --out $F/px_results_4x4_hold15_per_seed.csv
   for m in home picked; do
     $V baselines/diagnostics/px_results_4x4_plot.py --sampled --csv $F/px_results_4x4_sampled_per_seed.csv --out $F/fig_results_4x4_sampled_$m --metric $m
     $V baselines/diagnostics/px_results_4x4_plot.py --hold15 --csv $F/px_results_4x4_hold15_per_seed.csv --out $F/fig_results_4x4_hold15_$m --metric $m
   done
   ```
   The login nodes do not share `/tmp`; pipe scripts over ssh (`ssh … "python -" < script.py`).

## 7. Open decisions (user)

1. **State-based {Diffusion Policy} on the cleaned datasets.** Estimated in the session:
   - Already trained, needing only a `home` rescore: 8 pruned-human seeds and 4 machine seeds.
   - New work: 4 more machine seeds and 16 planner/teacher seeds, about 62 GPU-h plus CPU scoring.
   - Finish around midnight 09-17 if run ahead of the waiting pixel {Diffusion Policy} jobs, else Friday afternoon.
   - Proposed: pruned human; machine, planner and teacher as-is.
   - Planner has a state lerobot set to check; the teacher set probably needs a build.
   - Not started.
2. **Pruned-human pixel {Diffusion Policy}.** One dataset build plus 8 seeds. Not started.
3. **{RLPD} fixes** (frame stack, per-critic LayerNorm, UTD 20, deeper critics) and a short pilot to see which matter.
   Not started.
