# How to pull and plot the `nested_sparse10` PIXEL results (for a new agent, 2026-09-15)

Read alongside `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md` (what every run is and how it was
launched; §7 = the definitions behind every figure) and the two tallies
`paper/AF_PIXEL_CLUSTER_TALLY_2026-09-14.md` (cluster cells, per amendment) and
`paper/AE_PIXEL_HUMAN_VS_MACHINE_2026-09-13.md` (the local seeds' ten-cell series). Paths:
`LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`, cluster via `ssh jstale02@login.pax.tufts.edu`
(needs the Tufts VPN; the login host does not resolve without it).

## 1. Where the numbers live

| what | where | unit |
|---|---|---|
| world-model TRAINING RECORD (sampled actions, the policy's own starts, one line per episode) | `$W/runs/full_r2d_state_<set>_<rep>_s<N>/console.log`; local: `~/runs_dv3_local/dv3px_sparse10_<set>_rlDreamer_s<N>/console.log` | counter = env frames; online step = counter − origin (117 624 human sets, 149 952 machine sets = the first `[counter]` line of the log) |
| world-model scalar metrics | `metrics.jsonl` beside it (`step`, `fps/fps`, `train/loss/*`, `train/ret`, …) | same counter |
| world-model EVAL CELLS (deterministic actions, fixed starts, fresh process, pinned 64-core nodes) | `$W/ln_milestone_cells/<run>/online_<N>/{rnd30_mode,hold15_mode,rnd30_sample}/metrics.json` + one mp4 per episode; local series: `~/runs_dv3_local/dv3px_sparse10_series_<dH|dM>_s<N>/ck_<counter>/fresh_eval_{rnd30_mode,hold15_mode}/metrics.json` | `headline_stages` = rates; `per_episode[].outcome` ∈ {home, tipped, timeout} |
| RLPD training record | `$LAB/gp_pxr/baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_<dH|dDPfirst>_s<N>/episode_rollouts.jsonl` (has `ep_picked`, `ep_placed_v2`; NO farside/slide/home keys — a logging gap) and the Slurm log `$LAB/gp_pxr/e2e_rlpd_px_<jobid>.out` (`total_timesteps`, `[Q-WATCHDOG]`) | decisions; ×4 = sim frames |
| RLPD final cells | `<run>/fresh_eval_{hold15,rnd30}_{mode,sample}/metrics.json` (+ `_iso` shards — use the shared cell and the per-episode counts, the merged `_iso` headline is unreliable) | rates over 15 / 30 |
| {DP pixel} (ah) — other workstation | see PHASE_PLAN (ah); runs under `$LAB/gp_ah` | |

`<set>` = `dHfull_all_rns10h_img` (human) / `dDPfull_first_rns10h_img` (machine); `<rep>` = `dreamer` (DreamerV3 losses) /
`r2dreamer` (contrastive loss). Skip `*smoke*`, `*.preempted*`, `$W/runs_failed/`, and the `final/` cell dir (it is the
same checkpoint as the last milestone). The `_rnrh_img` runs are the `nested_ramp` control, not sparse10.

## 2. Pull (rsync only — the cluster is read-only for analysis; never open a Genesis world on the login node)

```bash
M=~/data/genesis_pickaplace/px_analysis_2026-09-14          # the mirror the analysis script reads
H=jstale02@login.pax.tufts.edu; LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03
# training records + metrics (small): every pixel run, both losses, both ladders
rsync -avz --include='*/' --include='console.log' --include='metrics.jsonl' --include='ladder_provenance.json' \
      --include='step_contract.json' --exclude='*' $H:$W/runs/ $M/W/runs/           # then prune non-*_img_* dirs if you like
# eval cells WITHOUT videos (metrics + provenance only)
rsync -avz --include='*/' --include='metrics.json' --include='provenance.json' --include='trees.json' --exclude='*' \
      $H:$W/ln_milestone_cells/ $M/W/ln_milestone_cells/
# RLPD records + final cells (no videos). Add --delete so seeds removed on the cluster leave the mirror too.
mkdir -p $M/LAB/gp_pxr/e2e_px
rsync -avz --include='*/' --include='episode_rollouts.jsonl' --include='metrics.json' --include='*.action_mode.json' \
      --exclude='*' $H:$LAB/gp_pxr/baselines/rl/checkpoints/e2e_px/ $M/LAB/gp_pxr/e2e_px/   # the script reads LAB/gp_pxr/e2e_px (fixed 09-16)
rsync -avz $H:'$LAB/gp_pxr/e2e_rlpd_px_*.out' $M/LAB/gp_pxr/
# {Diffusion Policy} human/machine (ah) cells (no videos, no checkpoints)
rsync -avz --prune-empty-dirs --include='*/' --include='metrics.json' --include='E2E_HEADLINE.txt' --exclude='wandb/' \
      --exclude='checkpoints/' --exclude='*' $H:$LAB/gp_ah/baselines/outputs/dp_px/ $M/LAB/gp_ah/dp_px/
# planner72 + r2teacher campaigns (all four learners): metrics/logs/ledgers only, dead runs excluded, --delete
FILT=(--prune-empty-dirs --exclude='runs_dead_*/' --exclude='*.preempted*/' --exclude='wandb/' --exclude='wandb_eval/' \
      --exclude='*_iso/' --exclude='data/' --exclude='artifacts/' --exclude='preparation/' --exclude='resubmit_*/' \
      --exclude='checkpoints/' --exclude='.hydra/' --include='*/' --include='console.log' --include='metrics.jsonl' \
      --include='metrics.json' --include='step_contract.json' --include='ladder_provenance.json' --include='provenance.json' \
      --include='episode_rollouts.jsonl' --include='E2E_HEADLINE.txt' --include='milestones/*.json' --include='slurm/*.out' \
      --include='/*.jsonl' --include='/QUEUE_HANDOFF.md' --exclude='*')
rsync -az --delete --delete-excluded "${FILT[@]}" $H:$LAB/planner72_px_2026-09-15/ $M/P72/   # NOT planner_px_2026-09-15 (old 70-tape cohort)
rsync -az --delete --delete-excluded "${FILT[@]}" $H:$LAB/r2teacher_px_2026-09-15/ $M/R2T/
# local seeds (already on pop-os)
rsync -a --include='*/' --include='console.log' --include='metrics.jsonl' --include='metrics.json' --exclude='*' \
      ~/runs_dv3_local/ $M/local/runs_dv3_local/
```

Add `--include='*.mp4'` to the cell rsync if you want the episode videos (≈ 10 GB for the pixel cells). The mirror
layout (`W/`, `LAB/`, `local/`) is what `px_phase_analysis.py` expects; keep it.

## 3. Plot

```bash
cd ~/workspace/genesis_pickaplace
~/workspace/genesis_sim2real/venv/bin/python baselines/diagnostics/px_phase_analysis.py \
  --data-root ~/data/genesis_pickaplace/px_analysis_2026-09-14 \
  --out-dir   paper/figures/px_phase_2026-09-14 \
  --tex       paper/figures/px_rise_time_by_phase.tex      # [--lag-thresholds 0.5,0.1] [--roll 30] [--grid 50000] [--boot N] [--seed 20260914]
```

Deterministic; it writes the learning-curve grids (rolling-30 phase rates vs online steps, mean ± bootstrap band per
learner × arm), the steady-state figure (training record last 20 % AND eval cells ≥ 0.5M, kept separate), the ignition
figure (training-record criterion and ≥ 1 `home` cell, Wilson CIs), the per-phase rise-time LaTeX table and the two
rise-time LAG figures (phase rise minus `picked` rise, at the 0.5 and 0.1 crossings), plus one CSV per figure with every
per-seed value. `paper/figures/px_phase_2026-09-14/README.md` lists each output and its source column. Runs that are
still training are included as far as they go and flagged incomplete; bands are cut where fewer than 3 seeds have data.

## 4. The statistics of record (compute from the cells, not from the figures)

- **Per seed, world models:** `home` rate in the rnd30 MODE cell. The pooled statistic every seed has = mean of the 0.5M
  and 1M cells (the local seeds' series contain both; the (af) seeds have only those two; the (ag) seeds four). Secondary:
  the local ten-cell series mean over 0.3–1.0M, the (ag) four-milestone mean, the single 2M cell.
- **Per seed, RLPD:** `home` in the final rnd30 MODE cell (ignition = ≥ 1 `home` in any final cell).
- **Test:** exact two-sided permutation on the per-seed statistic (all C(n, k) relabelings below ~10⁶, Monte-Carlo
  200–300k otherwise), ±0.15 null margin, ignition count per arm beside it. The tallies show the arithmetic for every
  readout so far (dreamer 16 v 16: 0.306 v 0.304, p 0.98; r2dreamer 15 v 15: 0.593 v 0.527, p 0.06).
- **Never** quote one checkpoint as a seed's result: the deterministic policy swings 0 ↔ 17/30 between milestones on
  the same seed (see the (ag) tables); quote the pooled/series statistic and say which.
- Rise time = first online step where the rolling-30 training-record rate of a phase reaches 0.5 (0.1 as the early
  crossing); it needs no checkpoints. The eval series answers a different question (deterministic policy on the fixed
  test starts) and only exists at the saved milestones.

## 5. Re-scoring or adding cells

The milestone sweep is idempotent and scores every unscored `milestones/online_*.pt` of every pixel run:
`SWEEP_MODE=cpu64pre RUN_FILTER=_img MS_FILTER=all MAXJOBS=48 bash $W/ln14_milestone_sweep.sh` (CPU nodes, 64/64-core
class, preempt QOS), then `bash $W/px_release.sh` (keeps my GPU queue inside Slurm's 20-job-per-user backfill window and
reserves slots for the (ah) DP jobs). A cell for a checkpoint that was never saved cannot be made — rerun the seed with a
denser `R2_MILESTONES` list. Local re-evaluation of a series checkpoint: the command in PROVENANCE §4b, one Genesis
world at a time, never beside a running `train.py` on pop-os.
