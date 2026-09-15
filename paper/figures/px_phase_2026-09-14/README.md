# `nested_sparse10` PIXEL conditions — by-phase analysis (2026-09-14)

Everything in this directory is produced by one command, from a local rsync mirror of the
cluster and of `~/runs_dv3_local`. Nothing was hand-edited; regenerate with:

```bash
~/workspace/genesis_sim2real/venv/bin/python baselines/diagnostics/px_phase_analysis.py \
  --data-root ~/data/genesis_pickaplace/px_analysis_2026-09-14 \
  --out-dir   paper/figures/px_phase_2026-09-14 \
  --tex       paper/figures/px_rise_time_by_phase.tex
```

`--lag-thresholds 0.5,0.1` (the default) picks which crossings the rise-time LAG figures use;
one `fig_rise_lag_thresh<t>.{png,pdf}` is written per value.

Definitions, caveats and the full data paths: `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md` §7
(the lag figures are §7.6a).

## Conditions

| key | {algorithm} | arms | seeds/arm | budget |
|---|---|---|---|---|
| `dreamer_local` | {dv3} = DreamerV3 losses in the r2dreamer chassis, pixels | human `dHfull_all` / machine `dDPfull_first` | 4 | 1 M online sim steps |
| `dreamer_cluster` | {dv3}, pixels | same | 12 (s4–s15) | 1 M (s4–7) / 2 M (s8–15) |
| `r2dreamer_cluster` | {r2dreamer} = the port's contrastive loss, pixels | same | 16 (s0–s15) | 1 M (s0–2) / 2 M (s3–15); all finished except human s15 (1.75 M) |
| `rlpd_cluster` | {RLPD}, DrQ-style shared encoder in the LN critic ensemble, pixels | `dH` / `dDPfirst` | 12 human / 11 machine (s0–s11) | 250 k decisions = 1 M sim frames; only s0/s1 per arm have finished, s2–s11 are 0.10–0.68 M in |
| `dreamer_ramp_control` | {dv3}, pixels, **`nested_ramp`** — a CONTROL, not a sparse10 condition | same | 1 | 1 M |

## Figures (PNG + PDF)

| file | what |
|---|---|
| `fig_learning_curves_dreamer_local.{png,pdf}` | {dv3} local: rolling-30 rate of each of the six phases vs online sim steps; mean + 95 % bootstrap CI over the 4 seeds per arm. TRAINING RECORD. |
| `fig_learning_curves_dreamer_cluster.{png,pdf}` | {dv3} cluster, same, 12 seeds per arm. |
| `fig_learning_curves_r2dreamer_cluster.{png,pdf}` | {r2dreamer} cluster, same, 16 seeds per arm; the band truncates where fewer than 3 seeds have reached that step. |
| `fig_learning_curves_rlpd_cluster.{png,pdf}` | {RLPD}, same, 12 human / 11 machine seeds; the band truncates near 0.4–0.65 M because only s0/s1 per arm have finished. `farside`/`slide_event`/`home` are ABSENT from its episode record. |
| `fig_learning_curves_dreamer_ramp_control.{png,pdf}` | the `nested_ramp` control, one line per seed (n = 1 per arm). |
| `fig_steady_state.{png,pdf}` | (a) TRAINING RECORD: mean `home` over the last 20 % of each run. (b) EVAL CELLS: mean `home` and `picked` over every rnd30 MODE cell at ≥ 0.5 M. One point per seed. |
| `fig_ignition.{png,pdf}` | per condition × arm: fraction of seeds whose rolling-30 `home` reaches ≥ 0.5 within budget (solid) and fraction with ≥ 1 `home` episode in any rnd30 MODE cell (hatched), with Wilson 95 % CIs. |
| `fig_rise_lag_thresh0p5.{png,pdf}` | per-phase rise-time LAG relative to `picked` at the ≥ 0.5 crossing: `lag(phase) = rise(phase) − rise(picked)`, in k online sim steps. One panel per learner condition ({dv3} local, {dv3} cluster, {r2dreamer}), human and machine side by side at each phase, one point per seed, thick bar = median over the seeds that crossed. A seed that never crosses is an open marker on the top edge, counted there, never plotted as 0. TRAINING RECORD. |
| `fig_rise_lag_thresh0p1.{png,pdf}` | the same figure at the ≥ 0.1 crossing. The sparse ladder makes the 0.5 crossings nearly simultaneous, so this variant separates the phases that the 0.5 variant compresses. |

## Table

| file | what |
|---|---|
| `../px_rise_time_by_phase.tex` | booktabs `table*`: rise time by phase, median [min, max] over seeds in k online sim steps, with (crossed/seeds) and the count still running. Needs `booktabs` and `graphicx` (`\resizebox`). |

## CSVs (every number in the figures and the table)

| file | what |
|---|---|
| `px_run_census.csv` | one row per run: condition, arm, seed, mirror path, prefill origin, nominal budget, achieved online steps, episodes, completeness, phases absent from the record, number of eval cells. |
| `px_learning_curves.csv` | the plotted bands: condition, arm, phase, 50 k-step grid point, mean, CI bounds, seeds contributing, whether the point is inside the drawn band. |
| `px_rise_time_per_seed.csv` | every per-seed rise value: `rise_step` (first step at which the rolling-30 rate ≥ threshold), `first_event_step` (first episode that reached the phase at all), `never`, `absent`, budget, achieved, note. |
| `px_rise_time_cells.csv` | the aggregated table cells: median/min/max, n crossed, n never, n pending (still running), n absent. |
| `px_rise_lag_per_seed.csv` | every point in the two lag figures: condition, arm, seed, run, threshold, phase, reference (`picked`), `rise_step_reference`, `rise_step_phase`, `lag_steps`, and `status` = `crossed` / `never` / `absent` / `no_reference` (the run's own `picked` never crossed, so no lag exists). |
| `px_steady_state.csv` | per-seed steady-state values, both kinds, tagged `training_record_last20pct` / `eval_rnd30_mode_ge0.5M`. |
| `px_ignition.csv` | per condition × arm and criterion: k, n, rate, Wilson bounds, number undetermined, total seeds. |
| `px_eval_cells.csv` | every eval cell used: run, cell kind, milestone, online step, episode count, each phase rate, and the `metrics.json` path in the mirror. |

## Data sources (local mirror, `~/data/genesis_pickaplace/px_analysis_2026-09-14/`)

| mirror path | rsynced from |
|---|---|
| `W/runs/<run>/console.log`, `step_contract.json`, `ladder_provenance.json` | `$W/runs/` on the pax cluster (`$W = $LAB/wm_fix_2026-09-03`) |
| `W/ln_milestone_cells/<run>/online_<N>/{rnd30_mode,hold15_mode}/metrics.json` | `$W/ln_milestone_cells/` |
| `LAB/gp_pxr/e2e_px/<run>/episode_rollouts.jsonl`, `fresh_eval_*/metrics.json` | `$LAB/gp_pxr/baselines/rl/checkpoints/e2e_px/` |
| `local/runs_dv3_local/<run>/console.log` and `.../dv3px_sparse10_series*/ck_*/fresh_eval_*/metrics.json` | `~/runs_dv3_local/` on pop-os |

The cluster was read only; no file on it was created or modified.

## Things the figures deliberately do not say

- `farside`, `slide_event` and `home` do not exist in the {RLPD} training record. They are
  marked "absent, not zero" and never plotted as 0. {RLPD} therefore has no panel in the
  lag figures at all.
- The {r2dreamer} (ag) seeds s3–s15 finished between the 2026-09-14 mirror and this one;
  only human s15 is still short of 2 M. The {RLPD} (ag) seeds s2–s11 have started but
  none has finished: their curves and their steady state are early-training numbers. A
  seed that has not yet crossed a threshold and has not finished is UNDETERMINED: it
  leaves the denominator and is counted separately (`+p?` in the ignition figure, "p run."
  in the table, a hollow point in the steady-state figure).
- In the lag figures, a seed is placed only by its OWN crossings; nothing is interpolated
  across seeds. The median is over the seeds that crossed, and the count of crossers is
  printed under each group, so a median never hides a non-crosser.
- The `nested_ramp` control pays a different ladder. Its numbers are not comparable with
  the sparse10 conditions.
- Training-record numbers come from sampled actions on the policy's own training starts.
  Eval-cell numbers come from deterministic actions on fixed starts in a fresh process.
  The two are never averaged together.
