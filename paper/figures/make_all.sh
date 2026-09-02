#!/bin/bash
# Re-render every paper figure (single-column format) from the repo root. Needs matplotlib+numpy
# (~/.wandb-venv/bin/python locally). Inputs (rsync-only, in paper/figures/): tape_stats.csv,
# tape_paths.npz, real_tape_stats.csv, real_tape_paths.npz, tape_dyn_metrics.csv,
# set_level_metrics.csv, speed_series_w3.npz, raw_vs_pruned_tapes.csv, raw_vs_pruned_sets.json;
# fig9 also needs the Bayes draws: python analysis/bayes_triple_2026-09-01.py --s84-best-rnd 1
set -euo pipefail
PY=${PY:-$HOME/.wandb-venv/bin/python}
cd "$(dirname "$0")/../.."
for s in make_figs_2026-08-31 make_fig6_tapes_2026-08-31 make_fig7_real_vs_sim_2026-08-31 \
         make_fig8_wm_metric_2026-09-01 make_fig9_advisor_2026-09-01 make_fig10_burstiness_2026-09-01 \
         make_fig11_raw_vs_pruned_2026-09-02 make_fig12_dataset_diff_2026-09-02; do
  $PY paper/figures/$s.py "$@" | grep '^wrote' || { echo "FAILED: $s"; exit 1; }
done
