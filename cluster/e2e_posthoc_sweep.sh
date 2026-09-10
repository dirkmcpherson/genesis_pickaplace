#!/usr/bin/env bash
# Post-hoc evals for the 2026-09-10 e2e long-run seeds ONLY.
#
# Why this exists: the in-job eval stage never ran for the ORIGINAL 16 r2dreamer seeds --
# wmfix_full.sbatch located its own Slurm log by an assumed job NAME, and every e2e run was
# submitted with -J e2eL_r2_*, so the check died (FATAL: no [sim-variant] line) AFTER training
# completed. Training artifacts are intact, so the evals simply run afterwards.
#
# The global full_eval_sweep.sh cannot do this: it greps slurm/wmfix_full_*.out for each run's
# logdir line, which is the SAME job-name assumption, so it finds nothing and skips every one of
# these runs. Here the log is located by globbing slurm/*.out instead. Scoped to these seeds so
# no unrelated experiment is touched.
set -u
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; cd "$W"
DRY=${DRY:-0}; n=0; skipped=0
for d in runs/full_r2d_state_dHfull_all_rx_s9* runs/full_r2d_state_dDPfull_first_rx_s9*; do
  [ -d "$d" ] || continue
  r=$(basename "$d")
  case "$r" in *_FAILED_*|*smoke*) continue;; esac
  [ -f "$d/latest.pt" ] || { skipped=$((skipped+1)); continue; }
  [ -f "$d/fresh_eval_rnd30_mode/metrics.json" ] && continue        # already has the cell
  [ -f "$d/.posthoc_submitted" ] && continue                        # already queued once
  LAST=$(tail -c 4000 "$d/metrics.jsonl" 2>/dev/null | grep -oE '"step": [0-9]+' | tail -1 | grep -oE "[0-9]+" || echo 0)
  [ "${LAST:-0}" -ge 3995000 ] || { skipped=$((skipped+1)); continue; }   # training must have finished
  out=$(grep -l "logdir=$W/runs/$r " slurm/*.out 2>/dev/null | head -1)
  [ -n "$out" ] && grep -q "# train rc=" "$out" || { echo "  no train-rc line for $r"; skipped=$((skipped+1)); continue; }
  if [ "$DRY" = "1" ]; then echo "  WOULD SUBMIT $r (step=$LAST)"; n=$((n+1)); continue; fi
  J=$(sbatch --parsable full_posthoc_evals.sbatch "$r") && { echo "$J" > "$d/.posthoc_submitted"; n=$((n+1)); echo "  post-hoc evals $r -> $J"; }
done
echo "e2e_posthoc_sweep: ${n} submitted/eligible, ${skipped} not ready"
