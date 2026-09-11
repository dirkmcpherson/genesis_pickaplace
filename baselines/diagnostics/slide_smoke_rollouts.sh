#!/usr/bin/env bash
# slide_success SMOKE TEST on {RLPD} policy rollouts under the unified ladder (Lane 9, 2026-09-11).
#
# Rolls three {RLPD} checkpoints out under `FullTaskEnv(scope='full', ladder='staged')` with
# SAMPLED actions -- the training-time statistic, and the only one that has ever produced a slide
# (the mode cells read 0) -- on both start sets, and renders EVERY episode with
# `baselines/eval_e2e_annot.py` (the shared annotate_demos overlay).
#
# Each (checkpoint x start-set x seed) is ONE process = ONE Genesis world, episodes in order
# (the shared-process protocol of PHASE_RESULTS 5.1). Genesis allows one world per process, so
# an alternative is one process per EPISODE (`--ic-index`); that is 2.05x the cost and is the
# right choice for a CELL. This is a predicate smoke test, not a cell: no number produced here
# is a rate of record, and the order-dependence is disclosed in the INDEX.
#
# Seeds vary the ACTION SAMPLING, not the starts: every seed replays the same 30 (rnd) / 15
# (hold) starts, so "120 episodes" is 30 starts x 4 draws, not 120 independent starts.
#
# usage: PY=<python> OUT=<dir> CKPTS=<dir> bash baselines/diagnostics/slide_smoke_rollouts.sh
set -u

PY="${PY:?set PY to a python with stable_baselines3 + genesis 0.2.1}"
OUT="${OUT:?set OUT to the rollout root}"
CKPTS="${CKPTS:?set CKPTS to the directory holding the fetched checkpoints}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PAR="${PAR:-6}"          # concurrent worlds; keep well under the core count, GPU untouched
THREADS="${THREADS:-2}"  # pinned before torch/taichi import, so reductions are reproducible

mkdir -p "$OUT/logs"

# checkpoint tag | checkpoint path | ic-set | seeds
#   dH_s940      unified-ladder pilot, staged, human arm, FINAL (100k decisions)
#   dH_s941_c040 the same pilot, second seed, ckpt_040 (40k decisions) -- the earlier policy
#   dH_s901      OLD-ladder human arm, FINAL (250k decisions): places and pushes, so it is the
#                positive-rich source. Its sidecar records no ladder (it predates the argument),
#                so the evaluator falls back to `staged` and says so -- i.e. this policy is
#                SCORED under the new ladder, it was not TRAINED under it.
# 120 episodes for the two FINAL checkpoints (30 rnd starts x 3 draws + 15 hold starts x 2),
# 75 for the 40k one (30 x 2 + 15 x 1) -- the brief's >= 120 / >= 60.
JOBS=(
  "dH_s940|$CKPTS/dH_s940/rlpd_final.zip|rnd|0 1 2"
  "dH_s940|$CKPTS/dH_s940/rlpd_final.zip|hold|0 1"
  "dH_s941_c040|$CKPTS/dH_s941_c040/rlpd_ckpt.zip|rnd|0 1"
  "dH_s941_c040|$CKPTS/dH_s941_c040/rlpd_ckpt.zip|hold|0"
  "dH_s901|$CKPTS/dH_s901/rlpd_final.zip|rnd|0 1 2"
  "dH_s901|$CKPTS/dH_s901/rlpd_final.zip|hold|0 1"
)

run_one() {   # tag ckpt icset seed
  local tag="$1" ck="$2" ics="$3" sd="$4"
  local cell="$OUT/${tag}_${ics}_s${sd}"
  if [ -f "$cell/metrics.json" ]; then echo "SKIP $tag $ics s$sd (done)"; return 0; fi
  CUDA_VISIBLE_DEVICES="" "$PY" "$REPO/baselines/eval_e2e_annot.py" \
      --kind sac --checkpoint "$ck" \
      --ic-file baselines/eval_ics.json --ic-set "$ics" \
      --mode sample --seed "$sd" --max-steps 1200 --threads "$THREADS" \
      --ladder staged --video --tag "$tag" --arm dH \
      --out "$cell" > "$OUT/logs/${tag}_${ics}_s${sd}.log" 2>&1
  local rc=$?
  echo "$( [ $rc -eq 0 ] && echo OK || echo "FAIL rc=$rc" ) $tag $ics s$sd"
  return 0                       # one dead cell must not stop the batch; the log says why
}

export -f run_one
export PY OUT REPO THREADS

cd "$REPO" || exit 1
# A QUEUE, not fixed-size batches with a barrier: cells differ 2x in length (30 starts vs 15),
# so a `wait` every PAR jobs would leave cores idle behind the slowest member of each batch.
# One line per cell, `xargs -n 1`, parsed back inside the worker.
for j in "${JOBS[@]}"; do
  IFS='|' read -r tag ck ics seeds <<< "$j"
  [ -f "$ck" ] || { echo "MISSING CHECKPOINT $ck" >&2; continue; }
  for sd in $seeds; do printf '%s|%s|%s|%s\n' "$tag" "$ck" "$ics" "$sd"; done
done | xargs -d '\n' -P "$PAR" -n 1 -I{} bash -c 'IFS="|" read -r t c i s <<< "{}"; run_one "$t" "$c" "$i" "$s"'
echo "ALL-CELLS-DONE"
