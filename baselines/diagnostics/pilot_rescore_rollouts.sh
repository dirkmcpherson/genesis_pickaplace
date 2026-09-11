#!/usr/bin/env bash
# {RLPD} rollouts WITH per-frame stage records, for the offline four-ladder re-score
# (Lane 11, PILOT_RESCORE_2026-09-11).
#
# Same protocol as Lane 9's slide smoke (`slide_smoke_rollouts.sh`) with two differences:
#   * `--records-out` writes one per-env-frame stage record per episode, so every predicate
#     question after this one is a 2-second offline re-score instead of a re-simulation;
#   * no `--video`, so the cells are cheaper and the clips are not re-rendered.
#
# Each (checkpoint x start-set x seed) is ONE process = ONE Genesis world, episodes in order
# (the shared-process protocol of PHASE_RESULTS 5.1). Seeds vary the ACTION SAMPLING, not the
# starts: each seed replays the same 30 (`rnd`) / 15 (`hold`) starts, so the episode counts are
# DRAWS, not independent starts.
#
# The seed sets for dH_s940 / dH_s941_c040 / dH_s901 on `rnd` (0 1 2) and `hold` (0 1) are the
# ones Lane 9 ran, so those cells are a direct reproducibility check against
# can_pos_recovery/videos_slide_smoke_2026-09-11/INDEX.md.
#
# usage: PY=<python> OUT=<rollout root> REC=<records root> CKPTS=<checkpoint dir> \
#        bash baselines/diagnostics/pilot_rescore_rollouts.sh
set -u

PY="${PY:?set PY to a python with stable_baselines3 + genesis 0.2.1}"
OUT="${OUT:?set OUT to the rollout root}"
REC="${REC:?set REC to the stage-record root}"
CKPTS="${CKPTS:?set CKPTS to the directory holding the fetched checkpoints}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PAR="${PAR:-8}"
THREADS="${THREADS:-2}"
VARIANT="${VARIANT:-gc_kp4_riser3_shelf6}"

mkdir -p "$OUT/logs" "$REC"

# tag | checkpoint | arm | ic-set | seeds
#   dH_s940       PILOT, ladder=staged, human arm, FINAL (100k decisions)
#   dH_s941_c040  PILOT, ladder=staged, human arm, ckpt_040 (40k) -- the only s941 on this box
#   dH_s901       LONG-RUN (250k), human arm. Its sidecar records NO ladder (it predates the
#                 argument) and it trained under the OLD picked1/contact2/nested4 ladder, so
#                 rolling it under `staged` SCORES it under a ladder it did not train on.
#   dDPfirst_s920 the same, MACHINE arm -- the only machine checkpoint on this box.
JOBS=(
  "dH_s940|$CKPTS/dH_s940/rlpd_final.zip|dH|rnd|0 1 2"
  "dH_s940|$CKPTS/dH_s940/rlpd_final.zip|dH|hold|0 1 2 3"
  "dH_s941_c040|$CKPTS/dH_s941_c040/rlpd_ckpt.zip|dH|rnd|0 1 2"
  "dH_s941_c040|$CKPTS/dH_s941_c040/rlpd_ckpt.zip|dH|hold|0 1 2 3"
  "dH_s901|$CKPTS/dH_s901/rlpd_final.zip|dH|rnd|0 1 2"
  "dH_s901|$CKPTS/dH_s901/rlpd_final.zip|dH|hold|0 1 2 3"
  "dDPfirst_s920|$CKPTS/dDPfirst_s920/rlpd_final.zip|dDPfirst|rnd|0 1 2"
  "dDPfirst_s920|$CKPTS/dDPfirst_s920/rlpd_final.zip|dDPfirst|hold|0 1 2 3"
)

run_one() {   # tag ckpt arm icset seed
  local tag="$1" ck="$2" arm="$3" ics="$4" sd="$5"
  local cell="$OUT/${tag}_${ics}_s${sd}"
  if [ -f "$cell/metrics.json" ]; then echo "SKIP $tag $ics s$sd (done)"; return 0; fi
  CUDA_VISIBLE_DEVICES="" "$PY" "$REPO/baselines/eval_e2e.py" \
      --kind sac --checkpoint "$ck" \
      --ic-file baselines/eval_ics.json --ic-set "$ics" \
      --mode sample --seed "$sd" --max-steps 1200 --threads "$THREADS" \
      --sim-variant "$VARIANT" --ladder staged --tag "$tag" --arm "$arm" \
      --records-out "$REC/${tag}_${ics}_s${sd}" \
      --out "$cell" > "$OUT/logs/${tag}_${ics}_s${sd}.log" 2>&1
  local rc=$?
  echo "$( [ $rc -eq 0 ] && echo OK || echo "FAIL rc=$rc" ) $tag $ics s$sd"
  return 0                       # one dead cell must not stop the batch; the log says why
}

export -f run_one
export PY OUT REC REPO THREADS VARIANT

cd "$REPO" || exit 1
for j in "${JOBS[@]}"; do
  IFS='|' read -r tag ck arm ics seeds <<< "$j"
  [ -f "$ck" ] || { echo "MISSING CHECKPOINT $ck" >&2; continue; }
  for sd in $seeds; do printf '%s|%s|%s|%s|%s\n' "$tag" "$ck" "$arm" "$ics" "$sd"; done
done | xargs -d '\n' -P "$PAR" -n 1 -I{} bash -c 'IFS="|" read -r t c a i s <<< "{}"; run_one "$t" "$c" "$a" "$i" "$s"'
echo "ALL-CELLS-DONE"
