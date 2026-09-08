#!/bin/bash
# END-TO-END (full-task) evaluation cells for one finished RLPD or DP run (PHASE_PLAN amendment (n), 2026-09-07);
# the re-runnable eval stage of cluster/sbatch_{rlpd,dp}_e2e.sh (in job) and the post-hoc path (rnd300, reels).
#   cells: fresh_eval_<set>_<mode>/metrics.json under $OUT   (sets: hold15 rnd30 spots60 [rnd300])
#   sac: MODES="sample mode"; dp: MODES="sample" (a diffusion policy has no deterministic mode)
# Usage (env):
#   KIND=sac|dp CKPT=<rlpd_final.zip | .../pretrained_model> OUT=<run dir> ARM=<dH|dDP> SEED=<n> \
#   [SETS="hold15 rnd30 spots60"] [MODES="sample mode"] [SIM_VARIANT=gc_kp4_riser3_shelf6] [EVAL_SEED=0] \
#   [VIDEO_SETS="rnd30"] [LIMIT=n] [REDO=0] [PAR=3] bash cluster/e2e_eval_cells.sh
# IC sets (amendment (n)): hold15 = baselines/eval_ics.json:hold (15 training starts -- in-distribution, NOT held
# out, REVIEW_GUIDE 8.7); rnd30 = eval_ics.json:rnd (the random box, out-of-distribution); spots60 =
# eval_ics_spots60.json:spots60 (amendment (k), the in-training-distribution test set); rnd300 =
# eval_ics_rnd300.json:rnd (post hoc, 300 starts -- hours per cell, never in job).
# Rules: a cell whose metrics.json exists is kept (resume-safe; REDO=1 wipes the listed cells first). Videos are
# written ONLY for the sets named in VIDEO_SETS (2026-09-07 disk incident: 300-decision mp4s add up fast).
set -uo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
: "${KIND:?KIND sac|dp}"; : "${CKPT:?CKPT}"; : "${OUT:?OUT}"; : "${ARM:?ARM}"; : "${SEED:?SEED}"
SETS=${SETS:-"hold15 rnd30 spots60"}
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; EVAL_SEED=${EVAL_SEED:-0}; PAR=${PAR:-3}
VIDEO_SETS=${VIDEO_SETS:-"rnd30"}
if [ -z "${MODES:-}" ]; then MODES="sample mode"; [ "$KIND" = dp ] && MODES="sample"; fi
[ "$KIND" = dp ] && case " $MODES " in *" mode "*) echo "FATAL: dp has no mode cell"; exit 1;; esac
if [ "$KIND" = sac ]; then [ -f "$CKPT" ] || { echo "FATAL: checkpoint $CKPT missing"; exit 1; }
else [ -d "$CKPT" ] || { echo "FATAL: DP checkpoint dir $CKPT missing"; exit 1; }; fi

ic_file_of() { case "$1" in
    hold15|rnd30) echo baselines/eval_ics.json ;;
    spots60)      echo baselines/eval_ics_spots60.json ;;
    rnd300)       echo baselines/eval_ics_rnd300.json ;;
    *) echo "" ;; esac; }
ic_set_of()  { case "$1" in
    hold15) echo hold ;; rnd30) echo rnd ;; spots60) echo spots60 ;; rnd300) echo rnd ;; *) echo "" ;; esac; }

for S in $SETS; do
  [ -n "$(ic_file_of "$S")" ] || { echo "FATAL: unknown IC set '$S' (hold15|rnd30|spots60|rnd300)"; exit 1; }
  [ -s "$(ic_file_of "$S")" ] || { echo "FATAL: IC file $(ic_file_of "$S") missing"; exit 1; }
done
if [ "${REDO:-0}" = 1 ]; then for S in $SETS; do for M in $MODES; do rm -rf "$OUT/fresh_eval_${S}_${M}"; done; done; fi
echo "== e2e_eval_cells kind=$KIND ckpt=$CKPT out=$OUT arm=$ARM seed=$SEED sets='$SETS' modes='$MODES' video='$VIDEO_SETS' variant=$SIM_VARIANT eval_seed=$EVAL_SEED par=$PAR $(date)"

run_cell() {
  local SET=$1 MODE=$2 D="$OUT/fresh_eval_${SET}_${MODE}"
  if [ -f "$D/metrics.json" ]; then echo "# cell $SET $MODE exists ($D/metrics.json), kept"; return 0; fi
  mkdir -p "$D"
  local VF=(); case " $VIDEO_SETS " in *" $SET "*) VF=(--video);; esac
  local LF=(); [ -n "${LIMIT:-}" ] && LF=(--limit "$LIMIT")
  python baselines/eval_e2e.py --kind "$KIND" --checkpoint "$CKPT" --ic-file "$(ic_file_of "$SET")" \
      --ic-set "$(ic_set_of "$SET")" --out "$D" --mode "$MODE" --seed "$EVAL_SEED" --max-steps 1200 \
      --sim-variant "$SIM_VARIANT" --arm "$ARM" --tag "${SET}_${MODE}" "${VF[@]}" "${LF[@]}" > "$D/eval.log" 2>&1
  local rc=$?; echo "# cell $SET $MODE rc=$rc $(date -Is)"; grep -E "^\[eval-e2e\] [0-9]+ episodes|FATAL|Traceback|Error" "$D/eval.log" | tail -3
  return $rc
}
for SET in $SETS; do
  for MODE in $MODES; do
    while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 5; done
    run_cell "$SET" "$MODE" &
  done
done
wait
python3 - "$OUT" "$KIND" "$ARM" "$SEED" "$SETS" "$MODES" <<'PY'
import json, os, sys
out, kind, arm, seed, sets, modes = sys.argv[1:7]
parts = []
for s in sets.split():
    for m in modes.split():
        f = os.path.join(out, f'fresh_eval_{s}_{m}', 'metrics.json')
        tag = f'{s}_{"S" if m == "sample" else "M"}'
        if os.path.exists(f):
            d = json.load(open(f)); n = int(d['episodes']); c = d['stage_counts']
            parts.append(f'{tag}=slide{c["slide_success"]}/{n}[p{c["picked"]},pv2{c["placed_v2"]},c{c["contact"]},'
                         f'nH{c["nested_honest"]},nP{c["nested_proxy"]}]')
        else:
            parts.append(f'{tag}=—')
line = f'E2E-HEADLINE learner={kind} arm={arm} seed={seed} key=slide_success ' + ' '.join(parts) + f' out={out}'
print(line); open(os.path.join(out, 'E2E_HEADLINE.txt'), 'w').write(line + '\n')
PY
echo "== e2e_eval_cells done $(date)"
