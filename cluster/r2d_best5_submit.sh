#!/bin/bash
# cluster/r2d_best5_submit.sh -- driver for the finding-10 repair (paper/AUDIT_approach_2026-09-02.md):
# for each run of the frozen corrected-world WM block (pick_v5d4c_delta_shaped_{dH,dDP}_s80-87) pick the
# FIVE fraction checkpoints (nearest recorded step to 20/40/60/80/100 % of STEPS among ckpt_*.pt still on
# disk -- the launcher's own keep rule, sbatch_r2dreamer.sh ~:489-503, verbatim) plus BEST_selected.pt (the
# BEST-of-K checkpoint of record, for a same-world same-process comparison), write a manifest, and submit
# one cluster/r2d_rescore5.sh job per (checkpoint, action mode) scoring hold-15 + rnd-30.
#
#   [MODES="sample mode"] [RUNS="dH:80 dDP:83 ..."] [DRYRUN=1] [SMOKE=1] bash cluster/r2d_best5_submit.sh
#   SMOKE=1 submits only the first (run, ckpt, mode); DRYRUN=1 prints the plan and manifests, submits nothing.
#   Never cancels or touches a running job; never deletes anything.
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02
GPR=$LAB/genesis_pickaplace; RUNS_DIR=$LAB/r2dreamer/runs
OUT=${OUT:-$GPR/baselines/outputs/best5_rescore}; mkdir -p "$OUT/logs" "$OUT/manifests"
MODES=${MODES:-sample}; STEPS=${STEPS:-3e6}; SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}
RUNS=${RUNS:-"dH:80 dH:81 dH:82 dH:83 dH:84 dH:85 dH:86 dH:87 dDP:80 dDP:81 dDP:82 dDP:83 dDP:84 dDP:85 dDP:86 dDP:87"}
NICE_MODE=${NICE_MODE:-500}   # deterministic pass queues behind the sampled pass of record
n_sub=0
for R in $RUNS; do
  A=${R%%:*}; S=${R#*:}; RUN=pick_v5d4c_delta_shaped_${A}_s$S; D=$RUNS_DIR/$RUN
  [ -f "$D/ckpt_scores.tsv" ] || { echo "FATAL: $D/ckpt_scores.tsv missing"; exit 1; }
  [ -f "$D/BEST_selected.pt" ] || { echo "FATAL: $D/BEST_selected.pt missing"; exit 1; }
  # launcher rule, verbatim (rows whose file exists; step from column 3; nearest to f*STEPS)
  SEL=$(python3 - "$D/ckpt_scores.tsv" "$STEPS" "$OUT/manifests/$RUN.json" "$RUN" "$A" "$S" <<'PY'
import sys, os, json
rows = []
for l in open(sys.argv[1]):
    p = l.rstrip('\n').split('\t')
    if len(p) < 2 or not os.path.exists(p[0]): continue
    try: sc = float(p[1])
    except ValueError: sc = -1.0
    st = int(p[2]) if len(p) > 2 and p[2].isdigit() else None
    rows.append((p[0], sc, st))
total = float(sys.argv[2]); withstep = [r for r in rows if r[2] is not None]
K = sum(1 for _ in open(sys.argv[1]))
picks = {}
for f in (0.2, 0.4, 0.6, 0.8, 1.0):
    r = min(withstep, key=lambda r: abs(r[2] - f * total))
    picks['F%d' % int(f * 100)] = dict(path=r[0], sel_score=r[1], step_name=r[2])
paths = [v['path'] for v in picks.values()]
assert len(set(paths)) == 5, ('fraction checkpoints not distinct', paths)
man = dict(run=sys.argv[4], arm=sys.argv[5], seed=int(sys.argv[6]), K=K, n_on_disk=len(rows), steps_total=total,
           fractions=picks, bestk=dict(path=os.path.join(os.path.dirname(sys.argv[1]), 'BEST_selected.pt')))
json.dump(man, open(sys.argv[3], 'w'), indent=1)
for k, v in picks.items(): print(k, v['path'])
print('BESTK', man['bestk']['path'])
PY
) || { echo "FATAL: selection failed for $RUN"; exit 1; }
  echo "## $RUN K=$(wc -l < "$D/ckpt_scores.tsv")"; echo "$SEL" | sed 's/^/   /'
  while read -r FRAC CK; do
    for MODE in $MODES; do
      TAG="${RUN}_${FRAC}"
      EXTRA=(); [ "$MODE" = mode ] && EXTRA=(--nice="$NICE_MODE")
      CMD=(sbatch "${EXTRA[@]}" -J "b5_${A}${S}_${FRAC}_${MODE}" --output "$OUT/logs/${TAG}_${MODE}_%j.out"
           --export="ALL,RUNDIR=$D,CK=$CK,TAG=$TAG,MODE=$MODE,OUT=$OUT,SIM_VARIANT=$SIM_VARIANT,GENESIS_PICKAPLACE_ROOT=$GPR"
           "$GPR/cluster/r2d_rescore5.sh")
      if [ "${DRYRUN:-0}" = 1 ]; then echo "   [dry] ${CMD[*]}"; else
        (cd "$GPR" && "${CMD[@]}") || { echo "FATAL: sbatch failed for $TAG $MODE"; exit 1; }
        n_sub=$((n_sub+1))
        [ "${SMOKE:-0}" = 1 ] && { echo "SMOKE: submitted 1 job, stopping"; exit 0; }
      fi
    done
  done <<< "$SEL"
done
echo "BEST5-SUBMITTED n=$n_sub modes='$MODES' out=$OUT"
