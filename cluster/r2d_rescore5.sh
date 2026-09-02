#!/bin/bash
# cluster/r2d_rescore5.sh -- paper/AUDIT_approach_2026-09-02.md finding 10 repair (BEST-of-K -> BEST-of-5).
#
# ONE job = ONE r2dreamer checkpoint x ONE action mode, scored on the frozen baselines/eval_ics.json
# `hold` (15 ICs) AND `rnd` (30 ICs), one fresh CPU process per episode. This is the cluster/r2d_rescore.sh
# loop re-implemented, because that script (a) cannot pass --mode (audit finding 26) and (b) neither sets
# nor verifies R2D_SIM_VARIANT.
#
# WORLD (found 09-02 while building this): the r2dreamer adapter picks the Genesis world from the env var
# R2D_SIM_VARIANT (r2dreamer/envs/genesis.py:135; default 'base'). sbatch_r2dreamer.sh exports it for the
# training + in-job evals (the runs' run.log and eval.log carry "[sim-variant] gc_kp4_riser3_shelf6 ...").
# NONE of the 2289 per-episode logs under baselines/outputs/n12_rescore/ carry that line: every r2d
# RESCORE-RESULT of record (the corrected-world WM block's hold/rnd headlines included) was scored in the
# BASE world with policies trained in the corrected world. This script exports the variant explicitly and
# REFUSES (counts as wrongworld, never averages in) any episode whose log lacks the adapter's line.
#
# Denominators (audit finding 23): RESCORE5-RESULT prints picked/present, expected, missing and wrongworld
# lists; a line with present < expected is INVALID and must be rerun, never averaged.
#
#   RUNDIR=<run dir> CK=<checkpoint .pt> TAG=<label> MODE=sample|mode OUT=<dir> \
#   [SIM_VARIANT=gc_kp4_riser3_shelf6] [MAXS=1200] [PAR=4] [ICF=baselines/eval_ics.json] \
#   sbatch [--output ...] cluster/r2d_rescore5.sh
#
# Protocol otherwise identical to cluster/r2d_rescore.sh: --seed 0, --device cpu, --max-steps 1200 (sim
# steps), config = the run's own .hydra/config.yaml (action_repeat/delta_joint from the run), per-episode
# metrics.json under OUT/<TAG>_<MODE>_<set>_<k>/. --torch-threads 2 (PAR 4 x 2 = the 8 cores; r2d_rescore.sh
# left the default 4 -> 16 threads on 8 cores) and --scale 1 (64px mp4 instead of 256px) are cost-only.
#SBATCH -J r2dbest5
#SBATCH -p batch
#SBATCH -N 1
#SBATCH -c 8
#SBATCH --mem=24g
#SBATCH --time=4:00:00
#SBATCH --exclude=pax007
#SBATCH --output=r2dbest5_%j.out
#SBATCH --error=r2dbest5_%j.out
set -uo pipefail
LAB=/cluster/tufts/shortlab/jstale02
GPR=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace}
R2D=${R2D_ROOT:-$LAB/r2dreamer}; PY=${VENV_PY:-$LAB/r2d_venv/bin/python}
: "${RUNDIR:?RUNDIR required}"; : "${CK:?CK required}"; : "${TAG:?TAG required}"; : "${MODE:?MODE required (sample|mode)}"; : "${OUT:?OUT required}"
SIM_VARIANT=${SIM_VARIANT:-gc_kp4_riser3_shelf6}; MAXS=${MAXS:-1200}; PAR=${PAR:-4}; ICF=${ICF:-$GPR/baselines/eval_ics.json}
case "$MODE" in sample|mode) ;; *) echo "FATAL: MODE must be sample|mode, got '$MODE'"; exit 1 ;; esac
CFG="$RUNDIR/.hydra/config.yaml"
[ -f "$CK" ]  || { echo "FATAL: checkpoint $CK missing"; exit 1; }
[ -f "$CFG" ] || { echo "FATAL: no .hydra/config.yaml in $RUNDIR"; exit 1; }
[ -f "$ICF" ] || { echo "FATAL: IC file $ICF missing"; exit 1; }
# TMPDIR on the shared fs (batch nodes' node-local /tmp can be full: rlpd_select_confirm.sh, job 3162556)
export TMPDIR=${TMPDIR_OVERRIDE:-$LAB/tmp/r2dbest5_${SLURM_JOB_ID:-$$}}; mkdir -p "$TMPDIR" 2>/dev/null || export TMPDIR=/tmp
trap 'rm -rf "${TMPDIR:?}" 2>/dev/null' EXIT
export GENESIS_PICKAPLACE_ROOT="$GPR" MUJOCO_GL=egl CUDA_VISIBLE_DEVICES="" R2D_SIM_VARIANT="$SIM_VARIANT"
mkdir -p "$OUT"
CK_STEP=$("$PY" -c "import torch,sys; print(torch.load(sys.argv[1], map_location='cpu', weights_only=False).get('step'))" "$CK" 2>/dev/null | tail -1)
echo "== r2dbest5 tag=$TAG ck=$CK ck_step=${CK_STEP:-NA} mode=$MODE variant=$SIM_VARIANT icf=$ICF max_steps=$MAXS par=$PAR node=$(hostname) job=${SLURM_JOB_ID:-none} $(date)"
rc_total=0
for SET in hold rnd; do
  NEP=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))[sys.argv[2]]))" "$ICF" "$SET")
  for k in $(seq 0 $((NEP-1))); do
    while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 2; done
    ( cd "$R2D" && "$PY" eval_genesis.py --checkpoint "$CK" --config "$CFG" --ic-file "$ICF" \
        --ic-set "$SET" --ic-index "$k" --max-steps "$MAXS" --mode "$MODE" --seed 0 --device cpu \
        --torch-threads 2 --scale 1 \
        --out "$OUT/${TAG}_${MODE}_${SET}_${k}" > "$OUT/${TAG}_${MODE}_${SET}_${k}.log" 2>&1 ) &
  done
  wait
  n_ok=0; n_pick=0; missing=(); wrongworld=()
  for k in $(seq 0 $((NEP-1))); do
    L="$OUT/${TAG}_${MODE}_${SET}_${k}.log"; M="$OUT/${TAG}_${MODE}_${SET}_${k}/metrics.json"
    if ! grep -aq "^\[sim-variant\] $SIM_VARIANT:" "$L" 2>/dev/null; then wrongworld+=("$k"); continue; fi
    [ -f "$M" ] || { missing+=("$k"); continue; }
    P=$(python3 -c "
import json,sys
d=json.load(open(sys.argv[1]))
assert d.get('mode')==sys.argv[2], ('mode', d.get('mode'))
v=d.get('picked', (d.get('metrics') or {}).get('eval/picked'))
print(1 if (v is True or (isinstance(v,(int,float)) and float(v)>0.5)) else 0)" "$M" "$MODE" 2>/dev/null || echo ERR)
    [ "$P" = "ERR" ] && { missing+=("$k"); continue; }
    n_ok=$((n_ok+1)); n_pick=$((n_pick+P))
  done
  [ "${#missing[@]}" -gt 0 ] || [ "${#wrongworld[@]}" -gt 0 ] && rc_total=1
  echo "RESCORE5-RESULT tag=$TAG mode=$MODE variant=$SIM_VARIANT ck=$(basename "$CK") ck_step=${CK_STEP:-NA} set=$SET picked=$n_pick/$n_ok expected=$NEP max_steps=$MAXS missing=[${missing[*]:-}] wrongworld=[${wrongworld[*]:-}] node=$(hostname)"
done
echo "== r2dbest5 done tag=$TAG mode=$MODE rc=$rc_total $(date)"
exit $rc_total
