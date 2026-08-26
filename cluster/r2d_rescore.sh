#!/bin/bash
# Re-score a trained r2dreamer checkpoint on the SHARED IC file, one fresh process per episode.
#
# WHY (paper/CRITIQUE_decisions_2026-08-26.md E1): the N12 world-model cells were selected AND
# reported on `sel`, while the RLPD rows they are compared against report `hold` -- disjoint uid
# sets, and `sel` carries a structural 14/15 ceiling (N11, uid 234). PREREG §5 requires the
# headline to be the selected checkpoint on hold+rnd. This re-scores existing BEST checkpoints.
#
#   RUNDIR=<r2dreamer run dir> SET=hold|rnd OUT=<dir> [MAXS=1200] [PAR=4] bash cluster/r2d_rescore.sh
#
# Writes OUT/<tag>_<set>_<k>/metrics.json per episode and prints RESCORE-RESULT with an ASSERTED
# denominator. Exits non-zero if any episode failed -- the first version of this loop printed
# "RESCORE-DONE" over 540 crashed processes because it never checked an exit code (silent-default
# bug family, 7th sighting). Requires GENESIS_PICKAPLACE_ROOT: the patched r2dreamer adapter
# imports sim_variant_hook from $GENESIS_PICKAPLACE_ROOT/baselines and dies without it.
set -uo pipefail
: "${RUNDIR:?RUNDIR required}"; : "${SET:?SET required (hold|rnd)}"; : "${OUT:?OUT required}"
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
GPR=${GENESIS_PICKAPLACE_ROOT:-$LAB/genesis_pickaplace}
R2D=${R2D_ROOT:-$LAB/r2dreamer}; PY=${VENV_PY:-$LAB/r2d_venv/bin/python}
MAXS=${MAXS:-1200}; PAR=${PAR:-4}; ICF=${ICF:-$GPR/baselines/eval_ics.json}
TAG=$(basename "$RUNDIR"); CK="$RUNDIR/BEST_selected.pt"; CFG="$RUNDIR/.hydra/config.yaml"
[ -f "$CK" ]  || { echo "FATAL: no BEST_selected.pt in $RUNDIR"; exit 1; }
[ -f "$CFG" ] || { echo "FATAL: no .hydra/config.yaml in $RUNDIR"; exit 1; }
NEP=$(python3 -c "import json,sys; print(len(json.load(open(sys.argv[1]))[sys.argv[2]]))" "$ICF" "$SET")
mkdir -p "$OUT"; export GENESIS_PICKAPLACE_ROOT="$GPR" MUJOCO_GL=egl CUDA_VISIBLE_DEVICES=""
echo "[rescore] $TAG set=$SET n=$NEP max_steps=$MAXS par=$PAR ck=$CK"
rc_total=0
for k in $(seq 0 $((NEP-1))); do
  while [ "$(jobs -rp | wc -l)" -ge "$PAR" ]; do sleep 2; done
  ( cd "$R2D" && "$PY" eval_genesis.py --checkpoint "$CK" --config "$CFG" --ic-file "$ICF" \
      --ic-set "$SET" --ic-index "$k" --max-steps "$MAXS" --device cpu \
      --out "$OUT/${TAG}_${SET}_${k}" > "$OUT/${TAG}_${SET}_${k}.log" 2>&1 ) &
done
wait
n_ok=0; n_pick=0; missing=()
for k in $(seq 0 $((NEP-1))); do
  M=$(ls "$OUT/${TAG}_${SET}_${k}"/metrics.json 2>/dev/null | head -1)
  if [ -z "$M" ]; then missing+=("$k"); continue; fi
  P=$(python3 -c "
import json,sys
d=json.load(open(sys.argv[1]))
v=d.get('picked', (d.get('metrics') or {}).get('eval/picked'))
print(1 if (v is True or (isinstance(v,(int,float)) and float(v)>0.5)) else 0)" "$M" 2>/dev/null || echo ERR)
  [ "$P" = "ERR" ] && { missing+=("$k"); continue; }
  n_ok=$((n_ok+1)); n_pick=$((n_pick+P))
done
if [ "${#missing[@]}" -gt 0 ]; then
  echo "RESCORE-INCOMPLETE $TAG $SET missing=${missing[*]} -- denominators below count PRESENT episodes only"
  rc_total=1
fi
echo "RESCORE-RESULT tag=$TAG set=$SET picked=$n_pick/$n_ok expected=$NEP max_steps=$MAXS"
exit $rc_total
