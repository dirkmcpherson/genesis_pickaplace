#!/bin/bash
# Repair sweep cells whose per-episode jsons are complete but whose sweep.json is missing, then regenerate the run's
# headline. Observed 2026-09-07: a sweep's parent shell can be killed between `wait` and the aggregation, so a fully
# computed cell is lost (headline reads MISSING) while every episode survives on disk. eval_sweep.sh is resume-safe,
# so re-invoking it skips the episodes and only aggregates -- seconds per cell, nothing is recomputed.
#
# usage: bash cluster/sweep_repair.sh '<run-dir-glob>' ...
#   e.g. bash cluster/sweep_repair.sh '/abs/.../baselines/rl/checkpoints/rlpd_g99v2fullw3_*'
# Only runs whose launcher FINISHED (HEADLINE_sampled.txt present) are touched; FORCE=1 overrides that guard.
set -uo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"; export GENESIS_PICKAPLACE_ROOT
CELLS=(final_sampled:hold,rnd:sampled final_det15:hold:det final_sampled_spots60:spots60:sampled final_det_spots60:spots60:det)
n_ok=0; n_fix=0; n_fail=0; n_head=0
for RUN in "$@"; do
  for D in $RUN; do
    SW=$D/sweep; [ -d "$SW" ] || continue
    if [ ! -f "$SW/HEADLINE_sampled.txt" ] && [ -z "${FORCE:-}" ]; then continue; fi
    CK=$D/ckpt_100/rlpd_ckpt.zip; [ -f "$CK" ] || continue
    ARM=$(basename "$D" | sed -E 's/^rlpd_[^_]+_(.*)_s[0-9]+$/\1/'); SEED=$(basename "$D" | grep -oE 's[0-9]+$' | tr -d s)
    for C in "${CELLS[@]}"; do
      CELL=${C%%:*}; REST=${C#*:}; SETS=${REST%%:*}; MODE=${REST##*:}
      OUT_D=$SW/$CELL; [ -d "$OUT_D" ] || continue
      if [ -f "$OUT_D/sweep.json" ]; then n_ok=$((n_ok+1)); continue; fi
      NJ=$(ls "$OUT_D"/*.json 2>/dev/null | grep -cv 'sweep.json$')
      [ "$NJ" -eq 0 ] && continue
      ICF=baselines/eval_ics.json; [ "$SETS" = spots60 ] && ICF=baselines/eval_ics_spots60.json
      SMP=(); [ "$MODE" = sampled ] && SMP=(--sample-actions)
      echo "REPAIR $OUT_D ($NJ episode jsons, no sweep.json)"
      bash cluster/eval_sweep.sh sac "$CK" "$OUT_D" --sets "$SETS" "${SMP[@]}" --no-video --tag "$CELL" \
        --ic-file "$ICF" --max-steps 1200 --arm "$ARM" --seed "$SEED" --ckpt-step ckpt_100 --reward sparse \
        > "$SW/${CELL}_repair.log" 2>&1
      if [ -f "$OUT_D/sweep.json" ]; then echo "  REPAIRED"; n_fix=$((n_fix+1)); else echo "  FAILED (see $SW/${CELL}_repair.log)"; n_fail=$((n_fail+1)); fi
    done
    # regenerate the headline from whatever sweep.jsons now exist (it was written when cells were missing)
    python3 - "$SW" "$ARM" "$SEED" <<'PY'
import json, os, sys
sw, arm, seed = sys.argv[1:4]
def rd(cell, key):
    f = os.path.join(sw, cell, 'sweep.json')
    if not os.path.exists(f): return 'MISSING'
    r = json.load(open(f))['sets'].get(key)
    if r is None: return 'MISSING'
    return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
p = os.path.join(sw, 'HEADLINE_sampled.txt')
old = open(p).read().strip() if os.path.exists(p) else ''
node = next((t for t in old.split() if t.startswith('node=')), 'node=?')
line = (f'SAMPLED-HEADLINE arm={arm} seed={seed} ckpt=ckpt_100 ic=baselines/eval_ics.json '
        f'smp_hold={rd("final_sampled","hold")} smp_rnd={rd("final_sampled","rnd")} '
        f'det_hold15={rd("final_det15","hold")} smp_spots60={rd("final_sampled_spots60","spots60")} '
        f'det_spots60={rd("final_det_spots60","spots60")} {node}')
if 'MISSING' in old and 'MISSING' not in line:
    line += ' repaired=1'
if line.strip() != old.strip():
    open(p, 'w').write(line + '\n'); print('HEADLINE-UPDATED ' + line)
PY
    n_head=$((n_head+1))
  done
done
echo "SWEEP-REPAIR done: $n_ok cells already complete, $n_fix repaired, $n_fail failed, $n_head headlines checked"
