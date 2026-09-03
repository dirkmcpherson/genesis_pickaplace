#!/bin/bash
# DP one-harness post-train stage, re-runnable standalone (extracted from sbatch_dp.sh 08-23 so a
# finished training can be (re)scored without retraining):
#   K numbered checkpoints under $OUT/checkpoints/<step>/pretrained_model (each with dp_sidecar.json)
#   -> each scored on `sel` (cluster/eval_sweep.sh dp ..., fresh process per episode, GPU policy if
#   visible) -> best (ties -> later) CONFIRMED on hold+rnd -> final also on hold+rnd -> DP-HEADLINE.
# Usage: OUT=baselines/outputs/dp_pilot/dH_DP_s0 ARM=dH SEED=0 RUN_NAME=dH_DP-pilot_s0 [PROJ=genesis_paper]
#        [ACTION_REPEAT=4 EVAL_HORIZON=1200 IC_FILE=baselines/eval_ics.json EVAL_CPUS=12 REDO=0]
#        bash cluster/dp_select_confirm.sh
# REDO=1 wipes $OUT/sweep first (default keeps finished per-episode jsons: eval_sweep is resume-safe).
set -uo pipefail
: "${OUT:?OUT required}"; : "${ARM:?ARM required}"; : "${SEED:?SEED required}"; : "${RUN_NAME:?RUN_NAME required}"
PROJ=${PROJ:-genesis_paper}; ACTION_REPEAT=${ACTION_REPEAT:-4}; EVAL_HORIZON=${EVAL_HORIZON:-1200}
IC_FILE=${IC_FILE:-baselines/eval_ics.json}; NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
export EVAL_CPUS=${EVAL_CPUS:-}
SWEEP_COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --wandb-run "$RUN_NAME" --wandb-project "$PROJ")
mapfile -t CK_DIRS < <(ls -d "$OUT"/checkpoints/[0-9]*/ 2>/dev/null | sort -V)
[ "${#CK_DIRS[@]}" -gt 0 ] || { echo "FATAL: no numbered checkpoints under $OUT/checkpoints"; exit 1; }
for D in "${CK_DIRS[@]}"; do [ -f "$D/dp_sidecar.json" ] || { echo "FATAL: $D/dp_sidecar.json missing (sbatch_dp.sh writes it post-train)"; exit 1; }; done
SW=$OUT/sweep; [ "${REDO:-0}" = "1" ] && rm -rf "$SW"; mkdir -p "$SW"
echo "== dp_select_confirm OUT=$OUT arm=$ARM seed=$SEED ckpts=${#CK_DIRS[@]} node=$NODE_CLASS cuda=${CUDA_VISIBLE_DEVICES-unset} eval_cpus=${EVAL_CPUS:-auto} $(date)"
BEST_TAG=""; BEST_CK=""; BEST_N=-1
for D in "${CK_DIRS[@]}"; do
  STEP=$(basename "$D"); TAG="ckpt_$STEP"; C="$D/pretrained_model"
  [ -d "$C" ] || { echo "WARN: $C missing, skipping"; continue; }
  bash cluster/eval_sweep.sh dp "$C" "$SW/$STEP" --sets sel --ckpt-step "$STEP" --tag "$TAG" "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/$STEP.log"
  N=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); s=d["sets"]["sel"]; print(s["picked"] if s["n_present"]==s["n_expected"] else -1)' "$SW/$STEP/sweep.json" 2>/dev/null || echo -1)
  echo "SELECT-RESULT arm=$ARM seed=$SEED ckpt=$STEP sel=$N/15"
  if [ "$N" -ge "$BEST_N" ]; then BEST_N=$N; BEST_TAG=$STEP; BEST_CK=$C; fi   # ties -> later checkpoint
done
FINAL_D=${CK_DIRS[-1]}; FINAL_TAG=$(basename "$FINAL_D"); FINAL_CK="$FINAL_D/pretrained_model"
echo "== selected $BEST_TAG (sel=$BEST_N/15); final=$FINAL_TAG"
bash cluster/eval_sweep.sh dp "$BEST_CK" "$SW/selected" --sets hold,rnd --ckpt-step "$BEST_TAG" --tag selected "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/selected.log"
if [ "$BEST_CK" = "$FINAL_CK" ]; then FINAL_SWEEP="$SW/selected/sweep.json"
else
  bash cluster/eval_sweep.sh dp "$FINAL_CK" "$SW/final" --sets hold,rnd --ckpt-step "$FINAL_TAG" --tag final "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/final.log"
  FINAL_SWEEP="$SW/final/sweep.json"
fi
python3 - "$SW/selected/sweep.json" "$FINAL_SWEEP" "$ARM" "$SEED" "$ACTION_REPEAT" "$BEST_TAG" "$BEST_N" "$FINAL_TAG" "$RUN_NAME" "$PROJ" "$NODE_CLASS" <<'PY'
import json, sys
sel_j, fin_j, arm, seed, rep, best, best_n, final_tag, run_name, proj, node = sys.argv[1:12]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception:
        return 'MISSING'
line = (f'DP-HEADLINE arm={arm} seed={seed} repeat={rep} selected={best} sel={best_n}/15 hold={rd(sel_j,"hold")} '
        f'rnd={rd(sel_j,"rnd")} final={final_tag} final_hold={rd(fin_j,"hold")} final_rnd={rd(fin_j,"rnd")} node={node}')
print(line)
open(sys.argv[1].rsplit('/', 2)[0] + '/HEADLINE.txt', 'w').write(line + '\n')
try:
    import wandb
    api = wandb.Api(timeout=60)
    runs = list(api.runs(f'jambotime/{proj}', filters={'display_name': run_name}))
    if runs:
        run = sorted(runs, key=lambda x: x.created_at)[-1]
        run.summary['sweep/headline'] = line; run.summary['sweep/selected_ckpt'] = best; run.summary['sweep/node'] = node
        run.summary.update(); print(f'wandb: pushed sweep/headline to {run.id}')
    else:
        print(f'WARN: wandb headline push skipped -- no run named {run_name!r}')
except Exception as e:
    print(f'WARN: wandb headline push failed ({type(e).__name__}: {e}) -- DP-HEADLINE line is authoritative')
PY
echo "SELECT-CONFIRM DONE $(date)"
