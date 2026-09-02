#!/bin/bash
# RLPD one-harness post-train stage, re-runnable standalone (extracted VERBATIM from
# cluster/sbatch_rlpd.sh lines ~245-302 on 09-02, so a finished training can be (re)scored
# without retraining -- the RLPD twin of cluster/dp_select_confirm.sh).
#
# WHY: the 09-02 v2 human-arm wave (dHv2raw, seeds 60-67, waves g99v2full / g99v2fullw3, jobs
# 3147221-36) finished TRAINING (K=5 archived ckpt_020..ckpt_100 + rlpd_final.zip) but died in
# train_rlpd.py's final log flush with `OSError: [Errno 116] Stale file handle` (a wandb log
# handle broken by the wandb_cache purge -- infrastructure, not code), so the launcher never
# reached its selection/confirmation block. This script IS that block, CPU-only (the eval is
# one fresh CPU process per episode; sac policy stays on CPU).
#
# Stage logic (identical to the launcher):
#   K archived checkpoints <RUN>/ckpt_*/rlpd_ckpt.zip (+ .action_mode.json sidecar) -> each
#   scored on `sel` (cluster/eval_sweep.sh sac ...) -> best by sel picked (ties -> LATER ckpt)
#   CONFIRMED on hold+rnd (tag selected) -> final (= last ckpt dir) also on hold+rnd (tag
#   final; skipped + copied when selected == final) -> ONE SWEEP-HEADLINE line in the
#   launcher's exact format (+ best-effort wandb summary push) -> <RUN>/sweep/HEADLINE.txt
#   (the launcher itself does not write the file; mirrored from dp_select_confirm.sh so the
#   harvest reads are uniform).
#
# Usage (env, no silent defaults for anything that changes the world or the denominators):
#   RUN=baselines/rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s60 ARM=dHv2raw SEED=60 WAVE=g99v2fullw3 \
#   REWARD=sparse IC_FILE=baselines/eval_ics_v2_w3.json [ACTION_REPEAT=4 EVAL_HORIZON=1200] \
#   [PROJ=genesis_paper] [REDO=0] [DRYRUN=1] [CONDA_ENV=...] \
#   sbatch cluster/rlpd_select_confirm.sh          # or: bash ... (DRYRUN=1 needs no conda)
#   REDO=1 wipes $RUN/sweep first (default keeps finished per-episode jsons: eval_sweep is resume-safe).
#
# Submit (from the checkout root, AFTER git pull; one batch CPU job per run):
#   for S in $(seq 60 67); do
#     RUN=baselines/rl/checkpoints/rlpd_g99v2full_dHv2raw_s$S   ARM=dHv2raw SEED=$S WAVE=g99v2full   REWARD=sparse IC_FILE=baselines/eval_ics_v2.json    sbatch cluster/rlpd_select_confirm.sh
#     RUN=baselines/rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s$S ARM=dHv2raw SEED=$S WAVE=g99v2fullw3 REWARD=sparse IC_FILE=baselines/eval_ics_v2_w3.json sbatch cluster/rlpd_select_confirm.sh
#   done
# Guards: RUN basename must equal rlpd_${WAVE}_${ARM}_s${SEED}[_dense] (the launcher's OUT);
# the IC file's sim_variant must equal the last checkpoint sidecar's sim_variant (world match);
# every ckpt_* dir must carry its sidecar (eval_sweep FATALs otherwise -- no SIDECAR_OPTIONAL).
# Cost: ~3.5 min per sel sweep + ~30 min per hold+rnd confirmation at max_jobs=4 (-c 8), i.e.
# 50-80 min per run (09-01 w3 s51 in-job timings).

#SBATCH -J rlpdsel
#SBATCH -p batch
#SBATCH -N 1
#SBATCH -c 8
#SBATCH --mem=24g
#SBATCH --time=6:00:00
#SBATCH --output=rlpdsel_%j.out
#SBATCH --error=rlpdsel_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl

RUN=${RUN:?set RUN (the launcher OUT dir, e.g. baselines/rl/checkpoints/rlpd_g99v2full_dHv2raw_s60)}
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}; WAVE=${WAVE:?set WAVE}
REWARD=${REWARD:?set REWARD (sparse|dense)}; IC_FILE=${IC_FILE:?set IC_FILE (v2 waves: baselines/eval_ics_v2.json old world / baselines/eval_ics_v2_w3.json corrected)}
ACTION_REPEAT=${ACTION_REPEAT:-4}; EVAL_HORIZON=${EVAL_HORIZON:-1200}; PROJ=${PROJ:-genesis_paper}
case "$REWARD" in sparse|dense) ;; *) echo "FATAL: REWARD must be sparse|dense, got '$REWARD'"; exit 1 ;; esac
SHAPE_SUFFIX=""; RUN_SUFFIX=""
if [ "$REWARD" = dense ]; then SHAPE_SUFFIX="_dense"; RUN_SUFFIX="-dense"; fi
OUT=$RUN
RUN_NAME="${ARM}_RLPD-${WAVE}_s${SEED}${RUN_SUFFIX}"
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"

# ---- guards ---------------------------------------------------------------------------
EXPECT_BASE="rlpd_${WAVE}_${ARM}_s${SEED}${SHAPE_SUFFIX}"
[ "$(basename "$OUT")" = "$EXPECT_BASE" ] || { echo "FATAL: RUN basename $(basename "$OUT") != launcher OUT name $EXPECT_BASE (ARM/SEED/WAVE/REWARD mismatch)"; exit 1; }
[ -d "$OUT" ] || { echo "FATAL: run dir $OUT missing"; exit 1; }
[ -f "$IC_FILE" ] || { echo "FATAL: IC file $IC_FILE missing"; exit 1; }
FINAL_CK=$OUT/rlpd_final.zip
[ -f "$FINAL_CK" ] || { echo "FATAL: no final checkpoint at $FINAL_CK (training did not finish -- this stage never retrains)"; exit 1; }
mapfile -t CK_DIRS < <(ls -d "$OUT"/ckpt_*/ 2>/dev/null | sort -V)
for D in "${CK_DIRS[@]}"; do
  Z=$(ls "$D"/*.zip 2>/dev/null | head -1)
  [ -n "$Z" ] && [ -f "${Z%.zip}.action_mode.json" ] || { echo "FATAL: $D lacks rlpd_ckpt.zip + .action_mode.json sidecar (eval_sweep needs both; no SIDECAR_OPTIONAL for the block of record)"; exit 1; }
done
# world match: IC file sim_variant vs the last checkpoint sidecar (the launcher trusts the
# submitter's IC_FILE; here the sidecar is the record of the world the run trained in)
if [ "${#CK_DIRS[@]}" -gt 0 ]; then
  LAST_SC=$(ls "${CK_DIRS[-1]}"/*.action_mode.json | head -1)
  python3 - "$IC_FILE" "$LAST_SC" <<'PY' || exit 1
import json, sys
ic, sc = [json.load(open(p)) for p in sys.argv[1:3]]
a = str(ic.get('sim_variant') or 'base'); b = str(sc.get('sim_variant') or 'base')
if a != b:
    print(f'FATAL: IC file sim_variant={a} but checkpoint sidecar sim_variant={b} (world mismatch: wrong IC_FILE for this run)'); sys.exit(1)
print(f'WORLD-OK sim_variant={a} ic_file={sys.argv[1]} sidecar={sys.argv[2]} sel={len(ic["sel"])} hold={len(ic["hold"])} rnd={len(ic["rnd"])}')
PY
fi

SWEEP_COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --reward "$REWARD"
              --wandb-run "$RUN_NAME" --wandb-project "$PROJ")

if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] RUN=$OUT ARM=$ARM SEED=$SEED WAVE=$WAVE REWARD=$REWARD ACTION_REPEAT=$ACTION_REPEAT EVAL_HORIZON=$EVAL_HORIZON IC_FILE=$IC_FILE RUN_NAME=$RUN_NAME NODE=$NODE_CLASS"
  echo "[dry] candidates (${#CK_DIRS[@]}): $(for D in "${CK_DIRS[@]}"; do basename "$D"; done | tr '\n' ' ')final=$FINAL_CK"
  echo "[dry] selection: for C in ${CK_DIRS[*]}: bash cluster/eval_sweep.sh sac <C>/*.zip $OUT/sweep/<C> --sets sel --ckpt-step <C> --tag <C> ${SWEEP_COMMON[*]}"
  echo "[dry] confirmation: bash cluster/eval_sweep.sh sac <SELECTED>.zip $OUT/sweep/selected --sets hold,rnd --tag selected ${SWEEP_COMMON[*]}; same for <FINAL> --tag final"
  echo "[dry] headline: SWEEP-HEADLINE arm=$ARM seed=$SEED reward=$REWARD repeat=$ACTION_REPEAT selected=<C> sel=a/15 hold=b/N rnd=c/30 final_hold=d/N final_rnd=e/30 -> $OUT/sweep/HEADLINE.txt"
  exit 0
fi

echo "== rlpd_select_confirm RUN=$OUT arm=$ARM seed=$SEED wave=$WAVE reward=$REWARD ckpts=${#CK_DIRS[@]} ic=$IC_FILE node=$NODE_CLASS host=$(hostname) $(date)"

module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
python -c 'import stable_baselines3' 2>/dev/null || { echo "FATAL: stable_baselines3 not importable in the env (the launcher pip-installs it; not done here)"; exit 1; }

# ======== below: cluster/sbatch_rlpd.sh post-train block, verbatim except the HEADLINE.txt write ========
# ---- K=5 archived checkpoints (train_rlpd.py writes <OUT>/ckpt_20..ckpt_100, each one
# .zip + its .action_mode.json sidecar); fallback = rlpd_final.zip only (short smokes or a
# trainer predating the archive) -- loudly.
CANDS=()   # "tag|zip"
for D in "${CK_DIRS[@]}"; do
  Z=$(ls "$D"/*.zip 2>/dev/null | head -1)
  [ -n "$Z" ] && CANDS+=("$(basename "$D")|$Z")
done
if [ "${#CANDS[@]}" -eq 0 ]; then
  echo "WARN: no archived ckpt_* dirs under $OUT -- selection degenerates to the final checkpoint only"
  CANDS+=("final|$FINAL_CK")
fi
SW=$OUT/sweep; [ "${REDO:-0}" = "1" ] && rm -rf "$SW"; mkdir -p "$SW"
set +e   # evals must never fail an already-trained job; every stage reports its rc
BEST_TAG=""; BEST_ZIP=""; BEST_N=-1
for C in "${CANDS[@]}"; do
  TAG=${C%%|*}; Z=${C#*|}
  bash cluster/eval_sweep.sh sac "$Z" "$SW/$TAG" --sets sel --ckpt-step "$TAG" --tag "$TAG" "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/$TAG.log"
  N=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); s=d["sets"]["sel"]; print(s["picked"] if s["n_present"]==s["n_expected"] else -1)' "$SW/$TAG/sweep.json" 2>/dev/null || echo -1)
  echo "SELECT-RESULT arm=$ARM seed=$SEED reward=$REWARD ckpt=$TAG sel=$N/15"
  if [ "$N" -ge "$BEST_N" ]; then BEST_N=$N; BEST_TAG=$TAG; BEST_ZIP=$Z; fi   # ties -> later checkpoint
done
FINAL_TAG=${CANDS[-1]%%|*}; FINAL_ZIP=${CANDS[-1]#*|}
echo "== selected $BEST_TAG (sel=$BEST_N/15); final=$FINAL_TAG"
bash cluster/eval_sweep.sh sac "$BEST_ZIP" "$SW/selected" --sets hold,rnd --ckpt-step "$BEST_TAG" --tag selected "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/selected.log"
if [ "$BEST_ZIP" = "$FINAL_ZIP" ]; then
  cp "$SW/selected/sweep.json" "$SW/final_sweep.json" 2>/dev/null
  FINAL_SWEEP="$SW/selected/sweep.json"
else
  bash cluster/eval_sweep.sh sac "$FINAL_ZIP" "$SW/final" --sets hold,rnd --ckpt-step "$FINAL_TAG" --tag final "${SWEEP_COMMON[@]}" 2>&1 | tee "$SW/final.log"
  FINAL_SWEEP="$SW/final/sweep.json"
fi
python3 - "$SW/selected/sweep.json" "$FINAL_SWEEP" "$ARM" "$SEED" "$REWARD" "$ACTION_REPEAT" "$BEST_TAG" "$BEST_N" "$FINAL_TAG" "$RUN_NAME" "$NODE_CLASS" "$PROJ" <<'PY'
import json, sys
sel_j, fin_j, arm, seed, reward, rep, best, best_n, final_tag, run_name, node, proj = sys.argv[1:13]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception as e:
        return 'MISSING'
line = (f'SWEEP-HEADLINE arm={arm} seed={seed} reward={reward} repeat={rep} selected={best} sel={best_n}/15 '
        f'hold={rd(sel_j,"hold")} rnd={rd(sel_j,"rnd")} final={final_tag} final_hold={rd(fin_j,"hold")} final_rnd={rd(fin_j,"rnd")} node={node}')
print(line)
open(sys.argv[1].rsplit('/', 2)[0] + '/HEADLINE.txt', 'w').write(line + '\n')   # <RUN>/sweep/HEADLINE.txt (dp_select_confirm.sh convention)
try:
    import wandb
    api = wandb.Api(timeout=60)
    runs = list(api.runs(f'jambotime/{proj}', filters={'display_name': run_name}))
    if runs:
        run = sorted(runs, key=lambda x: x.created_at)[-1]
        run.summary['sweep/headline'] = line; run.summary['sweep/selected_ckpt'] = best
        run.summary['sweep/node'] = node; run.summary.update()
        print(f'wandb: pushed sweep/headline to {run.id}')
    else:
        print(f'WARN: wandb headline push skipped -- no run named {run_name!r}')
except Exception as e:
    print(f'WARN: wandb headline push failed ({type(e).__name__}: {e}) -- SWEEP-HEADLINE line is authoritative')
PY
echo "SELECT-CONFIRM DONE $(date)"
