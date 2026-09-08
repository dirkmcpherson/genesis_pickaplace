#!/bin/bash
# DP pick checkpoints of record on the in-training-distribution start set `spots60` (PHASE_PLAN amendment (k),
# 2026-09-07; baselines/eval_ics_spots60.json: 60 starts, rnd schema). CPU only, one job per run dir, NEW dirs only:
#   <OUT>/sweep/selected_spots60/   the SELECTED checkpoint (the DP headline of record; step from sweep/HEADLINE.txt)
#   <OUT>/sweep/final_spots60/      the LAST checkpoint (100000) when it still exists on disk and differs from the
#                                   selected one (the dHv2raw arm's pruned runs keep only the selected checkpoint;
#                                   a missing LAST is reported as LAST-MISSING, never substituted)
#   <OUT>/sweep/HEADLINE_spots60.txt
# Uses cluster/eval_sweep.sh dp conventions (one fresh process per episode, DP's native sampling seeded by the episode
# index, 1200 sim steps, hold-4 execution from the checkpoint sidecar). On a CPU node the DDPM query runs on the CPU
# (~4 min per episode; 60 episodes at 4 parallel ~ 1 h per checkpoint).
# Usage: OUT=/abs/.../baselines/outputs/dp_w2final/dH_DP_s20 ARM=dH SEED=20 [SPOTS_FILE=baselines/eval_ics_spots60.json]
#        [SPOTS_SET=spots60] [REDO=0] [DRYRUN=1] sbatch -J dpspots_dH_s20 cluster/dp_eval_spots60.sh
#SBATCH -J dpspots
#SBATCH -p preempt
#SBATCH --qos=preempt
#SBATCH --requeue
#SBATCH --nice=2000
#SBATCH -N 1
#SBATCH -c 8
#SBATCH --mem=24g
#SBATCH --time=6:00:00
#SBATCH --output=dpspots_%j.out
#SBATCH --error=dpspots_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
export TMPDIR=${TMPDIR_OVERRIDE:-/cluster/tufts/shortlab/jstale02/tmp/dpspots_${SLURM_JOB_ID:-$$}}; mkdir -p "$TMPDIR" 2>/dev/null || export TMPDIR=/tmp
trap 'rm -rf "${TMPDIR:?}" 2>/dev/null' EXIT
OUT=${OUT:?set OUT (DP run dir)}; ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}
SPOTS_FILE=${SPOTS_FILE:-baselines/eval_ics_spots60.json}; SPOTS_SET=${SPOTS_SET:-spots60}
EVAL_HORIZON=${EVAL_HORIZON:-1200}; NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
[ -d "$OUT" ] || { echo "FATAL: run dir $OUT missing"; exit 1; }
[ -f "$SPOTS_FILE" ] || { echo "FATAL: $SPOTS_FILE missing"; exit 1; }
HL=$OUT/sweep/HEADLINE.txt; [ -f "$HL" ] || { echo "FATAL: $HL missing (no headline of record)"; exit 1; }
SEL=$(grep -o "selected=[0-9]*" "$HL" | head -1 | cut -d= -f2); FIN=$(grep -o "final=[0-9]*" "$HL" | head -1 | cut -d= -f2)
[ -n "$SEL" ] && [ -n "$FIN" ] || { echo "FATAL: cannot parse selected/final from $HL"; exit 1; }
SEL_CK=$OUT/checkpoints/$SEL/pretrained_model; FIN_CK=$OUT/checkpoints/$FIN/pretrained_model
[ -d "$SEL_CK" ] && [ -f "$OUT/checkpoints/$SEL/dp_sidecar.json" ] || { echo "FATAL: selected checkpoint $SEL_CK (+ sidecar) missing"; exit 1; }
DO_FIN=0
if [ "$FIN" != "$SEL" ]; then
  if [ -d "$FIN_CK" ] && [ -f "$OUT/checkpoints/$FIN/dp_sidecar.json" ]; then DO_FIN=1; else echo "LAST-MISSING $OUT: final=$FIN checkpoint pruned from disk; only the selected ($SEL) checkpoint is evaluated"; fi
fi
echo "RECORD-OK $HL"
SW=$OUT/sweep; [ "${REDO:-0}" = 1 ] && rm -rf "$SW/selected_spots60" "$SW/final_spots60"
COMMON=(--ic-file "$SPOTS_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED")
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] OUT=$OUT ARM=$ARM SEED=$SEED selected=$SEL final=$FIN do_final=$DO_FIN NODE=$NODE_CLASS"
  echo "[dry] selected: bash cluster/eval_sweep.sh dp $SEL_CK $SW/selected_spots60 --sets $SPOTS_SET --ckpt-step $SEL --tag selected_spots60 ${COMMON[*]}"
  [ "$DO_FIN" = 1 ] && echo "[dry] final:    bash cluster/eval_sweep.sh dp $FIN_CK $SW/final_spots60 --sets $SPOTS_SET --ckpt-step $FIN --tag final_spots60 ${COMMON[*]}"
  exit 0
fi
echo "== dp_eval_spots60 OUT=$OUT arm=$ARM seed=$SEED selected=$SEL final=$FIN do_final=$DO_FIN node=$NODE_CLASS host=$(hostname) $(date)"
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
python -c 'import lerobot' 2>/dev/null || { echo "FATAL: lerobot not importable"; exit 1; }
set +e
POLICY_CUDA=0 bash cluster/eval_sweep.sh dp "$SEL_CK" "$SW/selected_spots60" --sets "$SPOTS_SET" --ckpt-step "$SEL" --tag selected_spots60 --no-video "${COMMON[@]}" 2>&1 | tee "$SW/selected_spots60.log"
if [ "$DO_FIN" = 1 ]; then
  POLICY_CUDA=0 bash cluster/eval_sweep.sh dp "$FIN_CK" "$SW/final_spots60" --sets "$SPOTS_SET" --ckpt-step "$FIN" --tag final_spots60 --no-video "${COMMON[@]}" 2>&1 | tee "$SW/final_spots60.log"
fi
python3 - "$SW" "$ARM" "$SEED" "$SEL" "$FIN" "$DO_FIN" "$NODE_CLASS" "$SPOTS_SET" <<'PY'
import json, os, sys
sw, arm, seed, sel, fin, do_fin, node, sset = sys.argv[1:9]
def rd(p):
    try:
        r = json.load(open(p))['sets'][sset]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception:
        return 'MISSING'
fin_s = rd(os.path.join(sw, 'final_spots60', 'sweep.json')) if do_fin == '1' else ('=selected' if fin == sel else 'LAST-MISSING')
line = f'DP-SPOTS-HEADLINE arm={arm} seed={seed} selected={sel} sel_{sset}={rd(os.path.join(sw, "selected_spots60", "sweep.json"))} final={fin} final_{sset}={fin_s} node={node}'
print(line); open(os.path.join(sw, 'HEADLINE_spots60.txt'), 'w').write(line + '\n')
PY
echo "DP-SPOTS DONE $(date)"
