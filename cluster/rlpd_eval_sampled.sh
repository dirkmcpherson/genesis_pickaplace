#!/bin/bash
# SAMPLED-action re-evaluation of a finished RLPD run's LAST checkpoint (2026-09-07; user: sampled
# actions are the statistic of record for RLPD and r2dreamer). CPU only, one job per run, NEVER
# touches the deterministic cells: everything lands in NEW dirs under <RUN>/sweep/.
#
#   <RUN>/sweep/final_sampled/   hold + rnd of IC_FILE, wandb_eval --sample-actions (one draw per
#                                decision, torch seeded by the episode index k), cluster/eval_sweep.sh
#                                conventions (one fresh process per episode, CPU sim, 1200 sim steps)
#   <RUN>/sweep/final_det15/     hold of IC_FILE, DETERMINISTIC -- the comparator for the sampled hold
#                                cell on the SAME IC set (the cells of record used eval_ics_v2_w3.json
#                                hold = 66 training ICs; hold15 of eval_ics.json is NOT a subset: 294)
#   <RUN>/sweep/final_sampled_spots60/  spots60 of SPOTS_FILE (baselines/eval_ics_spots60.json, PHASE_PLAN amendment (k):
#                                       60 in-training-distribution starts), SAMPLED actions
#   <RUN>/sweep/final_det_spots60/      the same 60 starts, DETERMINISTIC
#   <RUN>/sweep/HEADLINE_sampled.txt   one SAMPLED-HEADLINE line (hold15/rnd30 sampled, hold15 det, spots60 sampled + det)
#
# Usage (from the checkout root; GENESIS_PICKAPLACE_ROOT may point at another checkout of the code):
#   RUN=/abs/path/baselines/rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s60 ARM=dHv2raw SEED=60 \
#     [IC_FILE=baselines/eval_ics.json] [CKPT_TAG=ckpt_100] [REDO=0] [DRYRUN=1] \
#     sbatch -J rlpdsmp_dHv2raw_s60 cluster/rlpd_eval_sampled.sh
# Guards: the checkpoint sidecar must carry action_mode/action_repeat/delta_ref (eval_sweep FATALs
# otherwise); the run's deterministic final cell of record must have used the SAME checkpoint file
# (final_sweep.json / final/sweep.json `ckpt`), else this job refuses -- the sampled cell must be
# the same weights as the deterministic one.
#SBATCH -J rlpdsmp
#SBATCH -p batch
#SBATCH -N 1
#SBATCH -c 8
#SBATCH --mem=24g
#SBATCH --time=6:00:00
#SBATCH --output=rlpdsmp_%j.out
#SBATCH --error=rlpdsmp_%j.out

set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"
export GENESIS_PICKAPLACE_ROOT PYTHONUNBUFFERED=1 MUJOCO_GL=egl
# TMPDIR on the shared fs (batch nodes' node-local /tmp can be full: rlpd_select_confirm.sh, 09-02)
export TMPDIR=${TMPDIR_OVERRIDE:-/cluster/tufts/shortlab/jstale02/tmp/rlpdsmp_${SLURM_JOB_ID:-$$}}; mkdir -p "$TMPDIR" 2>/dev/null || export TMPDIR=/tmp
trap 'rm -rf "${TMPDIR:?}" 2>/dev/null' EXIT

RUN=${RUN:?set RUN (run dir, e.g. baselines/rl/checkpoints/rlpd_g99v2fullw3_dHv2raw_s60)}
ARM=${ARM:?set ARM}; SEED=${SEED:?set SEED}
IC_FILE=${IC_FILE:-baselines/eval_ics.json}
SPOTS_FILE=${SPOTS_FILE:-baselines/eval_ics_spots60.json}; SPOTS_SET=${SPOTS_SET:-spots60}
CKPT_TAG=${CKPT_TAG:-ckpt_100}
EVAL_HORIZON=${EVAL_HORIZON:-1200}
NODE_CLASS="${SLURM_JOB_NODELIST:-$(hostname)}"
CK="$RUN/$CKPT_TAG/rlpd_ckpt.zip"
[ -d "$RUN" ] || { echo "FATAL: run dir $RUN missing"; exit 1; }
[ -f "$CK" ] || { echo "FATAL: checkpoint $CK missing"; exit 1; }
[ -f "${CK%.zip}.action_mode.json" ] || { echo "FATAL: sidecar ${CK%.zip}.action_mode.json missing"; exit 1; }
[ -f "$IC_FILE" ] || { echo "FATAL: IC file $IC_FILE missing"; exit 1; }
[ -f "$SPOTS_FILE" ] || { echo "FATAL: spots IC file $SPOTS_FILE missing"; exit 1; }
# the deterministic final cell of record must be THIS checkpoint (same weights, different action selection)
python3 - "$RUN" "$CKPT_TAG" <<'PY' || exit 1
import json, os, sys
run, tag = sys.argv[1:3]
cands = [os.path.join(run, 'sweep', 'final_sweep.json'), os.path.join(run, 'sweep', 'final', 'sweep.json')]
fs = [f for f in cands if os.path.exists(f)]
if not fs:
    print(f'FATAL: no deterministic final cell of record under {run}/sweep (final_sweep.json | final/sweep.json)'); sys.exit(1)
j = json.load(open(fs[0])); ck = str(j.get('ckpt', ''))
if not ck.replace('//', '/').endswith(f'{tag}/rlpd_ckpt.zip'):
    print(f'FATAL: the deterministic final cell {fs[0]} scored ckpt={ck}, not {tag}/rlpd_ckpt.zip'); sys.exit(1)
sc = json.load(open(os.path.join(run, tag, 'rlpd_ckpt.action_mode.json')))
print(f"RECORD-OK deterministic final cell {fs[0]} ckpt={ck} sim_variant={sc.get('sim_variant')} steps={sc.get('steps')} seed={sc.get('seed')}")
PY
SW=$RUN/sweep
if [ "${REDO:-0}" = "1" ]; then rm -rf "$SW/final_sampled" "$SW/final_det15" "$SW/final_sampled_spots60" "$SW/final_det_spots60"; fi
SPOTS_COMMON=(--ic-file "$SPOTS_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --ckpt-step "$CKPT_TAG" --reward sparse)
COMMON=(--ic-file "$IC_FILE" --max-steps "$EVAL_HORIZON" --arm "$ARM" --seed "$SEED" --ckpt-step "$CKPT_TAG" --reward sparse)
if [ -n "${DRYRUN:-}" ]; then
  echo "[dry] RUN=$RUN ARM=$ARM SEED=$SEED CK=$CK IC_FILE=$IC_FILE NODE=$NODE_CLASS"
  echo "[dry] sampled: bash cluster/eval_sweep.sh sac $CK $SW/final_sampled --sets hold,rnd --sample-actions --tag final_sampled ${COMMON[*]}"
  echo "[dry] det15:   bash cluster/eval_sweep.sh sac $CK $SW/final_det15 --sets hold --tag final_det15 ${COMMON[*]}"
  echo "[dry] spots60 sampled: bash cluster/eval_sweep.sh sac $CK $SW/final_sampled_spots60 --sets $SPOTS_SET --sample-actions --tag final_sampled_spots60 ${SPOTS_COMMON[*]}"
  echo "[dry] spots60 det:     bash cluster/eval_sweep.sh sac $CK $SW/final_det_spots60 --sets $SPOTS_SET --tag final_det_spots60 ${SPOTS_COMMON[*]}"
  exit 0
fi
echo "== rlpd_eval_sampled RUN=$RUN arm=$ARM seed=$SEED ckpt=$CK ic=$IC_FILE node=$NODE_CLASS host=$(hostname) $(date)"
module load anaconda/2025.06.0
conda activate "${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}"
python -c 'import stable_baselines3' 2>/dev/null || { echo "FATAL: stable_baselines3 not importable"; exit 1; }
set +e
bash cluster/eval_sweep.sh sac "$CK" "$SW/final_sampled" --sets hold,rnd --sample-actions --tag final_sampled "${COMMON[@]}" 2>&1 | tee "$SW/final_sampled.log"
bash cluster/eval_sweep.sh sac "$CK" "$SW/final_det15" --sets hold --tag final_det15 "${COMMON[@]}" 2>&1 | tee "$SW/final_det15.log"
bash cluster/eval_sweep.sh sac "$CK" "$SW/final_sampled_spots60" --sets "$SPOTS_SET" --sample-actions --tag final_sampled_spots60 "${SPOTS_COMMON[@]}" 2>&1 | tee "$SW/final_sampled_spots60.log"
bash cluster/eval_sweep.sh sac "$CK" "$SW/final_det_spots60" --sets "$SPOTS_SET" --tag final_det_spots60 "${SPOTS_COMMON[@]}" 2>&1 | tee "$SW/final_det_spots60.log"
python3 - "$SW/final_sampled/sweep.json" "$SW/final_det15/sweep.json" "$ARM" "$SEED" "$CKPT_TAG" "$NODE_CLASS" "$IC_FILE" "$SW/final_sampled_spots60/sweep.json" "$SW/final_det_spots60/sweep.json" "$SPOTS_SET" <<'PY'
import json, sys
smp, det, arm, seed, tag, node, icf, smp60, det60, sset = sys.argv[1:11]
def rd(p, s):
    try:
        r = json.load(open(p))['sets'][s]; return f"{r['picked']}/{r['n_present']}" + ('' if r['n_present'] == r['n_expected'] else f"(exp{r['n_expected']})")
    except Exception:
        return 'MISSING'
line = (f'SAMPLED-HEADLINE arm={arm} seed={seed} ckpt={tag} ic={icf} smp_hold={rd(smp,"hold")} smp_rnd={rd(smp,"rnd")} '
        f'det_hold15={rd(det,"hold")} smp_{sset}={rd(smp60,sset)} det_{sset}={rd(det60,sset)} node={node}')
print(line)
open(smp.rsplit('/', 2)[0] + '/HEADLINE_sampled.txt', 'w').write(line + '\n')
PY
echo "SAMPLED DONE $(date)"
