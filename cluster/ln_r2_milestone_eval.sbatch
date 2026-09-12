#!/usr/bin/env bash
# One {r2dreamer} MILESTONE cell-triple, evaluated on a COPY of the checkpoint.
# Submitted by cluster/ln_r2_milestone_sweep.sh, never by hand (the sweep does the copy, the
# sha256 verification against the milestone sidecar, and the provenance stamp).
#
#   sbatch [-p ... --qos=...] ln_r2_milestone_eval.sbatch <run_name> <milestone_label>
#
# It writes THREE cells under $CELLROOT/<run>/<milestone>/ :
#   rnd30_mode    30 episodes, ic-set rnd,  --mode mode     <- (aa) readout cell
#   hold15_mode   15 episodes, ic-set hold, --mode mode     <- (aa) readout cell
#   rnd30_sample  30 episodes, ic-set rnd,  --mode sample   <- the route census's statistic
# and SKIPS any cell whose metrics.json already exists, so the job is idempotent and a
# preempted job re-runs only what it had not finished.
#
# HARDWARE. The evaluator is CPU work end to end -- the r2dreamer adapter builds Genesis with
# backend="cpu" (envs/genesis.py:214) and the policy runs --device cpu -- so this job asserts a
# PHYSICAL core count instead of trusting a Slurm label (pax001 advertises `broadwell` and is
# Cascade Lake; pax012 advertises 64 CPUs and presents 128 logical). Full-scope outcomes track
# machine size in this project, so a cell produced on the wrong class is worse than no cell:
# the guard refuses BEFORE the first episode.
#SBATCH -J lnms
#SBATCH -N 1
#SBATCH -n 8
#SBATCH --mem=48g
#SBATCH -t 0-08:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/ln_milestone_cells/slurm_%x_%j.out
set -eo pipefail          # NOT -u: `conda activate` sources a cluster hook that reads an unset
                          # variable and dies under -u (Ladder-N log §6.2)
RUN=$1; MS=$2
LAB=${LAB:-/cluster/tufts/shortlab/jstale02}
W=${W:-$LAB/wm_fix_2026-09-03}
GP=${GP:-$LAB/gp_ladderN}
R2=${R2:-$W/r2dreamer_ladderN}
CELLROOT=${CELLROOT:-$W/ln_milestone_cells}
PY=$LAB/r2d_venv/bin/python
REQUIRE_CORES=${REQUIRE_CORES:-64}

CELL=$CELLROOT/$RUN/$MS
CKPT=$CELL/latest.pt
[ -f "$CKPT" ] || { echo "FATAL: no checkpoint copy at $CKPT (the sweep's copy step did not run)"; exit 2; }
[ -f "$CELL/.hydra/config.yaml" ] || { echo "FATAL: no .hydra/config.yaml beside the copy"; exit 2; }
[ -f "$GP/baselines/rl/full_env.py" ] || { echo "FATAL: GP=$GP is not a genesis_pickaplace tree"; exit 2; }
[ -f "$R2/eval_genesis.py" ] || { echo "FATAL: R2=$R2 is not an r2dreamer tree"; exit 2; }

PHYS=$(lscpu -p=Core,Socket 2>/dev/null | grep -v '^#' | sort -u | wc -l)
LOGI=$(grep -c '^processor' /proc/cpuinfo)
echo "[hw] host=$(hostname) physical=$PHYS logical=$LOGI model=$(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2-)"
if [ "$REQUIRE_CORES" != "0" ] && [ "$PHYS" != "$REQUIRE_CORES" ]; then
  echo "FATAL: --require-cores $REQUIRE_CORES but this machine has $PHYS physical cores -- refusing to write a cell on the wrong hardware class"; exit 2
fi

export GENESIS_PICKAPLACE_ROOT=$GP
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6

echo "# $(date -Is) RUN=$RUN MS=$MS CKPT=$CKPT"
echo "# gp_ladderN describe: $(git -C "$GP" describe --always --dirty 2>/dev/null || echo 'no git')"
echo "# r2dreamer_ladderN rev: $(git -C "$R2" rev-parse --short HEAD 2>/dev/null || echo '?')"
echo "# provenance:"; cat "$CELL/provenance.json" 2>/dev/null || echo "  (none)"

cd "$R2"
rc_all=0
# set  episodes  mode   -> cell dir
for SPEC in "rnd 30 mode" "hold 15 mode" "rnd 30 sample"; do
  set -- $SPEC; S=$1; N=$2; M=$3
  OUT=$CELL/${S}${N}_${M}
  if [ -f "$OUT/metrics.json" ]; then echo "# SKIP $OUT (metrics.json exists)"; continue; fi
  CMD=("$PY" eval_genesis.py --checkpoint "$CKPT" --episodes "$N" --mode "$M" --max-steps 1200 \
       --ic-file "$GP/baselines/eval_ics.json" --ic-set "$S" --seed 0 --out "$OUT" --device cpu)
  printf '%q ' "${CMD[@]}"; echo
  if "${CMD[@]}"; then echo "# cell $S$N $M rc=0 $(date -Is)"; else rc=$?; rc_all=$rc; echo "# cell $S$N $M rc=$rc $(date -Is)"; fi
done
echo "# DONE-MILESTONE $RUN $MS rc_all=$rc_all $(date -Is)"
exit $rc_all
