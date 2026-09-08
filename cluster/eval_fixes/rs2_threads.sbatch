#!/usr/bin/env bash
# Mechanism test: is the cross-node divergence driven by the Genesis/taichi CPU THREAD COUNT (which tracks the physical
# core count: 36c nodes reproduce the record, 40c/64c nodes do not)? Run the same ep0 with TI_NUM_THREADS pinned.
# If pinning makes a diverging node reproduce, every eval can simply export it and results become node-independent.
# usage: sbatch --nodelist=<node> rs2_threads.sbatch <ti_threads>
#SBATCH -J rs2_threads
#SBATCH -p preempt,batch
#SBATCH --qos=preempt
#SBATCH -N 1
#SBATCH -n 4
#SBATCH --mem=16g
#SBATCH -t 0-01:00:00
#SBATCH -o /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/slurm/%x_%N_%j.out
set -uo pipefail
T=$1
LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03; PY=$LAB/r2d_venv/bin/python
export TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/${SLURM_JOB_ID} MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
export GENESIS_PICKAPLACE_ROOT=$W/gp_root R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
export TI_NUM_THREADS=$T OMP_NUM_THREADS=$T
RUN=full_r2d_state_dHfull_all_bnormclampS8ent5_s3
echo "# $(date -Is) NODE=$(hostname) CPUTot(phys)=$(grep -c ^processor /proc/cpuinfo) cgroup_nproc=$(nproc) TI_NUM_THREADS=$T"
cd $W/r2dreamer_fix
"$PY" eval_genesis.py --checkpoint $W/runs/$RUN/latest.pt --episodes 1 --mode mode --max-steps 1200 \
  --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set rnd --ic-index 0 --seed 0 \
  --out $W/rs2_iso/thr_$(hostname)_$T --device cpu 2>&1 | grep -E "^ep0|episodes \(mode|Traceback|Error" | tail -2
echo "# THREADS-DONE $(hostname) T=$T   (record: nested/32)"
