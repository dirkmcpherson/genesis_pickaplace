#!/bin/bash
# Fill ~8 GPUs for a night, WITH SEEDS.
#
#   bash cluster/launch_8gpu_night.sh            # submit everything
#   DRYRUN=1 bash cluster/launch_8gpu_night.sh   # print the plan only
#
# Why seeds: joint DP spans 0.07-0.53 on identical configs (4 seeds measured), so a
# single-seed number here is not interpretable. Every learning condition below runs
# x3. That is the difference between "cartesian BC fails" and "we sampled a bad seed
# three times", which cost us most of a week.
#
# Allocation (1 GPU per sbatch unless noted):
#   GPU 1-4  the obs x action 2x2 -- 4 cells x 3 seeds, packed 3 seeds per GPU
#   GPU 5-6  dv3 cartesian, tip penalty 0.0: abs6 and delta6 (2 runs per GPU via multi)
#   GPU 7    dv3 joint 5M (the validated reference)
#   GPU 8    ouroboros lineages (DP + ACT), self-chaining
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:-$PWD}"
DV3=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
SEEDS=${SEEDS:-"0 1 2"}
STEPS=${STEPS:-100000}
sub() { if [ -n "${DRYRUN:-}" ]; then echo "  [dry] $*"; else "$@"; fi; }

echo "== 1-4: obs x action 2x2, seeds: $SEEDS"
for CELL in jobs_jact jobs_eact eobs_jact eobs_eact; do
  DS=baselines/lerobot_x2x2_${CELL}/genesis_pickaplace
  [ -d "$DS" ] || { echo "  SKIP $CELL: $DS missing"; continue; }
  case $CELL in eobs_*) CTRL_ARGS="--cartesian --control abs6";; *) CTRL_ARGS="";; esac
  # one sbatch per cell; the three seeds share that GPU sequentially
  sub sbatch --job-name="x2x2-$CELL" -p "${PARTITION:-gpu,preempt}" --requeue \
    --gres=gpu:1 --constraint="${GPU_CONSTRAINT:-l40s}" \
    -N 1 -n 8 --mem=48g --time=1-00:00:00 \
    --output="x2x2_${CELL}_%j.out" \
    --wrap="module load anaconda/2025.06.0; conda activate ${CONDA_ENV:-genesis}; \
            cd $PWD; export GENESIS_PICKAPLACE_ROOT=$PWD MUJOCO_GL=egl; \
            for S in $SEEDS; do \
              R=''; [ -d baselines/outputs/x2x2_${CELL}_s\$S/checkpoints/last ] && R='--resume=true'; \
              lerobot-train \$R --dataset.repo_id=local/x2x2_${CELL}_s\$S \
                --dataset.root=$DS --policy.type=diffusion --policy.push_to_hub=false \
                --seed=\$S --output_dir=baselines/outputs/x2x2_${CELL}_s\$S \
                --batch_size=64 --steps=$STEPS --job_name=x2x2_${CELL}_s\$S \
                --wandb.enable=true --wandb.project=genesis_x2x2 --wandb.disable_artifact=true; \
              python baselines/wandb_eval.py --kind dp $CTRL_ARGS --ic-mode both \
                --checkpoint baselines/outputs/x2x2_${CELL}_s\$S/checkpoints/last/pretrained_model \
                --random 15 --seed 0 --group x2x2_$CELL --name x2x2_${CELL}_s\$S-eval; \
            done"
done

echo "== 5-6: dv3 cartesian, TIP_PENALTY=0.0, seeds: $SEEDS"
for CTRL in abs6 delta6; do
  RUNS=""
  for S in $SEEDS; do
    RUNS="${RUNS}${RUNS:+ | }TAG=${CTRL}_tip0_s$S CART=1 VEC=1 CTRL=$CTRL SEED=$S"
  done
  sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-genesis}" WANDB=1 RUNS="$RUNS" \
    sbatch --job-name="dv3-$CTRL-tip0" "$DV3/sbatch_genesis_multi.sh"
done

echo "== 7: dv3 joint reference"
sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-genesis}" WANDB=1 VEC=1 TAG=joint5m_ref \
  sbatch --job-name="dv3-joint-ref" "$DV3/sbatch_genesis_pixels.sh"

echo "== 8: ouroboros lineages (self-chaining)"
sub bash cluster/launch_ouroboros.sh cluster/conditions_full.txt

echo
echo "== submitted. watch:  squeue -u \$USER"
echo "   wandb: genesis_x2x2 (the 2x2)  |  dreamer_v3 (dv3)  |  genesis_pickaplace_ouro"
