#!/bin/bash
# PAPER WEEK launcher — PICK-PHASE cross-algorithm demo matrix (2026-08-01).
# Design (user): each learner class trains on each source's demonstrations.
# DV3 and SACfD also consume FAILED demos (zero-reward negatives); DP never does.
#
#            learner:  DP (BC)      SACfD (RLfD)     DV3 (WM)
#   source dH          dH_DP        dH_SACfD         dH_DV3 (RUNNING s0-2)
#   source dDP         dDP_DP       dDP_SACfD        dDP_DV3
#   source dSACfD      dSACfD_DP    dSACfD_SACfD     dSACfD_DV3     (wave 2)
#   source dDV3        -- pending dv3-teacher harvest tooling + gate (wave 3)
#
# Sources: dH = 66 human pick-truncated (+25 fails for RLfD/WM); dDP = cap-1200
# target-66 verified harvest from the gen-0 DP teacher WITH --keep-fails 30;
# dSACfD = same harvest recipe from the best dH_SACfD checkpoint (exists only
# after wave 1 -- rerun this launcher and it fills in; override SACFD_TEACHER).
# 2026-08-11: SACfD trains in DELTA-JOINT action mode by default
# (SACFD_ACTION_MODE=absolute + SACFD_SUFFIX= restores the old geometry). Run
# names/out-dirs get SACFD_SUFFIX (default '-dj') so the delta wave cannot
# collide with the 16 absolute-mode 200k retrains of 08-10 (all 0.00 -- they
# are the matched control row for this wave).
# Missing harvests are auto-submitted; trainings chain behind them (afterok).
# BC datasets take ONLY success episodes (stems 1xxxxx; fails are 5xxxxx).
#
#   bash cluster/launch_paper_week.sh                      # N=3 (seeds 0 1 2)
#   SEEDS="0 1 2 3 4" bash cluster/launch_paper_week.sh    # N=5
#   DRYRUN=1 ...
#
# PREFLIGHT rsyncs (datasets travel by rsync, never git):
#   rsync -av <devbox>:.../baselines/lerobot_dH_pick/        <repo>/baselines/lerobot_dH_pick/
#   rsync -av <devbox>:.../baselines/episodes_pick_phase_all/ <repo>/baselines/episodes_pick_phase_all/
#   rsync -av <devbox>:.../dreamerv3-torch/demonstrations/genesis_pick/ <dv3>/demonstrations/genesis_pick/
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:-$PWD}"
export GENESIS_PICKAPLACE_ROOT="$PWD"
DV3=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
SEEDS=${SEEDS:-"0 1 2"}
STEPS=${STEPS:-100000}
PROJ=${PROJ:-genesis_paper}
PRE="module load anaconda/2025.06.0; conda activate ${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}; cd $PWD; export GENESIS_PICKAPLACE_ROOT=$PWD MUJOCO_GL=egl PYTHONUNBUFFERED=1"
sub() { if [ -n "${DRYRUN:-}" ]; then echo "  [dry] $*" >&2; echo "DRY"; else "$@"; fi; }

# per source: RAW harvest dir | teacher ckpt | teacher type | dreamer demo dir name
declare -A SRC
SRC[dH]="||"
SRC[dDP]="paper_smoke/m1all_harvest|ouroboros/ouro_dp_joint/gen0/dp/checkpoints/last/pretrained_model|dp|genesis_m1all"
SRC[dSACfD]="paper_smoke/sacall_harvest|${SACFD_TEACHER:-baselines/outputs/paper/dH_SACfD_s0/sacfd_final.zip}|sac|genesis_sacall"

echo "== paper week (cross-algo matrix): seeds [$SEEDS], wandb $PROJ"
for S_NAME in dH dDP dSACfD; do
  IFS='|' read -r RAW TEACHER TTYPE DNAME <<< "${SRC[$S_NAME]}"
  DEP=""
  if [ "$S_NAME" = dH ]; then
    NPZ=baselines/episodes_pick_phase_all          # SACfD/DV3 (66 succ + 25 fail)
    DS=baselines/lerobot_dH_pick/genesis_pickaplace # DP (success only, prebuilt)
    DDIR=genesis_pick                               # DV3 (91, fails included)
    [ -d "$NPZ" ] && [ -d "$DS" ] || { echo "  $S_NAME: rsync missing ($NPZ / $DS) -- SKIPPING all dH_* conditions"; continue; }
  else
    NPZ=$RAW
    DS=${RAW}_succ_lerobot/genesis_pickaplace
    DDIR=$DNAME
    if [ ! -f "$RAW/manifest.json" ]; then
      if [ -d "$RAW" ]; then
        echo "  $S_NAME: harvest $RAW IN FLIGHT -- rerun launcher when done. SKIPPING its conditions."; continue
      elif [ -e "$TEACHER" ]; then
        HID=$(sub sbatch --parsable --job-name="harv-$S_NAME" --time=0-12:00:00 \
          --export=ALL,GENESIS_PICKAPLACE_ROOT=$PWD,DV3_DIR=$DV3,TEACHER_CKPT=$TEACHER,TEACHER_TYPE=$TTYPE,SMOKE_KEPT=66,MIN_KEPT=40,HARVEST_N=320,CAP=1200,KEEP_FAILS=30,HARV_OUT=$RAW,DEMO_NAME=$DNAME,CONDA_ENV=${CONDA_ENV:-} \
          cluster/sbatch_paper_smoke_harvest.sh)
        echo "  $S_NAME: harvest submitted ($HID) from $TEACHER"
        DEP="--dependency=afterok:$HID"
      else
        echo "  $S_NAME: teacher not available yet ($TEACHER) -- rerun launcher after wave 1. SKIPPING its conditions."; continue
      fi
    fi
  fi

  # ---- DP learner (success-only) --------------------------------------------
  CONV=""
  [ "$S_NAME" != dH ] && CONV="if [ ! -d $DS ]; then T=\$(mktemp -d); cp $RAW/1*.npz \$T/; python baselines/convert_to_lerobot.py \$T $DS 8 4 none; rm -rf \$T; fi;"
  sub sbatch --job-name="${S_NAME}_DP" $DEP -p "${PARTITION:-gpu,preempt}" --requeue \
    --gres=gpu:1 --constraint="${GPU_CONSTRAINT:-l40s|a100|l40|h200}" \
    -N 1 -n 8 --mem=48g --time=0-14:00:00 --output="paper_${S_NAME}_DP_%j.out" \
    --wrap="$PRE; $CONV \
      for S in $SEEDS; do \
        O=baselines/outputs/paper/${S_NAME}_DP_s\$S; \
        TC=\$O/checkpoints/last/pretrained_model/train_config.json; \
        if [ -f \$TC ]; then lerobot-train --config_path=\$TC --resume=true; \
        else lerobot-train --dataset.repo_id=local/${S_NAME}_DP_s\$S \
          --dataset.root=$DS --policy.type=diffusion --policy.push_to_hub=false \
          --seed=\$S --output_dir=\$O --batch_size=64 --steps=$STEPS \
          --job_name=${S_NAME}_DP_s\$S --wandb.enable=true --wandb.project=$PROJ \
          --wandb.disable_artifact=true; fi; \
        python baselines/wandb_eval.py --kind dp --ic-mode both \
          --checkpoint \$O/checkpoints/last/pretrained_model --random 15 --seed 0 \
          --project $PROJ --group ${S_NAME}_DP --name ${S_NAME}_DP_s\$S-eval; \
      done" >/dev/null && echo "  ${S_NAME}_DP submitted ${DEP:+(waits on harvest)}"

  # ---- SACfD learner (successes + fails) ------------------------------------
  for S in $SEEDS; do
    sub sbatch --job-name="${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s$S" $DEP -p "${PARTITION:-gpu,preempt}" --requeue \
      --gres=gpu:1 --constraint="${GPU_CONSTRAINT:-l40s|a100|l40|h200}" \
      -N 1 -n 8 --mem=32g --time=0-10:00:00 --output="paper_${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s${S}_%j.out" \
      --wrap="$PRE; O=baselines/outputs/paper/${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s$S; \
        python baselines/rl/train_sacfd_full.py --scope pick \
          --demo-dir $NPZ --steps ${SACFD_STEPS:-200000} --seed $S \
          --action-mode ${SACFD_ACTION_MODE:-delta_joint} \
          --train-max-steps 900 --out-dir \$O --run-name ${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s$S \
          --project $PROJ --eval-freq 50000; \
        python baselines/wandb_eval.py --kind sac --ic-mode both \
          --checkpoint \$O/sacfd_final.zip --random 15 --seed 0 \
          --project $PROJ --group ${S_NAME}_SACfD${SACFD_SUFFIX:--dj} --name ${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s${S}-eval" \
      >/dev/null && echo "  ${S_NAME}_SACfD${SACFD_SUFFIX:--dj}_s$S submitted ${DEP:+(waits on harvest)}"
  done

  # ---- DV3 learner (fails included via the demo dir) -------------------------
  if [ "$S_NAME" = dH ]; then
    echo "  dH_DV3 s0-s2 = RUNNING hdv3_pick_s0/1/2 (not resubmitted)"
  else
    RUNS=""
    for S in $SEEDS; do
      [ "$S" -gt 2 ] && break
      RUNS="${RUNS}${RUNS:+ | }TAG=${S_NAME}_DV3_s$S VEC=1 SEED=$S SCOPE=pick DEMODIR=$DNAME"
    done
    # the harvest job converts RAW -> demonstrations/$DNAME (incl fails); afterok
    # ordering guarantees it exists when this starts
    sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-}" WANDB=1 RUNS="$RUNS" \
      GENESIS_PICKAPLACE_ROOT="$PWD" \
      sbatch ${DEP} --job-name="${S_NAME}_DV3" "$DV3/sbatch_genesis_multi.sh" >/dev/null \
      && echo "  ${S_NAME}_DV3 x3 submitted ${DEP:+(waits on harvest)}"
  fi
done

NEXTRA=$(echo $SEEDS | tr ' ' '\n' | awk '$1>2' | tr '\n' ' ')
if [ -n "$NEXTRA" ]; then
  RUNS=""
  for S in $NEXTRA; do RUNS="${RUNS}${RUNS:+ | }TAG=dH_DV3_s$S VEC=1 SEED=$S SCOPE=pick DEMODIR=genesis_pick"; done
  sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-}" WANDB=1 RUNS="$RUNS" \
    GENESIS_PICKAPLACE_ROOT="$PWD" \
    sbatch --job-name=dH_DV3_extra "$DV3/sbatch_genesis_multi.sh" >/dev/null \
    && echo "  dH_DV3 extra seeds [$NEXTRA] submitted"
fi
echo "== dDV3 source: pending the H4 gate + a dv3-teacher mode in harvest_ai_demos (wave 3)"
echo "== done. squeue -u \$USER | wandb: $PROJ"
