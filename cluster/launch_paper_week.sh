#!/bin/bash
# PAPER WEEK launcher — PICK-PHASE core matrix, naming d{source}_{algorithm}.
# (Decision Log 2026-08-01: phase-1 core; place later via specialists from banked
# picked entry states; both demo sources truncate at the pick grant.)
#
#   bash cluster/launch_paper_week.sh                      # N=3 (seeds 0 1 2)
#   SEEDS="0 1 2 3 4" bash cluster/launch_paper_week.sh    # N=5
#   DRYRUN=1 ...                                           # print the plan
#
# MATCHED-RULE DATASETS: the human set truncates at the pick grant (uncapped);
# every model set is a verified cap-1200 target-66 demo-IC harvest from its
# generation's teacher (the lineage datasets were cap-600 -- right-truncated fast
# picks -- so they are NOT used for paper conditions). If a condition's harvest
# does not exist yet, this launcher SUBMITS it (sbatch_paper_smoke_harvest.sh with
# that teacher) and chains the training behind it with --dependency=afterok.
# Conversion harvest->lerobot happens inside the training job if needed.
#
# THREE LEARNER CLASSES (user, 2026-08-01: ACT duplicates DP's class -- swap for
# SACfD): pure imitation (DP) / RL with demo buffer (SACfD) / world model (DV3).
#
#   condition   class demos-from                    teacher (for auto-harvest)
#   dH_DP       BC    66 human pick-truncated       -- (rsync lerobot_dH_pick)
#   dDP_DP      BC    gen-0 DP harvest (m1_full)    ouro_dp_joint/gen0
#   dDP2_DP     BC    gen-1 DP harvest (m2_full)    ouro_dp_joint/gen1 (the 0.87)
#   dH_SACfD    RLfD  66 human pick-truncated npz   -- (rsync episodes_pick_phase)
#   dDP_SACfD   RLfD  gen-0 DP harvest npz          (same m1_full harvest)
#   dDP_DV3     WM    genesis_m1_full images        (same demos as dDP_DP/dDP_SACfD)
#   dH_DV3      WM    = the RUNNING hdv3_pick_s0/1/2 (not resubmitted; N=5 adds s3 s4)
#
# PREFLIGHT rsyncs (datasets travel by rsync, never git):
#   rsync -av <devbox>:~/workspace/genesis_pickaplace/baselines/lerobot_dH_pick/ \
#       <repo>/baselines/lerobot_dH_pick/
#   rsync -av <devbox>:~/workspace/genesis_pickaplace/baselines/episodes_pick_phase/ \
#       <repo>/baselines/episodes_pick_phase/
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:-$PWD}"
export GENESIS_PICKAPLACE_ROOT="$PWD"
DV3=${DV3_DIR:-$(cd .. && pwd)/dreamerv3-torch}
SEEDS=${SEEDS:-"0 1 2"}
STEPS=${STEPS:-100000}
PROJ=${PROJ:-genesis_paper}

# cond: PTYPE|DS(lerobot)|RAW(harvest dir)|TEACHER ckpt|DEMO_NAME(dv3 demo dir)
declare -A C
C[dH_DP]="diffusion|baselines/lerobot_dH_pick/genesis_pickaplace|||"
C[dDP_DP]="diffusion|paper_smoke/lerobot_m1_full/genesis_pickaplace|paper_smoke/m1_harvest_full|ouroboros/ouro_dp_joint/gen0/dp/checkpoints/last/pretrained_model|genesis_m1_full"
C[dDP2_DP]="diffusion|paper_smoke/lerobot_m2_full/genesis_pickaplace|paper_smoke/m2_harvest_full|ouroboros/ouro_dp_joint/gen1/dp/checkpoints/last/pretrained_model|genesis_m2_full"
ORDER="dH_DP dDP_DP dDP2_DP"

sub() { if [ -n "${DRYRUN:-}" ]; then echo "  [dry] $*" >&2; echo "DRY"; else "$@"; fi; }

echo "== paper week: seeds [$SEEDS], $STEPS steps, wandb project $PROJ"
for COND in $ORDER; do
  IFS='|' read -r PTYPE DS RAW TEACHER DEMO_NAME <<< "${C[$COND]}"
  DEP=""
  # manifest.json is written at harvest END -- a dir without it is an IN-FLIGHT
  # harvest (e.g. the m1_full job): do not convert/train on a partial harvest.
  if [ ! -d "$DS" ] && [ -n "$RAW" ] && [ -d "$RAW" ] && [ ! -f "$RAW/manifest.json" ]; then
    echo "  $COND: harvest $RAW is IN FLIGHT (no manifest yet) -- rerun the launcher when it completes. SKIPPED."
    continue
  fi
  if [ ! -d "$DS" ] && [ -n "$RAW" ] && [ ! -d "$RAW" ]; then
    if [ -d "$TEACHER" ]; then
      HID=$(sub sbatch --parsable --job-name="harv-$COND" \
        --export=ALL,GENESIS_PICKAPLACE_ROOT=$PWD,DV3_DIR=$DV3,TEACHER_CKPT=$TEACHER,SMOKE_KEPT=66,MIN_KEPT=40,HARVEST_N=320,CAP=1200,HARV_OUT=$RAW,DEMO_NAME=$DEMO_NAME,CONDA_ENV=${CONDA_ENV:-} \
        --time=0-12:00:00 cluster/sbatch_paper_smoke_harvest.sh)
      echo "  $COND: harvest submitted ($HID) from $TEACHER"
      DEP="--dependency=afterok:$HID"
    else
      echo "  $COND: TEACHER not trained yet ($TEACHER) -- rerun the launcher when the chain banks it. SKIPPED."
      continue
    fi
  elif [ ! -d "$DS" ] && [ -z "$RAW" ]; then
    echo "  $COND: MISSING $DS -- rsync it (see header). SKIPPED."
    continue
  fi
  CONV=""
  # only harvest-backed conditions may convert inline; an empty \$RAW would hit
  # convert_to_lerobot's DEFAULT source -- a silent-default landmine
  [ -n "$RAW" ] && CONV="[ -d $DS ] || python baselines/convert_to_lerobot.py $RAW $DS 8 4 none;"
  JID=$(sub sbatch --parsable --job-name="$COND" $DEP -p "${PARTITION:-gpu,preempt}" --requeue \
    --gres=gpu:1 --constraint="${GPU_CONSTRAINT:-l40s|a100|l40|h200}" \
    -N 1 -n 8 --mem=48g --time=1-00:00:00 \
    --output="paper_${COND}_%j.out" \
    --wrap="module load anaconda/2025.06.0; conda activate ${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}; \
            cd $PWD; export GENESIS_PICKAPLACE_ROOT=$PWD MUJOCO_GL=egl PYTHONUNBUFFERED=1; \
            $CONV \
            for S in $SEEDS; do \
              O=baselines/outputs/paper/${COND}_s\$S; \
              TC=\$O/checkpoints/last/pretrained_model/train_config.json; \
              if [ -f \$TC ]; then lerobot-train --config_path=\$TC --resume=true; \
              else lerobot-train --dataset.repo_id=local/${COND}_s\$S \
                --dataset.root=$DS --policy.type=$PTYPE --policy.push_to_hub=false \
                --seed=\$S --output_dir=\$O --batch_size=64 --steps=$STEPS \
                --job_name=${COND}_s\$S --wandb.enable=true --wandb.project=$PROJ \
                --wandb.disable_artifact=true; fi; \
              python baselines/wandb_eval.py --kind dp --ic-mode both \
                --checkpoint \$O/checkpoints/last/pretrained_model \
                --random 15 --seed 0 --project $PROJ --group $COND --name ${COND}_s\$S-eval; \
            done")
  echo "  $COND: train+eval job $JID ${DEP:+(waits on harvest)}"
done

echo "== SACfD conditions (one job PER SEED: ~6-8h each; preemption restarts the seed)"
for COND in dH_SACfD dDP_SACfD; do
  case $COND in
    dH_SACfD)  DDIR=baselines/episodes_pick_phase;;
    dDP_SACfD) DDIR=paper_smoke/m1_harvest_full;;
  esac
  if [ ! -d "$DDIR" ] || { [ "$COND" = dDP_SACfD ] && [ ! -f "$DDIR/manifest.json" ]; }; then
    echo "  $COND: demos not ready ($DDIR) -- rsync/wait for harvest, rerun launcher. SKIPPED."
    continue
  fi
  for S in $SEEDS; do
    sub sbatch --job-name="${COND}_s$S" -p "${PARTITION:-gpu,preempt}" --requeue \
      --gres=gpu:1 --constraint="${GPU_CONSTRAINT:-l40s|a100|l40|h200}" \
      -N 1 -n 8 --mem=32g --time=1-00:00:00 \
      --output="paper_${COND}_s${S}_%j.out" \
      --wrap="module load anaconda/2025.06.0; conda activate ${CONDA_ENV:-/cluster/tufts/shortlab/jstale02/condaenv/genesis}; \
              cd $PWD; export GENESIS_PICKAPLACE_ROOT=$PWD MUJOCO_GL=egl PYTHONUNBUFFERED=1; \
              O=baselines/outputs/paper/${COND}_s$S; \
              python baselines/rl/train_sacfd_full.py --scope pick \
                --demo-dir $DDIR --steps ${SACFD_STEPS:-200000} --seed $S \
                --train-max-steps 900 --out-dir \$O --run-name ${COND}_s$S \
                --project $PROJ --eval-freq 50000; \
              python baselines/wandb_eval.py --kind sac --ic-mode both \
                --checkpoint \$O/sacfd_final.zip --random 15 --seed 0 \
                --project $PROJ --group $COND --name ${COND}_s${S}-eval" >/dev/null \
      && echo "  ${COND}_s$S submitted"
  done
done

echo "== dDP_DV3 (dv3 pick-scope, model image demos -- same set as dDP_DP)"
DVDEP=""
if [ ! -d "$DV3/demonstrations/genesis_m1_full" ]; then
  if [ -d paper_smoke/m1_harvest_full ]; then
    echo "  (harvest exists; converting demos inline is the harvest job's task -- if it"
    echo "   completed without the dreamer conversion, rerun it or convert by hand)"
  fi
  echo "  demonstrations/genesis_m1_full missing -- dDP_DV3 SKIPPED (rerun launcher after the m1 harvest lands)."
else
  RUNS=""
  for S in $SEEDS; do
    [ "$S" -gt 2 ] && break     # >3 dreamer runs per GPU does not fit; N=5 tail = 2nd job
    RUNS="${RUNS}${RUNS:+ | }TAG=dDP_DV3_s$S VEC=1 SEED=$S SCOPE=pick DEMODIR=genesis_m1_full"
  done
  sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-}" WANDB=1 RUNS="$RUNS" \
    GENESIS_PICKAPLACE_ROOT="$PWD" \
    sbatch --job-name=dDP_DV3 "$DV3/sbatch_genesis_multi.sh" >/dev/null && echo "  dDP_DV3 x3 submitted"
fi
NEXTRA=$(echo $SEEDS | tr ' ' '\n' | awk '$1>2' | tr '\n' ' ')
if [ -n "$NEXTRA" ]; then
  RUNS=""
  for S in $NEXTRA; do RUNS="${RUNS}${RUNS:+ | }TAG=dH_DV3_s$S VEC=1 SEED=$S SCOPE=pick DEMODIR=genesis_pick"; done
  [ -d "$DV3/demonstrations/genesis_m1_full" ] && \
    for S in $NEXTRA; do RUNS="${RUNS} | TAG=dDP_DV3_s$S VEC=1 SEED=$S SCOPE=pick DEMODIR=genesis_m1_full"; done
  sub env REPO_DIR="$DV3" CONDA_ENV="${CONDA_ENV:-}" WANDB=1 RUNS="$RUNS" \
    GENESIS_PICKAPLACE_ROOT="$PWD" \
    sbatch --job-name=dv3-extra-seeds "$DV3/sbatch_genesis_multi.sh" >/dev/null && echo "  dv3 extra seeds [$NEXTRA] submitted (dH + dDP)"
fi
echo "== dH_DV3 s0-s2 = the running hdv3_pick_s0/1/2 (not resubmitted)"
echo
echo "== done. squeue -u \$USER | wandb: $PROJ (lerobot) + dreamer_v3 (dv3)"
