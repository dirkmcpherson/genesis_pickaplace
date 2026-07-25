#!/bin/bash
# LOCAL ouroboros smoke: exercises every joint of the cluster chain with small
# numbers before committing 3 cluster-days. Mirrors sbatch_ouro_harvest.sh logic:
#   negctl gate -> DP harvest+verify -> SAC teacher probe -> image-save probe ->
#   convert -> gen1 mini-train (20k) -> gen1 mini-eval -> dv3-student conversion probe
# Success = all STAGE-OK lines present in the log; any FATAL aborts.
set -eo pipefail
cd "${GENESIS_PICKAPLACE_ROOT:-/home/j/workspace/genesis_pickaplace}"
LV=.venv-eval/bin
G=ouroboros/smoke
DP_T=baselines/outputs/dp_pick_pruned/checkpoints/last/pretrained_model
SAC_T=baselines/rl/checkpoints/sacfd_all_v2/sacfd_final.zip
IMG_T=baselines/outputs/dp_img2/checkpoints/last/pretrained_model
rm -rf $G; mkdir -p $G

if [ -n "${SKIP_NEGCTL:-}" ]; then echo "STAGE-OK negctl (cached from prior run)"; else
echo "== [1/7] negative control (random teacher, 20 ICs)"
$LV/python baselines/harvest_ai_demos.py --teacher-type random \
  --n 20 --scope pick --verify --seed 1 --outdir $G/negctl
NEG=$($LV/python -c "import json;print(json.load(open('$G/negctl/manifest.json'))['kept'])")
[ "$NEG" -gt 2 ] && { echo "FATAL: negctl kept $NEG/20"; exit 1; }
echo "STAGE-OK negctl ($NEG/20 kept)"
fi

echo "== [2/7] DP state-teacher harvest (40 ICs, verify, cap 600)"
$LV/python baselines/harvest_ai_demos.py --teacher-type dp --checkpoint $DP_T \
  --n 40 --scope pick --verify --seed 0 --cap 600 --outdir $G/harvest_dp
KEPT=$($LV/python -c "import json;print(json.load(open('$G/harvest_dp/manifest.json'))['kept'])")
[ "$KEPT" -lt 5 ] && { echo "FATAL: DP harvest kept only $KEPT/40"; exit 1; }
echo "STAGE-OK dp-harvest ($KEPT/40 kept)"

echo "== [3/7] SAC teacher probe (10 ICs)"
$LV/python baselines/harvest_ai_demos.py --teacher-type sac --checkpoint $SAC_T \
  --n 10 --scope pick --verify --seed 0 --outdir $G/harvest_sac
SKEPT=$($LV/python -c "import json;print(json.load(open('$G/harvest_sac/manifest.json'))['kept'])")
echo "STAGE-OK sac-harvest ($SKEPT/10 kept)"

echo "== [4/7] image-save probe (img2 teacher, 8 ICs, --images)"
$LV/python baselines/harvest_ai_demos.py --teacher-type dp --checkpoint $IMG_T \
  --n 8 --scope pick --verify --images --seed 3 --cap 600 --outdir $G/harvest_img || true
IKEPT=$($LV/python -c "import json;print(json.load(open('$G/harvest_img/manifest.json'))['kept'])" 2>/dev/null || echo 0)
if [ "$IKEPT" -ge 1 ]; then
  $LV/python -c "
import numpy as np, glob
f = sorted(glob.glob('$G/harvest_img/1*.npz'))[0]
d = np.load(f)
assert 'images' in d.files and d['images'].shape[1:] == (64,64,6), d['images'].shape
print('STAGE-OK image-save (images', d['images'].shape, ')')"
else
  echo "STAGE-SKIP image-save (0 kept from 8 ICs at ~0.2 pick rate -- rerun with more ICs on cluster)"
fi

echo "== [5/7] convert DP harvest -> gen1 dataset"
rm -rf $G/gen1_dataset
$LV/python baselines/convert_to_lerobot.py $G/harvest_dp $G/gen1_dataset 8 4 none video
echo "STAGE-OK convert"

echo "== [6/7] gen1 mini-train (20k steps) + mini-eval (5 ICs)"
$LV/lerobot-train \
  --dataset.repo_id=local/ouro_smoke_gen1 --dataset.root=$G/gen1_dataset \
  --policy.type=diffusion --policy.push_to_hub=false \
  --output_dir=$G/gen1_dp --batch_size=64 --steps=20000 --save_freq=20000 \
  --wandb.enable=false > $G/gen1_train.log 2>&1
$LV/python baselines/wandb_eval.py --kind dp \
  --checkpoint $G/gen1_dp/checkpoints/last/pretrained_model \
  --random 5 --seed 0 --group ouro-smoke --name ouro-smoke-gen1 \
  > $G/gen1_eval.log 2>&1
grep -aE "eval/(picked|placed)" $G/gen1_eval.log | tail -2
echo "STAGE-OK gen1-train-eval"

echo "== [7/7] dv3-student conversion probe (model-demos -> dreamer format)"
if [ "$IKEPT" -ge 1 ]; then
  $LV/python baselines/rl/to_dreamer_demos.py --src $G/harvest_img \
    --dst $G/dv3_demos && echo "STAGE-OK dv3-convert"
else
  echo "STAGE-SKIP dv3-convert (needs image harvest; state npz lack 'images')"
fi

echo "OURO-SMOKE-COMPLETE"
