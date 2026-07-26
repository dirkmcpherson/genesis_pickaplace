# Cluster launch — DV3 (joint + cartesian) & Ouroboros (cartesian DP)

All commands assume the Tufts layout:
`/cluster/tufts/shortlab/jstale02/{dreamerv3-torch, genesis_pickaplace}`.
Run blocks in order. `<box>` = this dev machine (rsync source).

> **Gate:** wait for my green-light message before §3/§4 — two local smokes
> (dv3-cartesian, ouroboros-cartesian) are finishing, and the gen-0 dataset gets
> its final `--picked-only` rebuild after they complete. §1/§2 are safe now.

## 1. One-time setup

```bash
# --- code: pull BOTH repos ---
cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch    && git pull   # branch: genesis
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace && git pull   # branch: 4dof-cartesian

# --- lerobot into the genesis conda env (once; needed by ouroboros) ---
module load anaconda/2025.06.0
conda activate /cluster/tufts/shortlab/jstale02/condaenv/genesis
bash cluster/install_lerobot.sh        # from genesis_pickaplace/; pins torchcodec 0.3 <-> torch 2.7
```

## 2. Data rsync (from the box — demos/datasets are gitignored)

```bash
# dv3 cartesian demos (85 eps, images + human vel_cmd actions, tip-truncated)
rsync -av ~/workspace/dreamerv3-torch/demonstrations/genesis_cartesian/ \
  <cluster>:/cluster/tufts/shortlab/jstale02/dreamerv3-torch/demonstrations/genesis_cartesian/

# ouroboros gen-0 dataset (STATE-only cartesian, proprio 9, picked-only pruned)
# NB: I rebuild this with --picked-only after the smoke finishes -- rsync AFTER green light.
rsync -av ~/workspace/genesis_pickaplace/baselines/lerobot_dataset_cart_pruned/ \
  <cluster>:/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/lerobot_dataset_cart_pruned/
```

## 3. DreamerV3

```bash
cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch

# (a) JOINT-space 5M continuation (when the current job ends): same TAG => resumes
#     from latest.pt; periodic policy evals (videos -> wandb) are in the sbatch now.
REPO_DIR=$PWD CONDA_ENV=genesis WANDB=1 VEC=1 TAG=run5m sbatch sbatch_genesis_pixels.sh
# resubmit the IDENTICAL line after each 48h time-limit kill until 5M steps.

# (b) CARTESIAN arm (new): 5-dim ee-velocity actions, tip-termination env,
#     human vel_cmd demos. No VEC (no batched IK yet) -> slower FPS, spans 2-3
#     allocations; same TAG-resume discipline.
REPO_DIR=$PWD CONDA_ENV=genesis WANDB=1 CART=1 TAG=cart5m sbatch sbatch_genesis_pixels.sh
```

## 4. Ouroboros (cartesian DP chain, self-perpetuating)

```bash
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace

TAG=ouro1 ACTIONS=cartesian CAMS=none CAP=1200 HARVEST_N=600 \
  DATASET=baselines/lerobot_dataset_cart_pruned/genesis_pickaplace \
  sbatch cluster/sbatch_ouro_train.sh
```

One submission runs the whole experiment: train gen-k -> eval (15 ICs + tiled
video -> wandb `genesis_pickaplace_ouro`) -> harvest job (negative control gate,
600 verified rollouts at eval-parity cap) -> gen-(k+1) dataset -> next train job,
up to `MAXGEN=3` (human -> gen1 -> gen2). Abort anytime: `scancel` the pending
job, or add `NO_CHAIN=1` to stop after the current stage.

## 5. Watching

```bash
squeue -u jstale02                      # ouro-train / ouro-harvest alternate; genesis-dv3 runs long
tail -f genesis_dv3_<jobid>.out         # dreamer: fps + train_return
tail -f ouro_train_<jobid>.out          # lerobot-train progress -> eval -> chain submit
```

- wandb: `dreamer_v3` project (train + `*-eval-step<N>` policy-eval runs with videos);
  `genesis_pickaplace_ouro` (per-generation DP train/eval).
- One-off eval of a live dv3 run from any GPU node:
  ```bash
  srun -p gpu --gres=gpu:1 -n 4 --mem=16g --time=0:30:00 --pty bash -c '
    module load anaconda/2025.06.0 && conda activate genesis
    cd /cluster/tufts/shortlab/jstale02/dreamerv3-torch
    export MUJOCO_GL=egl GENESIS_PICKAPLACE_ROOT=/cluster/tufts/shortlab/jstale02/genesis_pickaplace
    L=$(ls -dt logs_cluster/*/ | head -1)
    python genesis_eval.py --logdir "$L" --episodes 6 --name manual-eval-$(date +%H%M)'
  ```

## Failure modes to expect

- **dv3 resume check:** on every resubmission, the `.out` must show a checkpoint
  restore, NOT "Prefill dataset" from zero. Re-prefill = wrong TAG/logdir; kill it.
- **Ouroboros chain death is intentional** at two gates: negative control >2/50
  kept (predicate broken) or harvest <10 kept (teacher too weak). Both print FATAL
  with the reason in `ouro_harvest_*.out`.
- **cuDNN / NPP loader errors** on new nodes: the sbatch prepends the env's
  `nvidia/*/lib` dirs to `LD_LIBRARY_PATH`; if a node still fails, it's the shared
  site-packages clobber — re-run `cluster_bootstrap.sh`'s verify step.
