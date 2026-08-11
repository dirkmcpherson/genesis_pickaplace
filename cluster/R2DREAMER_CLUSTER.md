# r2dreamer on the cluster — runbook

Port of the dev-box r2dreamer/genesis stack (see `~/workspace/r2dreamer/GENESIS_PORT_STATUS.md`)
to Slurm. Target: the side-by-side pick runs at full spec — **msparity**
(`genesis_pick_v3`, absolute joint) vs **delta** (`genesis_pick_v4_delta`, delta-joint),
same 67 idle-pruned demos in two encodings.

The r2dreamer env is **separate** from the shared py3.10 conda env
(`/cluster/tufts/shortlab/jstale02/condaenv/genesis`) — r2dreamer needs py3.11 /
torch 2.8.0+cu126. Never install anything into the py3.10 env, and never
conda-install libraries into either (conda libstdc++ poisoned the py3.10 env on
07-30; pip-only is the law).

## 1. What to rsync (git is NOT enough for any of these)

From the dev box, into the cluster home (or wherever, then point env vars at it):

```bash
# r2dreamer: the ENTIRE genesis port is UNCOMMITTED in the local clone
# (envs/genesis.py, demo_prefill.py, eval_genesis.py, configs/env/genesis_*.yaml,
# + modified train/trainer/buffer/dreamer/tools/configs). A git clone of the
# upstream repo has NONE of it — the working tree must travel by rsync.
rsync -av --exclude runs/ --exclude .venv/ --exclude wandb/ --exclude '__pycache__' \
  <devbox>:~/workspace/r2dreamer/ ~/r2dreamer/

# Genesis: 0.2.1 + 269 upstream commits + local headless-render patch (f41427d).
# NOT pip-reproducible from any index/tag — rsync the checkout.
rsync -av --exclude '__pycache__' <devbox>:~/workspace/Genesis/ ~/Genesis/

# Demos (gitignored; datasets travel by rsync ONLY — house rule):
# both encodings, 67 eps / 67 rewarded pick terminals each.
rsync -av <devbox>:~/workspace/dreamerv3-torch/demonstrations/genesis_pick_pruned/ \
  /cluster/tufts/shortlab/jstale02/dreamerv3-torch/demonstrations/genesis_pick_pruned/
rsync -av <devbox>:~/workspace/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta/ \
  /cluster/tufts/shortlab/jstale02/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta/
```

`genesis_pickaplace` itself is on the cluster already
(`/cluster/tufts/shortlab/jstale02/genesis_pickaplace`) — `git pull` it; the
adapter reads it via `GENESIS_PICKAPLACE_ROOT` (the sbatch exports it).

## 2. Install (once; idempotent)

```bash
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
bash cluster/install_r2dreamer.sh            # -> ~/r2d_venv
# custom paths: R2D_DIR=~/r2dreamer GENESIS_DIR=~/Genesis bash cluster/install_r2dreamer.sh ~/r2d_venv
```

Interpreter: `python3.11` from PATH if present (check `module avail python`),
else `conda create -p ~/r2d_venv python=3.11` — interpreter only, pip-only after.
Installs torch 2.8.0+cu126 (pytorch index) → `pip -e r2dreamer` → `pip -e Genesis`
→ pyvista / moviepy==1.0.3 / wandb → numpy==2.4.6 (dev-box parity; the
numpy-2.x tensorboard shim is in-tree in r2dreamer's tools.py).

## 3. Verify (inside a GPU allocation)

```bash
srun -p preempt --gres=gpu:1 --constraint="l40s|a100|l40|h200" -n 8 --mem=32g \
  --time=0:30:00 --pty bash cluster/install_r2dreamer.sh   # installs no-op; verify runs
```

Stage 1 imports torch + the port modules; stage 2 builds the FULL kinova world on
the GPU (mirror of `verify_env.sh` stage 3 — the exact 07-30 crash point). All four
GPU types are verified for genesis. If stage 2 segfaults while stage 1 passes, the
env's C++ runtime is broken → rebuild the venv pip-only (never roll conda libs in).

wandb: `WANDB_API_KEY` must be set (or `~/r2d_venv/bin/wandb login` once).
Entity `jambotime`, project `r2dreamer_genesis` (hardcoded in
`sync_runs_to_wandb.py` / `eval_genesis.py --wandb`).

## 4. Launch — the side-by-side at full spec

One run per GPU. From the genesis_pickaplace root:

```bash
cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace
# 2026-08-10 night: v3 (absolute) FAILED locally (0/2479 eps at 850k, no entropy
# collapse) and v4_delta is SUPERSEDED by v5_delta (bounded_normal_clipped actor
# dist + delta_cap 0.025 + entropy 3e-5 — see r2dreamer/GENESIS_PORT_STATUS.md
# 2026-08-10 night). Launch v5 seeds; demo dir is the RE-ENCODED delta25 set.
CONFIG=genesis_pick_v5_delta SEED=1 DEMO_DIR=$HOME/demonstrations/genesis_pick_pruned_delta25 sbatch cluster/sbatch_r2dreamer.sh
CONFIG=genesis_pick_v5_delta SEED=2 DEMO_DIR=$HOME/demonstrations/genesis_pick_pruned_delta25 sbatch cluster/sbatch_r2dreamer.sh
# (local box runs SEED=0; rsync must include BOTH the r2dreamer tree -- new
# files: distributions/networks/_base_/envs changes, configs/env/genesis_pick_v5_delta.yaml,
# replay_gate.py -- and demonstrations/genesis_pick_pruned_delta25/)
```

Defaults = the dev-box full-spec recipe: `env.steps=3e6`, `env.env_num=6`,
`buffer.max_size=450000`, `trainer.pretrain=1000`, `env.demo_reinject_every=300000`,
demo dir auto-selected to match the CONFIG's encoding
(`genesis_pick_pruned` for v3, `genesis_pick_pruned_delta` for v4_delta — do not
cross them). Logdir: `~/r2dreamer/runs/pick_v3_s0`-style (= wandb run name).
`DRYRUN=1 bash cluster/sbatch_r2dreamer.sh` prints the resolved command.
Overrides: `STEPS SEED DEMO_DIR LOGDIR VENV R2D_DIR ENV_NUM BUFFER_MAX REINJECT
PRETRAIN EXTRA` (raw hydra), `EVAL_EPS`/`EVAL_MAX_STEPS`/`NO_EVAL`.

Each job: trains → background wandb sync every 30 min (idempotent
`sync_runs_to_wandb.py`) → final sync → honest eval (15 eps, sampled actions,
demo ICs, videos → wandb) via `eval_genesis.py --wandb`.

## 5. Preemption / requeue — the resume gap, stated honestly

`-p gpu,preempt --requeue`: preempted jobs go straight back in queue.
r2dreamer periodic-checkpoints `latest.pt` (atomic, every 1e5 env steps,
`trainer.save_every`). On requeue the sbatch passes `+resume=true` and train.py
(warm-restart block added 2026-08-10) reloads **agent weights + optimizer state**
from `latest.pt`.

**This is a WARM RESTART, not a true resume:** the replay buffer is not
persisted, so the buffer restarts at demo prefill and the step counter re-runs
the full `env.steps` budget with warm weights. Learned parameters survive
preemption; buffer contents and step progress do not. Consequence: a preempted
run costs extra wall time (budgeted in the 2-day `--time`), and its
metrics.jsonl step axis restarts — read wandb histories of requeued runs with
that in mind (the sync uses a custom `env_step` metric, so nothing is silently
dropped, but the axis is per-attempt). A full resume would need buffer +
step-counter serialization — not built.

## 6. Watch keys

Same as the dev-box runs (GENESIS_PORT_STATUS.md "ManiSkill-parity"):
`train/action_entropy` MUST fall (the MS fingerprint); `episode/score` 100.0 on
honest picks (predicate hardened 2026-08-09, fling-proof); `train/data/reward_frames`
~6+/batch. Browse: https://wandb.ai/jambotime/r2dreamer_genesis.


## 2026-08-11 — THE STATISTICS WAVE (champion recipe, d{src}_R2D matrix)

Champion recipe = `genesis_pick_v5d4_delta` (delta cap 0.025 == demo p99 speed,
bounded_normal_clipped actor, entropy 3e-5, demo_duplicate 4 + reinject 150k
BAKED IN). Local proof: CHAMPION_1576820 eval 0.91 sampled (n=45) / 1.00 mode.
Training is bistable -- the sbatch now archives+evals every checkpoint, prunes
to best-2+newest, and post-train CONFIRMS the best checkpoint on x3 fresh eval
seeds (+1 mode). Headline metric per run = the confirmation mean; secondary =
P(discovery) across seeds (local seed variance was severe: s0 discovered by
300k, s1 never in 1.9M).

### rsync manifest (dev box -> cluster)
    rsync -a <devbox>:~/workspace/r2dreamer/ ~/r2dreamer/ \
      --exclude runs --exclude .venv --exclude wandb
    rsync -a <devbox>:~/workspace/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta25/ \
      ~/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta25/
    # (new files this wave: configs/env/genesis_pick_v5d4_delta.yaml, replay_gate.py
    #  --raw-dir/--uids auto, distributions/networks/_base_/envs bounded-dist changes,
    #  eval_genesis action-mode fix -- ALL required)

### dDP demo set: convert + gate ON THE CLUSTER (harvest lives there)
    module load anaconda/2025.06.0 && conda activate <genesis env>
    cd ~/genesis_pickaplace
    python baselines/rl/to_dreamer_demos.py --src paper_smoke/m1all_harvest \
      --scope pick --action-encoding delta_joint --delta-cap 0.025 --grant-slack 48 \
      --dst ~/dreamerv3-torch/demonstrations/genesis_m1all_delta25
    # gate (r2d venv; model-demo ICs are random draws -> --raw-dir mode):
    cd ~/r2dreamer && ~/r2d_venv/bin/python replay_gate.py \
      --demo-dir ~/dreamerv3-torch/demonstrations/genesis_m1all_delta25 \
      --delta-cap 0.025 --leash-mult 5 --uids auto \
      --raw-dir ~/genesis_pickaplace/paper_smoke/m1all_harvest
    # GATE MUST PASS before launching dDP seeds.

### Launch (5 seeds per source; sbatch defaults resolve demo dir for v5*)
    cd ~/genesis_pickaplace
    for S in 0 1 2 3 4; do
      CONFIG=genesis_pick_v5d4_delta SEED=$S \
        LOGDIR=$HOME/r2dreamer/runs/dH_R2D_s$S sbatch cluster/sbatch_r2dreamer.sh
    done
    for S in 0 1 2 3 4; do
      CONFIG=genesis_pick_v5d4_delta SEED=$S \
        DEMO_DIR=$HOME/dreamerv3-torch/demonstrations/genesis_m1all_delta25 \
        LOGDIR=$HOME/r2dreamer/runs/dDP_R2D_s$S sbatch cluster/sbatch_r2dreamer.sh
    done
    # optional denser lottery sampling: SAVE_EVERY=50000 (default 1e5)

Official statistics = CLUSTER runs only (same-machine baseline rule); the local
s0/s1 pair is the pilot. Do NOT pass REINJECT/DUPLICATE for v5d4 (baked; the
sbatch passes them only if explicitly set).

## No-rsync path (2026-08-11): the port travels IN THIS REPO
`cluster/r2dreamer_port.tar.gz` = the complete uncommitted r2dreamer port
(30 source files, 86KB; modified + new files against upstream commit 546e4fa).
Apply on the cluster:

    git clone https://github.com/NM512/r2dreamer.git ~/r2dreamer 2>/dev/null || true
    cd ~/r2dreamer && git checkout 546e4fa
    tar xzf ~/genesis_pickaplace/cluster/r2dreamer_port.tar.gz -C ~/r2dreamer
    ls configs/env/genesis_pick_v5d4_delta.yaml replay_gate.py || echo APPLY-FAILED

Demo sets are NOT in git (rsync-only rule). For the dH delta25 set, in order of
preference:
1. If `~/genesis_pickaplace/baselines/episodes_pick_pruned_img/` exists on the
   cluster: regenerate CANONICALLY (byte-equivalent to the dev box):
       python baselines/rl/to_dreamer_demos.py --src baselines/episodes_pick_pruned_img \
         --scope pick --action-encoding delta_joint --delta-cap 0.025 --grant-slack 48 \
         --dst ~/dreamerv3-torch/demonstrations/genesis_pick_pruned_delta25
   then replay-gate it (uids 232 242 243, must pass 3/3).
2. Else: VPN rsync from the dev box (the set is only 75MB -- minutes).
The dDP set is generated on the cluster either way (harvest lives there; see
the STATISTICS WAVE section above).
