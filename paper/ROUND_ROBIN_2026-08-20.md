# Round robin {RLPD, dv3, r2dreamer, DP} x {dH, dDP, dR2D} — Wednesday vet, Thursday launch

Written 2026-08-18 night. Four hardened sbatches, one RUN_REGISTRY, all datasets
built. Every script: git/fixed-code gate -> dataset provenance gate -> stale-env
guard -> RUN_REGISTRY check+register (refuses exact repeats, warns on git-only
diffs, skipped on preemption requeue) -> train -> in-job eval -> ONE greppable
result line -> sidecar. DRYRUN=1 on every script exits before any module/conda.

Companion: CLUSTER_ROUND_2026-08-17.md (last round's results + amendments),
ALGORITHM_STATE_2026-08-18.md (per-algo state and expectations).

## 0. Two overnight verdicts feed Thursday's go/no-go (read Wednesday morning)
1. **dv3-MS-at-HEAD** (2 seeds, March demo recipe, seed-fixed code, local): bar
   >=0.8 by 200k on >=1 seed = March reproduced -> dv3 launches Thursday.
   <0.5 on both = HEAD regression suspect -> dv3 gets a bisect, launches later.
2. **RLPD dense** (3 seeds, pick_shaping, local): bar >=2/3 seeds >=3/15 fresh
   AND pooled >= 0.16 -> PICK_SHAPING=on becomes a 4th RLPD column Thursday.
   Miss -> dense stays off the round robin.

## 1. WEDNESDAY: rsync block (dev box, one shot; datasets travel by rsync ONLY)
```bash
LAB=jstale02@login.cluster.tufts.edu:/cluster/tufts/shortlab/jstale02
G=~/workspace/genesis_pickaplace/baselines
D=~/workspace/dreamerv3-torch/demonstrations
# --- RLPD (npz sets) ---
rsync -av $G/m1all_harvest/                 $LAB/genesis_pickaplace/baselines/m1all_harvest/
rsync -av $G/episodes_pick_phase_dppruned/  $LAB/genesis_pickaplace/baselines/episodes_pick_phase_dppruned/
#   (episodes_pick_phase_all + episodes_champion_pick already on cluster from the n=16 wave)
# --- DP (lerobot sets + their raw npz for provenance gates) ---
rsync -av $G/lerobot_dH_pick/               $LAB/genesis_pickaplace/baselines/lerobot_dH_pick/
rsync -av $G/episodes_pick/                 $LAB/genesis_pickaplace/baselines/episodes_pick/
rsync -av $G/lerobot_x2x2v2_jobs_jact/      $LAB/genesis_pickaplace/baselines/lerobot_x2x2v2_jobs_jact/
rsync -av $G/lerobot_dR2D_pick/             $LAB/genesis_pickaplace/baselines/lerobot_dR2D_pick/
#   (dDP lerobot set is built IN-JOB from m1all_harvest by sbatch_dp.sh)
# --- r2dreamer (dreamer-format, +100 term, delta25) ---
rsync -av $D/genesis_r2dchamp_delta25/      $LAB/dreamerv3-torch/demonstrations/genesis_r2dchamp_delta25/
rsync -av $D/genesis_m1all_delta25/         $LAB/dreamerv3-torch/demonstrations/genesis_m1all_delta25/
#   (genesis_pick_pruned_delta25 = dH already on cluster from wave 3)
# --- dv3 (dreamer-format, stride-4 stamped) + the seed-fixed code tree ---
rsync -av --exclude 'logdir/' --exclude 'logs*/' --exclude 'test_logs*/' --exclude 'venv/' \
  --exclude 'wandb/' --exclude 'demonstrations/' --exclude 'demo_videos/' --exclude 'random_episodes/' \
  --exclude 'debug_frame_stack*/' --exclude '__pycache__/' --exclude '.git/' --exclude '*MagicMock*' \
  ~/workspace/dreamerv3-torch/ $LAB/dreamerv3-torch/     # code only (~<100MB); tree is 37G otherwise
rsync -av $D/genesis_m1all_msr_delta25_r4/     $LAB/dreamerv3-torch/demonstrations/genesis_m1all_msr_delta25_r4/
rsync -av $D/genesis_champion_msr_delta25_r4/  $LAB/dreamerv3-torch/demonstrations/genesis_champion_msr_delta25_r4/
#   (genesis_pick_msr_delta25_r4 = dH already on cluster from msrecipe)
```
Then on the login node: `cd /cluster/tufts/shortlab/jstale02/genesis_pickaplace && git pull`
(needs the 08-18 hardening commits through e2e3a81 or later).

## 2. WEDNESDAY: smoke plan (login node, in this order; ~1.5h total)
Each step names its PROOF. If a proof is missing, paste the .out into the session
and stop — Wednesday is for fixing, Thursday for launching.

### 2a. DRYRUN every arm of every script (no GPU; ~1 min)
```bash
export GENESIS_PICKAPLACE_ROOT=$PWD; unset DEMO_DIR DEMO DATASET RAW
for A in dH dR2D dDP dHpruned; do ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd.sh | head -2; done
for A in dHpruned dH dDP dR2D;  do ARM=$A SEED=0 DRYRUN=1 bash cluster/sbatch_dp.sh   | head -2; done
for A in dH dDP dR2D; do ARM=$A SEED=0 CONFIG=genesis_pick_v5d4c_delta \
  VENV=/cluster/tufts/shortlab/jstale02/r2d_venv R2D_DIR=/cluster/tufts/shortlab/jstale02/r2dreamer \
  DRYRUN=1 bash cluster/sbatch_r2dreamer.sh | grep -E "^\[dry\] (ARM|demo_dir)"; done
cd ../dreamerv3-torch && for A in dH dDP dR2D; do REPO_DIR=$PWD CONDA_ENV=genesis \
  RUNS="TAG=smoke_$A ARM=$A SEED=0 STEPS=20000" DRYRUN=1 bash sbatch_genesis_multi.sh | grep -E "gate OK|\[dry\] TAG"; done; cd -
```
PROOF: every line is `[dry] ...` / `PROVENANCE-OK` / `fixed-code gate OK`; NO
FATAL, NO module/conda output. dv3's dR2D line must show NDEMO=52.

### 2b. Gate-refusal checks (no GPU; proves the guards bite)
```bash
ARM=bogus SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd.sh          # FATAL: ARM=bogus
DEMO_DIR=baselines/m1all_harvest ARM=dH SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd.sh   # FATAL: stale env
```

### 2c. One short REAL seed per script (GPU; each <30 min)
```bash
ARM=dH   SEED=0 STEPS=5000            sbatch cluster/sbatch_rlpd.sh
ARM=dR2D SEED=0 STEPS=3000 EVAL_EPS=2 sbatch cluster/sbatch_dp.sh
ARM=dR2D SEED=0 CONFIG=genesis_pick_v5d4c_delta STEPS=2e4 \
  VENV=/cluster/tufts/shortlab/jstale02/r2d_venv R2D_DIR=/cluster/tufts/shortlab/jstale02/r2dreamer \
  sbatch cluster/sbatch_r2dreamer.sh
cd ../dreamerv3-torch && REPO_DIR=$PWD CONDA_ENV=genesis WANDB=1 \
  RUNS="TAG=smoke_dH ARM=dH SEED=0 STEPS=20000" sbatch sbatch_genesis_multi.sh; cd -
```
PROOF per .out, in order: `REGISTRY-OK` -> config/demo-load line with the
expected count (RLPD dH: 91 eps/83465 transitions; DP dR2D: 66 eps; r2d dR2D:
`Demo prefill: {'episodes': 52 ...}`; dv3 dH: 67 demos + `SEED-OK ... recorded=0`)
-> `REGISTRY-REGISTERED` -> the result line (`SWEEP-RESULT` / `DP-RESULT` /
r2d eval line / `DV3-RESULT`) -> job exit 0. `cluster/RUN_REGISTRY.jsonl` gains
4 lines.

### 2d. Duplicate refusal (the registry's reason to exist)
Re-submit the RLPD line from 2c verbatim. PROOF: `.out` shows `REGISTRY-REFUSE
... full-key match`, no `[cfg]` line, exit 2. Then with `DUPLICATE_OK="smoke"`
it prints `REGISTRY-DUPLICATE-OK` and runs.

## 3. THURSDAY: submit map (after 2a-2d all pass and the §0 verdicts are in)
| algo | dH | dDP | dR2D | seeds/arm | notes |
|---|---|---|---|---|---|
| RLPD | done (16) | **NEW** | done (16) | 16 (top-up 16-19 = 20) | + PICK_SHAPING=on column if §0.2 passes; dHpruned optional |
| DP | dHpruned done (8); dH-unpruned **optional n=8** | done (8) | **NEW** | 8 | eval-of-record config verbatim |
| r2dreamer | done (34) | done (20) | **NEW** | 10 | 3M steps/seed; dR2D = 52 pixel demos (fresh image harvest — WM needs pixels; state 52-vs-66 in captions) |
| dv3 | **NEW on fixed seeds** | **NEW** | **NEW** | 3-5 | GATED on §0.1; ignition = nonzero picked at eval, entropy fingerprint secondary |

Commands (all from the genesis_pickaplace root; seeds shown for the NEW cells):
```bash
for S in $(seq 0 15); do ARM=dDP SEED=$S sbatch cluster/sbatch_rlpd.sh; done
for S in $(seq 0 7);  do ARM=dR2D SEED=$S sbatch cluster/sbatch_dp.sh; done
for S in $(seq 40 49); do ARM=dR2D SEED=$S CONFIG=genesis_pick_v5d4c_delta \
  VENV=/cluster/tufts/shortlab/jstale02/r2d_venv R2D_DIR=/cluster/tufts/shortlab/jstale02/r2dreamer \
  sbatch cluster/sbatch_r2dreamer.sh; done
# dv3 (only if §0.1 passes):
cd ../dreamerv3-torch && for A in dH dDP dR2D; do for S in 0 1 2; do REPO_DIR=$PWD CONDA_ENV=genesis WANDB=1 \
  RUNS="TAG=rr_${A}_s$S ARM=$A SEED=$S STEPS=300000" sbatch sbatch_genesis_multi.sh; done; done; cd -
# optional: RLPD top-up to n=20 both existing arms; RLPD dense; DP dH-unpruned n=8; RLPD dHpruned
```
Per-user GPU cap is ~10 concurrent — everything else queues; `--requeue` covers
preemption. Aggregation afterwards: `grep -h SWEEP-RESULT rlpd_*.out`,
`grep -h DP-RESULT dp_*.out`, `grep -h DV3-RESULT`, r2d via wandb.

## 4. Known caveats to carry into captions
1. dR2D for the WM arms = 52 pixel demos (fresh harvest); RLPD/DP dR2D = 66
   state tapes. Same teacher, same ICs, different render pass and count.
2. dDP DP row trains on the 63 SUCCESS stems only (BC convention); RLPD/r2d/dv3
   dDP consume all 93 (fails as zero-reward negatives). Registered convention.
3. dH means UNPRUNED for every RL/WM row; DP's functional human arm is
   dHpruned (see RESULTS_MATRIX notation).
4. Cluster absolute rates carry the same-machine caveat vs local history; the
   within-round contrasts are clean.

## §0 VERDICT UPDATE (08-19 16:40)
1. **dv3-MS-at-HEAD: PASS, 2/2 seeds at eval_success 0.9** (wandb
   dreamer_v3_maniskill, msHEAD_demo_s{0,1}-joint, finished 08-18 ~21:00,
   before the dev-box power loss). Bar was >=0.8 on >=1 seed. Code-era caveat
   CLOSED: HEAD reproduces the March positive; the msrecipe genesis null
   stands as task-x-recipe. **dv3 is GO for Thursday.**
2. RLPD dense: s1 completed 100k before the outage (checkpoint on disk);
   its fresh-process sweep is running now; s0/s2 relaunched (dev-box power
   loss killed them mid-run). Verdict expected late 08-19; if it slips past
   Thursday submit time, dense is NOT submitted (the bar requires the data,
   not the deadline).

## DENSE EXTENSION (08-19, user directive: if dense passes, repeat for r2d + dv3)
Plumbing landed (default-off, byte-identical; gamma matched per agent):
- RLPD: PICK_SHAPING=on (sbatch_rlpd.sh), gamma 0.998.
- r2dreamer: CONFIG=genesis_pick_v5d4c_delta_shaped (r2dreamer 4d98b56), gamma 0.999.
- dv3: SHAPED=1 in the spec (dreamerv3-torch d645765; overlays
  genesis_pick_msrecipe_shaped), gamma 0.997.
Submit lines IF the RLPD dense bar passes (verdict in wandb + handoff):
```bash
for S in 0 1 2 3 4 5; do ARM=dH SEED=$S PICK_SHAPING=on sbatch cluster/sbatch_rlpd.sh; done
for S in 50 51 52 53; do ARM=dH SEED=$S CONFIG=genesis_pick_v5d4c_delta_shaped \
  VENV=/cluster/tufts/shortlab/jstale02/r2d_venv R2D_DIR=/cluster/tufts/shortlab/jstale02/r2dreamer \
  sbatch cluster/sbatch_r2dreamer.sh; done
cd ../dreamerv3-torch && for S in 0 1 2; do REPO_DIR=$PWD CONDA_ENV=genesis WANDB=1 \
  RUNS="TAG=rr_dH_shaped_s$S ARM=dH SEED=$S SHAPED=1 STEPS=300000" sbatch sbatch_genesis_multi.sh; done; cd -
```
NOTE: requires ONE more rsync before submit (the dense plumbing edits):
```bash
rsync -av ~/workspace/dreamerv3-torch/sbatch_genesis_multi.sh ~/workspace/dreamerv3-torch/dreamer.py \
  ~/workspace/dreamerv3-torch/configs.yaml ~/workspace/dreamerv3-torch/envs/genesis.py \
  $LAB/dreamerv3-torch/ --relative 2>/dev/null || for f in sbatch_genesis_multi.sh dreamer.py configs.yaml envs/genesis.py; do \
  rsync -av ~/workspace/dreamerv3-torch/$f $LAB/dreamerv3-torch/$f; done
rsync -av ~/workspace/r2dreamer/envs/genesis.py ~/workspace/r2dreamer/envs/__init__.py $LAB/r2dreamer/envs/
rsync -av ~/workspace/r2dreamer/configs/env/genesis_pick_v5d4c_delta_shaped.yaml $LAB/r2dreamer/configs/env/
# + git pull in genesis_pickaplace (full_env pick_shaping_gamma)
```
