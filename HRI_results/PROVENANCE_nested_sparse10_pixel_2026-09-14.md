# Provenance: `nested_sparse10` from PIXELS — every run command, per algorithm (2026-09-14)

Written so a person can reproduce every run of amendments (ad)/(ae)/(af)/(ag) without this session. Every
command below is copied from a launch record, a Slurm log or a dry-run printout, not retyped from memory; the
file that holds the original is named beside each. Paths: `LAB=/cluster/tufts/shortlab/jstale02`,
`W=$LAB/wm_fix_2026-09-03`. Registrations: `paper/PHASE_PLAN_2026-09-04.md` amendments (ad) pixel recipe, (ae)
local 4 v 4, (af) cluster batch + rev 1, (ag) 16 v 16 + 2M. Tallies: `paper/AE_PIXEL_HUMAN_VS_MACHINE_2026-09-13.md`
(local), `paper/AF_PIXEL_CLUSTER_TALLY_2026-09-14.md` (cluster).

## 0. The conditions

| condition | learner | seeds (arm human `dH` / machine `dM`) | budget | where | trees |
|---|---|---|---|---|---|
| {dv3 = DreamerV3 losses in the r2dreamer chassis}, pixels | `model.rep_loss=dreamer` | local s0–3 (ae); cluster s4–7 (af); cluster s8–15 (ag) | 1M (ae, af), **2M** (ag) | pop-os / cluster | below |
| {r2dreamer = the port's contrastive loss}, pixels | `model.rep_loss=r2dreamer` | cluster s0–2 (af); s3–15 (ag) | 1M (af), 2M (ag) | cluster | below |
| ramp control, pixels (`nested_ramp`, not sparse10) | `dreamer` | cluster s0 both arms (af) | 1M | cluster | below |
| {RLPD} pixels | DrQ-style shared encoder in the LN-critic ensemble | cluster s0–1 (af); s2–15 (ag) | 250k decisions (= 1M sim frames) | cluster | below |

Commits (every run stamps its own): genesis_pickaplace `$LAB/gp_px` @ `9b50280` ((af) first submission), `2f33ac6`
((af) two resubmitted seeds: launcher compile-cache path only), `8492a2e` ((ag)); r2dreamer `$W/r2dreamer_px` @
`0b1b9d8` (all cluster world-model runs; the tree exists in the repo as
`cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle`, head `refs/heads/main` = 0b1b9d8); RLPD tree
`$LAB/gp_pxr` @ `9841633` ((af)) and `8492a2e` ((ag)); local r2dreamer `~/workspace/r2dreamer` @ `ada434d` (=
0b1b9d8's parent lineage + local-only commits; the local runs stamp `ada434d`). Branch of the repo:
`ladder-unify-2026-09-11`.

## 1. Environments

- Cluster world model + RLPD: `$LAB/r2d_venv` (python 3.11, torch with inductor/Triton; the venv the
  `wmfix_full.sbatch` launcher activates). RLPD trains in the same venv (`baselines/rl/train_rlpd.py`).
- Local world model: `~/workspace/r2dreamer/.venv`.
- Dataset builds and every evaluator that opens a Genesis world: `~/workspace/genesis_sim2real/venv` locally
  (python 3.10, genesis-world 0.2.1 editable); on the cluster the same interpreter as the launcher.
- World of record everywhere: `GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6` for the env and
  `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` for `eval_genesis.py` (it defaults to `base` otherwise — every cell must
  show a `[sim-variant]` line).

## 2. Demonstration sets (identical for every condition)

Source state sets: human `dHfull_all` (74 tapes, every in-the-wild attempt, `to_dreamer_native.py` from the stage
records) and machine `dDPfull_first` (72 tapes, the Diffusion-Policy teacher's FIRST attempt per start; teacher
trained on the pruned human set). The pixel sets re-execute each tape's action stream through `FullTaskEnv` with the
camera rig on and write the reward the `nested_sparse10` ladder pays (`paper/PX_IMAGE_DEMOS_2026-09-12.md`):

```bash
cd ~/workspace/genesis_pickaplace
export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace
export GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 R2D_SIM_VARIANT=gc_kp4_riser3_shelf6
~/workspace/genesis_sim2real/venv/bin/python baselines/rl/relabel_reward.py \
  --in  /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all \
  --out /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img \
  --ladder nested_sparse10 --tip-guard not_in_hand \
  --sim-variant gc_kp4_riser3_shelf6 --images --procs 8
# machine: --in .../dDPfull_first --out .../dDPfull_first_rns10h_img (same flags)
```

Results of record (from each set's `repeat.json`): human 74 tapes, Σ 130 = 10 × 13 `home`, actions sha256
`77bc4875…`; machine 72 tapes, Σ 140 = 10 × 14 `home`, actions sha256 `671614c5…`. **Built on pop-os (AMD 5950X,
32 cores, AVX2).** The cluster-built state sets `_rns10h` (64-core nodes) pay 12/12 `home`; the 1 human and 2 machine
extra `home` tapes are the known hardware-class divergence of re-execution — disclosed, not corrected. Cluster copies:
`$W/demos_state_full/{dHfull_all,dDPfull_first}_rns10h_img` by rsync (verified there: `manifest.json` `images:
rendered`, `state_only: false`, `total_reward` 130/140). Ramp-control sets `_rnrh_img` built the same way with
`--ladder nested_ramp` (human on pop-os with a +0.266 Σ disclosure, machine on pax019).

## 3. World-model runs (both representation losses)

### 3a. Cluster (af)/(ag) — `cluster/submit_px_batch.sh` → `cluster/wmfix_full.sbatch`

Submission (one job per seed, preempt QOS; `submit_px_batch.sh` pins the trees, checks the set's manifest, prints
`DISK-OK`, refuses an existing run dir):

```bash
cd $LAB/gp_px
# (af), 1M:
GP_PIN=9b50280 R2_PIN=0b1b9d8 REP=dreamer   ARM=dH SEED=4 bash cluster/submit_px_batch.sh
GP_PIN=9b50280 R2_PIN=0b1b9d8 REP=r2dreamer ARM=dM SEED=0 bash cluster/submit_px_batch.sh
GP_PIN=9b50280 R2_PIN=0b1b9d8 REP=dreamer   ARM=dH SEED=0 LADDER=nested_ramp bash cluster/submit_px_batch.sh   # control
# (ag), 2M:
STEPS=2000000 MILES='[500000,1000000,1500000,2000000]' GP_PIN=8492a2e R2_PIN=0b1b9d8 \
  REP=dreamer ARM=dH SEED=8 bash cluster/submit_px_batch.sh
```

What that runs (the `sbatch` line the script prints; from `$W/px_submit_2026-09-13.log` / `_2026-09-14_ag.log`):

```
env R2_TREE=$W/r2dreamer_px GENESIS_PICKAPLACE_ROOT=$LAB/gp_px GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 \
    LADDER=nested_sparse10 TIP_GUARD=not_in_hand ENVCFG=genesis_full_pixel TAG=dreamer EVAL_SETS="hold rnd" \
    R2_LONG_RUN=1 R2_MILESTONES='[500000,1000000]' \
  sbatch -J px_dreamer_dH_s4 -p gpu,preempt --qos=preempt --nice=0 $LAB/gp_px/cluster/wmfix_full.sbatch \
    dHfull_all_rns10h_img 4 1000000 model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8
```

The trainer command the launcher resolves and prints (copied from `$W/slurm/px_dreamer_dH_s8_3692502.out`; the
r2dreamer runs differ only in `model.rep_loss=r2dreamer`, seed, set and logdir):

```
$LAB/r2d_venv/bin/python train.py env=genesis_full_pixel seed=8 env.steps=2000000 \
  env.demo_dir=$W/demos_state_full/dHfull_all_rns10h_img env.ladder=nested_sparse10 env.far_release=false \
  env.tip_guard=not_in_hand env.return_clamp=10.0 model.return_clamp=10.0 buffer.max_size=5e5 \
  logdir=$W/runs/full_r2d_state_dHfull_all_rns10h_img_dreamer_s8 \
  env.actor_dist=bounded_normal env.act_entropy=3e-5 model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8
```

run from `$W/r2dreamer_px` (cwd) with `GENESIS_PICKAPLACE_ROOT=$LAB/gp_px`, `GENESIS_SIM_VARIANT` /
`R2D_SIM_VARIANT` = `gc_kp4_riser3_shelf6`, and (since 2f33ac6) `TORCHINDUCTOR_CACHE_DIR` /
`TRITON_CACHE_DIR` on the node's `/tmp`. `configs/env/genesis_full_pixel.yaml` = the state config with exactly:
`encoder/decoder cnn_keys: 'image'`, `state_slice: 8` (q(6) + grip motor + grip effort; the can/goal pose columns are
removed), `image_aug: shift4` (DrQ random shift, replicate-pad 4, before encoder and decoder). Milestones
`milestones/online_<N>.pt` + `.json`; `latest.pt` at the end; in-job final cells `fresh_eval_{hold15,rnd30}_{mode,sample}`.
Provenance stamps per run: `ladder_provenance.json` (ladder stamp, `r2dreamer_tree`, git rev, env config, rep_loss,
image_aug, state_slice, encoder keys), `step_contract.json`, `.hydra/`, and the `[obs]`/`[image]`/`[image_aug]`/
`[tree]`/`[cache]` lines in `$W/slurm/px_<name>_<jobid>.out`. Job ids: (af) 3685135–3685151 + resubmitted
3685494/3685495 (the originals 3685143/3685150 died in a Triton compile on NFS: `$W/runs_failed/`), (ag)
3692502–3692543.

### 3b. Local (ae) — `~/runs_dv3_local/launch_px_ae.sh <dH|dM> <seed>`

Exact command (from `~/runs_dv3_local/LAUNCH_px_ae_dH_s3.txt`, written by the launcher at each start):

```bash
cd ~/workspace/r2dreamer
export R2_LONG_RUN=1 R2_MILESTONES='[500000,1000000]' GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
       R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 MUJOCO_GL=egl
.venv/bin/python train.py env=genesis_full_pixel seed=3 env.steps=1000000 \
  env.demo_dir=/home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img \
  env.ladder=nested_sparse10 env.far_release=false env.tip_guard=not_in_hand env.return_clamp=10.0 \
  model.return_clamp=10.0 model.rep_loss=dreamer model.image_aug=shift4 env.state_slice=8 \
  env.actor_dist=bounded_normal env.act_entropy=3e-5 buffer.max_size=5e5 \
  logdir=/home/j/runs_dv3_local/dv3px_sparse10_dHfull_all_rns10h_img_rlDreamer_s3
```

Same recipe as the cluster runs (the local runs stamp r2dreamer `ada434d`, whose pixel code is the same commits
43a0e3c/066162a as 0b1b9d8 minus the provenance-stamp commit). The chain launcher runs one seed at a time (GPU
box, ~45 fps), order dM0 dH1 dM1 dH2 dM2 dH3 dM3 (human s0 = the (ad) run); each start writes
`LAUNCH_px_ae_<arm>_s<seed>.txt` and starts the series watcher (§4b).

## 4. Evaluation cells

### 4a. Cluster — the milestone sweep → `ln14_milestone_eval.sbatch` → `eval_genesis.py`

```bash
# hourly, from the login node (idempotent; pixel runs only; every milestone; CPU nodes through the preempt QOS)
SWEEP_MODE=cpu64pre RUN_FILTER=_img MS_FILTER=all MAXJOBS=24 bash $W/ln14_milestone_sweep.sh
```

(`$W/ln14_milestone_sweep.sh` = `cluster/ln_r2_milestone_sweep.sh`, md5 135ad0b1 at the (ag) deploy;
`$W/ln14_milestone_eval.sbatch` = `cluster/ln_r2_milestone_eval.sbatch`.) For each (run, milestone) it copies the
checkpoint to `$W/ln_milestone_cells/<run>/<milestone>/latest.pt`, verifies its sha256 against the milestone
sidecar, and submits one 8-CPU job pinned to 64-physical/64-logical-core nodes (`--exclude` = every other node;
the sbatch re-asserts both counts) which runs, with `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` and the r2dreamer tree
read from the run's `ladder_provenance.json`:

```
$PY eval_genesis.py --checkpoint $CELL/latest.pt --episodes 30 --mode mode   --max-steps 1200 \
    --ic-file $GP/baselines/eval_ics.json --ic-set rnd  --seed 0 --out $CELL/rnd30_mode   --device cpu
$PY eval_genesis.py --checkpoint $CELL/latest.pt --episodes 15 --mode mode   --max-steps 1200 \
    --ic-file $GP/baselines/eval_ics.json --ic-set hold --seed 0 --out $CELL/hold15_mode  --device cpu
$PY eval_genesis.py --checkpoint $CELL/latest.pt --episodes 30 --mode sample --max-steps 1200 \
    --ic-file $GP/baselines/eval_ics.json --ic-set rnd  --seed 0 --out $CELL/rnd30_sample --device cpu
```

Each cell dir holds `metrics.json` (`headline_stages`, `per_episode` with `outcome` ∈ {home, tipped, timeout},
`video`), one mp4 per episode (`ep<N>_rnd<N>_<outcome>.mp4`), `provenance.json`, `trees.json`,
`ladder_provenance.json`. Cell of record: `rnd30_mode` (30 random starts, deterministic actions); `hold15_mode` =
15 demonstration starts.

### 4b. Local — the series watcher (`~/runs_dv3_local/dv3px_sparse10_series_<arm>_s<seed>/snapshot_series.sh`)

Every ~100k online steps it copies `latest.pt` to `ck_<counter>/` (with the trainer's last-60-episode record beside
it) and, one Genesis world at a time, runs on the same box:

```bash
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
cd ~/workspace/r2dreamer && .venv/bin/python eval_genesis.py --checkpoint $D/latest.pt --episodes 30 --mode mode \
    --max-steps 1200 --ic-file $GENESIS_PICKAPLACE_ROOT/baselines/eval_ics.json --ic-set rnd  --seed 0 --device cuda --out $D/fresh_eval_rnd30_mode
# and --episodes 15 --ic-set hold --out $D/fresh_eval_hold15_mode
```

Statistic of record for a local seed = mean rnd30 MODE `home` over the 0.3–1.0M snapshots (8 cells); the pooled
16 v 16 statistic (ag) = mean of the 0.5M and 1.0M rnd30 MODE cells, which every local and cluster seed has.

## 5. {RLPD} pixels — `cluster/sbatch_rlpd_px.sh` → `baselines/rl/train_rlpd.py` → `cluster/sbatch_rlpd_px_eval.sh`

```bash
cd $LAB/gp_pxr
# (af):
GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr OBS=pixels LADDER=nested_sparse10 TIP_GUARD=not_in_hand ARM=dH SEED=0 \
  sbatch -J e2e_rlpd_px_dH_s0 cluster/sbatch_rlpd_px.sh          # ARM=dDPfirst for the machine set
# (ag), with dense checkpoints:
CKPT_EVERY=25000 GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr OBS=pixels LADDER=nested_sparse10 TIP_GUARD=not_in_hand \
  ARM=dH SEED=2 sbatch -J e2e_rlpd_px_dH_s2 cluster/sbatch_rlpd_px.sh
```

The trainer command the launcher runs (its `DRYRUN=1` printout, 2026-09-14; `--ckpt-every 0` for the (af) seeds):

```
python baselines/rl/train_rlpd.py --steps 250000 --scope full --ladder nested_sparse10 --tip-guard not_in_hand \
  --demo-format segment --demo-dir $W/demos_state_full/dHfull_all_rns10h_img --obs pixels --image-aug shift4 \
  --buffer-size 300000 --action-mode delta_joint --delta-ref target --action-repeat 4 \
  --train-max-steps 1200 --eval-max-steps 1200 --eval-freq 0 --gamma 0.99 --backup-entropy off \
  --per-member-ln off --pick-hold-reward off --pick-shaping off --utd 10 --ensemble-size 10 --subset-size 2 \
  --demo-batch 128 --demo-shaping off --pick-shaping-terminal-zero on --demo-terminal-guard on \
  --sim-variant gc_kp4_riser3_shelf6 --ckpt-every 25000 --ckpt-fracs 0.4,1.0 \
  --out-dir baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s2 --run-name e2e_rlpd_px_dH_s2 \
  --project genesis_paper --seed 2 --device cuda
```

Encoder: DrQ-v2-style shared CNN trained through the LayerNorm critic ensemble, actor detached
(`baselines/rl/rlpd_pixel.py`; the sidecar `rlpd_final.action_mode.json` records `encoder_wiring`). Outputs:
`ckpt_040/`, `ckpt_100/`, `rlpd_final.zip` + sidecar, `<N>_steps.zip` snapshots when `CKPT_EVERY` > 0,
`episode_rollouts.jsonl`; the Slurm log is `$LAB/gp_pxr/e2e_rlpd_px_<jobid>.out` (DEMO-SHA, `[ladder]`,
`[image]`, `[image_aug]`, `[Q-WATCHDOG]` lines). After `TRAIN-OK` the job submits its own pinned CPU eval
(64/64-core nodes, preempt QOS since 09-13 22:30):

```
env GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr KIND=sac CKPT=<run>/rlpd_final.zip OUT=<run> ARM=dH SEED=2 \
    SIM_VARIANT=gc_kp4_riser3_shelf6 SETS="hold15 rnd30" MODES="sample mode" ISO=1 ISO_SETS=rnd30 VIDEO_SETS=rnd30 \
    PAR=8 REQUIRE_CORES=64 sbatch -J e2e_rlpd_px_eval_dH_s2 --exclude=<non-64-core + SMT nodes> cluster/sbatch_rlpd_px_eval.sh
```

which is `cluster/e2e_eval_cells.sh` with `EVAL_SCRIPT=baselines/eval_e2e_px.py`, i.e. per (set, mode):

```
python baselines/eval_e2e_px.py --kind sac --checkpoint <run>/rlpd_final.zip --ic-file <ic file of the set> \
  --ic-set <rnd|hold> --out <run>/fresh_eval_<set>_<mode>[_iso] --mode <sample|mode> --seed 0 --max-steps 1200 \
  --sim-variant gc_kp4_riser3_shelf6 --arm dH --tag <set>_<mode> [--require-cores 64] [--role preview]
```

Job ids: (af) 3685153–3685156 (evals 3697480–3698039); (ag) 3692544–3692571.

## 6. Deviations a reproducer should know

- Two (af) world-model jobs died in their first `torch.compile` with `OSError: [Errno 116] Stale file handle`
  (Triton artifact read back from the per-job NFS cache); the launcher now puts the compile caches on node-local
  `/tmp` (2f33ac6). No effect on the computation.
- The CPU evaluations run through the preempt QOS (`SWEEP_MODE=cpu64pre`; RLPD eval sbatch header) because the
  normal QOS CPU cap was held by other runs; same node class, a preempted cell re-runs.
- The pixel demonstration sets carry `git=...-dirty` in their ladder stamp: the local repo had uncommitted
  (unrelated) files at build time. The build code path is `relabel_reward.py --images` as committed (PX-1 lane).
- Disk floor in the launchers: 150 GB until 2026-09-14 09:10, 100 GB after (user).
- The other workstation's state-based `nested_sparse10` runs (amendment (ac)) used the cluster-built `_rns10h`
  sets (12/12 `home`) and `$W/r2dreamer_ladderN` @ 0cf3d9e — the pixel tree is that tree plus the pixel commits.

## 7. Rise time, rise-time lag, learning curves, steady state, ignition — how they are computed

One script produces every figure, table and CSV of the by-phase analysis:
`baselines/diagnostics/px_phase_analysis.py`. It reads a **local rsync mirror** of the cluster and of
`~/runs_dv3_local`; the cluster is read-only and nothing on it was created or modified.

```bash
~/workspace/genesis_sim2real/venv/bin/python baselines/diagnostics/px_phase_analysis.py \
  --data-root ~/data/genesis_pickaplace/px_analysis_2026-09-14 \
  --out-dir   paper/figures/px_phase_2026-09-14 \
  --tex       paper/figures/px_rise_time_by_phase.tex
```

Outputs are listed file-by-file in `paper/figures/px_phase_2026-09-14/README.md`. The script is
deterministic: the bootstrap RNG is seeded (`--seed`, default 20260914) and the jitter of the
strip plots is a per-seed `RandomState`.

### 7.1 The two kinds of number, never mixed

- **TRAINING RECORD** — sampled actions, on the policy's own training starts, read from the sticky
  per-episode stage flags the trainer prints (`episode/train_ep_<phase>`). These are the learning
  curves, the rise times, steady state (a) and the solid ignition bars.
- **EVAL CELLS** — deterministic (`mode`) actions, fixed starts, a fresh process per cell, read from
  each cell's `metrics.json` `headline_stages`. These are steady state (b) and the hatched ignition
  bars. Cell of record: `rnd30_mode` (30 fixed random starts); `hold15_mode` is also harvested.

Every figure caption and the table caption state which kind it is.

### 7.2 Data paths

| mirror path (`~/data/genesis_pickaplace/px_analysis_2026-09-14/`) | source |
|---|---|
| `W/runs/full_r2d_state_{dHfull_all,dDPfull_first}_{rns10h,rnrh}_img_{dreamer,r2dreamer}_s*/console.log` (+ `step_contract.json`, `ladder_provenance.json`) | `$W/runs/` (§3a) |
| `W/ln_milestone_cells/<run>/online_<N>/{rnd30_mode,hold15_mode}/metrics.json` | `$W/ln_milestone_cells/` (§4a) |
| `LAB/gp_pxr/e2e_px/e2e_rlpd_px_{dH,dDPfirst}_s{0,1}/episode_rollouts.jsonl` and `fresh_eval_{rnd30_mode,hold15_mode}/metrics.json` | `$LAB/gp_pxr/baselines/rl/checkpoints/e2e_px/` (§5) |
| `local/runs_dv3_local/dv3px_sparse10_*_rlDreamer_s{0..3}/console.log` | `~/runs_dv3_local/` on pop-os (§3b) |
| `local/runs_dv3_local/dv3px_sparse10_series[_{dH,dM}_s{0..3}]/ck_<counter>/fresh_eval_{rnd30_mode,hold15_mode}/metrics.json` | the series watcher (§4b); human s0's series dir is `dv3px_sparse10_series/` |

Runs whose name contains `smoke` are skipped; so are `final/` cells (a duplicate of the last
milestone), `*_smt128` cells, `*.preempted*` and `runs_failed`.

### 7.3 The x-axis

`online step = counter − origin`, where `origin` is the first `[<counter>]` line of the run's
console log. Verified per log and cross-checked against `step_contract.json`
`prefill_counter_origin`: **117624** for every human (`dHfull_all`) run and **149952** for every
machine (`dDPfull_first`) run, on the cluster and locally alike. Local series checkpoints
`ck_<counter>` are converted with the same origin.

{RLPD} logs `step` in **decisions**. Its x-axis is `step × 4` (`--action-repeat 4`), so its
250 k-decision budget is 1 M sim frames and shares the world-model x-axis. This conversion is
stated in the figure caption and in the table caption.

### 7.4 Phases

`picked`, `placed_v2` (the `placed` field is used only if `placed_v2` is absent), `farside`,
`slide_event`, `home`, `tipped` — the sticky `episode/train_ep_*` flags. **The {RLPD} episode
record carries no `farside`, `slide_event` or `home` key at all.** They are reported as *absent*,
never as zero: the table prints `— (absent)`, the learning-curve panel prints "not recorded
(absent, not zero)", and {RLPD} has no bar in the training half of the ignition figure.

### 7.5 Learning curves

Per run, a rolling mean over the **last 30 training episodes** of each phase flag, plotted against
the online step of the window's last episode. Per condition (learner × arm), each run's curve is
linearly interpolated onto a common **50 k-step grid** (only inside the run's own range), then the
grid points are averaged over seeds; the band is a **95 % bootstrap CI over seeds** (2000
resamples, percentile). The band is **truncated where fewer than 3 seeds have data** — this is why
the {r2dreamer} bands stop near 1.05 M even though two seeds per arm reach 2 M. Where an arm has
fewer than 3 seeds ({RLPD}, the ramp control) no band is drawn at all: the individual seed traces
are plotted instead and the caption says so. Every plotted point is in `px_learning_curves.csv`.

### 7.6 Rise time

Per run and phase: the **first online step at which the rolling-30 rate reaches the threshold**
(≥ 0.5 for every phase; `home` is additionally reported at ≥ 0.1), and separately the online step
of the **first episode that reached the phase at all** (`first_event_step` in
`px_rise_time_per_seed.csv`; it is in the CSV, not in the table).

Table cells are the **median over seeds, with [min, max], in thousands of online sim steps**,
followed by `(crossed / determined seeds)`. A run that has neither crossed nor finished its budget
is **undetermined**, not a failure: it leaves the denominator and is reported as "p run." in the
cell. `n/a` means no determined seed crossed. LaTeX: `paper/figures/px_rise_time_by_phase.tex`
(booktabs `table*` inside `\resizebox`); per-seed values:
`paper/figures/px_phase_2026-09-14/px_rise_time_per_seed.csv`.

### 7.6a Rise-time LAG relative to `picked`

A rise time is an absolute clock reading, so it mixes two things: how fast a seed got going at
all, and the ORDER in which its phases arrived. The lag figures remove the first. For one run and
one threshold `t`,

```
lag(phase) = rise(t, phase) - rise(t, `picked`)
```

in online sim steps, plotted in thousands, for `placed_v2`, `farside`, `slide_event`, `home` and
`tipped`. `rise` is the same quantity as §7.6: the first online step at which the run's
rolling-`ROLL`-episode (30) rate for that flag reaches `t`. The reference is always the run's own
`picked` crossing, so no number is carried between seeds and nothing is interpolated. Code:
`lag_rows()` and `fig_rise_lag()` in `baselines/diagnostics/px_phase_analysis.py`; the thresholds
are the CLI flag `--lag-thresholds` (default `0.5,0.1`), one figure per value, written as
`paper/figures/px_phase_2026-09-14/fig_rise_lag_thresh{0p5,0p1}.{png,pdf}`. Every plotted point
and every non-crossing is in `px_rise_lag_per_seed.csv`.

Two thresholds are produced because the sparse ladder makes the ≥ 0.5 crossings of the late rungs
nearly simultaneous — under `nested_sparse10` a seed that reliably reaches `home` reaches
`farside` and `slide_event` on the way, in the same episodes — so the ≥ 0.1 variant, the first
crossing of a one-in-ten rate, is the one that separates the rungs. Neither is more correct; they
answer different questions and both are reported.

Layout: one panel per learner condition ({dv3} local, {dv3} cluster, {r2dreamer} cluster), human
and machine offset side by side at each phase position, x in task order, y = lag in k online sim
steps, with `y = 0` drawn as the `picked` reference line. One point per seed (deterministically
jittered by seed), the thick bar is the **median over the seeds that crossed**, and the count of
crossers is printed under each group. Status handling, which the figure never collapses into a
number:

- **never** — the reference crossed but the phase did not, inside the run's record. The seed is an
  OPEN marker on a dotted band at the top edge, labelled `Nx never` beside it, and it is excluded
  from the median. It is never drawn at lag 0. The band is drawn only when some seed needs it.
- **no_reference** — the run's own `picked` never crossed, so no lag is defined for it at all. The
  seed is not plotted and the count is stated in that panel's title.
- **absent** — the learner's episode record has no such flag. {RLPD} is for this reason excluded
  from the figure entirely (§7.4), and so is the `nested_ramp` control, which pays a different
  ladder.

Negative lags are meaningful and are plotted as such: `tipped` normally rises BEFORE `picked`,
because early policies knock the can over before they ever lift it.

### 7.7 Steady state

Two versions, in two panels of `fig_steady_state`, never combined:

- **(a) training record** — the mean of each phase flag over the episodes in the **last 20 % of the
  run's achieved online steps** (a run needs ≥ 5 episodes there). A run that has not finished its
  budget is drawn as a **hollow** point: its last 20 % is not a steady state.
- **(b) eval cells** — the mean of `home` (and `picked`) over **all `rnd30_mode` milestone cells at
  ≥ 0.5 M online steps** for that run. A seed with no such cell yet is absent from the panel, not
  zero.

### 7.8 Ignition

Per condition × arm, two criteria with Wilson 95 % CIs:

- **training**: the fraction of seeds whose rolling-30 `home` rate reaches **≥ 0.5** within budget.
  Undetermined seeds (see §7.6) are out of the denominator and shown as `+p?`.
- **eval**: the fraction of seeds with **≥ 1 `home` episode in any `rnd30_mode` cell** (any
  milestone). Seeds with no cell yet are out of this denominator.

### 7.9 Caveats a reader must carry

1. **Training record ≠ eval cells.** Sampled actions on self-generated starts run well above
   deterministic actions on fixed random starts for every condition here. Quote the kind.
2. **Mirror refreshed 2026-09-15; two seed counts moved.** The {r2dreamer} (ag) seeds s3–s15
   finished overnight, so that condition is now **16 seeds per arm** (only human s15 is short, at
   1.75 M of 2 M), and the {RLPD} (ag) seeds s2–s11 have started, so {RLPD} is now 12 human / 11
   machine seeds of which only s0/s1 per arm have finished — the rest are 0.10–0.68 M in and their
   curves, steady state and ignition are early-training numbers, not final ones. A seed that has
   not crossed a threshold and has not finished is undetermined rather than a failure (§7.6).
   In the {r2dreamer} panels of the lag figures every seed crosses every phase at both
   thresholds, so there are no open top-edge markers there.
3. **{RLPD} budget unit.** 250 k decisions, converted to 1 M sim frames by `× action_repeat 4`
   for a shared x-axis. Its training record also lacks the three late-phase flags (§7.4), so
   {RLPD} contributes only `picked`, `placed_v2` and `tipped` rise times, and only an eval
   ignition bar.
4. **The ramp control is a different ladder.** `nested_ramp` pays differently; it is plotted for
   reference and must not be pooled with the `nested_sparse10` conditions.
5. **Hardware is not uniform.** The 4 + 4 {dv3} local seeds ran on the pop-os workstation
   (AMD 5950X, AVX2, local GPU); every other condition ran on pax cluster GPU nodes. This is
   marked in every caption that shows them side by side.
6. **The local human s0 run** (the (ad) run) was launched at 2 M and deliberately stopped at its
   1 M milestone by the (ae) chain. The script carries that as an explicit note
   (`stopped_at_1M_by_chain`) so it is not mis-read as an interrupted run.
7. **Two seeds per arm is a pilot.** The {RLPD} pixel cells are n = 2 v 2; the (ag) {RLPD} seeds
   s2–s15 were still queued.

Commit: `8be9038` ("px phase analysis: rise time, learning curves, steady state, ignition for the
nested_sparse10 pixel conditions") on branch `ladder-unify-2026-09-11`. The rise-time LAG figures
(§7.6a) and the 2026-09-15 mirror refresh were added in the following commit on the same branch.
