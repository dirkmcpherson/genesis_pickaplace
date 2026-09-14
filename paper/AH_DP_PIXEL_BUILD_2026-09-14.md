# (ah) {Diffusion Policy} on pixel observations — build log

**Registration:** `paper/PHASE_PLAN_2026-09-04.md` § `### (ah)` (registered 2026-09-14 ~12:00 as "(ag)", renamed (ah) at 13:10 — the other box had already used (ag) that morning; before any build).
**Repo:** `genesis_pickaplace` branch `ladder-unify-2026-09-11`. **Nothing was submitted to the cluster from
this session** — the cluster was read-only here; the submit command is at the end.

---

## 1. The action question (settled before anything was built)

The DP end-to-end arm of record ((n)/(ab)) and the pixel demonstration sets do **not** carry the same action
stream, and the difference is not cosmetic:

| file | key | what it is |
|---|---|---|
| recorder tape `<uid>.npz` (`baselines/record_demos.py:310`) | `actions` (n,7) f32 | **ABSOLUTE command at the window end**: 6 joint targets in rad (`env._dj_target`, i.e. the delta integrator's target after the 4-step window) ++ grip motor in 0..1 |
| recorder tape `<uid>.npz` | `actions_delta` (n,7) f32 | the normalized `[-1,1]^7` decision the env EXECUTED |
| r2dreamer-native tape `genesis-<uid>-*.npz` (`baselines/rl/to_dreamer_native.py`) | `action` (T,7) f32, T = n+1 | `actions_delta` **backward-shifted**: `action[t]` led INTO `image[t]`, `action[0] = 0` |

`baselines/convert_to_lerobot.py` reads `d['actions']`, so **the DP action space is absolute joint targets +
grip**, and `baselines/eval_e2e.py` applies it as such (`d = clip((q* - env._dj_target) / (REPEAT * DJ_CAP), -1, 1)`,
grip `0..1 -> [-1,1]`), executed hold-4 through the env's delta integrator. `cluster/sbatch_dp_e2e.sh`'s own
header says the same thing.

So the `_img` native sets **cannot** supply the action column: they carry deltas, not absolute targets.
Reconstructing the absolute targets by integrating the deltas was NOT attempted — it would be a re-derivation of
a column that already exists.

**What was done instead, and why it makes gate 2 exact rather than approximate:** the pixel dataset keeps
`states` and `actions` from the SAME recorder tapes the (ab) dataset was built from
(`$LAB/genesis_pickaplace/baselines/matched_w3/{dHfull_all,dDPfull_first}/*.npz`) and takes ONLY the pixels from
the `_img` re-execution. The action column is therefore the (ab) column by construction, not by comparison.

**Measured, on 4 tapes (2 per arm) rsynced locally:**

- native `action[1:]` == recorder `actions_delta`, **byte for byte**, 4/4 tapes (max abs diff 0.0);
- native `state[:n]` == recorder `states`, **byte for byte**, 4/4 tapes — `relabel_reward.py` copies the source
  tape's columns and replaces only `reward` and `image`, so the state column is the recorded one;
- the (ab) lerobot parquet's `action` rows == recorder `actions` bytes, 2/2 human episodes checked directly;
- `raw actions[:, :6]` sits ~0.002–0.005 rad from `states[:, :6]` (a target the arm is tracking), range
  ±2.6 rad — absolute joint targets, not deltas, confirmed numerically.

**One disclosure that follows from the above.** The images come from the re-execution; the state/action columns
come from the original recording. The re-execution is not bit-identical to the recording: the set manifest
reports `can_dev_max_m 0.703`, `can_dev_p50_m 0.0091`, 36 of 74 tapes over 1 cm. This does **not** contaminate
this arm's observation, because (a) the can/goal pose is removed from the dataset entirely, and (b) the
proprioceptive part deviates by `rz_joint_dev_max_rad` — 0.00039 rad on the tape checked. It is the same
image/action pairing the (af) pixel world models trained on.

## 2. Converter path

`baselines/convert_to_lerobot.py` gained two **opt-in env vars**; unset, the script's output is byte-identical
to before (verified: the 2-episode state-only rebuild matches the (ab) dataset of record's `action`,
`observation.state` and `observation.environment_state` columns byte for byte, and its feature set and fps).

- `LEROBOT_IMAGES_FROM=<native _img dir>` — pixels from the native set, everything else from the recorder
  tapes. Three per-tape gates: native `T == n+1`; native `action[1:]` == tape `actions_delta` byte for byte;
  native `state[:n]` == tape `states` byte for byte; plus a non-placeholder (`im.any()`) check.
- `LEROBOT_NO_ENV_STATE=1` — `observation.environment_state` is not a dataset feature at all.

`baselines/verify_px_lerobot.py` gates the finished dataset: feature set exactly
`{observation.state (8,), observation.images.top (64,64,3), observation.images.wrist (64,64,3), action (7,)}`,
no `environment_state`, same fps, same episode list as the state dataset of record, and action bytes equal to
it episode for episode (plus the recorder tapes as a third witness). Negative control: pointing it at a state
dataset fails with exit 1 and a named reason.

`baselines/dp_runner.py` now sends `observation.environment_state` only when the checkpoint declares it
(every state checkpoint does — unchanged for them; a pixel checkpoint has no such feature).

**Image dtype is `image` (PNG), NOT `video`** — differs from the instruction and from the July image runs.
`torchcodec` cannot load its shared libraries in either environment: locally `OSError: libavutil.so.57: cannot
open shared object file`, on the cluster `RuntimeError: Could not load libtorchcodec` in
`$LAB/condaenv/genesis`. An mp4 dataset would build and then fail to decode at train time. Cost measured:
~5.2 KB per frame-pair, so ~150 MB (human, 29 221 frames) + ~170 MB (machine, 32 851 frames).

## 3. Evaluator

`baselines/eval_e2e.py --camera-rig` (default OFF, every existing call unchanged): builds `FullTaskEnv` with
`camera_rig=True` and passes `env.genv.rig_obs` as the DP runner's `rig_provider` (top = channels 0:3, wrist =
3:6 — the split the demonstrations were rendered with). Without the flag, a checkpoint that consumes image keys
is refused with a message naming the flag. No cropping is added: the centre crop at evaluation is lerobot's own
eval-mode behaviour inside `DiffusionRgbEncoder`. `metrics.json` stamps `camera_rig`.
`cluster/e2e_eval_cells.sh` gained `CAMERA_RIG=1` (default unset) which passes the flag through and prints it.

## 4. Launchers

- `cluster/ah_build_pixel_sets.sh` — builds `$DEMO_ROOT/{dHfull_all,dDPfull_first}/lerobot_px`, gates the
  sources (raw manifest contract/scope/n_kept; `_img` `repeat.json` `images == 'rendered'` and
  `nonzero_frac_min > 0`) and runs `verify_px_lerobot.py` on the result.
- `cluster/sbatch_dp_px.sh` — `sbatch_dp_e2e.sh` with the observation swapped. Same 100k steps, batch 64,
  registry check, preempt-safe resume, final-checkpoint-only disk rule, 150 GB disk guard. Adds
  `--policy.crop_shape=[56,56] --policy.crop_is_random=true`, re-runs the action gate in the job, and reads the
  **resolved** config back off the saved checkpoint, printing `POLICY-CONFIG` / `POLICY-INPUTS` and asserting
  `crop_shape == (56,56)`, `crop_is_random is True`, `pretrained_backbone_weights is None`, `input_features`
  exactly the three pixel/proprio keys, `observation.state` shape `(8,)`. Runs on `-p gpu --qos=normal`.
- `cluster/submit_ah_dp_px.sh` — refuses pinned trees, requires the new files in the clone, re-runs the dataset
  gate for both arms before submitting anything, disk guard, seeds 0–3 per arm, job names
  `ah_dp_px_<dH|dM>_s<seed>`. `DRYRUN=1` prints each job's plan and submits nothing.

## 5. Differences from the registration (each is a decision, not an oversight)

1. **Image storage is PNG, not video** — torchcodec is broken in both environments (§2).
2. **Augmentation is a crop, not a pad-and-shift.** The registration already says this ("lerobot crops rather
   than pads, disclosed"). 56 of 64 px keeps the ±4 px translation budget and discards ~23 % of the field.
3. **Evaluation ladder and tip guard are not named by the registration.** The launcher stamps
   `ladder=nested_sparse10`, `tip_guard=not_in_hand` into the sidecar (which `eval_e2e.py` takes as its source
   of truth) so the cells terminate where the (af) pixel world-model `home` cells terminate. The (ab) DP cells
   used the defaults (`staged`, `grip`). DP training reads no reward, so this changes only where an episode
   ends and which stage is the paid terminal — but it is a real choice and both are env-var overridable
   (`LADDER=`, `TIP_GUARD=`).
4. **In-job eval sets** are `hold15 rnd30` with `ISO=0` (the registration names rnd30 MODE and hold15 MODE).
   `spots60` and the isolated cells are not produced in job; add `SETS=`/`ISO=1` if wanted.
5. **"mode" for DP.** `e2e_eval_cells.sh` refuses a `mode` cell for `KIND=dp` ("a diffusion policy has no
   deterministic mode"); the cells are `sample`. The registration's "rnd30 MODE" is, for DP, this cell.

---

## 6. Local smoke (registration gate 1), 2026-09-14

Data staged under `/home/james/data/genesis_pickaplace/ag_pixel/` (never in the repo; the directory name
predates the (ag)→(ah) rename). Env `/home/james/workspace/genesis_pickaplace/.venv-eval/bin/python`
(lerobot 0.4.5 fork, genesis 0.2.1, torch 2.7.0+cu126).

**Inputs rsynced read-only from the cluster:** 2 recorder tapes + manifest per arm
(`$LAB/genesis_pickaplace/baselines/matched_w3/{dHfull_all,dDPfull_first}`), the matching native `_img`
tapes + `repeat.json` (`$W/demos_state_full/*_rns10h_img`), and the whole (ab) human lerobot dataset
(`matched_w3/dHfull_all/lerobot`, 4.1 MB) as the byte-identity reference.

### 6.1 The pairing gates, measured (4 tapes: dH 100000/100001, dM 100000/100003)

```
nat action[0] all-zero: True | nat action[1:] == raw actions_delta bytes: True | maxabs 0.0   (4/4)
nat state[:n]  == raw states       bytes: True | maxabs all-17 0.0 | maxabs proprio-8 0.0     (4/4)
raw actions[:, :6] range -2.59 .. 2.47 rad (absolute targets), grip 0.0 .. 1.0
raw states[:, :6] vs actions[:, :6] median |diff| 0.0018 .. 0.0054 rad  (a target being tracked)
```

### 6.2 Build

```
LEROBOT_IMAGES_FROM=<img dir> LEROBOT_NO_ENV_STATE=1 \
  python baselines/convert_to_lerobot.py <raw dir> <ds root> 8 4 top,wrist image
```
human 2 tapes → 425 frames, 2.2 MB; machine 2 tapes → 1200 frames, 6.2 MB (≈5.2 KB per frame-pair).
Both datasets' features:

```
observation.state float32 (8,) | action float32 (7,)
observation.images.top image (64,64,3) | observation.images.wrist image (64,64,3)
fps 7.5   — and NO observation.environment_state
```

### 6.3 Gate 2 — the action column

```
PX-DATASET-OK ds_dH  : episodes 2 frames 425  | action bytes == ds_dH_ref  on 2/2 episodes and == the recorder tapes on 2/2
PX-DATASET-OK ds_dDP : episodes 2 frames 1200 | action bytes == ds_dDP_ref on 2/2 episodes and == the recorder tapes on 2/2
negative control (a STATE dataset through the same gate): exit 1, "extra: ['observation.environment_state'],
  missing: ['observation.images.top', 'observation.images.wrist']"
```
and against the **real (ab) dataset of record**, directly on its parquet: its `action` rows equal the recorder
tapes' `actions` bytes for both human episodes checked — which is what makes the identity above transitive.

**Converter regression (the default path must not have moved):** rebuilding the 2 human tapes state-only with
no env vars reproduces the (ab) dataset of record's `action`, `observation.state` **and**
`observation.environment_state` columns byte for byte, with the same feature set and fps.

### 6.4 Train, 200 steps

```
lerobot-train --dataset.root=<ds_dH> --policy.type=diffusion --seed=0 --batch_size=64 --steps=200 \
  --save_freq=200 --policy.crop_shape="[56,56]" --policy.crop_is_random=true --policy.device=cpu
```
CPU (the box's GPU was 5.7 GB into an unrelated job and the run OOM'd at batch 64 on the first attempt;
it is a config smoke, not a timing measurement). 267 M params, ~11.5 s/step, 200/200 completed.

### 6.5 Gate 3 — the RESOLVED config, read back off the saved checkpoint

```
POLICY-CONFIG crop_shape=[56, 56] crop_is_random=True vision_backbone=resnet18
  pretrained_backbone_weights=None use_group_norm=True separate_rgb_encoder_per_camera=False
  spatial_softmax_num_keypoints=32 n_obs_steps=2 horizon=16 n_action_steps=8
POLICY-INPUTS observation.images.top(3, 64, 64) observation.images.wrist(3, 64, 64) observation.state(8,)
POLICY-CONFIG-OK: pixel inputs only, 8-d proprio, random 56x56 crop, backbone from scratch
```

### 6.6 Evaluation with the rig, one hold15 start

Negative control first — the same command **without** `--camera-rig`:
```
FATAL: checkpoint consumes ['observation.images.top', 'observation.images.wrist'] but no rig_provider given.
       This checkpoint consumes camera images but --camera-rig was not given; a pixel policy scored without
       its cameras is a different experiment.        (exit 1)
```
With the flag (CPU, `--limit 1`):
```
[eval-e2e] ladder 'nested_sparse10' from the checkpoint sidecar
[eval-e2e] tip_guard 'not_in_hand' from the checkpoint sidecar
[eval-e2e] camera rig ON: rig_obs() -> top ch0:3, wrist ch3:6 (64x64 each), rendered once per decision
[eval-e2e] shelf_top_z 0.170 ...; ladder nested_sparse10 {'home': 10.0} terminal ('home',)+tipped
[eval-e2e] dp policy on cpu; hold-4: q* -> delta clip((q*-target)/(4*0.025)), grip 0..1 -> [-1,1]
ep0: uid252 timeout ... (300 decisions, r=0.0, 306.9 s)
[eval-e2e] 1 episodes (sample, hold): picked 0/1 placed_v2 0/1 contact_push 0/1 slide_success 0/1
           nested_v2 0/1 farside 0/1 slide_event 0/1 home 0/1 nested_honest 0/1 | tipped 0 timeout 1
```
`metrics.json` carries `camera_rig: true`, `terminal_stage: home`, the full `stage_counts` block and the
ladder stamp. **A 200-step policy doing nothing is the expected outcome** — this smoke tests the pipeline,
not performance. Rig rendering costs ~1.0 s/decision on this 6-core CPU; in job the policy runs on the
allocated GPU, so the cell cost is dominated by the simulator as before.

Independently checked first, so a rig failure would not have been mistaken for a policy failure:
`rig_obs()` on a fresh CPU world returns `(64,64,6) uint8`, nonzero fraction 1.0, mean 107.3 — the same
value the demonstration tapes carry (`rz_image_mean` 107.43 on uid 100000, set minimum 99.4).

**Not done:** a 200-step train on the machine arm. Its dataset was built and passed the same gates; the
training configuration is arm-independent and the code path is identical, so a second 40-minute CPU train
would have added no information. Say so rather than imply both arms were trained.

## 7. To run it on the cluster

These commits are **local only** (not pushed). The clone needs them, so push the branch or rsync the tree
first. Then, on `pax`:

```bash
LAB=/cluster/tufts/shortlab/jstale02
# 0. fresh clone (never a pinned tree; submit_ah_dp_px.sh refuses gp_ladderN / gp_ac / gp_aa4 / gp_e2e / gp_root)
#    git clone <this repo> $LAB/gp_ah && cd $LAB/gp_ah && git checkout ladder-unify-2026-09-11

# 1. build the two pixel datasets (one CPU job, ~20 min; writes matched_w3/<set>/lerobot_px)
cd $LAB/gp_ah && GENESIS_PICKAPLACE_ROOT=$PWD sbatch cluster/ah_build_pixel_sets.sh

# 2. when it prints PX-DATASET-OK for both sets, submit the 8 runs
module load anaconda/2025.06.0 && conda activate $LAB/condaenv/genesis
cd $LAB/gp_ah
GP=$LAB/gp_ah DRYRUN=1 bash cluster/submit_ah_dp_px.sh    # prints each job's plan, submits nothing
GP=$LAB/gp_ah bash cluster/submit_ah_dp_px.sh             # 8 jobs: ah_dp_px_{dH,dM}_s{0..3}
```

`-p gpu --qos=normal`, one GPU per job, ~16 h walltime, final checkpoint only. Each job re-runs the action
gate, asserts the resolved policy config, and writes `hold15` + `rnd30` sample cells with the rig.
