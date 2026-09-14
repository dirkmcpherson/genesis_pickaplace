# {RLPD} pixel observation path (lane PXR-1, 2026-09-13)

**What this is.** `baselines/rl/train_rlpd.py --obs pixels`: the {RLPD} recipe of record (SB3 SAC, UTD 10,
E10/Z2 LayerNorm ensemble critics, 50/50 demo batches, γ 0.99, delta_joint cap 0.025 / leash 5×, repeat 4,
`FullTaskEnv(scope='full')` with the ladder and tip guard as constructor arguments) trained from **pixels
+ proprioception instead of the 17-dim state**, on the same recipe the pixel world model runs
(`nested_sparse10`: +10 on `home`, terminal `home`+tipped; tip guard `not_in_hand`; world
`gc_kp4_riser3_shelf6`; demos from the `_img` sets). Every state-based run is unchanged: `--obs state` is the
default and the state path's code is the same code (§5).

**Status.** Pipeline built and smoke-tested locally (2 000 decisions, 3 evaluation episodes). **No learning
claim** — the smoke shows the plumbing works, not that the policy learns (§7). Cluster launcher built and proven
by ONE smoke on a preempt L40S + a 64-core eval job (§9); no batch submitted.

## 1. Observation and recipe

| | pixel world model (`env=genesis_full_pixel`, PX_PIXEL_CONFIG_2026-09-12.md) | {RLPD} `--obs pixels` (this lane) |
|---|---|---|
| image | `genv.rig_obs()`: top RGB ++ wrist RGB, (64,64,6) uint8, rendered once after reset and once after every decision | same call, same points in the loop (`PixelObsWrapper.observe`) |
| proprio | `state_slice: 8` = state[:8] = q[:6], gripper motor, grip effort | `state[:8]`, same columns |
| can / goal pose | not observed | not observed |
| augmentation | `image_aug: shift4` (replicate-pad 4, random crop, one offset per sequence) | `--image-aug shift4` (replicate-pad 4, random crop, one offset per SAMPLE, obs and next_obs independently — the DrQ/DrQ-v2 form for a non-sequential learner) |
| encoder | Dreamer CNN encoder/decoder | DrQ-v2: 4 × Conv3×3 (32 ch, stride 2/1/1/1) + Linear→50, LayerNorm, Tanh; owned by the critic, actor detached (§3) |
| demos | `dHfull_all_rns10h_img` / `dDPfull_first_rns10h_img` (rendered image column) | the same sets, through `full_demos.segment_pixels_full` (§4) |
| ladder / tip guard / far_release | `nested_sparse10` / `not_in_hand` / off | same, passed explicitly (`--ladder --tip-guard`), in the sidecar and the `[ladder]` stamp |
| action | delta_joint, repeat 4, cap 0.025 | same (unchanged) |

## 2. Files

| file | what changed |
|---|---|
| `baselines/rl/rlpd_pixel.py` (NEW, ~330 lines) | `PixelObsWrapper` (gym.Wrapper over a `FullTaskEnv(camera_rig=True)`, the ONE observation function), `random_shift`, `DrQEncoder`, `PixelProprioExtractor` (SB3 features extractor: cat(CNN(image), proprio) → 58), `DetachedFeaturesActor`, `PixelRLPDPolicy` (DrQ-v2 wiring in `_build`, `encoder_wiring()` stamp), `PixelDemoData` (frames stored once per tape row, gathered by index, pinned host memory on cuda), `PixelRLPDSAC` (`_prep_batch` hook applies the shift and stamps/asserts on its first call), `make_rlpd_pixel` |
| `baselines/rl/rlpd_sac.py` | `_cat_obs` (dict-aware concat; `th.cat` for the flat state tensor as before), `RLPDSAC._prep_batch` no-op hook called once per gradient step after the 50/50 concat, `make_rlpd(..., policy_class, algo_class, buffer_size, policy_kwargs_extra, algo_kwargs_extra)` so the pixel path reuses the ONE list of pinned hypers. Defaults reproduce the previous behaviour exactly. |
| `baselines/rl/full_demos.py` | `segment_pixels_full(seg_dir, expect, ladder)`: calls `segment_transitions_full` (same files, order, reward/terminal checks and census) and adds the `image` column + `state[:, :8]`; refuses a set whose `repeat.json` is not `images: rendered` / `state_only: false`; asserts proprio == state[:8] on both ends of every transition; counts tapes at the ladder max. `print_pixel_census`. |
| `baselines/rl/train_rlpd.py` | flags `--obs {state,pixels}` (default state), `--image-aug {shift4,none}`, `--buffer-size` (default 300 000), `--cnn-feature-dim` (50); under pixels: env built with `camera_rig=True`, wrapped, `make_rlpd_pixel`, pixel demo loader + `PixelDemoData`, `[obs]` stamps with the wiring asserted, sidecar keys `obs / image_aug / image_shape / proprio_dim / image_source / cnn_feature_dim / encoder / encoder_wiring / buffer_size`. `[cfg]` line gains `obs=`. |
| `baselines/eval_e2e_px.py` (NEW) | a COPY of `baselines/eval_e2e.py` (the live one is read at run time by in-flight cluster jobs; untouched) with the `# PX:` differences only: `--kind sac`, sidecar must say `obs == 'pixels'`, env `camera_rig=True` + `PixelObsWrapper`, `act(o)` on the dict, `pix.observe()` after reset and after each decision, metrics.json gains `obs / image_shape / proprio_dim / n_renders / image_aug / cnn_feature_dim`, `amendment='n+ladder-unify+px'`. IC injection, predicates, settle, stage records, hardware stamps: identical. |
| `baselines/tests/test_rlpd_pixel.py` (NEW) | 5 offline checks (no Genesis, CPU): wiring, shift, demo buffer, two real `train()` calls, save/load round trip. Skips where stable-baselines3 is absent (the sim2real venv); passes under `.venv-eval`. |

Python: {RLPD} runs under **`.venv-eval`** (stable-baselines3 2.8.0, torch 2.7.0+cu126, genesis 0.2.1,
gymnasium 1.2.3, numpy 2.2.6) — `~/workspace/genesis_sim2real/venv` has no stable-baselines3.

## 3. Design choices (and where they deviate from the brief)

1. **Shared encoder, DrQ-v2 style, achieved.** SB3's own `share_features_extractor=True` does the
   opposite of DrQ-v2 (the actor's optimizer owns the shared extractor and the critic runs it without
   grad), so `PixelRLPDPolicy._build` wires it by hand: the critic is built with its own extractor (its
   optimizer covers every critic parameter incl. the encoder; `EnsembleCritic.forward` runs the extractor
   with grad because `share_features_extractor` is False), the actor is built ON the critic's extractor
   object and `DetachedFeaturesActor.extract_features` wraps it in `no_grad`, and the actor optimizer is
   built over the actor's non-extractor parameters only. `encoder_wiring()` proves all four facts at
   runtime and the trainer asserts them (smoke line `[obs] ... wiring={...}` in §6).
2. **Target encoder.** The target critic carries its own polyak-averaged encoder copy (SB3 builds
   `critic_target` with a separate extractor and `polyak_update` covers it), i.e. the DrQ-v1 / SAC-AE
   target-encoder scheme with the encoder tau equal to the critic tau (0.005). DrQ-v2 instead encodes
   next_obs with the live encoder. Kept SB3's structure; disclosed.
3. **Trunk shared; proprio raw.** DrQ-v2 shares the convs and gives actor and critic separate
   Linear→LayerNorm→Tanh trunks; here the trunk (feature_dim 50) is inside the shared extractor. The 8-dim
   proprio is concatenated RAW to the 50-dim image feature (features_dim 58) rather than through a proprio
   MLP inside the extractor: an MLP inside the shared extractor would be trained by the critic loss only,
   leaving the actor unable to shape its own proprio embedding; each head's first 256-unit layer is its
   proprio MLP. (The brief said "MLP on proprio"; this is the deviation.)
4. **Augmentation.** `random_shift` = replicate-pad 4 px, random integer crop back to 64×64, one offset per
   sample, applied INSIDE `train()` to the image of `obs` and `next_obs` of the concatenated online++demo
   batch (both halves, every gradient step). Integer shifts (DrQ v1), not DrQ-v2's bilinear sub-pixel
   shift. `--image-aug none` is the ablation switch; the setting travels in the sidecar. The first update
   prints `[image]` (online and demo halves: min/max/mean/non-zero-frame fraction; asserts both halves are
   real frames) and `[image_aug]` (max |aug−raw|, fraction of frames changed; asserts the augmented batch
   differs).
5. **Channel order.** SB3 treats the (64,64,6) uint8 Box as an image space (channels are not checked) and
   wraps the env in `VecTransposeImage`, so the replay buffer, the policy and the saved observation space
   are channel-first (6,64,64); `PixelDemoData` stores frames channel-first too; `predict()` on a raw
   (64,64,6) frame transposes back (`maybe_transpose`). `preprocess_obs` divides by 255; the encoder
   subtracts 0.5.
6. **Memory.** Online buffer: one row = 2 × 24 576 B of uint8, so the recipe's 300 000 rows = **14.7 GB of
   host RAM** (`--buffer-size` exists; the smoke used 5 000). Demo half: human 29 295 frames = 721 MB,
   machine 36 906 = 907 MB, pinned on cuda, gathered by index with a pinned staging buffer.
7. **No env-var levers.** `--obs`, `--image-aug`, `--buffer-size`, `--cnn-feature-dim` are flags; a state
   run that carries `--image-aug`/`--cnn-feature-dim` values other than the defaults is refused; `--obs
   pixels` is refused outside the end-to-end recipe (scope full + `--demo-format segment`, eval_freq 0).

## 4. Demonstration loader (verified, tool output)

`segment_pixels_full` on both `_img` sets (`expect` = the launcher's stamps, `ladder='nested_sparse10'`):

```
[segments] dHfull_all_rns10h_img: 74 tapes -> 29221 transitions, Sigma reward 130 over 13 rewarded transitions (0 tapes with >1 grant), 18 terminal; decisions p50 388 min 1 max 600; 74 distinct rollouts; reward values {10.0: 13}
[pixels] dHfull_all_rns10h_img: 74 tapes -> 29295 frames (720 MB uint8), 29221 transitions, proprio dim 8; image nonzero-frame frac min 1.000, frame mean 99.39..109.33; Sigma reward 130; 13 tapes rewarded, 13 tapes at the ladder max
  arrays: frames (29295, 64, 64, 6) uint8 proprio (29295, 8) obs_idx (29221,) actions (29221, 7) rewards sum 130 n_rewarded_transitions 13 dones 18 load 5.0s
[segments] dDPfull_first_rns10h_img: 72 tapes -> 36834 transitions, Sigma reward 140 over 14 rewarded transitions (0 tapes with >1 grant), 22 terminal; decisions p50 600 min 1 max 600; 72 distinct rollouts; reward values {10.0: 14}
[pixels] dDPfull_first_rns10h_img: 72 tapes -> 36906 frames (907 MB uint8), 36834 transitions, proprio dim 8; image nonzero-frame frac min 1.000, frame mean 102.02..109.50; Sigma reward 140; 14 tapes rewarded, 14 tapes at the ladder max
  arrays: frames (36906, 64, 64, 6) uint8 proprio (36906, 8) obs_idx (36834,) actions (36834, 7) rewards sum 140 n_rewarded_transitions 14 dones 22 load 2.4s
```

Σ 130 / 140 and 13 / 14 `home` tapes are what the loader sees (= PX_IMAGE_DEMOS_2026-09-12.md §3 for the
human set). The 18 / 22 terminals are the SOURCE tapes' tip terminals: the paid `home` frame carries no
terminal flag in these sets (`is_terminal` unchanged from the source — PX_IMAGE_DEMOS §4.5, HANDOFF §3e
`--cut-at-terminal`), so in the demo half `home` pays +10 and the tape continues with zero reward. Same rows
as the state path; not this lane's to change.

## 5. The state path is unchanged

* `train_rlpd.py --obs state` (default) builds the env with `camera_rig=False`, calls `make_rlpd` with the
  same arguments as before plus `buffer_size=300_000` (the old hard-coded value) and `DemoData` as before.
* `RLPDSAC.train`: `_cat_obs` is `th.cat` for tensors; `_prep_batch` returns its inputs. No numeric change.
* `baselines/tests`: **69 passed** (`~/workspace/genesis_sim2real/venv/bin/python -m pytest baselines/tests -q`,
  after the edits; the new pixel test skips there and passes under `.venv-eval`, 5/5).
* `eval_e2e.py` untouched; `eval_e2e_px.py` refuses a checkpoint whose sidecar is not `obs: pixels`, and
  `eval_e2e.py` on a pixel checkpoint fails before any episode (negative control in §6).

## 6. Smoke (pop-os, 2026-09-13; every line from tool output)

Box: pop-os, AMD Ryzen 9 5950X (16 cores / 32 threads, AVX2), one GPU (12 GB) owned by the (ae) pixel
world-model training run (3.6 GB, 6 CPU worker worlds) for the whole smoke; this smoke held **340 MiB** of GPU
memory (`nvidia-smi --query-compute-apps`), ONE Genesis world at a time (train, then eval, then the negative
control), `OMP_NUM_THREADS=8`, `.venv-eval`. Outputs: `/home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/`
(`train.log`, `eval.log`, `train/`, `eval_hold15_mode/`). Command = §8 with `--steps 2000 --buffer-size 5000`,
human `_img` set, seed 0, cuda.

### 6.1 Launcher-style `[ladder]` stamp (the `PYL` block of `cluster/sbatch_rlpd_e2e.sh`, run before training)

```
== PX-RLPD smoke start Sun Sep 13 08:07:36 PM EDT 2026 host=pop-os tree=known-good-2026-08-27-1007-g6128211-dirty venv=.venv-eval
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-1007-g6128211-dirty
[ladder] max_return 10.0
[ladder] tip_guard not_in_hand sustain 4 env frames
```

### 6.2 Trainer stamps (train.log, verbatim; the env-code hashes equal the `_img` set's, PX_IMAGE_DEMOS §3)

```
Using cuda device
Wrapping the env with a `Monitor` wrapper
Wrapping the env in a DummyVecEnv.
Wrapping the env in a VecTransposeImage.
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-1008-g2830405-dirty
[obs] pixels: Dict('image': Box(0, 255, (64, 64, 6), uint8), 'proprio': Box(-inf, inf, (8,), float32)) (image = genv.rig_obs() top RGB ++ wrist RGB, proprio = state[:8] = q[:6], gripper motor, grip effort; NO can/goal pose) image_aug=shift4 cnn_feature_dim=50 buffer_size=5000
[env] FullTaskEnv built in 20.1s | pick_z=0.1505 scope=full action_mode=delta_joint delta_cap=0.025 delta_leash=0.125 delta_ref=target action_repeat=4 pick_hold_reward=off pick_hold_k=- (~300 decisions/ep)
[obs] policy=PixelRLPDPolicy algo=PixelRLPDSAC extractor=PixelProprioExtractor features_dim=58 policy_obs_space=Dict('image': Box(0, 255, (6, 64, 64), uint8), 'proprio': Box(-inf, inf, (8,), float32)) wiring={'actor_sees_critic_encoder': True, 'encoder_in_actor_optimizer': False, 'encoder_in_critic_optimizer': True, 'target_encoder_separate': True, 'n_encoder_params': 1029654}
[cfg] RLPD | obs=pixels E=10 Z=2 UTD=10 gamma=0.99 ent_coef=auto target_entropy=-3.5 demo_batch=128/256 backup_entropy=off per_member_ln=off pick_hold_reward=off pick_hold_k=25 pick_shaping=off q_watchdog=20.00 scope=full action_mode=delta_joint delta_ref=target action_repeat=4 demo_dir=/home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img demo_format=segment demo_terminal_guard=on demo_shaping=off pick_shaping_terminal_zero=on train_max_steps=1200 ckpt_fracs=1.0
[demos] 74 npz in /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img content_sha256=626be556cdbb9cc3... format=segment terminal_guard=on demo_shaping=off
[segments] /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img: 74 tapes -> 29221 transitions, Sigma reward 130 over 13 rewarded transitions (0 tapes with >1 grant), 18 terminal; decisions p50 388 min 1 max 600; 74 distinct rollouts; reward values {10.0: 13}
[pixels] /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img: 74 tapes -> 29295 frames (720 MB uint8), 29221 transitions, proprio dim 8; image nonzero-frame frac min 1.000, frame mean 99.39..109.33; Sigma reward 130; 13 tapes rewarded, 13 tapes at the ladder max
[demos] pixel demo half: 29295 frames x (64, 64, 6) uint8 = 721 MB (pinned host memory), gathered by index per batch
[demos] 74 eps -> 29221 transitions in the IMMUTABLE demo buffer (50% of every batch), 13 rewarded, 18 terminal (18 zero-reward terminals = tip-guarded fails)
[ladder] wrote /home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/train/ladder_provenance.json
[ckpt] periodic snapshots disabled (--ckpt-every 0): only the 1.0 archive + rlpd_final.zip are written
```

### 6.3 First update batch: replay images non-zero, augmented batch differs (the two asserts, verbatim)

```
[image] first update batch: shape=(256, 6, 64, 64) dtype=torch.uint8 ONLINE(128) min=10 max=240 mean=86.32 nonzero_frames=1.000 | DEMO(128) min=10 max=255 mean=106.02 nonzero_frames=1.000
[image_aug] shift4 pad=4: max|aug-raw|=222.0/255 frames_changed=0.988 aug_mean=95.85 raw_mean=96.17 shape=(256, 6, 64, 64)  [the augmented batch differs from the raw batch]
```

Both are ASSERTS in `PixelRLPDSAC` (`nonzero_frames == 1.0` and `max > 0` on both halves; `frames_changed
> 0.5` and `max|aug-raw| > 0`), so a blank rig or a no-op shift ends the run. The online half's mean (86.3) is
below the demo half's (106.0): random-action frames see the arm elsewhere than the demonstrations do; not a
defect. `frames_changed` 0.988 ≈ 1 − 1/81 (the zero-offset draw).

### 6.4 End of training

```
[ckpt] archived /home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/train/ckpt_100/rlpd_ckpt.zip @ 2000 decisions
[rlpd] done in 0.2h -> /home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/train/rlpd_final.zip
	Elapsed (wall clock) time (h:mm:ss or m:ss): 9:08.95
	Maximum resident set size (kbytes): 4774360
== train exit 0 Sun Sep 13 08:16:49 PM EDT 2026
```

**Rough throughput: 2 000 decisions in 549 s wall = ~3.6 decisions/s over the whole run** (includes ~30 s of
imports + 20.1 s world build + demo load; 1 000 random-action decisions at sim + two 64×64 renders each, then
1 000 decisions each carrying 10 critic updates on a GPU shared with a training run). Not a cluster number.
`episode_rollouts.jsonl`: 7 training episodes ended at decisions 8, 308, 608, 908, 1208, 1508, 1808 —
`ep_picked` 0, `ep_tipped` 1. Checkpoint 30.3 MB (`rlpd_final.zip`). Sidecar (`rlpd_final.action_mode.json`):
`obs: pixels, image_aug: shift4, image_shape: [64, 64, 6], proprio_dim: 8, cnn_feature_dim: 50, buffer_size:
5000, encoder_wiring: {actor_sees_critic_encoder: true, encoder_in_actor_optimizer: false,
encoder_in_critic_optimizer: true, target_encoder_separate: true, n_encoder_params: 1029654}, ladder:
nested_sparse10, tip_guard: not_in_hand, max_return: 10.0, steps: 2000, demo_n_eps: 74, demo_sha256:
626be556…, git: 2830405, sim_variant: gc_kp4_riser3_shelf6`. `ladder_provenance.json`: `learner: rlpd, ladder:
nested_sparse10, tip_guard: not_in_hand, stamp: <the [ladder] line above>`.

### 6.5 Evaluation: 3 hold15 episodes, `eval_e2e_px.py`, `--mode mode --video --role preview` (eval.log, verbatim)

```
== PX eval start Sun Sep 13 08:17:20 PM EDT 2026 tree=known-good-2026-08-27-1009-g24a4da3-dirty
[eval-e2e] kind=sac mode=mode seed=0 repeat=4 sim_variant=gc_kp4_riser3_shelf6 ic=baselines/eval_ics.json:hold max_steps=1200 sidecar=/home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/train/rlpd_final.action_mode.json
[eval-e2e] 3 start(s) from baselines/eval_ics.json:hold (offset 0, isolation shared_process) (3 uid starts, 0 pose starts) node=pop-os pid=1759945 cores=16p/32l affinity=32 threads=8 isa=avx2 cpu="AMD Ryzen 9 5950X 16-Core Processor" role=preview
[eval-e2e] ladder 'nested_sparse10' from the checkpoint sidecar
[eval-e2e] tip_guard 'not_in_hand' from the checkpoint sidecar
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-1009-g24a4da3-dirty
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03
[eval-e2e] shelf_top_z 0.170 (placed_v2 band 0.180..0.240); delta cap 0.025 leash 0.125; tip_guard not_in_hand@4f; ladder nested_sparse10 {'home': 10.0} terminal ('home',)+tipped
[eval-e2e] PX obs: Dict('image': Box(0, 255, (64, 64, 6), uint8), 'proprio': Box(-inf, inf, (8,), float32)) (rig_obs() top++wrist, proprio=state[:8]; sidecar image_aug=shift4 cnn_feature_dim=50)
[eval-e2e] PX first frame: shape=(64, 64, 6) dtype=uint8 min=10 max=236 mean=107.40 proprio=[0.3269999921321869, -1.4470000267028809, 2.3450000286102295, -1.350000023841858, 2.2090001106262207, -1.5130000114440918, 0.0, 0.0]
ep0: uid252 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 39.3 s)
ep1: uid254 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 38.9 s)
ep2: uid256 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 38.5 s)
[eval-e2e] 3 episodes (mode, hold): picked 0/3  placed_v2 0/3  contact_push 0/3  slide_success 0/3  nested_v2 0/3  farside 0/3  slide_event 0/3  home 0/3  nested_honest 0/3  | legacy: placed 0 contact 0 nested_proxy 0 contact_push_legacy 0 slide_success_settle 0  | tipped 0 timeout 3  mean_steps 300  [117 s]
[eval-e2e] wrote /home/j/data/genesis_pickaplace/px_rlpd_smoke_2026-09-13/eval_hold15_mode/metrics.json + 3 mp4s
== PX eval exit 0 Sun Sep 13 08:19:39 PM EDT 2026
```

`metrics.json`: `obs: pixels, image_shape: [64, 64, 6], proprio_dim: 8, n_renders: 903` (= 3 × (1 + 300): one
render after each reset and one after each decision, the trainer's convention), `headline_stages` all 0.0,
`outcomes: {home: 0.0, tipped: 0.0, timeout: 1.0}`, `amendment: n+ladder-unify+px`, the same `ladder_stamp`
as training (up to the `git=` suffix). Eval throughput with the 640×560 per-decision video render: 300
decisions in 38.5–39.3 s ≈ 7.7 decisions/s. Videos: `eval_hold15_mode/ep{0,1,2}_uid{252,254,256}_timeout.mp4`
(~0.75 MB each). The first frame's `proprio` is the home pose (q = 0.327, −1.447, 2.345, −1.350, 2.209, −1.513;
motor 0; effort 0), i.e. the state[:8] columns, not the can/goal.

### 6.6 Negative control: the STATE evaluator on the pixel checkpoint (expected to fail before any episode)

```
    if observation[key].shape != subspace.shape:
IndexError: only integers, slices (`:`), ellipsis (`...`), numpy.newaxis (`None`) and integer or boolean arrays are valid indices
== negative control exit 1 Sun Sep 13 08:20:00 PM EDT 2026
```

`eval_e2e.py` feeds a 17-dim vector to a policy whose observation space is a Dict; SB3's `predict` fails on
the first decision (exit 1, no episode, no metrics.json). It fails rather than refuses because `eval_e2e.py` is
untouched (in-flight cluster jobs read it); the new evaluator refuses explicitly on the sidecar's `obs` key.

## 7. NOT validated / open

1. **No learning claim.** 2 000 decisions (1 000 of them before `learning_starts`), 7 training episodes,
   picked 0, eval picked 0/3, reward 0. The smoke shows the pipeline runs end to end, nothing else. Whether
   {RLPD} from pixels ignites on this recipe is unmeasured.
2. **Cluster launcher: DONE (§9), proven by ONE smoke; no batch submitted.** `cluster/sbatch_rlpd_px.sh` +
   `sbatch_rlpd_px_eval.sh`; the machine arm (`ARM=dDPfirst`) passes the gate by inspection of its manifest
   (§9 header) but has not been launched; `nested_ramp` needs `dDPfull_first_rnrh_img`, which PXC-1's
   `px_img_sets` job was still building.
3. **Throughput at the 250 000-decision budget is unmeasured.** The cluster smoke measured 8.16 decisions/s on an
   L40S over 2 000 decisions with the random-action phase averaged in (§9.1); a naive 250 000 / 8.16 ≈ 8.5 h is a
   lower bound because the update phase alone is slower than that average and its share grows with the
   budget. Measure one full seed before committing the batch.
4. **Host memory at 300k rows has not been exercised.** The launcher's `MEM-CHECK` does the arithmetic (14.7 GB
   buffer + 0.9 GB demo half + ~5 GB process against `--mem=48g`) and refuses above 85 %; the smoke ran at 5 000
   rows. SB3's `DictReplayBuffer` has no `optimize_memory_usage` (asserted off).
5. **Hardware class.** The smoke ran on the 16-core AVX2 box; cells of record are 64-core-class
   (`--require-cores 64`, `cluster/sbatch_e2e_rescore.sh`). `role: preview`, `isolation: shared_process`.
6. **Only `--mode mode` was evaluated** (3 of 15 hold starts). `--mode sample`, `rnd30`, `spots60`, the
   `_iso` protocol, `--records-out` and the machine `_img` set (loaded and verified in §4, not trained on)
   were not exercised.
7. **Deviations from DrQ-v2** (§3): shared trunk, raw proprio (no proprio MLP in the extractor), polyak
   target encoder, integer shift. None validated against an alternative.
8. **Demo terminals.** The paid `home` frame is not terminal in the `_img` sets (§4); the pixel demo half
   inherits that from the state path.
9. **`eval_e2e_px.py` is a copy** (45 differing lines vs `eval_e2e.py`); a later change to the live
   evaluator has to be mirrored by hand.
10. **RNG.** The shift offsets draw from torch's global RNG, so a pixel run's random stream differs from a
    state run's at equal seed (irrelevant across observation modes; within the pixel path a seed is
    reproducible up to CUDA non-determinism).
11. **The state path is untouched by construction, tested by the existing suite (69 passed), but no state
    run was re-executed after the edit** — the `_prep_batch` hook and `_cat_obs` are identity/`th.cat` for
    tensors (§5).

## 9. Cluster launch (2026-09-13 evening; every line from the Slurm logs)

**Tree:** `$LAB/gp_pxr` = a fresh clone of `origin/ladder-unify-2026-09-11` (7c23d04 → d8621a3 → 9841633);
`$LAB/gp_px` (lane PXC-1) and every tree in HANDOFF §4 untouched. Sets already on the cluster (PXC-1's rsync):
`$W/demos_state_full/dHfull_all_rns10h_img` (74 npz, sha `626be556cdbb9cc3` = the local set) and
`dDPfull_first_rns10h_img` (72 npz, sha `a0d65d74485ee1bc`), both `images: rendered`, `state_only: false`,
ladder `nested_sparse10`, tip guard `not_in_hand`, Σ 130 / 140.

**Launchers (commit d8621a3, fix 9841633):**
* `cluster/sbatch_rlpd_px.sh` — `cluster/sbatch_rlpd_e2e.sh` with: `OBS=pixels` required; the `_img` set of the
  run's ladder (`_rns10h_img` | `_rnrh_img`); the e2e provenance gate PLUS the pixel rules derived from PXC-1's
  `wmfix_full.sbatch` gate (`images == rendered` ∧ `state_only == false`; the set's `relabel.ladder` and
  `relabel.tip_guard` equal the run's `LADDER`/`TIP_GUARD`; first tape `(T,64,64,6)` uint8 with > 99.9 % non-blank
  frames) — nothing weakened; a host-memory check (`MEM-CHECK`: 2 × 24 576 B per row → 14.7 GB at 300k rows + 0.9 GB
  demo half + ~5 GB process, refused above 85 % of the allocation; `--mem=48g`); `-p gpu,preempt --qos=preempt
  --nice=0`, one GPU (`l40s|a100|l40|h200`), `--requeue`; `--obs pixels --image-aug shift4 --buffer-size 300000`
  appended to the recipe's `TRAIN_ARGS`; TRAIN-OK additionally asserts the sidecar's `obs/image_aug/buffer_size` and
  the encoder wiring; then it submits ONE dependent CPU job instead of evaluating on the GPU node.
* `cluster/sbatch_rlpd_px_eval.sh` — `-p batch --qos=normal`, `-n 8 --mem=24g`, submitted with
  `--exclude=<every non-64-physical-core node + the SMT nodes>` (the milestone sweep's `$LAB/gp_dp_e2e/.excl64.txt`
  + `SMT_EXCL` list, deduped; 115 nodes) and `REQUIRE_CORES=64`; runs `cluster/e2e_eval_cells.sh` with
  `EVAL_SCRIPT=baselines/eval_e2e_px.py` — hold15 + rnd30 × {sample, mode} (+ rnd30 `_iso`) by default, same cell
  names / metrics.json / `E2E-HEADLINE` line as the state path.
* `cluster/e2e_eval_cells.sh` — `EVAL_SCRIPT` (default `baselines/eval_e2e.py`, printed in the header line).

**Launch command of record (one seed; the batch is NOT submitted):**
```bash
cd $LAB/gp_pxr && GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr OBS=pixels LADDER=nested_sparse10 TIP_GUARD=not_in_hand \
  ARM=dH SEED=<s> sbatch -J e2e_rlpd_px_dH_s<s> cluster/sbatch_rlpd_px.sh        # ARM=dDPfirst for the machine arm
# defaults: STEPS=250000 BUFFER_SIZE=300000 IMAGE_AUG=shift4 GAMMA=0.99 EVAL_SETS="hold15 rnd30" EVAL_MODES="sample mode"
```
Smoke variant (what ran): `STEPS=2000 BUFFER_SIZE=5000 WAVE=pxsmoke EVAL_SETS=hold15 EVAL_LIMIT=3 EVAL_ISO=0
EVAL_VIDEO_SETS=hold15 SEED=9991 … -J pxr_smoke_dH_s9991`. `DRYRUN=1` prints the plan (gate + MEM-CHECK) and exits.

**Disclosed failure first:** job **3684599** (seed 9990, pax150) died at exit 127 — `/usr/bin/time: No such file or
directory` (my launcher wrapped the trainer in GNU time, which the compute nodes lack; the e2e launcher never used
it). Fixed in 9841633 (timer only if present + a bash wall clock). Its registry entry (`sbatch_rlpd_px.sh arm=dH
seed=9990`, stage start) stays in `$LAB/gp_pxr/cluster/RUN_REGISTRY.jsonl` as a run that produced nothing; the
clone shows `-dirty` in every later stamp because of those registry appends.

### 9.1 Training job 3684603 (`e2e_rlpd_px_3684603.out`, pax064, NVIDIA L40S)

```
DISK-OK 184 GB free
MEM-CHECK buffer 5000 rows x 49152 B = 0.2 GB; est. process 6.1 GB; allocation 48.0 GB
DEMO-SHA dH full-scope PIXEL segments n=74 sha=626be556cdbb9cc3 total_reward=130.0 pick=65 nopick=9 decisions_p50=388 ladder=nested_sparse10 tip_guard=not_in_hand images=rendered first_tape_frames=256 nonblank=1.0000 mean=107.43
== RLPD-E2E-PX e2e_rlpd_px_dH_s9991 start Sun Sep 13 08:35:44 PM EDT 2026 node=pax064 host=pax064 gpu=0 demo=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/demos_state_full/dHfull_all_rns10h_img sha=626be556cdbb9cc3 restart=0
== TREE /cluster/tufts/shortlab/jstale02/gp_pxr (known-good-2026-08-27-1012-g9841633e-dirty)
GPU NVIDIA L40S, 46068 MiB
[ladder] unified-2026-09-10 | ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-1012-g9841633e-dirty
[ladder] max_return 10.0
[ladder] tip_guard not_in_hand sustain 4 env frames
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03
[obs] pixels: Dict('image': Box(0, 255, (64, 64, 6), uint8), 'proprio': Box(-inf, inf, (8,), float32)) (image = genv.rig_obs() top RGB ++ wrist RGB, proprio = state[:8] = q[:6], gripper motor, grip effort; NO can/goal pose) image_aug=shift4 cnn_feature_dim=50 buffer_size=5000
[env] FullTaskEnv built in 20.1s | pick_z=0.1505 scope=full action_mode=delta_joint delta_cap=0.025 delta_leash=0.125 delta_ref=target action_repeat=4 pick_hold_reward=off pick_hold_k=- (~300 decisions/ep)
[obs] policy=PixelRLPDPolicy algo=PixelRLPDSAC extractor=PixelProprioExtractor features_dim=58 policy_obs_space=Dict('image': Box(0, 255, (6, 64, 64), uint8), 'proprio': Box(-inf, inf, (8,), float32)) wiring={'actor_sees_critic_encoder': True, 'encoder_in_actor_optimizer': False, 'encoder_in_critic_optimizer': True, 'target_encoder_separate': True, 'n_encoder_params': 1029654}
[segments] /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/demos_state_full/dHfull_all_rns10h_img: 74 tapes -> 29221 transitions, Sigma reward 130 over 13 rewarded transitions (0 tapes with >1 grant), 18 terminal; decisions p50 388 min 1 max 600; 74 distinct rollouts; reward values {10.0: 13}
[pixels] /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/demos_state_full/dHfull_all_rns10h_img: 74 tapes -> 29295 frames (720 MB uint8), 29221 transitions, proprio dim 8; image nonzero-frame frac min 1.000, frame mean 99.39..109.33; Sigma reward 130; 13 tapes rewarded, 13 tapes at the ladder max
[demos] pixel demo half: 29295 frames x (64, 64, 6) uint8 = 721 MB (pinned host memory), gathered by index per batch
[demos] 74 eps -> 29221 transitions in the IMMUTABLE demo buffer (50% of every batch), 13 rewarded, 18 terminal (18 zero-reward terminals = tip-guarded fails)
[ckpt] archived baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991/ckpt_040/rlpd_ckpt.zip @ 800 decisions
[image] first update batch: shape=(256, 6, 64, 64) dtype=torch.uint8 ONLINE(128) min=10 max=239 mean=88.93 nonzero_frames=1.000 | DEMO(128) min=10 max=254 mean=106.01 nonzero_frames=1.000
[image_aug] shift4 pad=4: max|aug-raw|=221.0/255 frames_changed=0.988 aug_mean=97.42 raw_mean=97.47 shape=(256, 6, 64, 64)  [the augmented batch differs from the raw batch]
[ckpt] archived baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991/ckpt_100/rlpd_ckpt.zip @ 2000 decisions
[rlpd] learn: 2000 decisions in 245.0 s = 8.16 decisions/s (1000 random-action decisions before updates, then UTD 10; obs=pixels)
[rlpd] done in 0.1h -> baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991/rlpd_final.zip
TRAIN-WALL 283 s (steps=2000 decisions, incl. world build + demo load) Sun Sep 13 08:40:45 PM EDT 2026
TRAIN-OK baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991: budget 2000 decisions reached (ckpt_100 at 2000), sidecar obs=pixels image_aug=shift4 buffer_size=5000 ladder=nested_sparse10 tip_guard=not_in_hand cap=0.025 leash=0.125
EVAL-SUBMITTED job=3684707 sets='hold15' modes='sample mode' iso=0 limit='3' require_cores=64 excluded=115 nodes -> baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991/e2e_eval_px.log
JOB DONE Sun Sep 13 08:40:45 PM EDT 2026
```
**Throughput on the cluster GPU: 8.16 decisions/s** over the 2 000-decision `learn()` (L40S; 1 000 random-action
decisions + 1 000 decisions with 10 critic updates each; the update-phase rate alone is not separated). The
coordinator's reference for state RLPD there is ~8/s. `sacct`: COMPLETED 0:0.

### 9.2 Eval job 3684707 (`e2e_rlpd_px_eval_3684707.out`, pax027, 64 physical cores, Intel Xeon Gold 6438M, avx512)

```
SIDECAR-OK obs=pixels image_aug=shift4 ladder=nested_sparse10 tip_guard=not_in_hand
== RLPD-PX-EVAL dH s9991 start Sun Sep 13 08:40:50 PM EDT 2026 node=pax027 cores=64 ckpt=baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991/rlpd_final.zip
== e2e_eval_cells kind=sac script=baselines/eval_e2e_px.py ckpt=... role=preview arm=dH seed=9991 sets='hold15' modes='sample mode' iso=0 iso_sets='rnd30' video='hold15' variant=gc_kp4_riser3_shelf6 eval_seed=0 par=8 node=pax027 cores=64 ...
# cell hold15 sample rc=0 2026-09-13T20:49:25-04:00
# cell hold15 mode rc=0 2026-09-13T20:49:54-04:00
E2E-HEADLINE learner=sac arm=dH seed=9991 role=preview key=terminal protocol=shared hold15_S=home0/3[p0,pv20,c0,far0,home0,nH0,nP0]@pax027/avx512 hold15_M=home0/3[p0,pv20,c0,far0,home0,nH0,nP0]@pax027/avx512 out=baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s9991
EVAL DONE rc=0 Sun Sep 13 08:49:54 PM EDT 2026
```
Per cell (`fresh_eval_hold15_{mode,sample}/eval.log`):
```
[eval-e2e] 3 start(s) from baselines/eval_ics.json:hold (offset 0, isolation shared_process) (3 uid starts, 0 pose starts) node=pax027 pid=1096900 cores=64p/64l affinity=8 threads=1 isa=avx512 cpu="Intel(R) Xeon(R) Gold 6438M" role=preview
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03
[eval-e2e] PX obs: Dict('image': Box(0, 255, (64, 64, 6), uint8), 'proprio': Box(-inf, inf, (8,), float32)) (rig_obs() top++wrist, proprio=state[:8]; sidecar image_aug=shift4 cnn_feature_dim=50)
[eval-e2e] PX first frame: shape=(64, 64, 6) dtype=uint8 min=10 max=236 mean=107.46 proprio=[0.3269999921321869, -1.4470000267028809, 2.3450000286102295, -1.350000023841858, 2.2090001106262207, -1.5130000114440918, 0.0, 0.0]
ep0: uid252 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 169.5 s)
ep1: uid254 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 162.5 s)
ep2: uid256 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 161.2 s)
[eval-e2e] 3 episodes (mode, hold): picked 0/3  placed_v2 0/3  contact_push 0/3  slide_success 0/3  nested_v2 0/3  farside 0/3  slide_event 0/3  home 0/3  nested_honest 0/3  | legacy: placed 0 contact 0 nested_proxy 0 contact_push_legacy 0 slide_success_settle 0  | tipped 0 timeout 3  mean_steps 300  [493 s]
ep0: uid252 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 141.8 s)
ep1: uid254 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 160.4 s)
ep2: uid256 timeout slide=0 nested_v2=0 nestedH=0 push=0 placed_v2=0 picked=0 (300 decisions, r=0.0, 160.6 s)
[eval-e2e] 3 episodes (sample, hold): picked 0/3  placed_v2 0/3  contact_push 0/3  slide_success 0/3  nested_v2 0/3  farside 0/3  slide_event 0/3  home 0/3  nested_honest 0/3  | legacy: placed 0 contact 0 nested_proxy 0 contact_push_legacy 0 slide_success_settle 0  | tipped 0 timeout 3  mean_steps 300  sample_dev_mean 0.5136  [463 s]
```
Both cells ran concurrently (PAR 8 on an 8-CPU allocation, `threads=1`, with the per-decision video render):
141.8–169.5 s per 300-decision episode, i.e. ~1.9 decisions/s per cell — four times slower than the local box
(§6.5). The full default cell set (hold15 + rnd30 × 2 modes + rnd30 `_iso` = 30 + 60 + 60 episodes) at that rate is
~7 h of an 8-CPU job; not measured. The mode and sample cells give identical stage flags on these 3 starts
(`sample_dev_mean 0.5136` says the sampled actions did differ).

## 8. How to run

```bash
cd ~/workspace/genesis_pickaplace && export GENESIS_PICKAPLACE_ROOT=$PWD
# train (local smoke shape; the cluster recipe is --steps 250000 --buffer-size 300000 --device cuda)
.venv-eval/bin/python baselines/rl/train_rlpd.py --obs pixels --steps 2000 --buffer-size 5000 \
  --scope full --ladder nested_sparse10 --tip-guard not_in_hand --demo-format segment \
  --demo-dir /home/j/data/genesis_pickaplace/demos_state_full/dHfull_all_rns10h_img \
  --action-mode delta_joint --delta-ref target --action-repeat 4 --train-max-steps 1200 --eval-max-steps 1200 --eval-freq 0 \
  --gamma 0.99 --backup-entropy off --per-member-ln off --pick-hold-reward off --pick-shaping off \
  --utd 10 --ensemble-size 10 --subset-size 2 --demo-batch 128 --demo-shaping off --pick-shaping-terminal-zero on \
  --demo-terminal-guard on --sim-variant gc_kp4_riser3_shelf6 --ckpt-every 0 --ckpt-fracs 1.0 \
  --out-dir <out> --run-name <name> --no-wandb --seed 0 --device cuda
# evaluate (the pixel evaluator; same IC files / sets as cluster/e2e_eval_cells.sh)
.venv-eval/bin/python baselines/eval_e2e_px.py --kind sac --checkpoint <out>/rlpd_final.zip \
  --ic-file baselines/eval_ics.json --ic-set hold --out <out>/eval_hold15_mode --mode mode --seed 0 \
  --max-steps 1200 --sim-variant gc_kp4_riser3_shelf6 [--limit 3] [--video]
# offline checks
.venv-eval/bin/python baselines/tests/test_rlpd_pixel.py
```
