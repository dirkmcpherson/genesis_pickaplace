# Item 03 — Diffusion Policy: paper text vs code, appendix hyperparameter section, data true to source

**Status: DONE (Fable lane, 2026-09-17 late).** Every hyperparameter below was read back from the saved
`train_config.json` / `config.json` of the runs the paper reports (one run quoted per dataset, all 8 seeds per dataset
checked for identical configs), from the lerobot fork installed in the cluster env, and from the launchers of record.
The four pixel datasets were re-gated read-only tonight (`verify_px_lerobot.py`, action column byte-identical to the
recorder tapes on every episode). Three `\llm{}` notes inserted (03-method DP paragraph, 04 DP paragraph, empty appendix
subsection). Headline discrepancies: **the paper's "ResNet18 pretrained on ImageNet" is false for every DP run in the
paper (backbone trained from scratch, `pretrained_backbone_weights: null`)**, and **"trains on successful demonstrations
only and prunes out zero-actions" is true ONLY of the state-based DP teacher that generated the machine dataset; the
four reported pixel DP arms train on the RAW, failure-inclusive, unpruned sets.** Details in §Discrepancies.

---

## 1. What the code actually is (code of record)

| item | value | source |
|---|---|---|
| Implementation | lerobot **0.4.5**, fork `dirkmcpherson/lerobot`, branch `genesis-fixes`, commit `b63920e7` (= upstream `d324ffe8` + one PNG-writer mkdir patch); editable install in the cluster conda env | `ssh pax: pip show lerobot; git -C $LAB/lerobot_src log -1` → `Version: 0.4.5`, `Editable project location: /cluster/home/jstale02/lerobot`, `__file__ = $LAB/lerobot_src/src/lerobot/__init__.py`, `b63920e7 2026-07-25 image_writer: mkdir parent before PNG save`. Local mirror `~/workspace/lerobot` is at `d324ffe8` (same diffusion code). |
| Policy class | `lerobot.policies.diffusion.modeling_diffusion.DiffusionPolicy` (`--policy.type=diffusion`), stock; no fork edits to the policy | `cluster/sbatch_dp_px.sh` `TRAIN_ARGS`; `git -C $LAB/lerobot_src log --oneline -12` (only the image-writer patch is non-upstream) |
| torch / torchvision / diffusers | 2.7.0+cu126 / 0.22.0+cu126 / 0.35.2 | `pip show` in the cluster env |
| Trainer | `lerobot-train` (`src/lerobot/scripts/lerobot_train.py`), single GPU, `use_amp=false`, gradient-norm clip 10.0, **no EMA** (grep `ema` over `policies/diffusion/`, `scripts/lerobot_train.py`, `configs/train.py`: no hits) | train_config.json `optimizer.grad_clip_norm: 10.0`, `policy.use_amp: false` |

## 2. Hyperparameters read back from the runs of record

One run per dataset; all other seeds of each dataset have a byte-identical `train_config.json` apart from
`seed`/`job_name`/`output_dir`/`repo_id`/wandb id (checked: human 8/8, machine 7/7 finished, planner72 8/8, r2teacher 8/8).

| run (checkpoint `100000`) | `dataset.root` | sidecar |
|---|---|---|
| `$LAB/gp_ah/baselines/outputs/dp_px/ah_dp_px_dH_s0` | `$LAB/genesis_pickaplace/baselines/matched_w3/dHfull_all/lerobot_px` | git `41af68dc`, demo_sha `70027c899be26f60`, node pax009, amendment (ah), 2026-09-15T22:40Z |
| `$LAB/gp_ah/baselines/outputs/dp_px/ah_dp_px_dM_s0` | `.../matched_w3/dDPfull_first/lerobot_px` | git `41af68dc`, demo_sha `ebb7bd848fb03137`, pax009, (ah), 2026-09-15T23:00Z |
| `$LAB/planner72_px_2026-09-15/runs/dp/planner72_dp_px_dPlanner72_s0` | `$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals/lerobot_pixels` | git `e8287af0`, demo_sha `9aff5c95…` (archive sha), pax026, 2026-09-17T22:13Z |
| `$LAB/r2teacher_px_2026-09-15/runs/dp/r2teacher_dp_px_dR2fromH_px_s0` | `$LAB/r2teacher_px_2026-09-15/data/training_v1/lerobot_pixels` | git `e8287af0`, demo_sha `008a06e6…`, pax050, 2026-09-17T11:14Z |

Identical `policy` block in all four (quoted from `train_config.json`):

```
n_obs_steps 2 | horizon 16 | n_action_steps 8 | drop_n_last_frames 7 | do_mask_loss_for_padding false
input_features: observation.state [8], observation.images.top [3,64,64], observation.images.wrist [3,64,64]
output_features: action [7]
normalization_mapping: VISUAL MEAN_STD (dataset.use_imagenet_stats true), STATE MIN_MAX, ACTION MIN_MAX
vision_backbone resnet18 | pretrained_backbone_weights null | use_group_norm true | spatial_softmax_num_keypoints 32
use_separate_rgb_encoder_per_camera false | resize_shape null | crop_shape [56,56] | crop_is_random true
down_dims [512,1024,2048] | kernel_size 5 | n_groups 8 | diffusion_step_embed_dim 128 | use_film_scale_modulation true
noise_scheduler_type DDPM | num_train_timesteps 100 | beta_schedule squaredcos_cap_v2 | beta_start 1e-4 | beta_end 0.02
prediction_type epsilon | clip_sample true | clip_sample_range 1.0 | num_inference_steps null (= 100)
optimizer adam lr 1e-4 betas [0.95,0.999] eps 1e-8 weight_decay 1e-6 grad_clip_norm 10.0
scheduler diffuser cosine, num_warmup_steps 500 | batch_size 64 | steps 100000 | save_freq 50000 | seed = run seed
use_amp false | device cuda | dataset.image_transforms.enable false | num_workers 4
```

The state-based **teacher** that generated the machine dataset (`$LAB/genesis_pickaplace/baselines/outputs/dp_phase/dHfull_pruned_DP_s0/checkpoints/100000/pretrained_model/train_config.json`):
`dataset.root = baselines/matched_w3/dHfull_pruned/lerobot`, `input_features: observation.state [8], observation.environment_state [9]` (no cameras),
`crop_shape null`, otherwise the same policy/optimizer block; `save_freq 20000` (5 checkpoints; the 100k one is the teacher, sidecar `arm dHfull_pruned`, `demo_sha 841c5dd547c565e6`, git `c4da67e`, 2026-09-04).

Derived quantities (from `modeling_diffusion.py` at `d324ffe8`, lines 173–186, 448–509): per-camera encoder = torchvision
ResNet-18 trunk with every BatchNorm replaced by GroupNorm(num_features//16), SpatialSoftmax with 32 keypoints → 64-d →
Linear(64,64); one encoder shared by both cameras; global conditioning = (8 + 64 + 64) × 2 observation steps = 272-d,
concatenated with the 128-d diffusion-step embedding and injected by FiLM (scale + bias) into a 1-D conditional U-Net
(channels 512/1024/2048, kernel 5, GroupNorm 8, Mish). Loss = MSE on the predicted noise, unmasked. Inference = 100
DDPM reverse steps (`num_inference_steps None` → `num_train_timesteps`), clipped to [−1, 1] in normalized action space.

## 3. Datasets the reported DP runs trained on — true to source?

| dataset (paper name) | recorder tapes → DP episodes / frames | converter call | one-decision tapes | action column vs source | fps metadata |
|---|---|---|---|---|---|
| Human (`dHfull_all`, raw, all 74 teleop attempts incl. 10 no-pick) | 74 → **72 / 29,219** (drops `102000.npz`, `105007.npz`, n=1 each; 29,221 decisions total) | `LEROBOT_IMAGES_FROM=$W/demos_state_full/dHfull_all_rns10h_img LEROBOT_NO_ENV_STATE=1 convert_to_lerobot.py $RAW lerobot_px 8 4 top,wrist image` (`cluster/ah_build_pixel_sets.sh`) | dropped (`MIN_FRAMES=4`) | `PX-DATASET-OK … action bytes == …/lerobot on 72/72 episodes and == the recorder tapes on 72/72` (re-run tonight) | 7.5 |
| DP-teacher machine (`dDPfull_first`, first attempt per start, de-selected) | 72 → **70 / 36,832** (drops `102000.npz`, `103017.npz`) | same | dropped | `… 70/70 and == the recorder tapes on 70/70` | 7.5 |
| Planner (`planner72_matched72`) | 72 → **72 / 17,872** | `convert_to_lerobot.py dp_raw lerobot_pixels 8 1 top,wrist image 8.3333` (`experiments/full_state_planner_2026-09-12/matched72_completed_2026-09-15/finish.py:80`) | **kept** (`MIN_FRAMES=1`; `000018.npz`, `000034.npz`) but they yield 0 training anchors (`DP_TEMPORAL_LOCAL.json`: sampled_tapes 70, anchors 17,380) | `… 72/72 and == the recorder tapes on 72/72` | **8.333** |
| R2 teacher (`r2teacher_matched72`, `dR2fromH_px`) | 72 → **72 / 2,278** | `validate_and_export.py:23`, same args | kept; tapes 18, 34, 38 (n = 1, 1, 5) have 0 anchors → 69 sampled tapes, 1,788 anchors (campaign HANDOFF.md line 30) | `… 72/72 and == the recorder tapes on 72/72` | **8.333** |
| (teacher's set) Human pruned (`dHfull_pruned`) | 64 → 64 / 23,307 (25,123 before idle collapse, 7.2 % removed, all pre-pick) | `prune_full_v1.py --margin 38 --idle-eps 1e-3` over `demos_v2/dHfull_w3_picked`, then `convert_to_lerobot.py … 8 4 none` | none (min n = 129) | not a pixel set; state DP only | 7.5 |

Sources: cluster `manifest.json` / `genesis_source.json` / `meta/info.json` of each set (quoted in §Sources); gate re-run
tonight from `$LAB/gp_ah` with `python baselines/verify_px_lerobot.py --px … --ref … --raw …` for all four.
Raw-tape action sha256 (first 16 hex, all `actions` arrays concatenated in stem order): human `640c49c9b192ffc4`,
machine `0a2566ced6470c7b`, pruned-human `4fc39a0da130cbd4`.

**"Trained with each dataset":** on the cluster at 23:30 09-17 — human 8/8 runs finished (rnd30 cells 5/8, hold15 8/8);
machine 7/8 finished + `ah_dp_px_dM_s6` re-running (job 3773978, started 20:49) (rnd30 cells 4/8); planner72 8/8
finished (rnd30 cells 5/8); r2teacher 8/8 finished (rnd30 8/8, hold15 8/8). Every DP cell stamps `kind dp, mode sample,
camera_rig true, max_steps 1200, action_repeat 4` (`fresh_eval_rnd30_sample/metrics.json`). Per-seed rnd30 `home`:
human 0, 0, .07, .03, .03; machine 0, .07, 0, 0; planner72 .60, .60, .57, .57, .57; r2teacher .60 ×6, .60, .60.
(These are `role: preview` in-job cells; there is no pinned `rec/` pass for DP.)

## 4. Deliverable (a): appendix `\subsection{Diffusion Policy}` — paste-ready

```latex
\subsection{Diffusion Policy}\label{appx:dp}

We use the Diffusion Policy implementation in LeRobot~0.4.5~\citep{cadene2024lerobot} (fork
\texttt{dirkmcpherson/lerobot}, branch \texttt{genesis-fixes}, commit \texttt{b63920e7}; the policy code is
unmodified upstream code) with the library's default architecture and optimisation preset. The policy is a
conditional 1-D U-Net denoiser~\citep{chi2024diffusionpolicy} that maps Gaussian noise to a window of
$H=16$ consecutive actions, conditioned on the two most recent observations. Each $64\times64$ RGB view is
encoded by a ResNet-18 trunk with GroupNorm in place of BatchNorm and a 32-keypoint spatial-softmax head
(64-d per view); one encoder is shared by the top and wrist cameras. \textbf{The backbone is trained from
scratch} (\texttt{pretrained\_backbone\_weights=None}); pixels are only standardised with ImageNet channel
statistics. The image, proprioceptive and (for the state-based teacher) privileged-state features of the two
observation steps are concatenated (272-d for the pixel policies) and injected into every U-Net block by FiLM
together with the diffusion-step embedding. Training minimises the unmasked mean-squared error of the predicted
noise under a 100-step DDPM forward process. At test time the policy is queried once, denoises for 100 reverse
steps, executes the first $8$ of the 16 predicted actions (one per 0.12\,s decision, i.e.\ four simulator frames
each), and re-plans. Actions are the demonstrations' absolute window-end joint targets (6 joints, rad) plus a
gripper command in $[0,1]$; the evaluator converts each target into the environment's delta-joint command,
$\Delta q=\mathrm{clip}\big((q^{*}-q^{\mathrm{tgt}})/(4\cdot0.025),-1,1\big)$, so every learner drives the
same delta integrator (cap $0.025$\,rad per frame, leash $0.125$\,rad). Table~\ref{tab:dp-hparams} lists
every setting; all values are read back from the saved \texttt{train\_config.json} of the reported runs and
are identical across datasets and seeds.

\textbf{Data.} A LeRobot dataset holds one row per decision (frames of the two cameras stored as PNG, 8-d
proprioception, 7-d action) at the demonstrations' decision rate. The pixel policies never receive the
simulator's can or goal pose. Tapes shorter than four decisions cannot form a training window and are
dropped by the converter (2 of 74 human and 2 of 72 DP-teacher tapes, each a single decision); the planner
and R2 datasets keep their one-decision tapes, which contribute no training windows. The action column of
every pixel dataset is byte-identical to the recorder tapes and to the corresponding state-observation
dataset (verified per episode by \texttt{baselines/verify\_px\_lerobot.py} at build time and in every
training job). The four reported pixel DP arms train on the raw, unfiltered datasets (all attempts, including
failed ones, no idle pruning). The state-based teacher that produced the DP machine dataset was trained on
the pruned human set: the 64 human tapes that reach a grasp (the 10 no-pick attempts removed), with runs of
identical consecutive actions collapsed only in the segment ending 38 decisions ($\approx5$\,s) before the
grasp (25{,}123\,$\to$\,23{,}307 decisions, $-7.2\%$); everything from that point through the place and the
slide is unchanged.

\textbf{Evaluation.} Each seed's final (100k-update) checkpoint is evaluated in a fresh process on the 30
random starts and on the 15 demonstration starts, one episode per start, sampled (100-step DDPM, evaluation
seed 0), 1200 simulator steps (300 decisions) per episode, with the same two-camera rig the demonstrations
were rendered with. Episodes end on the terminal stage \texttt{home} or on the tip rule
(\texttt{nested\_sparse10}, \texttt{tip\_guard=not\_in\_hand}); DP itself never reads a reward, so the ladder
only fixes where an episode ends, and it is where the other learners' episodes end.

\begin{table}[htbp]\centering\small
\caption{Diffusion Policy hyperparameters (LeRobot 0.4.5 \texttt{DiffusionConfig}), identical for all four
demonstration datasets and all seeds. Values read from the runs' saved \texttt{train\_config.json}.}
\label{tab:dp-hparams}
\begin{tabular}{ll}\toprule
Observation inputs & \texttt{observation.images.top}, \texttt{observation.images.wrist} ($3\times64\times64$ each), \\
 & \texttt{observation.state} (8-d: 6 joint angles, gripper motor, grip effort); no object/goal pose \\
Image preprocessing & no resize; random $56\times56$ crop in training, centre crop at evaluation \\
Image normalisation & per-channel mean/std (ImageNet statistics) \\
Vision backbone & ResNet-18, GroupNorm (groups $=C/16$), spatial softmax with 32 keypoints, \\
 & 64-d per view, one encoder shared by both cameras, \textbf{no pretrained weights} \\
Observation horizon $n_{\mathrm{obs}}$ & 2 decisions \\
Prediction horizon $H$ & 16 decisions \\
Executed per query $n_{\mathrm{act}}$ & 8 decisions (re-plan every 8 decisions; 1 decision $=$ 4 simulator frames $=0.12$\,s) \\
Denoiser & 1-D conditional U-Net, channels (512, 1024, 2048), kernel 5, GroupNorm 8, Mish, \\
 & FiLM scale+bias conditioning, diffusion-step embedding 128 \\
Global conditioning & $(8+64+64)\times 2=272$-d (pixel policies); $(8+9)\times2=34$-d (state teacher) \\
Noise schedule & DDPM, 100 training steps, \texttt{squaredcos\_cap\_v2}, $\beta\in[10^{-4},0.02]$, $\epsilon$-prediction \\
Inference & 100 DDPM reverse steps, samples clipped to $[-1,1]$ \\
Action space & 7-d: absolute window-end joint targets (rad) $+$ gripper $[0,1]$; min--max normalised \\
State normalisation & min--max \\
Loss & MSE on predicted noise, no padding mask; last 7 frames of each episode not used as window starts \\
Optimiser & Adam, lr $10^{-4}$, $\beta=(0.95,0.999)$, $\epsilon=10^{-8}$, weight decay $10^{-6}$ \\
LR schedule & cosine with 500 warm-up steps \\
Gradient clipping & global norm 10 \\
EMA / mixed precision & none / none (fp32) \\
Batch size, updates & 64, 100{,}000 gradient updates \\
Checkpoints & saved at 50k and 100k; only the 100k checkpoint is kept and evaluated \\
Seeds & 8 per dataset (0--7); the seed sets model initialisation and data shuffling \\
Evaluation & fresh process; 30 random starts and 15 demonstration starts; sampled actions (seed 0); \\
 & 1200 simulator steps; episode ends on \texttt{home} or tip; two-camera rig; final checkpoint only \\
Dataset rows & Human 72 ep./29{,}219; DP 70/36{,}832; Planner 72/17{,}872; R2 72/2{,}278 (decisions) \\
\bottomrule\end{tabular}
\end{table}
```

## 5. Deliverable (b): corrected main-text paragraphs

### `sections/03-method.tex`, paragraph "Diffusion Policy" (line 33) — replace with

```latex
\paragraph{Diffusion Policy}

We use LeRobot's implementation~\citep{cadene2024lerobot} of Diffusion Policy (DP), a sequence-prediction imitation learner~\citep{chi2024diffusionpolicy}. It learns a denoising model that turns noise into short action sequences, conditioned on recent observations. The learned model represents a distribution rather than a single function, allowing DP to learn multi-modal demonstration distributions and capture the different ways people perform a task. At test time DP predicts a chunk of 16 actions, executes the first 8, and re-plans. DP encodes each camera view with a ResNet18 backbone~\citep{torchvision2016} trained from scratch, with random-crop augmentation. DP is pure imitation and is sensitive to the demonstration distribution it is shown: the state-based DP used to generate the machine dataset was trained, following common practice, on the successful human demonstrations only with pre-grasp idle time removed; the DP arms of the main comparison are trained on the same raw datasets as every other learner (Appendix~\ref{appx:dp}).
```

Rationale: (i) "pretrained on ImageNet" → "trained from scratch" (every run: `pretrained_backbone_weights: null`);
(ii) "trains on successful demonstrations only and prunes out zero-actions" is true of the teacher only; the reported
pixel DP arms train on raw sets. If the authors would rather keep the general-method sentence, it must be scoped to the
teacher, as above. (iii) "zero-actions" is not what the pruner removes: it collapses runs of *repeated* (unchanged)
actions before the grasp (`|a[t+1]-a[t]| < 1e-3` over all 7 channels), and `MUST_HAVE_RESULTS.md` (user correction
2026-09-09) records that no decision is truly idle over all channels — "zero-action" should not be used.

### `sections/04-evaluation.tex`, paragraph "DP" under Machine Demonstrations (line 78) — replace with

```latex
\paragraph{DP}
The DP demonstrations come from a state-based DP (privileged can and goal pose plus proprioception, the observation space on which DP performed best) trained on a cleaned human dataset: the 64 human trials that reach a grasp, with idle time before the grasp collapsed (Appendix~\ref{appx:dp}). Its first attempt on each of the 72 human starts is kept, with no selection among attempts. Trained on the cleaned data, DP reproduces its demonstration distribution: the DP dataset matches the human dataset at every phase (Table~\ref{tab:phases}), including 14 versus 13 trials reaching the goal. The DP learner in the main comparison is then trained on pixel observations, on the raw datasets, like every other learner.
```

Numbers: `HRI_results/DEMO_SETS_2026-09-11.md` §1 (teacher, `dHfull_pruned`, harvest 195 rollouts, FIRST attempt per
start, PHASE_PLAN (v)) and §3 (pixel sets: `home` 13 v 14, picked 65 v 64, placed_v2 42 v 44, slide_event 26 v 24).
Note: the paper's `tab:phases` human row (88.9/54.2/33.3/15.3 over "72 demonstrations") does not match DEMO_SETS §3
(human n = 74: picked 65 = 0.88, placed 42, slide 26, home 13); the DP row (64/72, 41/72, 23/72, 12/72) matches the
STATE-set counts with `home` (12) in the "Nested" column. Whoever owns the phase table should state which 72 human
tapes and which predicate each column uses (not settled by this lane).

## 6. Discrepancies (paper vs code/notes)

| # | paper says | code / artefact says | evidence |
|---|---|---|---|
| D1 | 03-method: "ResNet18 backbone pretrained on ImageNet" | `pretrained_backbone_weights: null` in every reported DP run (human, machine, planner72, r2teacher, and the state teacher, which has no cameras at all); launcher asserts it (`sbatch_dp_px.sh` "assert cfg.get('pretrained_backbone_weights') in (None,'null')"); registration (ah) says "no pretrained weights — the (af) encoder is also trained from scratch". Only `use_imagenet_stats: true` (pixel normalisation) involves ImageNet. | `train_config.json` of the four runs in §2; `paper/PHASE_PLAN_2026-09-04.md` (ah) |
| D2 | 03-method: "DP trains on successful demonstrations only and prunes out zero-actions" (stated as DP's method) | True only for the teacher `dp_phase/dHfull_pruned_DP_s0` (`dataset.root = matched_w3/dHfull_pruned/lerobot`, 64 picked tapes, pre-pick idle collapse). The four DP arms of the 4×4 comparison train on the RAW sets: human `dHfull_all` (74 tapes incl. 10 no-pick, idle_frac 0.365), machine `dDPfull_first` (72, no selection), planner72, r2teacher — `dataset.root` fields in §2. `HANDOFF_PIXEL_4x4` §3.5 and open decision 7.1/7.2 say the same ("The human arm trains on raw demonstrations … a pruned pixel set has not been built"). `MUST_HAVE_RESULTS.md:78` still says DP "trains on the PRUNED human set" as a standing requirement — the pixel study does not follow it. | cluster `train_config.json`; `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`; `paper/MUST_HAVE_RESULTS.md` |
| D3 | "prunes out zero-actions" | The pruner (`baselines/prune_full_v1.py`) collapses runs of *unchanged* actions (`max|a[t+1]-a[t]| < 1e-3`) only before `j_pick − 38`; it never removes a "zero action" and never touches the grasp/place/slide. Per the user's 2026-09-09 correction, no decision in any set is idle over all 7 channels. | `baselines/prune_full_v1.py`; `paper/MUST_HAVE_RESULTS.md:86` |
| D4 | 04-eval "DP" paragraph: "cleaned human dataset" undefined; "We then train DP on pixel-demonstrations" | The pixel DP trains on the same raw tapes (pixels rendered by re-execution, actions byte-identical) — not on a separate "pixel demonstration" set, and not cleaned. See corrected paragraph. | `cluster/ah_build_pixel_sets.sh`; gate output §3 |
| D5 | 04-eval line 17 (simulated section, but reads as global): "DfD, RLPD, and Diffusion Policy each use the same fixed human and machine datasets" | For the robot task this is true at the tape level, but DP's dataset drops 2 one-decision tapes per arm (72/74, 70/72) and the planner/R2 DP sets keep 1-decision tapes with zero training windows. Disclose in the appendix (done in §4). | manifests `short_tapes`; `DP_TEMPORAL_LOCAL.json` |
| D6 | (ah) registration: "rnd30 MODE and hold15 MODE (DP is deterministic given the seed)" | The evaluator refuses `--mode mode` for DP and labels its cells `sample` (`eval_e2e.py:138`, `e2e_eval_cells.sh:19`); all DP cells are `mode: sample`, seeded DDPM. Same computation, different label — the paper should say "sampled (seeded)". | `baselines/eval_e2e.py`; metrics.json |
| D7 | Appendix subsection empty; `\ref{appx:hyperparameters}` in 03-method has no label target (04 uses `appx:training`, which exists) | Fill with §4; fix the ref to `appx:training` or add `\label{appx:hyperparameters}`. | `sections/03-method.tex:29`, `appendix/a-research-methods.tex:38` |
| D8 | Not in the paper, must be disclosed somewhere: planner72/r2teacher DP datasets carry `fps 8.333` metadata (0.12 s per decision, 4 × 0.03 s) while human/machine carry `fps 7.5` (30/4) | LeRobot indexes windows by integer frame offsets, so training is unaffected (`DP_TEMPORAL_LOCAL.json: fps_to_frame_offsets_identical true`), but the physical-time metadata differs. | `experiments/.../finish.py:79-80`; `experiments/wm_teacher_matched_2026-09-15/write_handoff.py:39` |
| D9 | Not in the paper: augmentation | Pixel DP uses random 56×56 crop (train) / centre crop (eval) as the analogue of the world models' `shift4` pad-and-shift; lerobot crops (discards ~23 % of the field) rather than pads — a disclosed difference between learners. `dataset.image_transforms.enable: false` (no colour jitter). | `cluster/sbatch_dp_px.sh` header; `paper/AH_DP_PIXEL_BUILD_2026-09-14.md` §102-104 |
| D10 | Not in the paper: checkpoint selection | Earlier state-DP tables (pick, `tables/phase_performance.tex` "Pick / DP", "Place / DP") used a selected-checkpoint protocol (K=5 checkpoints scored on a selection set, `cluster/sbatch_dp.sh`); the e2e/pixel DP runs use the FINAL 100k checkpoint only, no selection. If the phase table stays in the paper, its DP rows come from `dp_w2final/dH_DP_*` (human = `episodes_pick_phase_dppruned`, PRUNED) v `dDP_DP_*`, and `pl_dp_*` (`matched_w3/dH_place` v `dDP_place_n39`), i.e. the earlier human-DP rows ARE on pruned data while the 4×4 rows are not. | `cluster/sbatch_dp.sh:56-60,203`; `HRI_results/VERIFICATION.md:24,30,40` |
| D11 | Cells of record | All DP pixel cells are `role: preview` in-job cells (no pinned `rec/` re-score exists for any DP run, pixel or e2e; `PHASE_PLAN (ab).1(a)`). Node/ISA are stamped; a pinned pass is available post hoc. | metrics.json `role`; `paper/PHASE_PLAN_2026-09-04.md` (ab).1 |

## 7. `\llm` notes inserted

- `sections/03-method.tex` line 33, appended after "...to prevent state-action aliasing." — `\llm{Item 03: two claims here are contradicted by the runs' train\_config.json: the ResNet18 backbone is trained FROM SCRATCH (pretrained\_backbone\_weights=null in every DP run) and only the state-based DP TEACHER trained on success-only, idle-pruned data; the four reported pixel DP arms train on the raw datasets. Corrected paragraph + appendix table: HRI\_results/late\_night\_9\_18/03\_diffusion\_policy\_appendix.md.}`
- `sections/04-evaluation.tex` line 78, appended after "...like-to-like comparisons" — `\llm{Item 03: define "cleaned" (64 human tapes reaching a grasp, pre-grasp idle collapsed; teacher dp\_phase/dHfull\_pruned\_DP\_s0), say FIRST attempt per start with no selection, and say the pixel DP trains on the RAW datasets (same tapes as the other learners, no separate pixel-demo set). Corrected paragraph in HRI\_results/late\_night\_9\_18/03\_diffusion\_policy\_appendix.md section 5.}`
- `appendix/a-research-methods.tex` line 45 (new line after `\subsection{Diffusion Policy}`) — `\llm{Item 03: paste-ready text + hyperparameter table (tab:dp-hparams) in HRI\_results/late\_night\_9\_18/03\_diffusion\_policy\_appendix.md section 4; all values read back from the runs' train\_config.json (backbone from scratch, no EMA, 100k updates, batch 64, random 56 crop, final checkpoint only).}`

## 8. Sources

Paper: `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sections/03-method.tex:29-33`, `sections/04-evaluation.tex:17,74-90`,
`appendix/a-research-methods.tex:38-46`, `tables/phase_performance.tex`, `preamble.tex:42` (`\llm`).

Code (local, branch `ladder-unify-2026-09-11`): `cluster/sbatch_dp_px.sh`, `cluster/submit_ah_dp_px.sh`, `cluster/sbatch_dp_e2e.sh`,
`cluster/sbatch_dp.sh`, `cluster/sbatch_dp_place.sh`, `cluster/ah_build_pixel_sets.sh`, `cluster/e2e_eval_cells.sh`,
`cluster/planner72_px_2026-09-15/{launch/dp.sbatch,CAMPAIGN.json,CONTRACT.json,DP_DATA_CONTRACT.json,DP_TEMPORAL_LOCAL.json,HANDOFF.md,eval_final.py}`,
`cluster/r2teacher_px_2026-09-15/{diffs/dp.sbatch.diff,CAMPAIGN.json,CONTRACT.json,DP_DATA_CONTRACT.json,HANDOFF.md}`,
`baselines/convert_to_lerobot.py`, `baselines/prune_full_v1.py`, `baselines/verify_px_lerobot.py`, `baselines/rl/full_demos.py:310-337`,
`baselines/record_demos.py:21,559`, `baselines/dp_runner.py:16-110`, `baselines/eval_e2e.py:37-39,61-63,80,138,285-291,336-360`,
`experiments/full_state_planner_2026-09-12/matched72_completed_2026-09-15/finish.py:79-80`, `experiments/wm_teacher_matched_2026-09-15/{validate_and_export.py:22-23,write_handoff.py:39}`.

lerobot: `~/workspace/lerobot` @ `d324ffe8` — `src/lerobot/policies/diffusion/configuration_diffusion.py` (defaults, lines 88-160, presets 214-230),
`modeling_diffusion.py` (173-186, 204-207, 246-282, 311-362, 448-509, 602-720), `scripts/lerobot_train.py:125-140`, `configs/train.py:50-63`,
`optim/optimizers.py:86-95`; `grep -rni "\bema\b|ExponentialMovingAverage|use_ema"` over the diffusion policy + trainer → none.

Cluster (read-only, `ssh pax`): `pip show lerobot torchvision diffusers torch`; `git -C $LAB/lerobot_src log/branch/remote`;
`$LAB/gp_ah/baselines/outputs/dp_px/ah_dp_px_{dH,dM}_s{0..7}/checkpoints/100000/{pretrained_model/train_config.json,pretrained_model/config.json,dp_sidecar.json}`;
`$LAB/planner72_px_2026-09-15/runs/dp/planner72_dp_px_dPlanner72_s{0..7}/…`; `$LAB/r2teacher_px_2026-09-15/runs/dp/r2teacher_dp_px_dR2fromH_px_s{0..7}/…`;
`$LAB/genesis_pickaplace/baselines/outputs/dp_phase/dHfull_pruned_DP_s0/checkpoints/100000/…`;
`$LAB/genesis_pickaplace/baselines/matched_w3/{dHfull_all,dDPfull_first,dHfull_pruned,dDPfull}/{manifest.json,prune_manifest.json,lerobot*/meta/info.json,lerobot*/genesis_source.json}`;
`$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals/{dp_raw/manifest.json,lerobot*/…}`; `$LAB/r2teacher_px_2026-09-15/data/training_v1/…`;
`fresh_eval_{rnd30,hold15}_sample/metrics.json` under each DP run; `sacct --name=ah_dp_px_dM_s6`;
`python baselines/verify_px_lerobot.py --px … --ref … --raw …` ×4 (all `PX-DATASET-OK`).

Notes: `HRI_results/late_night_9_18/00_BRIEF.md`, `HRI_results/DEMO_SETS_2026-09-11.md` §1,§3,§4, `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`,
`HRI_results/STATE_PIXEL_4x4_2026-09-16.md`, `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`, `HRI_results/{results,VERIFICATION}.md`,
`paper/PHASE_PLAN_2026-09-04.md` (ab), (ah), (P-MP-20260915), `paper/AH_DP_PIXEL_BUILD_2026-09-14.md`, `paper/MUST_HAVE_RESULTS.md:65-90`,
`paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md`, `paper/PHASE_RESULTS_2026-09-05.md:199-239`, `paper/figures/px_phase_2026-09-14/px_results_4x4_sampled_per_seed.csv`.
