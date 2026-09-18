# Item 02 — RLPD: paper text vs code/training procedure; appendix hyperparameter section

**Status: DONE (2026-09-17/18).** The RLPD paragraph in `sections/03-method.tex`, the Results paragraph in
`sections/04-evaluation.tex`, the empty appendix subsection and the HANDOFF notes were checked against the code of
record (`baselines/rl/{train_rlpd,rlpd_sac,rlpd_pixel,full_env,full_demos}.py`), the launchers that produced the
4×4 pixel study, and — read-only on the cluster — the sidecars, `ladder_provenance.json` and Slurm logs of one run
per dataset plus a completion census of every RLPD run dir. The four demonstration sets were hashed and censused on the
cluster. Deliverables below: (a) a paste-ready appendix `\subsection` with a hyperparameter table, (b) a corrected
main-text paragraph, (c) a DISCREPANCIES list, (d) bib entries the paper is missing, (e) the data-true-to-source
evidence. Two `\llm{}` notes were inserted (§"\llm notes inserted"). Nothing else in the .tex was touched; nothing
committed.

Partial: the r2teacher-dataset RLPD cell had 4 of 8 seeds scored (5 trained) at audit time — see §5.

---

## 1. What actually ran (one command per dataset, from the cluster)

All four datasets ran the SAME trainer command; only `--demo-dir`, `--out-dir`/`--run-name`, `--seed` and
(for the (ag)/planner/teacher jobs) `--ckpt-every 25000` differ. The launcher builds `TRAIN_ARGS` and the trainer
prints a `[cfg]` line and writes a sidecar; both were read for every dataset.

**Launcher of record (human, machine):** `cluster/sbatch_rlpd_px.sh` lines 151–159 (`TRAIN_ARGS`), tree
`$LAB/gp_pxr` @ `9841633e` (Slurm log `== TREE ... g9841633e-dirty`). Submission (PROVENANCE §5):

```
GENESIS_PICKAPLACE_ROOT=$LAB/gp_pxr OBS=pixels LADDER=nested_sparse10 TIP_GUARD=not_in_hand ARM=dH SEED=0 \
  sbatch -J e2e_rlpd_px_dH_s0 cluster/sbatch_rlpd_px.sh              # ARM=dDPfirst = machine (DP-teacher) set
```

**Trainer command the launcher runs** (reconstructed from `TRAIN_ARGS`; the `[cfg]` line in each log confirms every
value — quoted below):

```
python baselines/rl/train_rlpd.py --steps 250000 --scope full --ladder nested_sparse10 --tip-guard not_in_hand \
  --demo-format segment --demo-dir <DEMO> --obs pixels --image-aug shift4 --buffer-size 300000 \
  --action-mode delta_joint --delta-ref target --action-repeat 4 \
  --train-max-steps 1200 --eval-max-steps 1200 --eval-freq 0 --gamma 0.99 --backup-entropy off \
  --per-member-ln off --pick-hold-reward off --pick-shaping off --utd 10 --ensemble-size 10 --subset-size 2 \
  --demo-batch 128 --demo-shaping off --pick-shaping-terminal-zero on --demo-terminal-guard on \
  --sim-variant gc_kp4_riser3_shelf6 --ckpt-every <0|25000> --ckpt-fracs 0.4,1.0 \
  --out-dir <OUT> --run-name <RUN> --project genesis_paper --seed <S> --device cuda
```

| dataset (paper name) | `ARM` / `--demo-dir` | run dir (cluster) | log with the `[cfg]` line | tree |
|---|---|---|---|---|
| Human | `dH` → `$W/demos_state_full/dHfull_all_rns10h_img` | `$LAB/gp_pxr/baselines/rl/checkpoints/e2e_px/e2e_rlpd_px_dH_s0` | `$LAB/gp_pxr/e2e_rlpd_px_3685153.out` | gp_pxr @ 9841633e |
| DP (machine) | `dDPfirst` → `$W/demos_state_full/dDPfull_first_rns10h_img` | `.../e2e_px/e2e_rlpd_px_dDPfirst_s0` | `$LAB/gp_pxr/e2e_rlpd_px_3685154.out` | gp_pxr @ 9841633e |
| Planner | `dPlanner72` → `$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals/native_rns10h_img` | `$LAB/planner72_px_2026-09-15/runs/rlpd/e2e_rlpd_px_dPlanner72_s0` | `$LAB/planner72_px_2026-09-15/slurm/planner72_rlpd_px_s0_3738544.out` | `$LAB/planner_px_2026-09-15/gp` @ e8287af0 |
| R2 (teacher) | `dR2fromH_px` → `$LAB/r2teacher_px_2026-09-15/data/training_v1/native_rns10h_img` | `$LAB/r2teacher_px_2026-09-15/runs/rlpd/e2e_rlpd_px_dR2fromH_px_s0` | `.../slurm/r2teacher_rlpd_px_s0_3765113.out` (first attempt 3738576 segfaulted at world build; re-run) | `$LAB/planner_px_2026-09-15/gp` @ e8287af0 |

The `[cfg]` line, identical in all four logs except `demo_dir`:

```
[cfg] RLPD | obs=pixels E=10 Z=2 UTD=10 gamma=0.99 ent_coef=auto target_entropy=-3.5 demo_batch=128/256 backup_entropy=off per_member_ln=off pick_hold_reward=off pick_hold_k=25 pick_shaping=off q_watchdog=20.00 scope=full action_mode=delta_joint delta_ref=target action_repeat=4 demo_dir=<...> demo_format=segment demo_terminal_guard=on demo_shaping=off pick_shaping_terminal_zero=on train_max_steps=1200 ckpt_fracs=0.4,1.0
```

Planner/teacher submissions (`cluster/planner72_px_2026-09-15/SUBMIT_COMMANDS.txt`, same for r2teacher with
`ARM=dR2fromH_px`), environment values embedded in a sealed copy of the same launcher:

```
env ARM=dPlanner72 BUFFER_SIZE=300000 CKPT_EVERY=25000 FAR_RELEASE=0 GAMMA=0.99 GENESIS_PICKAPLACE_ROOT=$LAB/planner_px_2026-09-15/gp \
  GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 IMAGE_AUG=shift4 LADDER=nested_sparse10 OBS=pixels SEED=0 STEPS=250000 TIP_GUARD=not_in_hand \
  sbatch --partition=gpu,preempt --qos=preempt --job-name=planner72_rlpd_px_s0 ... preparation/launch/rlpd.sbatch
```

Every run's `ladder_provenance.json` stamps
`ladder=nested_sparse10 | home=10 | max_return=10 | terminal=home+tipped | tip=tilt>60deg&not_in_hand@4f |
full_env=584bea6d9c6d genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632` (same three code hashes on all
four datasets; census in §5).

Evaluation job each training job submits (`cluster/sbatch_rlpd_px_eval.sh` → `cluster/e2e_eval_cells.sh` with
`EVAL_SCRIPT=baselines/eval_e2e_px.py`, 64-physical-core CPU nodes):

```
python baselines/eval_e2e_px.py --kind sac --checkpoint <run>/rlpd_final.zip --ic-file baselines/eval_ics.json \
  --ic-set <rnd|hold> --out <run>/fresh_eval_<rnd30|hold15>_<sample|mode>[_iso] --mode <sample|mode> --seed 0 \
  --max-steps 1200 --sim-variant gc_kp4_riser3_shelf6 --arm <ARM> --require-cores 64
```

---

## 2. (a) Paste-ready appendix subsection

```latex
\subsection{Reinforcement Learning with Prior Data}
\label{appx:rlpd}

\paragraph{Algorithm.}
We re-implement RLPD~\citep{ball2023efficient} on top of the Soft Actor-Critic
implementation in Stable-Baselines3~2.8~\citep{raffin2021sb3}
(\texttt{baselines/rl/rlpd\_sac.py}, \texttt{rlpd\_pixel.py}, \texttt{train\_rlpd.py}). The three
ingredients of RLPD are kept: (i) \emph{symmetric sampling} -- every update batch of 256 transitions
is 128 drawn uniformly from the online replay buffer and 128 drawn uniformly from an immutable
demonstration buffer, with no behaviour-cloning term; (ii) a \emph{critic ensemble} of $E{=}10$
$Q$-networks with LayerNorm after every hidden layer, whose Bellman target is the minimum over a
random subset of $Z{=}2$ target critics; (iii) a \emph{high update-to-data ratio}: ten critic
gradient steps per environment decision. The actor and the entropy coefficient are updated once per
decision (on the last of the ten critic steps), and the actor is trained against the ensemble mean
$Q$. Following RLPD's setting for sparse-reward domains, the entropy term enters only the actor
objective, not the critic target (\texttt{backup\_entropy = off}). The entropy coefficient is learned
(SAC's automatic tuning, initial value~1) against a target entropy of $-|\mathcal{A}|/2 = -3.5$.
The LayerNorm affine parameters are shared across the ten ensemble members (a departure from a
per-member LayerNorm that we did not correct for this study).

\paragraph{Observation, encoder and augmentation.}
The policy observes two $64{\times}64$ RGB images (a fixed top camera and a wrist camera, concatenated
to a $64{\times}64{\times}6$ tensor) and an 8-d proprioceptive vector (six joint positions, the gripper
motor position and the grip effort). It receives no object or goal pose. Images are encoded by a
DrQ-v2 style convolutional encoder~\citep{yarats2022drqv2}: four $3{\times}3$ convolutions with 32
channels (stride 2,1,1,1, ReLU), flattened ($32{\times}25{\times}25$) and projected by a linear layer to
50 units followed by LayerNorm and $\tanh$; the raw proprioceptive vector is concatenated to give a
58-d feature. The encoder is owned by the critic and trained through the critic loss only; the actor
reads the same encoder with gradients detached; the target critic keeps a Polyak-averaged copy of
the encoder. Every update batch (online and demonstration halves, current and next observation) is
augmented with a random shift~\citep{kostrikov2021drq}: replicate-pad by 4 pixels and crop back to
$64{\times}64$ at an integer offset drawn per sample. There is no frame stacking; each observation is a
single rendered frame. Demonstration frames are rendered by the same camera call as the online
frames, once after reset and once after every decision.

\paragraph{Environment interface.}
Actions are 7-d in $[-1,1]$: six per-decision joint-target deltas, scaled by a cap of $0.025$\,rad per
simulation frame and integrated onto a running target (leash $5{\times}$ the cap), plus one gripper
command. Each decision is held for four simulation frames (action repeat~4), so the 1200-frame
horizon is 300 decisions and the discount $\gamma{=}0.99$ is applied per decision. Reward is the
\texttt{nested\_sparse10} ladder: $+10$ once, when the can has been set down, pushed from the far side
and rests in contact with the goal can (\texttt{home}), which ends the episode; a can that tips more
than $60^\circ$ while not in the hand (sustained for four frames) also ends the episode with reward~0;
the horizon truncates the episode without a terminal flag (the value is bootstrapped). The first
1{,}000 decisions of a run are uniformly random actions; thereafter actions are sampled from the
policy. Demonstrations are loaded verbatim from the shared demonstration files (state, action, reward
and terminal columns as recorded; \S\ref{appx:datasets}); in every dataset the paid \texttt{home}
transition is stored as \emph{non-terminal} (only tip endings are terminal), so the critic bootstraps
through the demonstrated $+10$ while the online environment terminates on it.

\paragraph{Budget, checkpoints and evaluation.}
Each seed trains for 250{,}000 decisions ($10^6$ simulation frames, $2.5\times10^6$ critic updates),
one seed per GPU (L40S/A100/L40/H200; 3.3--5.0 decisions per second, 14--21 hours), with the online
buffer holding the last 300{,}000 decisions. The checkpoint at 250{,}000 decisions is evaluated. Each
checkpoint is rolled out once from each of 30 random can starts (\texttt{rnd30}: can position uniform
over the bounding box of the demonstrated starts with a 1\,cm margin, goal fixed) and once from each
of the 15 demonstrated starts (\texttt{hold15}), for at most 1200 simulation frames, both with sampled
actions (one draw per decision, seed~0) and with the policy mean (deterministic). The phase success
rates in the main text are the sampled-action \texttt{rnd30} cells; the registered human-versus-machine
comparison for RLPD used the deterministic \texttt{rnd30} cell. Evaluation runs on CPU nodes of one
hardware class (64 physical cores) because contact simulation is not bit-reproducible across classes.

\begin{table}[htbp]\centering\small\setlength{\tabcolsep}{4pt}
\begin{tabular}{ll}\toprule
\textbf{RLPD hyperparameter} & \textbf{Value} \\\midrule
Base algorithm / implementation & SAC, Stable-Baselines3 2.8.0 (PyTorch 2.7) \\
Observation & top + wrist RGB $64{\times}64{\times}6$ (uint8) + 8-d proprio; no object/goal pose \\
Frame stack & none (1 frame) \\
Image encoder & DrQ-v2 conv$\times$4 (32 ch, $3{\times}3$, stride 2/1/1/1, ReLU) $\to$ Linear(20000$\to$50) $\to$ LayerNorm $\to$ $\tanh$; $1.03{\times}10^{6}$ parameters \\
Encoder training & through the critic loss; actor detached; Polyak target copy ($\tau{=}0.005$) \\
Image preprocessing / augmentation & $x/255-0.5$; random shift (pad 4, integer crop) on obs and next obs, both batch halves \\
Feature vector & 50 (image) $\oplus$ 8 (proprio, raw) $= 58$ \\
Actor & MLP $58\!\to\!256\!\to\!256$ (ReLU), Gaussian head, $\log\sigma\in[-20,2]$, $\tanh$-squashed \\
Critic ensemble & $E{=}10$ members: $65\!\to\!256\!\to\!256\!\to\!1$, LayerNorm after each hidden layer (affine shared across members), ReLU \\
Bellman target & $r + \gamma(1-d)\min_{i\in\mathcal{Z}}Q^{\text{targ}}_i(s',a')$, $|\mathcal{Z}|{=}2$ drawn per update, $a'\sim\pi$; no entropy term in the target \\
Critic loss & sum over the 10 members of the mean squared TD error \\
Actor loss & $\mathbb{E}[\alpha\log\pi(a|s) - \bar{Q}(s,a)]$ with $\bar{Q}$ the ensemble mean \\
Entropy coefficient $\alpha$ & learned (auto), initial 1.0, target entropy $-3.5$ ($=-|\mathcal{A}|/2$) \\
Discount $\gamma$ & 0.99 per decision \\
Polyak $\tau$ & 0.005 (critic and its encoder) \\
Optimiser / learning rate & Adam, $3{\times}10^{-4}$ for actor, critic (incl.\ encoder) and $\alpha$ \\
Batch composition & 256 $=$ 128 online $+$ 128 demonstration (uniform within each buffer) \\
Update-to-data ratio & 10 critic updates per decision; actor and $\alpha$ once per decision \\
Online replay buffer & 300{,}000 decisions (FIFO); demonstration buffer immutable, never evicted \\
Random-action warm-up & 1{,}000 decisions \\
Action space & $[-1,1]^7$: 6 joint deltas (cap 0.025 rad/frame, leash $5\times$) + gripper \\
Action repeat / horizon & 4 frames per decision; 1200 frames $=$ 300 decisions; horizon $=$ truncation (bootstrapped) \\
Reward / terminals & \texttt{nested\_sparse10}: $+10$ at \texttt{home} (terminal); tip $>60^\circ$ not-in-hand, 4 frames (terminal, 0) \\
Demonstration terminals & as recorded: tip endings terminal; the $+10$ \texttt{home} row non-terminal (all four datasets) \\
Training budget & 250{,}000 decisions ($10^6$ frames); one GPU per seed \\
Checkpoints & 100k and 250k decisions (+ every 25k for the planner, R2 and later human/DP seeds); the 250k checkpoint is scored \\
Seeds & 8 per dataset (design), seeds 0--7 \\
Evaluation & \texttt{rnd30} (30 random starts) and \texttt{hold15} (15 demonstrated starts), 1 episode each, 1200 frames, sampled and deterministic actions, seed 0, 64-core CPU class \\
World & Genesis 0.2.1, \texttt{gc\_kp4\_riser3\_shelf6}, identical for training, demonstrations and evaluation \\
\bottomrule\end{tabular}
\caption{RLPD hyperparameters and training protocol for the restocking task. Every value is the one
recorded in the run sidecars and Slurm logs of the reported seeds.}
\label{tab:rlpd-hparams}
\end{table}
```

Note for the head session: `\S\ref{appx:datasets}` is a placeholder for whichever appendix label the datasets end up
under (item 3/4 lanes); replace or drop.

---

## 3. (b) Corrected main-text RLPD paragraph (`sections/03-method.tex`, replaces lines 48–50)

```latex
\paragraph{RLPD}

Reinforcement Learning with Prior Data (RLPD)~\citep{ball2023efficient} is a model-free
reinforcement-learning-from-demonstrations method built on Soft Actor-Critic~\citep{haarnoja2018sac}:
a stochastic actor trained with a maximum-entropy objective against a learned critic. RLPD adds three
things to SAC. Half of every training batch is drawn from the demonstrations, which sit in a separate
buffer that online experience never overwrites; the critic is an ensemble of ten LayerNorm networks
whose target is the minimum over a random pair of members; and the critic takes ten gradient steps
per environment decision. RLPD imposes no behaviour-cloning loss and makes no assumption about
demonstration quality, so its demonstration set keeps every attempt, including unsuccessful ones
(Diffusion Policy, by contrast, trains on successful demonstrations only). For the pixel-based robot
task the actor and critic share a DrQ-v2 convolutional encoder~\citep{yarats2022drqv2} trained through
the critic loss, with random-shift augmentation~\citep{kostrikov2021drq} and no frame stacking. We use
RLPD on the robot task only, as the model-free counterpart to the world-model learners; all three
train on the same sparse terminal reward. Hyperparameters are in Appendix~\ref{appx:rlpd}.
```

Sentences deliberately removed and why:
- "trains with an entropy reward (like SAC) with additions" → the entropy term is in the actor objective only
  (`backup_entropy off`, `rlpd_sac.py:280-290`); reworded to "maximum-entropy objective".
- "RLPD benefits from both successful and failed demonstrations" → no citation exists for a benefit; our own
  all-data control found none (D5 below). Replaced by the neutral "makes no assumption about demonstration quality".
- "where a strong non-imitation baseline is needed and dense reward is not available" → every RL learner here
  trains on the same sparse reward (D6). Replaced.

---

## 4. (c) DISCREPANCIES — paper vs code / notes

| # | Where | Paper says | Code / record says | Evidence |
|---|---|---|---|---|
| D1 | 03-method.tex:50 | `\citep{ball2023efficient}` and `SAC\todo{cite}` | **`ball2023efficient` is cited but defined in NEITHER `sample-base.bib` nor `software.bib`** (undefined citation at build); no SAC entry either. Entries supplied in §6. | `grep -n -i 'ball\|haarnoja' *.bib` → only `zhou2024efficient` |
| D2 | 03-method.tex:50 | "RLPD samples a stochastic actor-critic and trains with an entropy reward (like SAC)" | Garbled. RLPD = SAC + symmetric sampling + LN critic ensemble + high UTD. The entropy bonus is in the **actor** loss; the **critic target carries no entropy term** (`--backup-entropy off`, RLPD's sparse-domain setting). | `rlpd_sac.py:271-320`; `[cfg] ... backup_entropy=off` in all four logs |
| D3 | 03-method.tex:50; HANDOFF §1 | "ensemble of ten networks with layer normalization" | True, but the LayerNorm **affine parameters are shared across the 10 members** (`nn.LayerNorm(h)` in `EnsembleCritic`; `per_member_ln=off`). HANDOFF lists this as a recipe gap. | `rlpd_sac.py:71-119`; sidecar `"per_member_ln": "off"` |
| D4 | 03-method.tex:50 | "the critic updates ten times per environment step" | Ten critic updates per **decision** (= 4 simulator frames, action repeat 4); actor and α once per decision. RLPD's reference recipe is UTD 20 (HANDOFF §4 "where the paper uses 20"). | `rlpd_sac.py:256-323`; `train_rlpd.py:55-57` |
| D5 | 03-method.tex:50 | "RLPD benefits from both successful and failed demonstrations \todo{citation?}" | No citation supports a *benefit* from failures. Our own all-data control (state-based pick, human set + 40 non-pick/failed recordings vs raw): **RLPD 0.554 vs 0.600, p 0.567 — no gain**. What is true: the demo sets used here keep unsuccessful attempts (human 9 no-pick tapes; DP 8), which is a *design choice* (fail tapes omitted from the human FULL set by label are a separate disclosure, memory note). | `paper/MORNING_TABLE_2026-09-04.md:129`; DEMO-SHA lines `pick=65 nopick=9` / `pick=64 nopick=8` |
| D6 | 03-method.tex:50 | "used on the robot task only, where ... dense reward is not available" | All RL learners on the robot task (RLPD, DfD, R2Dreamer) train on the same sparse terminal +10; PinPad4/PushT are sparse too (04-evaluation:30 "1.0 for a successful trial"). Reward density is not the reason. | `full_env.py:149-176` (`nested_sparse10`); 04-evaluation.tex:93 |
| D7 | appendix (empty); 04-eval:93 | "sparse reward of +10 for a successful slide" | Online: `home` pays +10 and **terminates**; tip terminates with 0; horizon truncates. In the **demonstration buffers the +10 row is NOT terminal** in all four datasets (0 rewarded terminals; the 18/22/2/4 terminals are all tip endings), so RLPD's TD target bootstraps through the demonstrated +10 while the online env terminates on it. Consistent across datasets (fair comparison), but a demo/online mismatch the appendix must state. Planner/R2 sets had their 68 `home` terminal flags explicitly cleared to match the human/DP convention. | census §5; `[demos] ... 13 rewarded, 18 terminal (18 zero-reward terminals = tip-guarded fails)`; `TERMINAL_REVISION.json` (planner: `rewarded_terminals: 0`; r2: `rewarded_terminals_cleared: 68`); `paper/PX_RLPD_PIPELINE_2026-09-13.md` §4, §8.8; `LADDER_IMPL_NOTES` §4 |
| D8 | HANDOFF §1/§4 vs paper | paper silent on the pixel recipe | No frame stack (single frame); integer shift (DrQ-v1) not DrQ-v2's bilinear shift; trunk shared by actor and critic; proprio concatenated raw; Polyak target encoder. None ablated. The user has flagged the missing frame stack as a limitation. | `rlpd_pixel.py:121-231`; `PX_RLPD_PIPELINE` §3, §8.7 |
| D9 | 04-eval:93 "8 seeds"; PROVENANCE §5 | 250k-decision budget | Human s2–s8 and DP s2–s7 were launched with `STEPS=2000000` (the world-model budget), hit the 30 h wall clock (400–592k decisions) and were **scored at `rlpd_250000_steps.zip`**; no `rlpd_final.zip`/`ckpt_100` exists for them. Equivalent to a 250k run because the learning rate is constant and nothing else depends on the total budget, but the launcher defect is real and should be disclosed. s0/s1 (human, DP) and all planner/R2 seeds stopped at 250k. | cluster census §5; `e2e_rlpd_px_3692544.out` "CANCELLED ... DUE TO TIME LIMIT"; sidecar `"steps": 2000000` |
| D10 | 04-eval:93 "8 seeds" | 8 per cell | Human 8 (+ an extra s8 outside the design, excluded from tables), DP 8, Planner 8, **R2-teacher 4 scored / 5 trained / 3 not started (s4, s6, s7 deprioritised `Nice=10000`)** at audit time. | census §5; `STATE_PIXEL_4x4` 2026-09-17 08:30 |
| D11 | 04-eval table caption "Sampled-action evaluation" | RLPD row = sampled rnd30 | Correct for the table, but the **registered** RLPD readout is the deterministic (MODE) rnd30 cell (human 0.000 v machine 0.125, exact p 0.20, 8 v 8), and HANDOFF §2 records that sampled was chosen for the chart *after* seeing it scores higher. Say which cell each number is, and do not attach p-values to the sampled RLPD cell. | `PX_SPARSE10_RESULTS_SUMMARY` §2; `HANDOFF_PIXEL_4x4` §2 |
| D12 | 04-eval:93 "30 new can positions chosen randomly from the demo support" | — | rnd30 = uniform over the **bounding box** of demonstrated can positions + 1 cm margin (not the demo distribution); 4 of the 30 starts lie inside the shelf footprint (CONFOUNDS row 82) and ~10/30 outside the pruned DP set's support. Wording defensible; "bounding box" is more exact. | `baselines/ic_sampling.py:4-26`; `paper/CONFOUNDS.md` row 82 |
| D13 | 04-eval:71 "12/40 slid the can into contact" | human twin census | The pixel human set RLPD trained on has **13** `home` tapes (Σ 130) and the DP set 14 (Σ 140); the state sets have 12/12. The difference is re-execution on another CPU class (disclosed in DEMO_SETS §2). State which set the sentence counts. | `[segments] ... Sigma reward 130 over 13 rewarded`; `DEMO_SETS_2026-09-11.md` §3 |
| D14 | 03-method.tex:29 | `Appendix~\ref{appx:hyperparameters}` | Label does not exist; the appendix section is `\label{appx:training}`. | `grep -rn appx:hyperparameters` (see §"\llm notes") |
| D15 | appendix | — | Observed but undocumented: the critic overestimates — `[Q-WATCHDOG]` mean actor-state Q 20–46 vs max return 10 through most of training on human/DP seeds. A diagnostic, not a hyperparameter; worth one sentence if the RLPD failure on human data is discussed. | `e2e_rlpd_px_3685153.out`, `..._3685154.out` |
| D16 | 03-method.tex:50 "half of every training batch comes from the demonstrations" | — | Matches (128/256, uniform draws, immutable demo buffer). No discrepancy. | `rlpd_sac.py:248-267`; `[cfg] demo_batch=128/256` |

---

## 5. Data true to source — the four RLPD demonstration sets

Method: on the cluster, for each set, every `*.npz` was loaded (sorted by file name) and the `action` and `state`
arrays hashed (sha256 over the concatenated float32 bytes), and the `reward`/`is_terminal` columns censused
(script: this session's `scratchpad/census.py`, run with `$LAB/condaenv/genesis/bin/python`, 2026-09-17).
The launcher's own gate hashes the whole files (`DEMO-SHA ... sha=`), which is what the sidecar `demo_sha256` records.

| set (paper name) | path | tapes | rows / transitions | Σ reward | rewarded rows | terminal rows | rewarded terminals | actions sha (this audit) | states sha |
|---|---|---|---|---|---|---|---|---|---|
| Human source (staged ladder) | `$W/demos_state_full/dHfull_all` | 74 | 29,295 / 29,221 | 118 | 86 | 18 | 3 | `314a7bb820ff8ace` | `bf1fc66a0f26b386` |
| Human, state relabel | `.../dHfull_all_rns10h` | 74 | same | 120 | 12 | 18 | 0 | `314a7bb820ff8ace` | `bf1fc66a0f26b386` |
| **Human, pixels (RLPD trained on this)** | `.../dHfull_all_rns10h_img` | 74 | same | **130** | 13 | 18 | 0 | `314a7bb820ff8ace` | `bf1fc66a0f26b386` |
| DP source (staged) | `.../dDPfull_first` | 72 | 36,906 / 36,834 | 131 | 86 | 22 | 8 | `c5ece17f21b5b7ef` | `9d59d0c5169e45e9` |
| DP, state relabel | `.../dDPfull_first_rns10h` | 72 | same | 120 | 12 | 22 | 0 | `c5ece17f21b5b7ef` | `9d59d0c5169e45e9` |
| **DP, pixels (RLPD)** | `.../dDPfull_first_rns10h_img` | 72 | same | **140** | 14 | 22 | 0 | `c5ece17f21b5b7ef` | `9d59d0c5169e45e9` |
| **Planner, pixels (RLPD)** | `$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals/native_rns10h_img` | 72 | 17,944 / 17,872 | **680** | 68 | 2 | 0 | `06c928d362b0d387` | `5137ce97b23b12df` |
| **R2 teacher, pixels (RLPD)** | `$LAB/r2teacher_px_2026-09-15/data/training_v1/native_rns10h_img` | 72 | 2,350 / 2,278 | **680** | 68 | 4 | 0 | `ab1ba148efcbdc54` | `e295269cd57bd62d` |

Findings:
- **Human and DP:** the action AND state arrays of the pixel sets are byte-identical to the relabelled state sets and to
  the staged-ladder source sets (same hash on every stage; 74/74 and 72/72 tapes). The manifests carry their own
  per-file action hashes (`relabel.actions_sha256` human `77bc4875…`, machine `671614c5…`, identical for `_rns10h`
  and `_rns10h_img`), agreeing. Only the reward column and the image column differ. Σ reward 130/140 (13/14 `home`
  tapes) versus 120/120 on the state sets is the disclosed re-execution difference on another CPU class
  (`DEMO_SETS_2026-09-11.md` §2–3).
- **Planner:** transferred as one archive, sha256 `9aff5c95…` verified on arrival (`DATA_TRANSFER.json`
  `ARCHIVE_TRANSFER_VERIFIED`; same sha in `cluster/planner72_px_2026-09-15/CONTRACT.json` and
  `HRI_results/HANDOFF_PLANNER72_NONEMPTY_2026-09-15.md`). 72 tapes / 17,872 decisions / 67 initial geometries
  (`SUMMARY.json`: home 68, tip 2, controller-abort 2). Native actions are the delta-joint encoding of the planner's
  absolute targets in `dp_raw/` (`audit.json` `dp_action_decode` 1.19e-6). Relabel = `10 × native recorded reward`
  (`nested_sparse` → `nested_sparse10`, "no new predicate").
- **R2 teacher:** archive sha256 `008a06e6…` verified on arrival (`DATA_TRANSFER.json`; same in
  `HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`). 72 first-attempt tapes / 2,278 decisions / 67 geometries; teacher n=1
  (R2Dreamer seed 15 trained on the human pixel set, checkpoint sha `84b2e72c…`). Reward unchanged (+10 at home).
- **Terminal flags (what RLPD bootstraps on):** in all four sets `rewarded_terminals = 0`: the +10 `home` row is
  non-terminal (`done=False` in the transition tuple, `full_demos.py:129`), so the target is
  `10 + γ·min Q(s',a')`; only tip endings are `done=True` (human 18, DP 22, planner 2, R2 4 — the trainer prints them
  as "zero-reward terminals = tip-guarded fails"). Human/DP inherit this from the recorded tapes
  (`LADDER_IMPL_NOTES` §4: `is_terminal` left as recorded); planner/R2 had their 68 `home` terminals cleared by a
  versioned transform to match (`TERMINAL_REVISION.json`). Online, `home` terminates. See D7.
- **Tape-length asymmetry** (not a source-fidelity issue, but a confound the RLPD comparison inherits): median
  decisions per tape human 388, DP 600 (the cap), planner 255, R2 32; success rate human 13/74, DP 14/72, planner
  68/72, R2 68/72. The demo buffer is sampled uniformly over transitions, so the fraction of rewarded rows in the demo
  half is 0.04 % (human), 0.04 % (DP), 0.38 % (planner), 3.0 % (R2).

**Trained with each dataset — completion census (cluster, 2026-09-17):**

| dataset | run dirs | trained to 250k | scored (rnd30 sample + mode) | notes |
|---|---|---|---|---|
| Human | s0–s8 (+ smoke s9991) | 9 | 9 | s2–s8 via `rlpd_250000_steps.zip` (D9); s8 outside the design |
| DP | s0–s7 | 8 | 8 | s2–s7 via `rlpd_250000_steps.zip` (D9) |
| Planner | s0–s7 | 8 | 8 | all `rlpd_final.zip`, budget reached |
| R2 teacher | s0–s7 | 5 (s0,1,2,3,5) | 4 (s0,1,3,5) | s2 trained, cells not yet written; s4/s6/s7 not started (`Nice=10000`); s0 first attempt segfaulted at world build (job 3738576), re-run as 3765113 |

Every run dir's `ladder_provenance.json` reads `ladder=nested_sparse10` (33/33 incl. smoke).

---

## 6. (d) Bib entries the paper needs (none present in `sample-base.bib` / `software.bib`)

```bibtex
@inproceedings{ball2023efficient,
  title={Efficient Online Reinforcement Learning with Offline Data},
  author={Ball, Philip J. and Smith, Laura and Kostrikov, Ilya and Levine, Sergey},
  booktitle={Proceedings of the 40th International Conference on Machine Learning},
  series={Proceedings of Machine Learning Research},
  volume={202},
  pages={1577--1594},
  year={2023},
  publisher={PMLR}
}

@inproceedings{haarnoja2018sac,
  title={Soft Actor-Critic: Off-Policy Maximum Entropy Deep Reinforcement Learning with a Stochastic Actor},
  author={Haarnoja, Tuomas and Zhou, Aurick and Abbeel, Pieter and Levine, Sergey},
  booktitle={Proceedings of the 35th International Conference on Machine Learning},
  series={Proceedings of Machine Learning Research},
  volume={80},
  pages={1861--1870},
  year={2018},
  publisher={PMLR}
}

@inproceedings{yarats2022drqv2,
  title={Mastering Visual Continuous Control: Improved Data-Augmented Reinforcement Learning},
  author={Yarats, Denis and Fergus, Rob and Lazaric, Alessandro and Pinto, Lerrel},
  booktitle={International Conference on Learning Representations},
  year={2022}
}

@inproceedings{kostrikov2021drq,
  title={Image Augmentation Is All You Need: Regularizing Deep Reinforcement Learning from Pixels},
  author={Kostrikov, Ilya and Yarats, Denis and Fergus, Rob},
  booktitle={International Conference on Learning Representations},
  year={2021}
}

@article{raffin2021sb3,
  title={Stable-Baselines3: Reliable Reinforcement Learning Implementations},
  author={Raffin, Antonin and Hill, Ashley and Gleave, Adam and Kanervisto, Anssi and Ernestus, Maximilian and Dormann, Noah},
  journal={Journal of Machine Learning Research},
  volume={22},
  number={268},
  pages={1--8},
  year={2021}
}
```
(Page numbers/volumes are from memory of the published proceedings; verify before the camera-ready.)

---

## \llm notes inserted

1. `~/workspace/overleaf/6a96f0e5337340dfd4edea88/sections/03-method.tex` line 50, appended after the anchor
   `...dense reward is not available.`:
   `\llm{RLPD paragraph audited against the code (rlpd\_sac.py / rlpd\_pixel.py / train\_rlpd.py + cluster sidecars): corrected paragraph, appendix subsection with hyperparameter table, bib entries and 16 discrepancies in HRI\_results/late\_night\_9\_18/02\_rlpd\_appendix.md. Headline fixes: ball2023efficient is cited but missing from both .bib files; entropy term is actor-only (no entropy backup); LayerNorm affine shared across the 10 critics; UTD 10 per decision (RLPD reference uses 20); no citation for ``benefits from failed demos'' and our own all-data control shows none; every RL learner here trains on the same sparse +10 terminal reward, so ``dense reward is not available'' is not the reason; Appendix ref appx:hyperparameters is undefined (appendix label is appx:training).}`
2. `.../appendix/a-research-methods.tex`, new line after the anchor `\subsection{Reinforcement Learning with Prior Data}`:
   `\llm{Paste-ready subsection (four paragraphs + hyperparameter table, label appx:rlpd) in HRI\_results/late\_night\_9\_18/02\_rlpd\_appendix.md \S2, every value taken from the run sidecars and Slurm logs of the reported seeds. Note for the writer: in all four demonstration sets the +10 home row is stored non-terminal (only tip endings are terminal) while the online environment terminates on home; and human s2--s8 / DP s2--s7 were scored from the 250k-decision snapshot of over-budget runs (launcher defect, disclosed in \S4 D9 of that file).}`

---

## Sources

Paper: `sections/03-method.tex` (lines 23–50), `sections/04-evaluation.tex` (lines 61–105), `appendix/a-research-methods.tex`
(36–48), `preamble.tex:42` (`\llm`), `sample-base.bib`, `software.bib` (grep for ball/haarnoja/yarats/raffin: none).

Code (local repo, branch `ladder-unify-2026-09-11`): `baselines/rl/rlpd_sac.py` (all), `baselines/rl/rlpd_pixel.py` (all),
`baselines/rl/train_rlpd.py` (all), `baselines/rl/full_demos.py` (all), `baselines/rl/full_env.py` (74–260, 654–670, 723–890),
`baselines/eval_e2e_px.py` (52–76, 146, 340–365), `baselines/ic_sampling.py` (1–30), `cluster/sbatch_rlpd_px.sh`,
`cluster/sbatch_rlpd_e2e.sh`, `cluster/sbatch_rlpd_px_eval.sh`, `cluster/e2e_eval_cells.sh`,
`cluster/planner72_px_2026-09-15/{launch/rlpd.sbatch,diffs/rlpd.sbatch.diff,SUBMIT_COMMANDS.txt,RUN_PLAN.json,CONTRACT.json,DATA_FILES.json,HANDOFF.md}`,
`cluster/r2teacher_px_2026-09-15/{diffs/rlpd.sbatch.diff,CONTRACT.json,DATA_FILES.json}`,
`cluster/cohort_revision_2026-09-15/REGISTRATION.md`, `.venv-eval/.../stable_baselines3` 2.8.0 (`common/buffers.py:193`,
`sac/policies.py:21-22,57`, `sac/sac.py:180-189`, `common/off_policy_algorithm.py:398`).

Docs: `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`, `HRI_results/STATE_PIXEL_4x4_2026-09-16.md`,
`HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md` §5, §7.3–7.4, `HRI_results/DEMO_SETS_2026-09-11.md` §1–3,
`HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`, `HRI_results/HANDOFF_PLANNER72_NONEMPTY_2026-09-15.md`,
`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`, `paper/PX_RLPD_PIPELINE_2026-09-13.md` §2–4, §8,
`paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md` §2, `paper/MORNING_TABLE_2026-09-04.md:129`,
`paper/LADDER_IMPL_NOTES*.md` §4, `paper/CONFOUNDS.md` row 82.

Cluster (read-only, `ssh pax`, 2026-09-17): run sidecars `rlpd_final.action_mode.json` / `rlpd_250000_steps.action_mode.json`,
`ladder_provenance.json` and Slurm logs for `e2e_rlpd_px_dH_s0` (3685153), `e2e_rlpd_px_dDPfirst_s0` (3685154),
`e2e_rlpd_px_dH_s2` (3692544), `e2e_rlpd_px_dPlanner72_s0` (3738544), `e2e_rlpd_px_dR2fromH_px_s0` (3738576 segfault, 3765113);
`repeat.json` + full npz census of the 8 sets in §5; `DATA_TRANSFER.json`, `TERMINAL_REVISION.json`, `SUMMARY.json`, `audit.json`
(planner72), `TERMINAL_REVISION.json`, `METADATA_REVISION.json` (r2teacher); `ls` census of all RLPD run dirs.
