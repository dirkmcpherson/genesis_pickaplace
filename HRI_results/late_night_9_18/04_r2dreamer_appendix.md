# Item 04 — R2Dreamer: paper text vs. code, appendix hyperparameter section, data true to source

**Status: DONE (audit complete; LaTeX ready to paste; `\llm{}` notes inserted).** Every hyperparameter below was read
from the tree of record (`$W/r2dreamer_px` @ `0b1b9d8`, clean; identical copy `$LAB/planner_px_2026-09-15/r2dreamer`
@ `0b1b9d8`, clean; local copy = `cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle`, head `0b1b9d8`) and
confirmed against the SAVED config (`.hydra/config.yaml`), `ladder_provenance.json`, `step_contract.json` and the
Slurm/console stamps of one {r2dreamer} run per dataset, plus a provenance sweep over all 48 {r2dreamer} runs
(16 human, 16 machine, 8 planner, 8 r2teacher): every one stamps `rep_loss=r2dreamer git=0b1b9d8 state_slice=8
image_aug=shift4 return_clamp=10.0 ladder=nested_sparse10 tip_guard=not_in_hand`. What R2-Dreamer *is* was read from
the ICLR 2026 paper PDF (`~/workspace/r2dreamer/17677_R2_Dreamer_Redundancy_Re.pdf`, §3) and from `dreamer.py`.
Two things the paper text gets wrong or leaves out are in **Discrepancies** (the "contrastive" label used in our own
tables; the demo-eviction timing; the missing bib entry; the augmentation the R2 paper's title says it does not need).
Nothing was committed; the cluster was read-only.

Paths: `LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`, `P72=$LAB/planner72_px_2026-09-15`,
`R2T=$LAB/r2teacher_px_2026-09-15`. The local scratch clone of the bundle used for line numbers below is
`/tmp/claude-1000/-home-james-workspace-genesis-pickaplace/30232c01-75cf-49d2-b2e3-edb5866cd994/scratchpad/r2px`
(= `0b1b9d8`).

---

## 1. What R2Dreamer is (from the paper, matched to the code)

Morihira et al., *R2-Dreamer: Redundancy-Reduced World Models without Decoders or Augmentation*, ICLR 2026
(openreview `Je2QqXrcQq`; code github.com/NM512/r2dreamer). It keeps DreamerV3's RSSM, reward/continue heads and
actor-critic unchanged and **replaces the image decoder + reconstruction loss** with (i) a lightweight linear
projector $k_t = f_\phi(s_t)$ from the latent state $s_t=(h_t,z_t)$ to the space of the encoder embedding $e_t$, and
(ii) a Barlow-Twins redundancy-reduction loss between $k_t$ and $e_t$:

$\mathcal{L}_{BT} = \sum_i (1 - C_{ii})^2 + \alpha \sum_{i \ne j} C_{ij}^2$, with $C$ the cross-correlation matrix of
$k_t$ and $e_t$ standardised over the $B \times T$ steps of a mini-batch, target $e_t$ detached (paper §3.2, eq. 5).
World-model loss $= \beta_{BT}\mathcal{L}_{BT} + \mathcal{L}_{pred} + \beta_{dyn}\mathcal{L}_{dyn} + \beta_{rep}\mathcal{L}_{rep}$ (eq. 4).
It is a **non-contrastive** objective (no negatives; the paper positions it against InfoNCE-style contrastive losses
and against augmentation-based decoder-free methods such as DreamerPro/TD-MPC2).

Code (`dreamer.py` 452–467, tree `0b1b9d8`): `x1 = self.prj(feat)` (linear, no bias, `networks.Projector`, from
`feat_size` 2560 = deter 2048 + 32×16 stoch to `embed_size`), `x2 = embed.detach()`, both standardised over $B T$,
`c = x1ᵀx2/(BT)`, `losses["barlow"] = Σ(diag(c)−1)² + lambd·Σ_offdiag c²` with `lambd = model.r2dreamer.lambd = 5e-4`
($\alpha$) and `loss_scales.barlow = 0.05` ($\beta_{BT}$). Under `rep_loss=r2dreamer` **no decoder module is
constructed at all** (`dreamer.py` 118–132; run stamp `[obs] decoder cnn_keys=[] mlp_keys=[] (no decoder: model.rep_loss
is not 'dreamer')`) — neither the image nor the 8-d state is reconstructed. Note one difference from the paper's
setting: in our runs the encoder embedding $e_t$ is the concatenation of the CNN image embedding (1024-d) and a
3×256 MLP embedding of the 8-d proprioception (symlog inputs), so the redundancy-reduction target is the joint
image+proprio embedding, not the image alone.

The {DfD} arm of the robot study is the SAME chassis with `rep_loss=dreamer` (decoder + `mse` image / `symlog_mse`
state reconstruction, `loss_scales.recon 1.0`); the two world-model arms differ in nothing else (same yaml, launcher,
demo sets, seeds, budget, evaluation).

## 2. Deliverable (a): paste-ready appendix subsection

```latex
\subsection{DfD and R2Dreamer (robot task)}
\label{appx:r2dreamer}

\paragraph{Implementation.}
Both world-model learners of the robot study run on the PyTorch DreamerV3 reproduction released with
R2Dreamer~\citep{morihira2026r2d}\footnote{\url{https://github.com/NM512/r2dreamer}; our fork adds a Genesis
adapter, demonstration prefill, milestone checkpoints and provenance stamps. Every run records the tree and
git revision it trained under.} and differ in exactly one configuration key, the representation loss.
\emph{DfD} uses the DreamerV3 objective: a convolutional decoder reconstructs the two camera images (MSE) and
the proprioceptive vector (symlog-MSE) from the latent state. \emph{R2Dreamer} constructs no decoder; a
linear projector $k_t=f_\phi(s_t)$ maps the latent state $s_t=(h_t,z_t)$ to the space of the encoder
embedding $e_t$ and the world model minimises the Barlow-Twins redundancy-reduction loss
\begin{equation}
\mathcal{L}_{\mathrm{BT}}=\sum_i\bigl(1-C_{ii}\bigr)^2+\alpha\sum_{i\neq j}C_{ij}^2,
\end{equation}
where $C$ is the cross-correlation matrix between $k_t$ and a detached copy of $e_t$, both standardised over
the $B\times T$ steps of a mini-batch ($\alpha=5\times10^{-4}$, loss weight $\beta_{\mathrm{BT}}=0.05$). It is a
non-contrastive objective: no negative pairs and no second augmented view. In our runs $e_t$ is the
concatenation of the image embedding and the proprioception embedding, so both are regularised. Reward,
continuation, dynamics and representation (KL) losses, imagination and actor-critic learning are those of
DreamerV3 and identical in the two arms.

\paragraph{Observations, actions and world.}
Two $64\times64$ RGB cameras (overhead and wrist, stacked to a $64\times64\times6$ tensor) and an 8-d
proprioceptive vector (six joint positions, gripper motor position, grip effort); the can pose, can orientation
and goal position present in the simulator state are \emph{not} observed. A DrQ-style random shift (replicate-pad
4\,px, random crop back to $64\times64$, one offset per training sequence) is applied to the images before the
encoder in both arms; in DfD the decoder reconstructs the shifted frame. Actions are 7-d in $[-1,1]$: six
joint-target deltas (cap $0.025$\,rad per simulator frame, target leashed to $5\times$ the cap from the measured
joint position) and a gripper command; the simulator repeats each action for 4 frames (one \emph{decision}
$=4$ frames; 1200-frame horizon $=300$ decisions). Six Genesis worlds (\texttt{gc\_kp4\_riser3\_shelf6}) are
stepped in parallel. Reward is the \texttt{nested\_sparse10} ladder: $+10$ once at \texttt{home}, terminal;
a tipped can (tilt $>60^\circ$, not in hand, sustained 4 frames) terminates with 0.

\paragraph{Demonstrations.}
Every demonstration tape is inserted once into the replay buffer before the first environment step
(``prefill''), as rows at the decision clock with the same reward, observation slice and image augmentation as
online rows. There is no re-injection, duplication, behaviour-cloning term or fixed demonstration sampling
ratio: batches are drawn uniformly from the buffer, so the demonstration share of a batch falls from 100\,\%
to $P/(P+500{,}000)$ by the end of training, where $P$ is the number of demonstration rows
(human 29{,}406; machine 37{,}488; planner 18{,}642; R2 teacher 2{,}430, including the padding rows that
equalise the six buffer columns). The buffer holds $5\times10^5$ rows in FIFO order, i.e.\ $2.0$\,M online
frames, so demonstrations remain in replay until the final $4P$ frames of the 2\,M budget (first demonstration
row overwritten at 1.88\,M/1.85\,M/1.93\,M/1.99\,M online frames for human/machine/planner/R2-teacher; all gone
at 2.0\,M). The 0.5, 1 and 1.5\,M checkpoints therefore train with every demonstration present. Demonstration
rows carry the recorded terminal flags only for tipped endings; the \texttt{home} reward is stored as a
non-terminal transition in all four sets (see \S\ref{appx:r2d-terminals}).

\paragraph{Return clamp.}
$\lambda$-return targets, both from imagined rollouts and from replayed sequences, are clipped from above at the
ladder's maximum return (10). With sparse terminal reward the continuation head's small error at terminals let
targets compound past the true maximum and preceded every observed entropy collapse in earlier runs; because
the maximum return of the task is known, clipping at it is exact rather than heuristic. The clamp is applied in
both world-model arms and was required for the DreamerV3-loss arm to learn the pick sub-task at all (its
original code path has the same trigger but a bounded actor, so the clamp alone repaired it).

\paragraph{Budget, checkpoints, evaluation.}
Each run trains for $2\times10^6$ online simulator frames after the prefill ($5\times10^5$ decisions), one
gradient update per 8 frames ($2.5\times10^5$ updates), and writes immutable checkpoints at 0.5, 1, 1.5 and
2\,M online frames. A checkpoint is evaluated on CPU in a fresh process on 30 random starts (\texttt{rnd30}) and
on the 15 demonstration starts (\texttt{hold15}, 14 of which occur in the training set), for at most 1200 frames,
with the actor either sampled or at its mean (``mode''). An episode scores \texttt{home} if the can is slid into
contact with the goal can and nested upright; tipped and timeout are the other outcomes. The per-seed statistic
of the four-dataset comparison is the mean \texttt{home} rate of the 1.5\,M and 2\,M \texttt{rnd30} sampled cells;
the registered human-vs-machine readouts use the 0.5\,M and 1\,M \texttt{rnd30} mode cells. Eight seeds per
dataset (human and machine: the eight lowest seed numbers that trained to 2\,M).

\begin{table}[t]
\centering\small
\caption{World-model hyperparameters shared by DfD and R2Dreamer on the robot task (size-12M preset of the
R2Dreamer code; values read from the saved run configurations). The only difference between the arms is the
representation loss row.}
\label{tab:r2d-hparams}
\begin{tabular}{ll}
\toprule
\textbf{World model} & \\
RSSM deterministic state & 2048 (8 blocks of 256, block-GRU) \\
RSSM stochastic state & 32 categoricals $\times$ 16 classes, unimix 0.01, learned initial state \\
Hidden / MLP units & 256, SiLU, RMSNorm \\
Image encoder & CNN, kernel 5, channels 32/48/64/64, max-pool 2, $4\times4\times64$ output \\
Proprioception encoder & MLP 3 $\times$ 256, symlog inputs \\
Representation loss & DfD: image MSE + state symlog-MSE decoder (weight 1.0); \\
 & R2Dreamer: linear projector + Barlow Twins, $\alpha=5\times10^{-4}$, weight 0.05 \\
KL losses & dynamics 1.0, representation 0.1, free bits 1.0 \\
Reward head & 1 $\times$ 256, symexp two-hot, 255 bins \\
Continuation head & 1 $\times$ 256, Bernoulli \\
\midrule
\textbf{Actor--critic} & \\
Imagination horizon & 15 steps from every posterior state of the batch \\
Discount $\gamma$ & $1-1/333=0.997$; $\lambda=0.95$ \\
Critic & 3 $\times$ 256, symexp two-hot 255 bins; slow-critic EMA 0.02 per update; replay-value loss 0.3 \\
Actor & 3 $\times$ 256, Gaussian with $\tanh$ mean, std in $[0.1,1]$; entropy coefficient $3\times10^{-5}$ \\
Return normalisation & EMA of the 5--95th percentile range (decay 0.99), floored at 1 \\
Return clamp & $\lambda$-return targets $\le 10$ (imagined and replayed) \\
\midrule
\textbf{Optimisation} & \\
Batch & 16 sequences $\times$ 64 decisions \\
Replay ratio & 512 replayed steps per environment step (1 update / 8 frames) \\
Optimiser & LaProp, lr $4\times10^{-5}$, $\beta=(0.9,0.999)$, $\epsilon=10^{-20}$, 1000-update warm-up \\
Gradient clipping & adaptive (AGC 0.3) \\
Precision & fp16 autocast with loss scaling; \texttt{torch.compile} \\
\midrule
\textbf{Environment and data} & \\
Parallel worlds & 6; action repeat 4; horizon 1200 frames (300 decisions) \\
Observations & $64\times64\times6$ (top + wrist RGB) and 8-d proprioception; random shift 4\,px \\
Reward & \texttt{nested\_sparse10}: $+10$ at \texttt{home} (terminal); tip terminal, 0 \\
Replay buffer & $5\times10^5$ rows (decisions), FIFO, CPU storage \\
Demonstrations & prefill once, uniform sampling, no re-injection / BC term \\
Budget & $2\times10^6$ online frames; checkpoints at 0.5/1/1.5/2\,M \\
Seeds & 8 per dataset \\
\bottomrule
\end{tabular}
\end{table}

\paragraph{Terminal flags in the demonstration sets.}
\label{appx:r2d-terminals}
Relabelled human and machine tapes keep the terminal flags of the original recordings (18 and 22 tipped
endings) and were re-executed without termination, so the 13 (human) and 14 (machine) tapes that reach
\texttt{home} continue for a median of 53 and 290 further decisions after the $+10$. The planner and R2-teacher
tapes end at \texttt{home}; their 67 and 68 rewarded terminal flags were cleared before training to match the
human/machine convention (2 and 4 tip terminals retained). Consequently, in every dataset the world model sees
the demonstrated $+10$ as a non-terminal transition, whereas online \texttt{home} transitions are terminal; the
value target bootstraps past the demonstrated \texttt{home} (into the post-\texttt{home} frames of human/machine
tapes, or across the tape boundary for planner/teacher tapes). The effect is the same for the human and machine
arms and fades as online data comes to dominate the buffer.
```

## 3. Deliverable (b): corrected main-text "Dreamer" paragraph (R2Dreamer part)

Replace the last two sentences of the `\paragraph{Dreamer}` in `sections/03-method.tex` (line 37) with:

```latex
We follow previous work that inserts demonstrations into Dreamer's replay buffer before training starts
\citep{staley2024agent} and refer to it as Dreamer from Demonstrations (DfD). On PinPad4 and PushT we use a
forked PyTorch implementation~\citep{dreamerv3torch2024}. On the robot task both world-model learners run on the
PyTorch DreamerV3 reproduction released with R2Dreamer~\citep{morihira2026r2d}: DfD keeps the DreamerV3 losses,
while R2Dreamer removes the image decoder and trains the latent state with a Barlow-Twins redundancy-reduction
loss against the encoder embedding (a non-contrastive objective that needs no negatives and, in the original
work, no augmentation). The two robot-task world models differ in that loss only; every other setting, including
the random-shift image augmentation we apply in both, is shared (Appendix~\ref{appx:r2dreamer}).
```

Bib entry to add to `sample-base.bib` (the key `morihira2026r2d` is cited in 03-method.tex but is NOT defined in
`sample-base.bib` or `software.bib` — BibTeX will print `[?]`):

```bibtex
@inproceedings{morihira2026r2d,
  title     = {R2-Dreamer: Redundancy-Reduced World Models without Decoders or Augmentation},
  author    = {Morihira, Naoki and Nahar, Amal and Bharadwaj, Kartik and Kato, Yasuhiro and Hayashi, Akinobu and Harada, Tatsuya},
  booktitle = {The Fourteenth International Conference on Learning Representations (ICLR)},
  year      = {2026},
  url       = {https://openreview.net/forum?id=Je2QqXrcQq}
}
```

Suggested wording for `sections/05-conclusion.tex` line 7 ("R2Dreamer, which is even more robust due its
decoder-free loss"): "R2Dreamer, whose redundancy-reduction objective replaces the pixel decoder, is
demonstration-source agnostic on this task" — the two arms differ only in the representation loss, so the arm
difference is attributable to that loss, but our runs do not separate "no decoder" from "the Barlow-Twins
objective" (the R2 paper's own ablation, *Dreamer without Decoder*, is the control for that and is theirs, not ours).

## 4. Data true to source — per dataset

Verified on the cluster with numpy over every `.npz` (no Genesis world opened) and from each set's
`repeat.json`/`manifest.json`; the launcher's in-job `[demo-gate]` and the trainer's `Demo prefill:` /
`[image] first ONLINE transition` stamps agree with these numbers in every run inspected.

| dataset (set dir) | tapes | rows (decisions) | prefill rows (+padding) | Σ reward | tapes with +10 | +10 on last row | rows after +10 (median/max) | is_terminal tapes (all on last row, all unrewarded) | image non-blank frac (min over tapes) | actions vs. source |
|---|---|---|---|---|---|---|---|---|---|---|
| human `$W/demos_state_full/dHfull_all_rns10h_img` | 74 | 29,295 | 29,406 (+111) | 130 = 10×13 | 13 | 0 | 53 / 91 | 18 (tips) | 1.0000 | `actions_sha256 77bc4875…` identical to the state set `dHfull_all_rns10h` (manifest.json); source `dHfull_all` |
| machine `$W/demos_state_full/dDPfull_first_rns10h_img` | 72 | 36,906 | 37,488 (+582) | 140 = 10×14 | 14 | 0 | 290 / 388 | 22 (tips) | 1.0000 | `actions_sha256 671614c5…` identical to `dDPfull_first_rns10h`; source `dDPfull_first` |
| planner `$P72/data/training_v3_reference_terminals/native_rns10h_img` | 72 | 17,944 | 18,642 (+698) | 680 = 10×68 | 68 | 68 | — | 2 (tips) | 1.0000 | three-way pixel / state-witness / raw action gate 72/72 (`cluster/planner72_px_2026-09-15/HANDOFF.md`, `DATA_FILES.json`, `CONTRACT.json`); 67 rewarded terminals CLEARED (`TERMINAL_REVISION.json`) |
| r2teacher `$R2T/data/training_v1/native_rns10h_img` | 72 | 2,350 | 2,430 (+80) | 680 = 10×68 | 68 | 68 | — | 4 (tips) | 1.0000 | same gate 72/72 (`cluster/r2teacher_px_2026-09-15/HANDOFF.md`); 68 rewarded terminals cleared, 4 tip terminals kept (`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`) |

Every `Demo prefill:` stamp reads `rewarded_terminals: 0`, `demo_duplicate: 1`, `reward_scale: 1.0`;
`terminal_reward_values` is `[]` for human/machine (the +10 is interior) and `[10.0]` for planner/teacher (the +10
is on the last row, unflagged). All four sets: `sim_variant gc_kp4_riser3_shelf6`, `action_repeat 4`,
`images: rendered`, `state_only: false`, `relabel.ladder nested_sparse10`, `tip_guard not_in_hand`,
`far_release false`. The pixel human/machine sets were built on pop-os (AVX2, 32 cores) — 13/14 `home` vs the
cluster-built state sets' 12/12, the known hardware-class re-execution divergence (disclosed in
`HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md` §2).

**Was R2Dreamer trained on each dataset?** Yes — 48 run directories with `rep_loss=r2dreamer`:
human s0–15 (s0–2 at the 1M budget, s3–15 at 2M), machine s0–15 (same), planner72 s0–7 (2M), r2teacher s0–7 (2M);
each 2M run has all four milestone checkpoints (`milestones/online_{500000,1000000,1500000,2000000}.pt`). Per-seed
cells of record for the 4×4 table: `paper/figures/px_phase_2026-09-14/px_results_4x4_sampled_per_seed.csv`
({r2dreamer}: human s3–10 and machine s3–10 complete; planner72 s0–6 complete, s7 pending; r2teacher s0–2 and s7
complete, s3–6 pending at the time of reading).

## 5. Eviction — which unit is right

`buffer.py`: `LazyTensorStorage(max_size=5e5, ndim=2)`; torchrl documents `ndim` as "the number of dimensions to
be accounted for when measuring the storage size … a storage of shape [3, 4] has capacity 12 if ndim=2"
(`torchrl/data/replay_buffers/storages.py:480`), and `Buffer.count()` returns `storage.shape.numel()`. One stored
row is one agent decision (the adapter repeats the action 4 simulator frames), and `trainer.py:137` converts
`step = buffer.count() * action_repeat`. So `max_size=5e5` is **500,000 decisions = 2.0 M online frames**. The
`eviction_note` string in `demo_prefill.py:337` ("first demo frame evicted after {max_size − total} online env
steps") is written in rows but labelled as env steps: multiply by 4. **CLAUDE.md's correction is right; the
statement in `paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` §7.1 ("all demo rows are evicted by 0.5M online
steps … for ≥ 87 % of training neither arm has any demonstration in replay") is wrong by the factor 4 and should
be corrected** — under the 4M state budget of that audit, demos left at ~1.9–2.0M (half-way), not at 0.5M; under
the 2M pixel budget they leave only during the last 4P frames (human 1.88M → 2.0M). The state-yaml header comment
"nothing is ever evicted" was written for the 1M budget and is true there.

## Discrepancies

1. **"decoder-free" — correct; "contrastive" — wrong.** `03-method.tex:37` ("decoder-free … redundancy-reduction
   representation loss") is accurate: under `rep_loss=r2dreamer` no decoder module exists (`dreamer.py:118–132`;
   stamp `[obs] decoder cnn_keys=[] mlp_keys=[]`). But our own result documents and tables call the arm
   "{r2dreamer contrastive loss}" (`paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md:17,28,73`,
   `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md:36`, `HRI_results/STATE_PIXEL_4x4_2026-09-16.md:26`,
   `cluster/submit_px_batch.sh:106`). Barlow Twins is non-contrastive (no negatives); the code's contrastive
   option is `rep_loss=infonce`, which was never run. Rename to "redundancy-reduction loss" wherever it reaches the
   paper.
2. **Which implementation.** `03-method.tex:37` says DfD uses "a forked PyTorch implementation
   \citep{dreamerv3torch2024}" and lists R2Dreamer as a separate variant. For the ROBOT task both world-model arms
   are the R2Dreamer authors' chassis (`$W/r2dreamer_px` @ 0b1b9d8, `model.rep_loss=dreamer|r2dreamer`); the
   NM512 dreamerv3-torch fork was used only for PinPad4/PushT (`~/workspace/dreamerv3-torch`, per the brief). The
   paragraph should say so (text in §3).
3. **Missing bib entry.** `morihira2026r2d` is cited but not defined in `sample-base.bib` / `software.bib`
   (`grep -n morihira sample-base.bib software.bib` → nothing). Entry provided in §3.
4. **Demonstration insertion.** "inserts demonstrations directly into Dreamer's initial replay buffer at the start of
   training" is true (`demo_prefill.py`, `train.py:255`) but the paper says nothing about sampling or eviction: uniform
   sampling (no demo ratio, unlike RLPD's 50 %), no re-injection (`demo_reinject_every 0`), no duplication
   (`demo_duplicate 1`), no BC term (`actor_bc_lambda 0.0`), FIFO eviction during the last 4P frames of the 2M budget
   (§5). The audit doc's "evicted by 0.5M" is a unit error (§5).
5. **Augmentation.** The R2 paper's title claims no augmentation is needed; our runs apply `image_aug=shift4` to
   BOTH world-model arms (`genesis_full_pixel.yaml:66`, stamp `[image_aug] shift4 pad=4 … frames_changed=1.000`).
   Not mentioned anywhere in the LaTeX; the appendix text above discloses it. `05-conclusion.tex:7` attributes
   R2Dreamer's robustness to "its decoder-free loss": the arms differ only in the loss, so the between-arm
   difference IS attributable to it, but "decoder-free" vs "redundancy-reduction objective" is not separated by
   our runs (see §3 wording).
6. **Return clamp sentence** (`a-research-methods.tex:40`, "We clamp imagined returns"). The clamp applies to
   BOTH the imagined λ-return targets (`dreamer.py:531`) and the replayed-sequence λ-return targets
   (`dreamer.py:581`), at the ladder's maximum return (10.0; `env.return_clamp` and `model.return_clamp` set from
   `full_env.max_return(LADDER)` by `cluster/wmfix_full.sbatch:150–167` and asserted equal by `train.py:78–85`),
   in both arms. Text in §2 says so.
7. **Terminal flags differ in kind across datasets** (§4 table): human/machine tapes run past `home` (recorded
   tips are the only terminals); planner/teacher tapes end at `home` with the rewarded terminal CLEARED
   (`training_v3_reference_terminals`, `TERMINAL_REVISION.json`; teacher `training_v1`). All four give
   `rewarded_terminals: 0`, so the continuation head is trained on demo rows to "continue" at `home` while online
   `home` rows are terminal, and the value target bootstraps past the demonstrated `home` (`_lambda_return`,
   `dreamer.py:653–663`: `is_last` cuts the λ-recursion but `live = (1−is_terminal)·γ` keeps bootstrapping from the
   next row — the post-home frames for human/machine, the next tape's first row for planner/teacher, since demo
   streams share one `episode` id per buffer column, `demo_prefill.py:281`). Same in both human/machine arms; not
   identical between the pairs. The paper's "the same reward" is still true (0/+10 everywhere, `reward_scale 1`).
8. **Brief's config guesses corrected.** Stochastic state is 32 × **16** classes (`size12M.yaml: discrete: 16`), not
   32 × 32. Actor is `bounded_normal` and entropy `3e-5` only because the launcher's `RECIPE` overrides the yaml
   (`genesis_full_pixel.yaml:70,79` say `bounded_normal_clipped` / `3e-4`; `wmfix_full.sbatch:144`; saved configs
   confirm `env.actor_dist = bounded_normal`, `env.act_entropy = 3e-05`). Milestones are 0.5/1/1.5/2M for the 2M
   runs (`R2_MILESTONES='[500000,1000000,1500000,2000000]'`), 0.5/1M for the six 1M pilot seeds (s0–2 per arm).
9. **"Trained to convergence"** (`03-method.tex:27`): the world models train to a fixed 2M-frame budget with a
   per-checkpoint series, not to convergence (rnd30 `home` still moves between 1.5M and 2M on several seeds).
10. **The local `~/workspace/r2dreamer` named in the brief is not the code of record on this machine:** it is the
    upstream repo on branch `sigreg` (HEAD `7971a00`, 2026-04-16), with no `envs/genesis.py`, no
    `genesis_full_pixel.yaml`, and commit `ada434d` (the stamp of the local (ae) runs) is not an object in it. The
    Genesis-adapted tree exists locally only as `cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle`
    (head `0b1b9d8`), and on the cluster as `$W/r2dreamer_px` and `$LAB/planner_px_2026-09-15/r2dreamer` (both at
    `0b1b9d8`, `git status` clean).
11. **Planner/teacher {r2dreamer} s0 have two Slurm logs each** (`planner72_px_r2dreamer_s0_3738546.out` with no
    prefill line, then `_3765084.out` complete; teacher `_3738578` → `_3765117`): a restart. The run dirs are single
    and complete (4 milestones); whether a `.preempted*` dir was kept was not checked (cluster read-only, not needed
    for the appendix). Disclose as a requeue if the campaign log confirms it.

## `\llm` notes inserted

- `sections/03-method.tex`, anchor: end of the `\paragraph{Dreamer}` sentence `…representation loss~\citep{morihira2026r2d}. ` — note:
  `\llm{R2Dreamer audit (HRI_results/late_night_9_18/04_r2dreamer_appendix.md): on the robot task BOTH world models run on the R2Dreamer authors' PyTorch DreamerV3 chassis (dreamerv3torch2024 = PinPad4/PushT only); the R2 loss is Barlow-Twins redundancy reduction, non-contrastive (our tables say "contrastive" -- wrong); bib key morihira2026r2d is undefined (entry in the MD); demos are prefilled once, sampled uniformly, FIFO-evicted only in the last ~0.1M of the 2M budget; both arms use shift-4 image augmentation. Replacement paragraph in the MD \S3.}`
- `appendix/a-research-methods.tex`, anchor: `We clamp imagined returns to stabilize training.` — note:
  `\llm{Paste-ready \subsection{DfD and R2Dreamer (robot task)} + hyperparameter table + terminal-flag paragraph in HRI_results/late_night_9_18/04_r2dreamer_appendix.md \S2 (all values from the saved run configs, tree 0b1b9d8). The clamp applies to imagined AND replayed lambda-return targets, at the ladder max return 10, in both arms.}`
- `sections/05-conclusion.tex`, anchor: `due its decoder-free loss` — note:
  `\llm{typo "due its"; and our runs separate only "DreamerV3 losses vs R2 redundancy-reduction loss" (the single config difference between the arms), not "decoder-free" per se -- see 04_r2dreamer_appendix.md \S3 for wording.}`

## Sources

Code of record (bundle clone at `0b1b9d8`): `configs/configs.yaml`, `configs/model/_base_.yaml`,
`configs/model/size12M.yaml`, `configs/env/genesis_full_pixel.yaml`, `configs/env/genesis_full_state.yaml`,
`dreamer.py` (40–70, 100–145, 418–600, 632–720), `networks.py` (99–234 encoders, 339–406 heads/ReturnEMA),
`rssm.py` (1–120, 222–230), `distributions.py` (217–252), `buffer.py`, `demo_prefill.py`, `trainer.py` (1–60,
127–300), `train.py` (22–125, 250–290), `envs/genesis.py` (61–140, 225–300), `eval_genesis.py` (argparse, 396),
`longrun_milestones.py`. torchrl semantics: `~/workspace/r2dreamer/.venv/lib/python3.10/site-packages/torchrl/data/replay_buffers/storages.py:480`.
Paper: `~/workspace/r2dreamer/17677_R2_Dreamer_Redundancy_Re.pdf` pp. 1–8; `~/workspace/r2dreamer/README.md`.
Launchers: `cluster/wmfix_full.sbatch`, `cluster/submit_px_batch.sh`, `cluster/r2_milestones.sh`,
`cluster/planner72_px_2026-09-15/{launch/world_model.sbatch,diffs/world_model.sbatch.diff,SUBMIT_COMMANDS.txt,HANDOFF.md}`,
`cluster/r2teacher_px_2026-09-15/{launch/world_model.sbatch,SUBMIT_COMMANDS.txt,HANDOFF.md}` (launcher diff vs the
repo copy: QOS/paths/gate only). Docs: `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`,
`HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`, `HRI_results/DEMO_SETS_2026-09-11.md`,
`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`, `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`,
`paper/AUDIT_R2D_NESTED_SPARSE_HvM_2026-09-13.md` (108–130, 160–176), `paper/LADDER_IMPL_NOTES_2026-09-10.md` (348–353),
`paper/PX_SPARSE10_RESULTS_SUMMARY_2026-09-16.md`, `paper/figures/px_phase_2026-09-14/px_results_4x4_sampled_per_seed.csv`.
LaTeX: `sections/03-method.tex:25–50`, `sections/05-conclusion.tex:7`, `appendix/a-research-methods.tex:36–48`,
`sample-base.bib` (grep for morihira / staley2024agent / dreamerv3torch2024), `preamble.tex:42`.
Cluster (read-only, `ssh pax`): `git -C $W/r2dreamer_px log -1; status`; same for `$LAB/planner_px_2026-09-15/r2dreamer`;
run dirs `$W/runs/full_r2d_state_{dHfull_all,dDPfull_first}_rns10h_img_r2dreamer_s3`,
`$P72/runs/full_r2d_state_native_rns10h_img_r2dreamer_s0`, `$R2T/runs/full_r2d_state_native_rns10h_img_r2dreamer_s0`
(`ladder_provenance.json`, `step_contract.json`, `.hydra/config.yaml`, `milestones/`, `console.log`); Slurm logs
`$W/slurm/px_r2dreamer_{dH,dM}_s3_36925{18,19}.out`, `$P72/slurm/planner72_px_r2dreamer_s0_{3738546,3765084}.out`,
`$R2T/slurm/r2teacher_px_r2dreamer_s0_{3738578,3765117}.out`; provenance sweep over all 48 `*_r2dreamer_s*` run dirs;
demo sets' `repeat.json`/`manifest.json` and a numpy census of every `.npz` (reward position, terminal flags, image
blankness); `$W/ln_milestone_cells/full_r2d_state_dHfull_all_rns10h_img_r2dreamer_s3/online_{1500000,2000000}/*/metrics.json`.
