# Item 05 — DreamerV3 / DfD: paper text vs code, appendix hyperparameters, data true to source

**Status: DONE (partial on the simulated tasks).** Lane E (Fable), 2026-09-17/18. The robot-task (restocking, pixel
4×4) DfD arm is fully audited against the cluster's saved run configs: all 32 DfD runs of record (human s8–15,
machine s8–15, planner72 s0–7, r2teacher s0–7) stamp `rep_loss=dreamer`, r2dreamer tree `0b1b9d8`, budget 2M online
sim steps, four milestones + `latest.pt`, and identical overrides except seed/demo set (§A.4). The four demonstration
sets were checked for tape count, action-stream identity, real images and terminal-flag handling (§A.5). The
simulated PinPad4/PushT DfD runs (2024) are reconstructed from the local `fastrl-pusht` launcher and demo directories;
several paper claims about them (100 demos, 200k steps, sampled actions) do not match what the launch scripts and
directories show, and the exact trial sets behind the figures could not be identified locally — those are marked
UNKNOWN rather than guessed (§B). No `.tex` was edited except `\llm{}` notes; no cluster write.

**Headline discrepancy.** The paper's Dreamer paragraph describes ONE implementation ("a forked PyTorch
implementation", NM512/dreamerv3-torch) for every DfD result. In fact there are TWO code bases: (A) the robot task's
"DfD" is the **r2dreamer chassis run with `model.rep_loss=dreamer`** (reconstruction decoder + KL, DreamerV3 losses;
same optimizer, RSSM, actor, critic, replay and clamp as the R2Dreamer arm); (B) the PinPad4/PushT DfD/IBC/VMAIL runs
used the **fastrl fork of NM512/dreamerv3-torch** (`~/workspace/fastrl-pusht`, 2024). The r2dreamer README itself
calls its DreamerV3 path "an efficient PyTorch DreamerV3 reproduction" of the same NM512 codebase, so the lineage is
shared, but the two arms are different code with different sizes, optimizers, precision and demo handling.

---

## 1. Deliverable (a): paste-ready appendix subsection

```latex
\subsection{DreamerV3 from Demonstrations (DfD)}\label{appx:dfd}

DfD is DreamerV3~\citep{hafner2023mastering} with the demonstration set placed in the replay buffer before online
training begins~\citep{staley2024agent}. The world model, actor and critic are unchanged; the demonstrations are
ordinary replay rows (image, proprioception, action, reward, continuation flags) that the world model reconstructs
and that imagination starts from, and nothing forces the actor to imitate them. Two implementations were used and
must not be conflated.

\paragraph{Restocking task (Sec.~\ref{sec:eval:results}).}
DfD is the DreamerV3 loss configuration of the R2Dreamer code base~\citep{morihira2026r2d}
(\texttt{model.rep\_loss=dreamer}: pixel and proprioception reconstruction plus the KL dynamics/representation
losses, no redundancy-reduction term), so that DfD and R2Dreamer differ in the representation loss and in nothing
else. Every setting below is read from the saved Hydra configuration of the runs of record; the four demonstration
datasets were trained with byte-identical overrides except for the demonstration directory and the seed.
Table~\ref{tab:dfd-robot} lists the hyperparameters.

\emph{Demonstration insertion.} Each tape is loaded at the decision clock (one row per agent decision =
4 simulator steps, \texttt{action\_repeat} 4), the 17-d recorded state is truncated to the 8-d proprioception the
policy sees, actions are shifted to the trainer's forward-in-row convention, and the tapes are packed into the
six parallel replay columns (greedy longest-first, padded by looping whole tapes). The replay buffer holds
$5\times10^5$ rows and is a FIFO ring; a row is one decision, so the ring spans $2.0\times10^6$ online simulator
steps. The first demonstration row is therefore evicted after $(5\times10^5 - P)\times4$ online steps, where $P$ is
the number of prefilled rows (human $29{,}406$; DP $37{,}488$; planner $18{,}642$; R2 $2{,}430$), i.e.\ at
1.88M / 1.85M / 1.93M / 1.99M online steps, and the last demonstration row leaves at exactly 2.0M, the end of the
budget. Demonstrations are thus present in replay for essentially the whole run. No demonstration re-injection,
duplication, actor behaviour-cloning term or world-model pre-training on demonstrations was used
(\texttt{demo\_reinject\_every 0}, \texttt{demo\_duplicate 1}, \texttt{actor\_bc\_lambda 0}, \texttt{pretrain 0}).

\emph{Reward and terminals.} All four sets pay the same ladder (\texttt{nested\_sparse10}: $+10$ once when the can
is slid home, $0$ otherwise; tip-over terminates with $0$). In the human and DP sets the $+10$ row is followed by
the rest of the recorded tape and carries no terminal flag; in the planner and R2 sets the $+10$ is the last row
and its terminal flag was cleared to match that convention (\texttt{rewarded\_terminals} $=0$ in every prefill
stamp). The world model therefore sees \emph{home} as a rewarded, non-terminal event in every dataset, whereas the
online environment terminates at \emph{home}; this is identical across datasets and is disclosed rather than
corrected.

\emph{Return clamp.} Imagined and replayed $\lambda$-return targets are clamped at the ladder's maximum return
($10$) before they reach the critic and the actor's advantage. This clamp was the single change that made the
separate NM512-fork port learn the sparse restocking rewards at all (it learned the reach proxy $15/15$ on $4/4$
seeds only with the clamp and never without), and it is applied identically to DfD and R2Dreamer here.

\emph{Budget and evaluation.} $2\times10^6$ online simulator steps after prefill, one gradient update every eight
simulator steps (two decisions) on batches of $16\times64$ rows, six parallel environments, immutable checkpoints
at 0.5, 1, 1.5 and 2M. Each checkpoint is evaluated in a fresh process on CPU (64-core nodes) with the world of
record, 1200-step horizon: 30 random starts (\texttt{rnd30}) and the 15 demonstration starts (\texttt{hold15}),
each with sampled and with mode actions. The per-seed statistic in Table~\ref{tab:phase-success-pixel-sparse10-sampled-eval}
is the mean of the 1.5M and 2M \texttt{rnd30} sampled-action cells; the registered human-vs-DP readout used the
0.5M and 1M mode cells with 16 seeds per arm. Design seeds: human and DP s8--15, planner and R2 s0--7.

\begin{table}[htbp]\centering\small\setlength{\tabcolsep}{4pt}
\begin{tabular}{ll}\toprule
\multicolumn{2}{l}{\textbf{DfD, restocking task} (R2Dreamer code base, \texttt{rep\_loss=dreamer}, tree \texttt{0b1b9d8})}\\\midrule
Observation & two $64{\times}64$ RGB cameras (top, wrist) stacked to $64{\times}64{\times}6$; 8-d proprioception (6 joints, gripper motor, grip effort); no object or goal pose \\
Image augmentation & DrQ random shift: replicate-pad 4\,px, random $64{\times}64$ crop, one offset per sequence, applied before the encoder; the decoder reconstructs the shifted frame \\
Action & 6 delta joint targets (cap 0.025\,rad per simulator step, leash $5\times$cap) + gripper; \texttt{action\_repeat} 4; episode 1200 simulator steps (300 decisions) \\
Reward & \texttt{nested\_sparse10}: $+10$ terminal on \emph{home}; tip (tilt $>60^\circ$, not in hand, 4 frames) terminal with 0; \texttt{reward\_scale} 1 \\
RSSM & deterministic 2048 (8 blocks), stochastic $32\times16$ discrete, hidden 256, unimix 0.01, learned initial state, 2 image / 1 obs / 1 dyn layers \\
Encoder / decoder & CNN depth 16, kernel 5, min-res 4, mults $[2,3,4,4]$; MLP 3 layers $\times$ 256 units; symlog inputs; image MSE, state symlog-MSE \\
Heads & reward and continue: 1 layer $\times$ 256, symexp two-hot (255 bins) / Bernoulli; critic 3 $\times$ 256, symexp two-hot (255 bins), slow target EMA 0.02 every update \\
Actor & 3 $\times$ 256, \texttt{bounded\_normal} (tanh mean, std in $[0.1, 1.0]$), output scale 0.01; entropy coefficient $3\times10^{-5}$; advantage normalised by the 5--95\,\% return percentile EMA \\
Loss scales & recon 1.0, reward 1.0, continue 1.0, dynamics KL 1.0, representation KL 0.1 (free bits 1.0), policy 1.0, value 1.0, replay value 0.3 \\
Imagination & horizon 15, $\gamma = 1 - 1/333 \approx 0.997$, $\lambda = 0.95$, return clamp 10 \\
Optimiser & LaProp, lr $4\times10^{-5}$, $\beta=(0.9, 0.999)$, $\epsilon=10^{-20}$, 1000 warm-up steps, adaptive gradient clipping 0.3; fp16 autocast, \texttt{torch.compile} \\
Replay / updates & buffer $5\times10^5$ rows (CPU), FIFO; batch $16\times64$; \texttt{train\_ratio} 512 = one update per 8 simulator steps; 6 environments; no pre-training \\
Demonstrations & prefilled once at start; human 74 tapes / 29\,295 decisions; DP 72 / 36\,906; planner 72 / 17\,944; R2 72 / 2\,350; no re-injection, duplication or BC term \\
Budget & $2\times10^6$ online simulator steps; checkpoints at 0.5/1/1.5/2M; seeds human, DP s8--15, planner, R2 s0--7 \\
Evaluation & fresh process, CPU, \texttt{rnd30} and \texttt{hold15}, sampled and mode actions, 1200 steps; statistic = mean of the 1.5M and 2M \texttt{rnd30} sampled cells \\
\bottomrule\end{tabular}
\caption{DfD hyperparameters on the restocking task. Values are from the saved run configuration
(\texttt{.hydra/config.yaml}) and the launch overrides of the runs of record; they are identical for the four
demonstration sources.}\label{tab:dfd-robot}
\end{table}

\paragraph{Simulated tasks (PinPad4, PushT).}
These runs used our fork of the NM512 PyTorch DreamerV3~\citep{dreamerv3torch2024}, trained through the same
launcher as the IBC and VMAIL baselines. Demonstrations are loaded into the replay buffer at start-up as complete
episodes and are never evicted (dataset size $10^6$ steps). Under the sparse-reward setting used for PushT the
loader relabels every demonstration to $0$ at all steps and $+1$ on the final step of a successful episode;
PinPad4 keeps its recorded reward ($+10$ per completed sequence plus a small shaping term of $0.01\times$ the
sequence position for each correct pad). Machine demonstrations were collected from a trained DfD policy at the end
of its run (100 episodes per run). Table~\ref{tab:dfd-sim} lists the settings; entries marked $\dagger$ were
recovered from the launch scripts and demonstration directories rather than from a saved run configuration and
should be checked against the original logs (see the notes in \texttt{HRI\_results/late\_night\_9\_18/05\_dreamerv3\_dfd\_appendix.md}).

\begin{table}[htbp]\centering\small\setlength{\tabcolsep}{4pt}
\begin{tabular}{lll}\toprule
\textbf{DfD, simulated tasks} (NM512/dreamerv3-torch fork) & PinPad4 & PushT \\\midrule
Observation & $64{\times}64$ RGB & $64{\times}64$ RGB + 2-d agent position \\
Action & 5 discrete (no-op, 4 moves), one-hot actor, REINFORCE gradient & 2-d continuous, Gaussian actor (std learned in $[0.1,1]$), dynamics gradient \\
Episode length & 250 steps$^\dagger$ & 300 steps \\
Reward & $+10$ per completed pad sequence, $+0.01k$ shaping for the $k$-th correct pad & sparse: $+1$ at success (coverage $\ge 0.93$), else 0 \\
RSSM & deterministic 512, stochastic $32\times32$ discrete, hidden 512 & same \\
Encoder / decoder & CNN depth 32, kernel 4, min-res 4; MLP 2 $\times$ 512 & same \\
Heads / actor / critic & 2 layers $\times$ 512 units & same \\
Learning rates & model $10^{-4}$; actor and critic $3\times10^{-4}$ & model $10^{-4}$; actor $3\times10^{-5}$ \\
Loss scales & dynamics KL 0.5, representation KL 0.1, free bits 1.0 & same \\
Imagination & horizon 16, $\gamma\,0.997$ (imagination discount 0.99), $\lambda\,0.95$, entropy $3\times10^{-4}$, return-percentile EMA normalisation, slow critic EMA 0.02 & same \\
Replay / updates & batch $16\times64$; \texttt{train\_ratio} 256; 2500 random prefill steps; 100 pre-training updates & batch $16\times64$; \texttt{train\_ratio} 512 \\
Demonstrations & 15 human episodes per trial (13 in trial 0); 15 machine episodes per trial$^\dagger$ & 70--78 human episodes per trial; 100 machine episodes per trial$^\dagger$ \\
Budget & 25\,000 environment steps per trial$^\dagger$ & 60\,000 environment steps in the launch script; 200\,000 stated in the paper$^\dagger$ \\
Seeds & 8 trials (\texttt{imi} 0--7), each with its own demonstration set & 4 trials (\texttt{imi} 11--14)$^\dagger$ \\
\bottomrule\end{tabular}
\caption{DfD hyperparameters on the simulated tasks, from the fork's configuration file and launch scripts.
$\dagger$: recovered from scripts/directories, not from a saved run record; see the audit note.}\label{tab:dfd-sim}
\end{table}
```

## 2. Deliverable (b): corrected "Dreamer" main-text paragraph (`sections/03-method.tex`)

```latex
\paragraph{Dreamer}

Dreamer is a model-based RL algorithm~\citep{hafner2023mastering}. It learns a recurrent world model that predicts
future latent states from past observations and actions, and an actor-critic trains inside imagined rollouts of
that model. Following~\citet{staley2024agent} we place the demonstrations in Dreamer's replay buffer before online
training begins and refer to the result as DreamerV3 from Demonstrations (DfD); the demonstrations are ordinary
replay data, with no imitation loss. On the simulated tasks DfD is our fork of a PyTorch DreamerV3
implementation~\citep{dreamerv3torch2024}. On the restocking task DfD and R2Dreamer share one code
base~\citep{morihira2026r2d}: DfD is that code base with the DreamerV3 losses (pixel reconstruction and the KL
dynamics losses), and R2Dreamer is the same code base with its decoder-free redundancy-reduction representation
loss, so the two differ in the representation loss and in nothing else (hyperparameters, replay, return clamp and
evaluation are shared; Appendix~\ref{appx:dfd}). On the restocking task both arms clamp imagined returns at the
task's maximum return, without which neither world model learned the sparse reward.
```

Also for `sections/02-related-work.tex` §World-model learning: no change needed for correctness; if the sentence "Dreamer-family methods ... demonstrations can bootstrap their learning without an explicit actor imitation loss" is kept, it is consistent with the code (no BC term on either code base; `actor_bc_lambda 0` on the robot task).

## 3. Deliverable (c): audit of the paper text against the code — see `## Discrepancies` below.

---

## A. Robot task (restocking, pixel 4×4) — what was verified

### A.1 Which code is "DfD" here
- Tree of record: `$W/r2dreamer_px` @ `0b1b9d8` (every run's `ladder_provenance.json: r2dreamer_git 0b1b9d8`; in the repo as
  `cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle`, extracted read-only to the scratchpad for this audit).
  The local `~/workspace/r2dreamer` on this box is a DIFFERENT checkout (sigreg branch, no `genesis_full_pixel.yaml`) —
  do not quote it for the paper.
- `dreamer.py:31` `self.rep_loss = str(config.rep_loss)`; branches `"dreamer"` (decoder built, reconstruction loss `recon`),
  `"r2dreamer"` (Barlow/redundancy term, no decoder), `"infonce"`, `"sigreg"`, `"dreamerpro"`. Common to all: `losses["rep"]`,
  `losses["dyn"]` (KL, `rssm.kl_loss`, free bits 1.0), reward/cont heads, imagination actor-critic (`dreamer.py:513-560`),
  replay value learning (`repval`, scale 0.3), return clamp (`:531-532`, `:581-582`), ReturnEMA advantage normalisation.
- r2dreamer README line 3: "an efficient PyTorch DreamerV3 reproduction ... than a widely used codebase [dreamerv3-torch]".
- Launch line (every DfD run, `.hydra/overrides.yaml`): `env=genesis_full_pixel seed=S env.steps=2000000 env.demo_dir=<set>
  env.ladder=nested_sparse10 env.far_release=false env.tip_guard=not_in_hand env.return_clamp=10.0 model.return_clamp=10.0
  buffer.max_size=5e5 logdir=… env.actor_dist=bounded_normal env.act_entropy=3e-5 model.rep_loss=dreamer model.image_aug=shift4
  env.state_slice=8`. The R2Dreamer arm differs only in `model.rep_loss=r2dreamer` (`PROVENANCE_… §3a`).

### A.2 Hyperparameters (source: `$W/runs/full_r2d_state_dHfull_all_rns10h_img_dreamer_s8/.hydra/config.yaml`)
Model: `deter 2048, hidden 256, discrete 16, stoch 32, blocks 8, depth 16, units 256, act SiLU, norm true, unimix 0.01,
initial learned`; encoder CNN `kernel 5, minres 4, mults [2,3,4,4]`, MLP `layers 3, units 256, symlog_inputs true`; decoder
CNN mirror (`cnn_dist mse`), MLP `symlog_mse`; reward `layers 1, symexp_twohot 255, outscale 0`; cont `layers 1, binary`;
critic `layers 3, symexp_twohot 255`; actor `layers 3, bounded_normal min_std 0.1 max_std 1.0, outscale 0.01`.
Training: `act_entropy 3e-5, return_clamp 10, kl_free 1.0, imag_horizon 15, horizon 333 (γ = 1 − 1/333), lamb 0.95, lr 4e-5,
agc 0.3, pmin 1e-3, eps 1e-20, betas 0.9/0.999, warmup 1000, slow_target_update 1, slow_target_fraction 0.02, loss_scales
recon 1 rew 1 con 1 dyn 1 rep 0.1 policy 1 value 1 repval 0.3, compile true`; `batch_size 16, batch_length 64,
buffer.max_size 5e5, storage cpu, trainer.pretrain 0, save_every 1e5, train_ratio 512, env_num 6, action_repeat 4,
time_limit 1200, delta_cap 0.025, delta_leash_mult 5, state_slice 8, image_aug shift4, demo_reinject_every 0, demo_duplicate 1,
actor_bc_lambda 0, reward_scale 1`.
- Optimiser = LaProp (`dreamer.py:184`, `optim/laprop.py`), AGC (`optim/agc.py`); fp16 autocast (`dreamer.py:388`);
  `torch.set_float32_matmul_precision("high")` (`train.py:19`).
- Update cadence: `trainer.py:33` `Every(batch_steps / train_ratio * action_repeat)` = 1024/512×4 = one update per 8 sim steps.
- Actor distribution: the pixel env YAML defaults to `bounded_normal_clipped`, but the launcher's `RECIPE`
  (`cluster/wmfix_full.sbatch:144`) overrides `env.actor_dist=bounded_normal env.act_entropy=3e-5` — the saved config confirms
  `bounded_normal` (tanh mean, `distributions.py:217-222`).
- Return clamp requirement is enforced at start: `train.py::_write_ladder_provenance` refuses a run whose `env.return_clamp`
  / `model.return_clamp` ≠ `full_env.max_return(ladder, scope)` (= 10 for `nested_sparse10`).

### A.3 Demo insertion, eviction, terminal handling (source: `demo_prefill.py` in the tree of record + each run's `console.log`)
- `train.py:128 prefill_from_demos` → `demo_prefill.add_demo_set`: loads every `*.npz` in `env.demo_dir` (native stride-4 dirs
  with `repeat.json`, `action_repeat 4` asserted, `terminal_reward 1.0` asserted), truncates `state` 17→8 (`state_slice`),
  shifts actions forward one row, packs into `env_num=6` streams (greedy longest-first, padded by looping whole tapes),
  writes `is_demo=True` rows (unused: `actor_bc_lambda 0`), zero latents (self-healed after sampling).
- Prefill stamps (`Demo prefill: {...}` in `console.log`):
  | set | episodes | frames_raw | rows added | padding | terminal_reward_values | rewarded_terminals | trainer starts at counter |
  |---|---|---|---|---|---|---|---|
  | human `dHfull_all_rns10h_img` s8 | 74 | 29,295 | 29,406 | 111 | [] | 0 | 117,624 |
  | machine `dDPfull_first_rns10h_img` s8 | 72 | 36,906 | 37,488 | 582 | [] | 0 | 149,952 |
  | planner72 `training_v3_reference_terminals/native_rns10h_img` s0 | 72 | 17,944 | 18,642 | 698 | [10.0] | 0 | 74,568 |
  | r2teacher `training_v1/native_rns10h_img` s0 | 72 | 2,350 | 2,430 | 80 | [10.0] | 0 | 9,720 |
- Eviction: `buffer.py` `LazyTensorStorage(max_size=5e5, ndim=2)` = FIFO ring of ROWS; `trainer.py:137` `step = count()*action_repeat`
  → one row = one decision = 4 sim frames. The prefill's `eviction_note` ("first demo frame evicted after N online env steps")
  mislabels rows as env steps (`paper/AUDIT_TRAIL_R2D_NESTED_SPARSE_HvM_2026-09-13.md` §B2). Correct: first demo row leaves at
  (5e5 − rows)×4 = human 1,882,376 / machine 1,850,048 / planner 1,925,432 / r2teacher 1,990,280 online sim steps; all demo rows
  gone at 2,000,000 = the budget. So demos are present for the whole 2M run.
- Terminal flags: human/machine sets keep `is_terminal` as recorded (`paper/LADDER_IMPL_NOTES_2026-09-10.md` §4) and the tape
  continues after the +10 (`terminal_reward_values []` = the +10 is never on the last row). Planner/r2teacher exports put the +10
  on the last row but CLEARED the home terminal flag ("versioned nonterminal home bootstrap convention", `repeat.json.relabel`;
  67 / 68 flags cleared per `TERMINAL_REVISION.json`, tip terminals kept 2 / 4). Consequence identical in all four: `cont=1` on
  the rewarded row, the critic can bootstrap past home; the online env terminates at home. Disclosed in the appendix text.

### A.4 Trained with each dataset — verified on the cluster (read-only)
`ssh pax` loop over the 32 run dirs: every one has `ladder_provenance.json` with `rep_loss dreamer`, `r2dreamer_git 0b1b9d8`,
`budget_steps 2000000`, `env_config genesis_full_pixel`, `image_aug shift4`, `state_slice 8`, `encoder_cnn_keys image`,
`return_clamp 10.0`, `tip_guard not_in_hand`; `milestones/` holds 4 `.pt` and `latest.pt` exists:
- human `$W/runs/full_r2d_state_dHfull_all_rns10h_img_dreamer_s{8..15}` (gp tree `8492a2e`), machine
  `…dDPfull_first_rns10h_img_dreamer_s{8..15}` (`8492a2e`);
- planner72 `$LAB/planner72_px_2026-09-15/runs/full_r2d_state_native_rns10h_img_dreamer_s{0..7}` (gp `e8287af0-dirty`, repo
  `$LAB/planner_px_2026-09-15/gp`); r2teacher `$LAB/r2teacher_px_2026-09-15/runs/…dreamer_s{0..7}` (same trees).
- Also on the cluster: human/machine DfD s4–7 (the 1M (af) pilots) — not design seeds for the 4×4 table; local (ae) s0–3 are not on
  the cluster.
- Console stamps per run confirm real pixels: `[image] first ONLINE transition … nonzero_frac=1.0000`, `[image_aug] shift4 …
  [REAL frames]`, `[obs] encoder cnn_keys=['image'] mlp_keys=['state'] shapes={'image': (64, 64, 6), 'state': (8,)}`.

### A.5 Data true to source
| set | tapes | decisions | Σ reward (home) | actions | images | source |
|---|---|---|---|---|---|---|
| human `dHfull_all_rns10h_img` | 74 | 29,221 | 130 (13) | sha256 `77bc4875…` = the state set's stream (`manifest.json`; DEMO_SETS: identical on all six relabelled sets) | rendered 74/74, `image_nonzero_frac_min 1.0` | `dHfull_all` (every attempt of the 74 successful trials; the 16 fail-labelled trials excluded — DEMO_SETS disclosure), src sha `3b478c9a…` |
| machine `dDPfull_first_rns10h_img` | 72 | 36,834 | 140 (14) | sha256 `671614c5…` | rendered 72/72 | `dDPfull_first` (state DP teacher trained on pruned human, FIRST attempt per start; `one_per_ic_first true`, 123 duplicate-IC tapes skipped), src sha `db95084f…` |
| planner72 `training_v3_reference_terminals/native_rns10h_img` | 72 | 17,872 | 680 (68) | native planner actions; `repeat.json` builder `matched72_completed_2026-09-15/finish.py`, world commit `91fba145…`; relabel = `10 × native nested_sparse reward` | rendered | motion planner on the DP/human-matched 67 geometries (`HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`); OLD 70-tape cohort excluded |
| r2teacher `training_v1/native_rns10h_img` | 72 | 2,278 | 680 (68) | three-way action gate 72/72 (`cluster/r2teacher_px_2026-09-15/HANDOFF.md`) | rendered | first attempts of r2dreamer seed 15 (human-trained, 1M, MODE actions), teacher n=1 (`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`) |
The two `_rns10h_img` sets were built on pop-os (AVX2); the cluster-built state sets pay 12/12 home vs 13/14 here — known
hardware-class divergence, disclosed in PROVENANCE §2, not corrected.

### A.6 Result numbers the paper quotes (for the conclusion check)
`paper/figures/px_phase_2026-09-14/px_results_4x4_sampled_per_seed.csv` (mean of the 1.5M+2M `rnd30_sample` cells):
{DreamerV3 losses} `home` human 0.306 (8) / machine 0.348 (8) / planner72 0.043 (7) / r2teacher 0.425 (2); `picked`
0.652 / 0.650 / 0.388 / 0.608. Matches the paper's table row (0.31/0.35/0.04/0.42; 0.65/0.65/0.39/0.61). Pairwise
(`px_results_4x4_pairwise.csv`, Anderson–Darling permutation): human v machine p 0.69; human v planner p 0.0019 (Holm 0.037);
machine v planner p 0.0008 (Holm 0.016); human v r2teacher p 0.42 with n = 8 v **2**; machine v r2teacher p 0.82 (n 8 v 2).

---

## B. Simulated tasks (PinPad4, PushT) — what could be reconstructed locally

- **Code:** `~/workspace/fastrl-pusht` (git, last commit `5c20ad1` 2024-12-10 "unify setup between pinpad and pusht"). Runs are
  launched by `train.sh`, which converts `corl_superteleop.ipynb` to a script and runs it with flags; the fork's own
  `configs.yaml` is used (not `~/workspace/dreamerv3-torch/dreamerv3-torch/configs.yaml`, whose 2026 `pinpad_four`/`pusht`
  blocks belong to later work). The Dreamer core is the NM512 fork (`dreamerv3torch2024`, commit `a27711a` in the bib).
- **Flags** (`corl_superteleop_21:17:11_0913.py:93-110`): `-c` config, `-ts` train steps, `-o` use offline (demo) data,
  `-imi N` demo-folder trial → `HD_<task>_N` (human) or, with `-ai`, `AD_<task>_N` (machine) under `~/workspace/fastrl/logs`,
  `-fs` force sparse, `-pret` pretrain updates (default 100), `-alr` actor lr (default 3e-4), `-bc/-ibc/-vmail` baselines.
- **Demo loading** (`fastrl_utils.py:110-131, 165-175`): all npz episodes of the chosen dir become `train_eps` (no success filter,
  no cap other than `dataset_size` 1e6); `force_sparse` relabels reward to 0 everywhere and 1.0 on the last row iff `is_last[-1]`
  (checked: in these tapes `is_last[-1]` is True only on successes; timeouts carry `is_terminal` instead — e.g. HD_pusht_12: 24/70
  `is_last`, 47 timeouts with `is_last` False). No eviction (`dataset_size` 1e6 ≫ run length).
- **Machine demonstrations:** collected at the end of a run by `tools.simulate(... final_eps ...)` with
  `force_sample=config.final_data_collection_bsample` (**False** in `configs.yaml:33`) → `AnotherDirector.py:623`
  `sample = training or human or force_sample` → **mode (deterministic) actions**; a second, sampled collection goes to
  `final_eps_stoch`. The `AD_pusht_11–13` dirs contain `final_eps`, `AD_pusht_14` `final_episodes`, `ai_imis/imi*` `final_eps`.
- **Config of record** (`fastrl-pusht/configs.yaml`): defaults `dyn_deter 512, dyn_hidden 512, dyn_stoch 32, dyn_discrete 32,
  units 512, reward/cont/value/actor layers 2, cnn_depth 32, kernel 5 (pinpad/pusht blocks: 4), minres 4, mlp 2×512,
  LayerNorm, dyn_scale 0.5, kl_free 1.0, batch 16×64, train_ratio 512, prefill 2500, pretrain 100, model_lr 1e-4,
  grad_clip 100, precision 32, use_amp False, time_limit 1000`. `pinpad_four`: `steps 2e5, train_ratio 256, imag_gradient
  reinforce, onehot actor lr 3e-4, envs 1, action_repeat 1, train_every 4, time_limit 250, prefill 2500`. `pusht`: `train_ratio
  512, eval_every 1e4 ("For RSS we have a budget of 60k environment steps"), eval_episode_num 20, step 1e5 (sic), envs 1,
  time_limit 300, size 64, worker_actor normal lr 3e-5 std learned [0.1,1.0], encoder/decoder mlp_keys agent_pos,
  final_data_collection_eps 100`. Defaults also set `discount 0.997, imag_discount 0.99, discount_lambda 0.95, imag_horizon 16,
  actor_entropy 3e-4, rep_scale 0.1, reward_EMA True, slow_target_update 1, slow_target_fraction 0.02` (`configs.yaml:60-132`);
  UNKNOWN whether the notebook overrides any of them at run time.
- **Environments:** `envs/pinpad.py` (dense shaping `0.01·k` per correct pad + 10 per completed sequence, reset countdown 4);
  `envs/pusht.py` (gym_pusht `PushT-v0`, `force_sparse` → 1.0 at `is_success` else 0; success = coverage ≥ 0.93 per the
  fork's wrapper).
- **Launch scripts:** `pinpad_train.sh` / `pinpad_train_ai.sh`: `-c pinpad_four -ts 25000 -o [-ai] -imi 0..7` (8 trials each,
  P100). `HDpusht_job.sh` / `AIDpusht_job.sh` (active line): `-c pusht -ts 60000 -o -imi 10 -fs [-ai]`; a commented loop used
  `-ts 200000 -o -imi $i -pret 500 -alr 3e-5` for `i` in 7..8. `results.ipynb` references `imi10–14` and `imi4–7` for PushT.
- **Demo sets on disk** (`~/workspace/fastrl/logs`): PinPad human `HD_pinpad_four_0..11` = 13, 15×11 npz (251 steps each,
  reward Σ median 20 = two sequences); machine `ai_imis/imi0..11` = 15 npz each (Σ median 50). PushT human `HD_pusht_10` 50
  (info.txt: "sparse reward, action_repeat=2"), `HD_pusht_11` 78 (78 successes), `_12` 70 (24), `_13` 70 (70), `_14` 72 (72),
  `_16/_22/_23` 100 each (Nov 2024, 94×94 px, 95/81/87 successes), `HD_pusht_4` 200 (137); machine `AD_pusht_10` 50 (48),
  `_11` 100 (93), `_12` 100 (94), `_13` 100 (32), `_14` 100 (86), `AD_pusht_4` 250. (Success = `is_last[-1]`, the sparse
  relabel's criterion.) No `AD_pusht_16/22/23` exist, so the 100-tape human sets have no machine counterpart under the `imi`
  convention.
- **UNKNOWN:** which `imi` trials produced `pusht_scalars_psuccess.png` / `pinpad_scalars_train_return.png` and Table
  `tbl:pusht` (wandb runs, not local); the exact notebook revision at collection time; whether the paper's "200k steps" refers to
  the commented `-ts 200000` runs or to something else; how the 400-episode PushT evaluation sets in `tbl:pusht` were built
  (100 `final_eps` × 4 trials is consistent with `final_data_collection_eps 100` and n=4).

---

## Discrepancies (paper vs code/notes)

1. **`sections/03-method.tex:37` "We use a forked PyTorch implementation~\citep{dreamerv3torch2024}"** — true only for
   PinPad4/PushT. The restocking DfD is the r2dreamer chassis with `model.rep_loss=dreamer` (`.hydra/overrides.yaml` of all 32
   runs; `dreamer.py:476`). Fix: paragraph in §2 above. The same paragraph's "R2Dreamer, a decoder-free DreamerV3-family variant"
   is right; add that on the robot task the two share every other setting.
2. **`appendix/a-research-methods.tex:38-40` "DreamerV3 and R2Dreamer — We clamp imagined returns to stabilize training."** —
   correct but incomplete: the clamp is at the ladder's max return (10), applied to imagined AND replay λ-returns
   (`dreamer.py:531,581`), enforced by `train.py::_write_ladder_provenance`, and it applies to the robot task only (the fastrl fork
   has no `return_clamp`). Lineage: `paper/DV3_DEBUG_2026-09-05.md` §5 — the NM512-fork port learned the reach proxy 15/15 on 4/4
   seeds only with `--return_clamp 1.0`; that port was then superseded by the r2dreamer chassis for the pixel study.
3. **`sections/04-evaluation.tex` Simulated Experiments: "100 human demonstrations from paper authors and 100 machine-generated
   trajectories … sampled actions … eight training seeds per source"** —
   (a) PinPad4 trials used 15 (trial 0: 13) human and 15 machine demonstrations EACH, one dataset per trial
   (`HD_pinpad_four_N`, `ai_imis/imiN`, `pinpad_train*.sh -imi 0..7`), not 100; (b) the PushT trials referenced by
   `results.ipynb` (`imi11–14`) have 70–78 human tapes v 100 machine tapes — the only 100-tape human sets (`HD_pusht_16/22/23`)
   have no machine counterpart; (c) machine demos were collected with `force_sample=False` → mode actions
   (`configs.yaml:33`, `AnotherDirector.py:623`), unless the notebook at collection time differed (UNKNOWN); the paper says
   sampled; (d) "eight seeds" holds for PinPad (imi 0–7) but the PushT caption says n=4. (e) "Both datasets retain unsuccessful
   attempts": consistent with the loader (no success filter) — e.g. HD_pusht_12 24/70 successes, AD_pusht_13 32/100.
4. **`tbl:pusht` caption "Demonstrations were produced after 200k steps of training" and `fig:results_pusht_train_return`
   "over 200k training steps"** — the active launch line is `-ts 60000` (and the config comment says the RSS budget was 60k env
   steps); `-ts 200000` appears only in a commented loop for `imi 7..8`. UNKNOWN which applies to the reported runs; state the
   budget from the wandb run config, not from memory.
5. **`sections/04-evaluation.tex` PushT text "DfD … received a reward of 1.0 for a successful trial and 0 at all other steps"**
   — matches `-fs` (`fastrl_utils.py:124-126`, `envs/pusht.py:46-47`). But the human PushT demos were RECORDED with the dense
   coverage reward (+600 success bonus in the recorder wrapper, `envs/pusht.py:43`) and relabelled at load; fine, but say
   "relabelled".
6. **PinPad reward**: the paper does not say PinPad4 used dense shaping (`envs/pinpad.py:129-142`: +0.01·k per correct pad,
   +10 per sequence, `use_dense_reward=True`). The figure is "train_return" on that reward. Add to the appendix table (done).
7. **`sections/05-conclusion.tex` "DfD, which uses pixel reconstruction loss, learns equally well from the human, DP, and
   R2Dreamer demonstrations, but do not learn well from motion planning demonstrations"** — human v DP: supported (0.306 v
   0.348, p 0.69, 8 v 8). Planner: supported (0.043, p 0.002 v human, 7 seeds). **R2Dreamer-teacher: n = 2 seeds** (0.425;
   p 0.42 v human at 8 v 2) — "equally well" is not testable at n=2; write "the two R2-teacher seeds so far are in the
   human/DP range". Also "DfD … learns equally well" contradicts the same paragraph's next sentence only if R2Dreamer is
   claimed "more robust" on the strength of the planner cell alone — that part IS supported (R2Dreamer planner 0.540, 7 seeds).
8. **`sections/04-evaluation.tex` restocking Results "We train 8-seeds … DV3"** and the table label "DreamerV3": rename to DfD
   (item 14 lane); the DfD cells are 8/8/7/2 seeds, so "8 seeds" is not yet true for planner (7) and R2 (2) — the table should
   carry per-cell n (the `px_results_4x4.md` row does).
9. **`sections/03-method.tex:27` "Within one environment every algorithm is trained to convergence"** — the robot-task DfD has
   a fixed 2M-step budget with milestone checkpoints; several planner-data DfD seeds only start reaching home after ~1.1M and one
   scores 0.28 at 2M (`STATE_PIXEL_4x4 §` and the per-seed CSV), so "to convergence" is not established for that cell. Say
   "fixed budget".
10. **Terminal-flag convention** (not in the paper): in every demonstration set the +10 `home` row is non-terminal to the world
    model while the online env terminates at home (§A.3). Same for all four sources, so not a source confound; disclose in the
    appendix (done in the text above).
11. **`appendix/a-research-methods.tex:79` "human dHfull with all_bnormclampS8ent5, machine dDPfull …"** describes the older
    state-based staged-ladder cells, not the pixel 4×4 — outside this item's scope but will confuse a reader of the DfD appendix;
    flag for the head session.
12. **Bib:** `dreamerv3torch2024` is defined twice in `sample-base.bib` (lines 308 and 622) — BibTeX warning; item-16 lane.

## \llm notes inserted
- `sections/03-method.tex`, Dreamer paragraph, appended after lane D's note (anchor `Replacement paragraph in the MD S3.}`) —
  note: robot-task DfD is the r2dreamer chassis with `rep_loss=dreamer`; corrected paragraph + appendix in this MD; "trained to
  convergence" is a fixed 2M budget.
- `sections/04-evaluation.tex`, anchor `Both datasets retain unsuccessful attempts.` — note: demo counts (PinPad 15/trial; PushT
  70–78 v 100), mode-not-sampled machine demos, 60k v 200k budget; see this MD §B / Discrepancies 3–4.
- `sections/05-conclusion.tex`, anchor `but do not learn well from motion planning demonstrations.` — note: R2-teacher cell n=2;
  planner p 0.002 at 7 seeds.
- `appendix/a-research-methods.tex`, anchor `We clamp imagined returns to stabilize training.` — note: paste-ready
  `\subsection{DreamerV3 from Demonstrations (DfD)}` with two tables in this MD §1.

## Sources
Cluster (read-only, `ssh pax`): `$W/runs/full_r2d_state_{dHfull_all,dDPfull_first}_rns10h_img_dreamer_s{8..15}/{.hydra/config.yaml,
.hydra/overrides.yaml,ladder_provenance.json,step_contract.json,console.log,milestones/}`;
`$LAB/{planner72,r2teacher}_px_2026-09-15/runs/full_r2d_state_native_rns10h_img_dreamer_s{0..7}/{…same…}`;
`$W/demos_state_full/{dHfull_all,dDPfull_first}_rns10h_img/{manifest.json,repeat.json}`;
`$LAB/planner72_px_2026-09-15/data/training_v3_reference_terminals/native_rns10h_img/repeat.json`;
`$LAB/r2teacher_px_2026-09-15/data/training_v1/native_rns10h_img/repeat.json`.
r2dreamer tree of record: `cluster/bundles/r2dreamer_px_full_main_2026-09-13.bundle` (head `0b1b9d8`, cloned to the scratchpad):
`configs/configs.yaml`, `configs/model/{_base_,size12M}.yaml`, `configs/env/genesis_full_pixel.yaml`, `dreamer.py`, `demo_prefill.py`,
`buffer.py`, `trainer.py`, `train.py`, `distributions.py`, `networks.py`, `envs/genesis.py`, `eval_genesis.py`, `optim/`, `README.md`.
Repo: `cluster/wmfix_full.sbatch`, `baselines/rl/full_env.py` (`LADDERS`, `max_return`), `HRI_results/HANDOFF_PIXEL_4x4_2026-09-17.md`,
`HRI_results/STATE_PIXEL_4x4_2026-09-16.md`, `HRI_results/PROVENANCE_nested_sparse10_pixel_2026-09-14.md`,
`HRI_results/DEMO_SETS_2026-09-11.md`, `HRI_results/PLANNER_MATCHED_STARTS_2026-09-14.md`,
`HRI_results/HANDOFF_R2_TEACHER_DATASET_2026-09-15.md`, `cluster/{planner72,r2teacher}_px_2026-09-15/HANDOFF.md`,
`paper/DV3_DEBUG_2026-09-05.md`, `paper/PX_PIXEL_CONFIG_2026-09-12.md`, `paper/AUDIT_TRAIL_R2D_NESTED_SPARSE_HvM_2026-09-13.md` (§B2),
`paper/LADDER_IMPL_NOTES_2026-09-10.md` (§4), `paper/figures/px_phase_2026-09-14/{px_results_4x4_sampled_per_seed.csv,
px_results_4x4_pairwise.csv,px_results_4x4.md}`.
Simulated tasks: `~/workspace/fastrl-pusht/{configs.yaml,train.sh,corl_superteleop_21:17:11_0913.py,fastrl_utils.py,AnotherDirector.py,
envs/pusht.py,envs/pinpad.py,pinpad_train.sh,pinpad_train_ai.sh,HDpusht_job.sh,AIDpusht_job.sh,pusht_default_job.sh,results.ipynb,
for_september_2024.txt}`; `~/workspace/fastrl/logs/{HD_pinpad_four_*,ai_imis/imi*,HD_pusht_*,AD_pusht_*}` (npz counts and
`is_last`/reward statistics computed with `~/workspace/genesis_sim2real/venv/bin/python`);
`~/workspace/dreamerv3-torch/dreamerv3-torch/{configs.yaml,dreamer.py,envs/pinpad.py,envs/pusht.py}` and `git show 365220c:{dreamer.py,configs.yaml}`
(later fork state, for contrast only). Paper: `sections/{02-related-work,03-method,04-evaluation,05-conclusion}.tex`,
`appendix/a-research-methods.tex`, `sample-base.bib`.
