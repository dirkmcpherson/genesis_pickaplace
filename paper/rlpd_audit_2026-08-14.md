# Adversarial implementation audit: RLPD (joint pick scope)

**Date:** 2026-08-14 · **Branch:** `4dof-cartesian` · **Auditor:** Fable (read-only; no code changed, no training launched)
**Scope:** `baselines/rl/rlpd_sac.py`, `baselines/rl/train_rlpd.py`, `baselines/rl/full_env.py`,
`baselines/rl/train_sacfd_full.py`, `baselines/rl/sacfd_delta_gate.py`, `baselines/wandb_eval.py`,
sb3 2.8.0 in `.venv-eval`, all `scratchpad/rlpd_*.log`.
**Reference:** Ball et al. 2023 (arXiv 2302.02948; ICML PMLR v202) **and the official code**
`github.com/ikostrikov/rlpd` (verified this session — see §3 for exact file/line evidence).

---

## Verdict summary

| # | Suspect | Verdict |
|---|---|---|
| 1 | Truncation bootstrapping (online + demo) | **CLEAN** (both paths correct, verified by repro) |
| 2 | Q-watchdog "fired at 1001" | **CONFIRMED BUG (wrongly waved off).** Fired at STEP 1001 with Q=2.82; Q went on to **441–2400** (max task return = 1). The watchdog is one-shot, so the explosion it was built to catch was never re-flagged. |
| 3 | Paper parity vs Ball et al. | **TWO ACCIDENTAL DEVIATIONS CONFIRMED:** (a) **entropy backup ON — the paper turns it OFF for every sparse task** (Table 2); (b) **LayerNorm affine params SHARED across all 10 ensemble members** (RLPD's are per-member). Plus one minor: 2-layer vs the paper's 3-layer sparse-task critic. Everything else matches. |
| 4 | Silent family (demo/online parity) | **CLEAN** — demo tensors, done semantics, grip sign, reward scale, action_repeat units all verified. |
| 5 | Learning dynamics | **CONFIRMED PATHOLOGY:** α and Q co-diverge (positive feedback); the pre-registered fixed-α fallback diverges *worse* (Q→1.6e5, critic loss 3.2e7). Also surfaced: **an eval/train horizon mismatch unique to the RLPD row** — real as a code fact, UNCERTAIN in size (§6). |

**Headline:** `rlpd_sac.py:220` — `next_q = next_q - ent_coef * next_logp` — is an
**accidental deviation from RLPD's published configuration for exactly this task class**, and it is
quantitatively sufficient to explain the arm's behaviour on its own.

---

## 1. Truncation bootstrapping — CLEAN

### 1a. Online path — correct

`FullTaskEnv._step_once` (`full_env.py:426`) emits `truncated = (not terminated) and self._t >= self.max_steps`,
a genuine gymnasium 5-tuple. The env is wrapped `Monitor → DummyVecEnv` (printed in every run log).
SB3 `DummyVecEnv.step_wait` (`common/vec_env/dummy_vec_env.py:63-66`):

```python
self.buf_dones[env_idx] = terminated or truncated
self.buf_infos[env_idx]["TimeLimit.truncated"] = truncated and not terminated
```

`ReplayBuffer.add` stores it (`common/buffers.py:278`) and `_get_samples` masks it
(`common/buffers.py:322`): `dones * (1 - timeouts)`. `RLPDSAC.train` consumes exactly that
tensor (`rlpd_sac.py:202,207,221`), so `target_q = r + (1-dones)*gamma*next_q` **bootstraps
through timeouts and not through genuine terminations.**

**Repro (deterministic, CPU):**

```
stored dones   : [1. 1. 1. 1.]
stored timeouts: [1. 0. 1. 0.]
dones handed to train(): [0. 1. 0. 1.]   <- 0 == bootstrapped
```

Full end-to-end repro through `make_rlpd().learn()` on a 17/7-dim dummy env with mixed
timeout/terminal endings: 1200 transitions, 76 episode ends, 50 recorded as `timeouts=1`,
26 as genuine terminals — i.e. the machinery is live in the real training loop, not just in the buffer class.

### 1b. Demo path — correct

`train_rlpd.py:150` → `train_sacfd_full.delta_encode_transitions(..., scope='pick', ...)`.
Measured over the real demo set (`baselines/episodes_pick_phase_all`, `pick_z=0.1505`, `cap=0.025`):

```
91 episodes -> 83,465 transitions
rewarded (r>0)        : 66
done=True             : 66
done & rewarded       : 66     <- every terminal is a pick
done & unrewarded     : 0      <- no false terminals
rows after a pick-done: 0      <- pick is the last row of its tape
episodes with NO done : 25     <- the 25 no-pick tapes end non-terminal => bootstrapped (correct)
```

The 25 no-pick demo tapes end with `done=False`, which is the *correct* treatment of a
recording truncation. (Note the plan's "post-pick rows kept" never actually applies:
the pick-phase demos are truncated at the pick, so there are zero post-pick rows.)

### Quantification asked for

Fraction of buffer transitions that are timeout-ends **treated as terminal: 0.000.**
Fraction that are timeout-ends at all: online ≈ `1/ep_len_mean` ≈ **0.12 %** (ep_len_mean 810 in
`rlpd_dH_s0.log`); demo half: 0 (all demo terminals are picks). Even had this been broken, it
could not have been the dominant effect. **Rule this suspect out.**

---

## 2. The Q-watchdog — CONFIRMED BUG (the warning was correct and was waved off)

**It fired at STEP 1001, at Q value 2.82.** `rlpd_sac.py:266-269` prints
`mean actor-state Q={qm:.2f} ... at step {self.num_timesteps}`. Every run log line reads:

```
[Q-WATCHDOG] mean actor-state Q=2.82 > 2.0 at step 1001 -- suspected value explosion
```

`step 1001` = `learning_starts=1000` + 1, i.e. the *first* `train()` call. So the two
readings offered ("step 1001" vs "Q 1001") resolve to: **step 1001, Q≈2.8**. Fired in
**every** run: `rlpd_dH_s0`, `rlpd_dH_s0_fixa` (step 3350), `rlpd_laneA`/`laneB` (all 3 lanes each),
`rlpd_ar4_s0..s5`, `rlpd_ar8_s0..s5`, both smokes.

**The waving-off is the bug.** `_watchdog_tripped` is one-shot (`rlpd_sac.py:264`), so the
2.82 was the *only* line ever printed — while `train/actor_q_mean` kept climbing. Parsed from
the SB3 log blocks in the run logs (max task return at pick scope = **1.0**):

| run | step | ep_len | ep_rew | **actor_q_mean** | critic_loss | ent_coef |
|---|---|---|---|---|---|---|
| dH_s0 | 8.1k | 810 | 0.00 | **441** | 35 | 0.122 |
| dH_s0 | 88k | 635 | 0.05 | **744** | 4,650 | 0.597 |
| dH_s0 | 162k | 609 | 0.13 | **873** | 22,300 | 0.903 |
| laneA lane3 | 107k | 658 | 0.02 | **2,400** | 60,800 | 2.23 |
| ar4_s0 | 99k | 203 | 0.02 | **269** | 84 | 0.115 |
| ar8_s0 | 7.3k | 104 | 0.00 | **427** | 49 | 0.156 |
| **dH_s0_fixa** (ent_coef 0.005) | 196k | 817 | **0.00** | **160,000** | **3.16e7** | 0.005 |

The critic's own residual (√critic_loss ≈ 30–250 per member) is **30–250× the entire task
reward**. The learning signal sits far below the critic's fitting error.

---

## 3. Paper parity vs Ball et al. 2023 — two accidental deviations

Verified against the **official code** (`github.com/ikostrikov/rlpd`) and paper Tables 1–2.
Note the requested `droq_config.py` / `adroit_config.py` do not exist; the real chain is
`configs/td_config.py → sac_config.py → rlpd_config.py` + README CLI overrides.

### 3.1 CONFIRMED ACCIDENTAL DEVIATION — entropy backup is ON; RLPD turns it OFF for sparse tasks

`rlpd_sac.py:216-221`:

```python
next_q, _ = th.min(next_q_all[subset], dim=0)
next_q = next_q - ent_coef * next_logp.reshape(-1, 1)      # <-- entropy backup
target_q = rewards + (1.0 - dones) * self.gamma * next_q
```

This is inherited verbatim from SB3 2.8 SAC. RLPD *has* the same code path
(`rlpd/agents/sac/sac_learner.py`, `if self.backup_entropy:`) and its config default is `True` —
**but the README launch commands for both sparse domains pass `--config.backup_entropy=False`,
and paper Table 2 (p.18) states it outright:**

| Environment | CDQ | **Entropy Backups** | MLP Architecture |
|---|---|---|---|
| Locomotion | True | True | 2 Layer |
| AntMaze | False | **False** | 3 Layer |
| **Adroit** (the sparse-manipulation domain) | True | **False** | 3 Layer |
| DMC (Pixels) | False | **False** | 2 Layer |

Entropy backups are ON only for *dense-reward locomotion*. Every sparse task in the paper —
i.e. every task structurally like ours — runs with them **off**. Our code has them on, with no
comment, no flag, and no mention in `RLPD_PLAN.md`. **Classification: ACCIDENTAL.**

**Why it is fatal here, quantitatively.** With the entropy backup on, the critic's fixed point
for any non-terminal transition in a zero-reward region is

&nbsp;&nbsp;&nbsp;&nbsp;`Q* ≈ γ·α·H / (1 − γ)` = **500 · α · H** at γ = 0.998.

Checked against the logs (H = policy entropy, bounded above by 7·ln2 = 4.85 for a
tanh-bounded 7-dim action):

| run/step | measured Q | measured α | implied H = Q/(500α) | plausible? |
|---|---|---|---|---|
| dH_s0 @162k | 873 | 0.903 | 1.93 | ✓ |
| laneA lane3 @107k | 2,400 | 2.23 | 2.15 | ✓ |
| ar4_s0 @99k | 269 | 0.115 | 4.68 | ✓ (near max) |
| laneA lane2 @150k | 1,020 | 1.43 | 1.43 | ✓ |

**The formula reproduces the observed Q in every window.** The Q "explosion" is not an
instability to be debugged — it is the exact analytic fixed point of a term the paper deletes.

Three consequences, all matching the symptom:

1. **Scale.** Values live at 50–2,400 while the entire task reward is 1.0. The +1 is
   **0.04 %–2 %** of the regression target and, per §2, is 30–250× *smaller than the critic's own
   RMSE*. The task objective is numerically invisible.
2. **Anti-terminal gradient — the perverse one.** `scope='pick'` terminates on success
   (`full_env.py:370-374`), and the 66 demo picks are `done=True` (§1b). At a terminal,
   `target = r = 1.0` exactly. At any non-terminal, `target ≈ 400`. So the critic is taught that
   **succeeding is worth ~1 and dithering is worth ~400** — a 400:1 incentive *against*
   completing the task. RLPD never meets this because (a) entropy backups are off for sparse
   tasks and (b) its Adroit offline loader deletes terminals entirely
   (`rlpd/data/binary_datasets.py`: `remove_terminals=True` → every demo transition `mask=1.0`),
   and Adroit episodes do not terminate on success at all.
3. **α↔Q positive feedback (see §5).** The actor's gradient scales with |Q| ≈ 400, so it
   collapses entropy to chase Q; auto-α then *raises* α to restore entropy; a higher α raises the
   floor 500·α·H, which raises Q. Measured co-divergence: laneA lane3 α 0.39→2.23 while Q 594→2,400.

**Minimal fix:** drop the entropy term from the Bellman target (equivalently, expose
`backup_entropy=False` and default it off for `scope='pick'`), i.e. delete `rlpd_sac.py:220`.
With `backup_entropy=False` the critic's range becomes `[0, 1]` for this task and the +1 becomes
100 % of the signal instead of 0.25 %.

**Expected effect size:** value-target SNR improves by **~2.5 orders of magnitude**; the anti-terminal
incentive (currently 400:1 against success) *inverts* to 1:0 in favour. This does not guarantee
the arm clears the noise floor, but no honest statement about "RLPD on this task" can be made
until it is re-run without it. Note this is a *within-paper* fix, not a novel intervention.

**What it invalidates:** every RLPD number produced so far — stride-1 (0.039), ar4 (0.023),
ar8 (0.000), the 0.40 selected max, the 7-seed "ignition" story, and the **repeat-N verdict in
handoff §4a-3**. The action-repeat sweep varied horizon while the value scale was pinned 400×
above the reward in all three arms; a flat, at-floor result across N is exactly what a
signal-invisible critic produces regardless of N. "Action-repeat bought nothing" is not safe as a
clean negative result — it is confounded and should be re-run after the fix or reported with this caveat.
The **RLPD > SACfD contrast survives** (SACfD is strictly zero and shares the same defect).

### 3.2 CONFIRMED DEVIATION (moderate) — the 10 critics share one LayerNorm

`rlpd_sac.py:92`: `layers.append(nn.LayerNorm(h))` applied to an `(E, B, h)` tensor.
`nn.LayerNorm` normalizes the last axis per row (correct) but its **affine parameters have no
ensemble axis**, so all 10 members share them. Measured parameter shapes:

```
qnet.0.weight  (10, 24, 256)   <- per-member  ✓
qnet.1.weight  (256,)          <- LayerNorm: SHARED across all 10 members  ✗
qnet.3.weight  (10, 256, 256)  <- per-member  ✓
qnet.4.weight  (256,)          <- SHARED  ✗
qnet.6.weight  (10, 256, 1)    <- per-member  ✓
```

In RLPD each of the 10 critics is an independently-parameterised `MLP` (flax `Ensemble` over
`num_qs` separate parameter trees), so **every member has its own LayerNorm scale/shift**.
Here the shared affine receives the *summed* gradient of all 10 members, injecting a common
component into every member's function during training. At init the coupling is nil (LN affine is
identity: measured inter-member output correlation mean −0.015), so this is a *training-time*
diversity loss, not an init defect.

Why it matters: the paper's own ablation (Fig. 7) is that removing LayerNorm gives
"significantly higher variance across seeds" and, on expert-only data, "no progress on any task."
Ensemble diversity is what makes min-over-Z pessimism real. The `_fixa` run's runaway
(Q → 1.6e5, critic loss 3.2e7 — far above the α=0.005 entropy floor of ≈12, so *genuine*
overestimation divergence, not the §3.1 fixed point) is the signature of insufficient
ensemble pessimism. **Effect size: UNCERTAIN** — plausible contributor to the fixed-α divergence
and to seed variance; not independently measurable without a rerun.
**Minimal fix:** give LayerNorm an ensemble axis (per-member affine of shape `(E, 1, h)`), or
drop the affine (`elementwise_affine=False`) so nothing is shared.

### 3.3 Minor deviation — 2-layer critic vs the paper's 3-layer for sparse tasks

`train_rlpd.py`/`make_rlpd` default `net_arch=(256, 256)`. Paper Table 2: Adroit and AntMaze both
use **3 Layer**; README passes `--config.hidden_dims="(256, 256, 256)"`. 2-layer is the
*locomotion* setting. Undocumented → **ACCIDENTAL**, low impact, cheap to align.

### 3.4 Point-by-point on the requested checklist — all CLEAN

| Item | Ours | RLPD | Verdict |
|---|---|---|---|
| (a) 50/50 draw per **gradient step** | fresh 128+128 draw inside the UTD loop (`rlpd_sac.py:202-208`) | one `256·UTD` batch, **interleaved** `[0::2]=offline, [1::2]=online`, sliced contiguously → each minibatch exactly 128/128 | **CLEAN** — per-grad-step 50/50 achieved; ours resamples rather than slices (benign, arguably more data) |
| (b) min over **target** critics | `self.critic_target(nobs, next_act)` then `min` over `randperm(E)[:Z]` (`rlpd_sac.py:217-219`), redrawn every grad step, without replacement | `subsample_ensemble(key, self.target_critic.params, num_min_qs, num_qs)`, `jax.random.choice(..., replace=False)`, fresh key per step | **CLEAN** |
| (c) LayerNorm after each hidden Linear, incl. first, before activation | `EnsembleLinear → LayerNorm → ReLU` ×2, bare `EnsembleLinear(→1)` head | `Dense → LayerNorm → ReLU` for every hidden incl. the last (`activate_final=True`), bare `Dense(1)` head, critic-only | **CLEAN** (placement identical; see §3.2 for the sharing defect) |
| (d) actor vs ensemble **mean** | `self.critic(obs, actions_pi).mean(dim=0)` (`rlpd_sac.py:244`), live critic | `qs.mean(axis=0)` over all 10, live critic; Algorithm 1 line 23 | **CLEAN** |
| (e) `target_entropy` | `-act_dim/2 = -3.5` (`rlpd_sac.py:295`); α₀ = exp(0) = 1.0 (SB3 default) | `target_entropy = -action_dim / 2`; `init_temperature = 1.0` (Table 1) | **CLEAN** — deliberate, documented, and *matches the paper* (SB3's own default −dim would have been the deviation) |
| (f) `tau` / target-update cadence | 0.005, polyak every grad step (`rlpd_sac.py:253`) | 0.005, EMA inside each `update_critic` | **CLEAN** |
| UTD, actor/α ×1 per env step | actor+α on final grad step only (`rlpd_sac.py:233`) | `update_actor`/`update_temperature` once per UTD cycle on the last minibatch | **CLEAN** |
| E, Z | 10, 2 | Adroit 10, 2 | **CLEAN** |
| critic dropout | none | `critic_dropout_rate` set by no config → always `None` | **CLEAN** (the "DroQ dropout" belief is wrong; dropout is only an *ablation* in Fig. 9 and performs *worse* on sparse tasks) |
| critic loss reduction | `sum_E(mean_B MSE)` | `((qs - target)**2).mean()` | **CLEAN** — differs by a constant factor 1/E on each member's own gradient; Adam-invariant, documented as deliberate |
| `gamma` | 0.998 | 0.99 | **DELIBERATE**, documented (`train_rlpd.py:63-66`) — but see §7: it is the multiplier on the §3.1 defect (500× vs 100×) |
| offline data terminals | 66 demo picks `done=True` | Adroit loader forces `mask=1.0` on **all** offline transitions | **DELIBERATE-BY-TASK** (our env genuinely terminates on the pick) — but it is the half of §3.1 that creates the anti-terminal gradient |

Also noted (paper-vs-code, not ours): Algorithm 1 line 17 prints the entropy backup with a `+`
where the code subtracts, and line 21 has ρ/(1−ρ) swapped in the EMA. The code is right in both cases.

---

## 4. Silent family (demo↔online parity) — CLEAN

- **Gate path == train path.** `sacfd_delta_gate._encode` and `train_rlpd.py:143-152` both dispatch
  on `(action_repeat > 1)` into `delta_encode_transitions_repeat` / `delta_encode_transitions`
  with the same `(pick_z, scope, delta_cap, repeat)` and both build `DemoData(..., action_transform=None, ...)`.
  Same functions, same arguments — **not** a parallel construction. The gate's
  `np.array_equal(dd.actions, ref_act)` assert therefore certifies the tensors the trainer actually uses.
- **`action_transform`.** `delta_joint` passes `norm=None` (actions already normalized);
  `absolute` passes `pick_env.normalize_action`. Explicit per branch, no fallback. ✓
- **Grip sign.** Demo: `clip(cmd,0,1)*2-1` → `[-1,1]` (`train_sacfd_full.py:112`).
  Env: `(clip(a[6],-1,1)+1)/2` → `[0,1]` (`full_env.py:344`). Exact inverses. ✓
- **Reward scale.** Demo pick `STAGE_REWARD['picked'] = 1.0`; env pick `+1.0`. Identical. ✓
  Measured demo density `train/demo_rew_per_batch` = 0.0–0.2 per 128 — consistent with 66/83,465.
- **Obs parity.** Both halves are the 17-dim `genv._obs()['state']` float32; the delta gate's
  open-loop replay (which re-earns the demonstrated lift from demo actions through the live env)
  is the end-to-end evidence that layout and scale agree. ✓
- **`action_repeat` units.** `train_rlpd.py:110-121` asserts `repeat>1 ⇒ delta_joint` and asserts the
  env got the value; `max_steps` stays a **sim-step** budget (`full_env.py:319-324`), SB3 counts
  **decisions**, so `--steps`/`--eval-freq`/`save_freq=50_000` are all decisions and consistent.
  The sidecar (`train_rlpd.py:173`) travels to every snapshot and `wandb_eval` reads
  `action_repeat` from it (`wandb_eval.py:125-127`). ✓
  *Book-keeping note, not a bug:* "100k decisions" is 100k sim steps at stride 1, 400k at ar4,
  800k at ar8 — the arms are matched on gradient steps, not on physical experience.

---

## 5. Learning dynamics — s0 (ignited) vs the flat seeds

Parsed from the SB3 blocks (`scratchpad/rlpd_dH_s0.log`, `rlpd_laneA.log` 3 lanes, `rlpd_ar*`).

**What differs first:** nothing in the *first* 25k. All seeds start at ep_len ≈ 810–900,
ep_rew 0, α decaying from 1.0, Q already 400+. The seeds separate only in whether α turns
around and how far Q rides up with it:

| lane | α trajectory | Q trajectory | ep_rew @ end | eval picked @200k (400 steps) |
|---|---|---|---|---|
| laneA-1 | 0.096 → 0.62 → 0.19 → 0.43 | 423 → 748 → 123 → 160 | 0.06 | 0.00 |
| laneA-2 | 0.26 → 0.55 → 1.17 → 1.44 | 513 → 778 → 1,060 → 875 | 0.15 | 0.00 |
| laneA-3 | 0.39 → 0.66 → 2.23 → 1.27 | 594 → 1,280 → 2,400 → 551 | **0.41** | 0.20 |
| dH_s0 | 0.12 → 0.60 → 0.90 → 0.80 | 441 → 744 → 873 → 685 | 0.28 | 0.00 (0.40 @150k) |

Two findings:

1. **α does not collapse — it *rises*, up to 2.23**, and Q rises with it in lockstep, exactly as
   `Q ≈ 500·α·H` predicts. This is the §3.1 feedback loop, not exploration death. (The memory
   note "auto-alpha collapsing to ~0 = exploration death" is the *opposite* of what happened here.)
2. **The pre-registered fallback makes it worse, not better.** `rlpd_dH_s0_fixa` (fixed α=0.005)
   never scored (ep_rew 0.00 throughout 200k) and diverged outright: Q 5.8 → 6,100 → 19,500 →
   53,200 → 143,000 → **160,000**, critic loss → **3.16e7**. The α=0.005 entropy floor is only
   ≈12, so this is genuine overestimation runaway, i.e. the ensemble pessimism is not holding
   (→ §3.2). **The "restart with fixed --ent-coef 0.005" prescription in `rlpd_sac.py:260-261`
   and `train_rlpd.py:68-70` should be retired**; it treats a symptom of the entropy backup by
   removing the actor's only regularizer.

No gradient norms are logged; that is the one dynamics channel this audit could not read.

---

## 6. Additional finding (not on the list): the RLPD row is evaluated at a 3× shorter horizon than the DP row

Not a code bug, but it directly inflates the reported gap and belongs with the numbers.

- **RLPD trains at `--train-max-steps 900` sim steps** (`train_rlpd.py:73`) and is
  **evaluated at 400 sim steps** — `--eval-max-steps` default 400 (`train_rlpd.py:100`), and the
  handoff §4a-2/§4a-3 100k protocol fixes 400.
- **The official DP/SACfD matrix evals pass no `--max-steps`**
  (`cluster/launch_paper_week.sh:92-94, 108-110`; `cluster/sbatch_reeval_sacfd.sh:40-41`), so they
  take `wandb_eval.py:30`'s default **1200**.
- The **median demo pick frame is 662** — *past the RLPD eval cutoff and inside the DP one.*

So `RLPD 0.039 @400 steps` vs `dH_DP 0.62 @1200 steps` is not a matched measurement. Suggestive
(not conclusive) corroboration from the logs: training `ep_rew_mean` (900-step budget, stochastic
policy) reaches 0.06/0.15/0.41 in the three laneA lanes while the 400-step deterministic eval
reports 0.00/0.00/0.20 at the same steps. The r2dreamer comparison **is** matched (it trains and
evals at 400 sim steps, METHODOLOGY §"time limit"), so this caveat rescues nothing against 0.91;
it applies to the DP/BC columns.

**I attempted a direct 400-vs-1200 A/B and killed it as invalid — reporting that here because it
is a trap the next person will also fall into.** I launched two `wandb_eval.py` runs on
`rlpd_dH_s0/rlpd_100000_steps.zip` (demo-IC, n=15, deterministic, delta_joint via sidecar)
differing only in `--max-steps`. This is **the same experiment newbox_supp already ran and
retired as confounded** (handoff, "DOSE-RESPONSE ... CONFIRMED 08-14"): `GenesisCanEnv.reset`
never re-issues `control_dofs_position`, so each episode inherits the previous episode's
controller target; residue **accumulates across resets**, and changing the horizon changes the
residue schedule. Quote: *"The 400-vs-1600 'horizon effect' was never physics — longer
predecessors leave different residue."* A single-process `--max-steps` A/B therefore cannot
isolate truncation from residue, at any n. **The horizon question is UNCERTAIN and stays open.**

Two things the aborted run did establish, both protocol observations rather than horizon results:

- The 400-step arm completed 15/15 and produced **3 picks (uids 243, 244, 246 — episodes 7, 8, 10
  of the sequence)** for s0 @100k. Handoff §4a-3 reports the fresh-process 100k readout as
  stride-1 4/105 with *"no seed exceeds 2/15."* My sequence protocol yields 3/15 on a single seed.
  This is **not** a contradiction — it is another instance of the residue artifact, and it points
  the same direction as newbox_supp's dose-response (uid 243 picks with 7 predecessors, not alone;
  my picks are at exactly episodes 7/8/10). Sequenced evals read **higher** than isolated ones.
- Consequently the fixed-checkpoint stride-1/ar4/ar8 table is protocol-sensitive in both
  directions and the correct 400-vs-1200 measurement is **one episode per fresh process at each
  horizon** (the adopted standard), which is worth spending only *after* the §3.1 fix.

---

## 7. What to do next (ranked)

1. **Turn the entropy backup off** (§3.1) — one line, restores paper parity for the sparse regime,
   converts the value scale from ~400 to ~1. Re-run ≥3 seeds at stride 1, 100k, and re-measure.
   Nothing else on this list matters until this is done.
2. **Re-declare the repeat-N verdict provisional** (handoff §4a-3). It was measured entirely inside
   the defect; a flat result across N is what a signal-invisible critic produces at any N.
3. **Give LayerNorm a per-member affine** (§3.2) and go to 3 hidden layers (§3.3) — both are
   free paper-parity fixes, both bear on the divergence and on seed variance.
4. **Retire the "fixed ent_coef 0.005" fallback** (§5) and re-scope the watchdog: make it
   *recurring* (or a hard abort) rather than one-shot, and thresholded on the task's return range.
5. **Caption every RLPD-vs-DP comparison with the 400-vs-1200 horizon gap** (§6), and if the
   horizon is to be settled, measure it with one episode per fresh process at each horizon —
   never a single-process `--max-steps` A/B (that experiment is already dead by residue).
6. Only then revisit reward density (handoff §3a options 1/2) — the lit doc's #1 explanatory delta.
   It is still probably real, but it was never testable while the reward was 0.25 % of the value scale.

## 8. What this audit did NOT find

- No bug in truncation handling, online or demo (§1).
- No demo/online tensor mismatch, grip-sign, reward-scale, dtype, or `action_repeat` unit bug (§4).
- No error in the ensemble/subset/target/actor-mean/UTD/α machinery (§3.4) — the RLPD-specific
  code is a faithful port. The defect is in a term the port *inherited from SB3 and never questioned*,
  which is the same shape as this repo's other silent-default incidents.
