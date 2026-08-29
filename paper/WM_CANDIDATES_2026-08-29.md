# World-model learner candidates for the human-vs-model-demos paper (scout report, 2026-08-29)

Scope: published, unmodified model-based RL methods for the Genesis can-pick task (2x64x64 RGB + 17-d proprio,
7-d joint deltas, action_repeat 4, 300 decisions/episode, sparse +1 on pick, 56 human / 56 DP demos, 1-3M sim steps,
1 GPU, 2-day job cap, ~3 weeks to deadline). Every claim is tagged **[E]** evidence (paper/code/issue I read) or
**[I]** inference. No cluster commands were run; nothing outside this file was edited.

## 0. Headline

1. **Our DreamerV3 failure is most likely port-specific, not a DreamerV3 property** — but the published fix is a
   *recipe*, not a single knob. NM512/dreamerv3-torch is explicitly declared stale by its author ("implemented prior to
   major updates to DreamerV3 and does not reflect those changes, which accounts for several GitHub Issues") [E:
   NM512 README]. The current official recipe (arXiv v2 2024 / Nature 2025) trains the critic on **replay
   trajectories as well as imagination** (beta_repval = 0.3), regularises the critic to its own EMA, uses percentile
   return normalisation with the max(1,S) floor, and zero-inits the reward/critic output layers "because large
   predicted rewards can delay the onset of learning" [E: arXiv 2301.04104v2; danijar/dreamerv3 agent.py has
   `repl_loss`, `slowreg`, `retnorm{impl: perc, limit: 1.0, perclo 5, perchi 95}`]. The in-house r2dreamer "replay-buffer
   critic loss" therefore re-implements a *published* component; the **return-target clamp does not exist in the
   official code** (agent.py `lambda_return` has no clipping) [E]. So: r2dreamer-with-clamp is not "as published";
   r2dreamer-with-replay-critic-loss-only arguably is (see §2.1 for the caveat on scale 0.3).
2. **No published method matches all our constraints exactly** (sparse reward, pixels, <=3M steps, demos consumed
   *without* actor BC). The two methods with actual published sparse-reward+pixels+few-demos results at our budget —
   **MoDem** (100k interaction steps, 5 demos, Adroit/Meta-World sparse) and **DEMO3** (100k-500k steps, 5-100 demos,
   16 sparse manipulation tasks) — both *require BC pretraining of the actor* as phase 1 [E]. That collides with the
   user's 2026-08-11 rule "NO actor-BC in the world-model arm" (PAPER_PLAN.md). This is the central decision, §3.
3. **Recommendation (A, most likely to learn):** DEMO3 (TD-MPC2 backbone + demo oversampling + BC pretrain + learned
   stage reward), run as published with the pick as a 1-stage task, *if* the H4 framing is relaxed to "model-based
   learner that consumes demos as published". **Recommendation (B, cheapest & rule-compatible):** official
   danijar/dreamerv3 (JAX) with demos pre-loaded into replay — the RLC 2024 Staley/Sinapov paper is a published
   precedent for exactly this usage on sparse-reward pixel tasks [E], and it needs no actor BC.

## 1. Candidate-by-candidate

### 1.1 DreamerV3, official JAX (danijar/dreamerv3; Hafner et al. arXiv 2301.04104 v1 2023 / v2 2024, Nature 2025)
- Demos: **no native demo loading** — configs.yaml has no `demo`/`offline` keys [E]; replay dir pre-fill is the
  standard hack (used by Staley et al. RLC 2024, who "let a human teleoperate... D <- (s, pi_human(s), s', r)" and
  train W and pi_dream on D, i.e. demos only enter via the world model, no BC) [E: rlc2024_james.pdf Alg. 1]. That
  paper reports sparse-reward PinPad5 / MemoryMaze (image obs, 1000-step episodes), 4-6x faster to 90% of max
  reward; **no manipulation, no proprio, no code URL found** [E/I].
- Reward: sparse OK in principle (paper's percentile floor is explicitly for sparse) [E], BUT open issue #214
  (2026-07): default DreamerV3 gets "near-zero return after 5M steps" on DMC manipulator_bring_ball (sparse,
  contact-rich) while "TD-MPC2 converged without issues"; no maintainer reply [E]. This is the closest public
  analogue to our task and it is a negative result for vanilla DV3.
- Pixels+proprio: yes (dict obs, image + vector keys) [E: configs]. Framework: JAX (separate env from our torch stack).
- Value runaway: I found **no** issue on danijar/dreamerv3 or NM512/dreamerv3-torch reporting critic/lambda-return
  divergence past max return [E: gh issue search over both repos, titles listed in scout log]. NM512 #16 documents a
  return-indexing deviation from official (fixed), #6 terminal-value bug (closed) [E]. Absence of a report is weak
  evidence; the official recipe's *mechanisms* (EMA-regularised critic, replay critic loss, symexp-twohot with
  zero-init) are the published answer to slow/unstable value onset, not an explicit anti-runaway guarantee [I].
- Wall-clock: our sim is the bottleneck (~60k sim steps/h/proc => 1M sim steps ~17h/proc; 3M > 2-day cap unless
  the VEC facade runs >=2 envs) [E: task brief]. Dreamer's own train step at train_ratio 32-512 is the second cost;
  a third-party paper reports ~7h/1M env steps on a 3080 Ti for DreamerV3 (source not verified, treat as order of
  magnitude) [I].
- Blockers: embodied API wrapper (not gym/gymnasium); `FullTaskEnv` is gymnasium [E: baselines/rl/full_env.py]
  so a ~100-line adapter; action_repeat is done inside our env already (keep repeat=1 on the Dreamer side) [I].

### 1.2 NM512/r2dreamer (ICLR 2026 R2-Dreamer + "up-to-date, ~5x faster PyTorch DreamerV3 baseline")
- Author-declared successor to dreamerv3-torch [E: NM512 README]. README does not say whether the critic replay
  loss / percentile retnorm of the 2024 recipe are implemented; a GitHub code search for `repval`/`percentile`
  returned no hits (code search indexing is unreliable — **verify by reading dreamer.py/trainer.py locally**) [E/I].
- No demo/offline support natively [E: README]. Our port added demo pre-fill, return clamp, replay critic loss,
  entropy 3e-5, horizon 333, action_repeat 4 (R2DREAMER_V2_FIXES_EXPLAINED.md). Of those, only "replay critic loss"
  and horizon 333 (= official `horizon: 333`, gamma 0.997) are published-recipe items; entropy 3e-5 (official 3e-4),
  the clamp, and demo re-injection are modifications [E].
- Status in project: the arm has produced no honest pick under the hardened predicate (PAPER_PLAN 08-10) except the
  dense-reward+clamp variant (13-15/15 at selected ckpts, bistable) [E: task brief].

### 1.3 TD-MPC2 (Hansen et al. ICLR 2024; nicklashansen/tdmpc2, PyTorch, MIT)
- Demos: **none** in single-task online RL [E: paper]; the offline mode is multitask-dataset training, not demo
  bootstrapping. Reward: paper's sparse tasks are DMC cartpole/reacher sparse, cup catch etc.; Meta-World/ManiSkill2
  in the paper use *dense* shaped rewards with success metrics [E]. Pixels: 64x64 RGB with shallow conv + random
  shift, DMC only; no combined pixel+proprio config in the paper [E]. Budgets: 2M (Meta-World) to 4-14M (DMC/MS2)
  [E] — above ours for sparse.
- Cost: MPPI 512 samples x 6 iterations x horizon 3 per decision; the planner dominates wall-clock (~15 Hz on a
  4090 reported by a third party) [E/I]. Gymnasium-based env examples [E: README].
- Verdict: valid backbone, but **as published it is demo-free** — does not answer H4 alone.

### 1.4 MoDem (Hansen et al. 2022, facebookresearch/modem, PyTorch on TD-MPC v1, MIT)
- Published sparse-reward pixel manipulation at **100k interaction steps (200k env steps, action_repeat 2) with 5
  demos**: Adroit door/hammer/pen, 15 Meta-World tasks; obs = 224x224 RGB (2-frame stack) + proprio [E: paper].
- Demo use: phase 1 BC pretrain (encoder+policy), phase 2 seeding (5k steps, 75% demo oversampling), phase 3
  interactive with demo ratio annealed 75%->25% [E]. Reported wall-clock ~8h total per task (55 min + 34 min +
  6.5 h, "primarily planning overhead") [E].
- Blockers: single-commit repo, MuJoCo 2.1 / mj_envs / mujoco-py toolchain, old gym; demos as repo-release pickles;
  224px encoder (would need a config change to 64px, minor) [E/I]. MoDem-V2 repo is **archived 2025-08-06,
  CC-BY-NC**, real-robot focused [E] — skip.
- Verdict: the canonical "MBRL + demos, sparse, pixels" citation, but superseded by DEMO3 whose codebase *contains*
  a MoDem baseline on TD-MPC2 with a maintained toolchain.

### 1.5 DEMO3 (Lopez Escoriza, Hansen et al., ICML 2025, arXiv 2503.01837; adrialopezescoriza/demo3, PyTorch, MIT)
- Closest published match: TD-MPC2 backbone; 16 **sparse** manipulation tasks (ManiSkill3, Meta-World, Robosuite,
  MS humanoids), RGB 128/224px + proprio, 5-100 demos, **100k-500k interaction steps**, action_repeat 2 (1 for
  Robosuite); +40% avg over MoDem/TD-MPC2/LaNE [E: paper html]. Wall-clock **5.19 h per 100k steps on one RTX 3090**
  (their sims; ours would add the Genesis CPU cost) [E].
- Demo use: phase 1 joint BC pretrain of policy+encoder; phase 2 50% demo sampling ratio; plus per-stage
  discriminators giving a bounded dense bonus `r + beta*tanh(delta(z))`, beta<=1/3 — for our 1-stage pick this is a
  single learned discriminator [E]. Flags: `obs=rgb`, `enable_reward_learning`, `demo_sampling_ratio`,
  `policy_pretraining`; demos are `.pkl` trajectories with state+RGB; TD-MPC2 and MoDem baselines run from the same
  repo by flag [E: README]. CUDA 12.4+, 32 GB RAM for pixels; Genesis-vs-their-sim (ManiSkill3 uses gymnasium)
  compat unverified — custom-task guide absent [E].
- Verdict: **(A) most likely to learn** within 1-3M sim steps (their budgets are 4-10x smaller than ours, sparse,
  pixels, demos). Validity caveat: it *is* demo-augmented RL with actor BC — see §3.

### 1.6 Not viable / not relevant (one line each)
- DIAMOND: Atari/discrete; authors list continuous control as future work [E]. IRIS/STORM: discrete [E]. DreamerPro:
  robustness to distractors, no demo mechanism [I]. DreamerFD (needle picking, DreamerV2 + demo replay buffer): no
  code found [E]. "Dreaming with demos"-style DV3 variants: only the RLC 2024 paper surfaced, code not found [E].

## 2. Is our value runaway port-specific?

- **[E]** NM512/dreamerv3-torch predates the 2024 recipe and its author says so. The 2024 recipe adds exactly the
  components that pin the critic to observed data: replay critic loss (0.3), EMA-critic regulariser (slowreg 1.0),
  percentile retnorm with floor 1, zero-init critic/reward heads.
- **[E]** No public issue on either repo reports "lambda-return exceeds max attainable return"; #214 reports vanilla
  DV3 failing outright on a sparse contact task, which is consistent with our from-scratch pick nulls but says nothing
  about runaway.
- **[I]** Our x100 terminal reward with gamma 0.997 and an actor that quickly makes "pick" imagined-likely is the
  classic setup for imagination-only bootstrapping to overshoot (value of a state = 100 regardless of how many
  steps away, once the model predicts eventual pick); the replay critic loss exists to anchor exactly that. The fact
  that r2dreamer-with-replay-loss learns with dense reward is consistent. So: likely port-specific *recipe gap*, not
  an algorithmic property — but unproven until an official-recipe run (the standard-recipe arm now running, or
  danijar JAX) either reproduces or avoids it. Pre-register: if the standard-recipe torch arm still runs away, the
  next test is danijar JAX with `repl_loss` on (default `repval_loss` true — verify in configs) at reward scale 1.

## 3. Ranking and the H4 tension

| Rank | Method | Learns pick in budget (as published)? | Rule-compatible (no actor BC)? | Integration |
|---|---|---|---|---|
| A1 | DEMO3 (1-stage) | **Likely** [I from 16 sparse tasks <=500k] | No (BC pretrain is phase 1) | ~4-6 pd |
| A2 | MoDem via DEMO3 repo flags | Likely [E published sparse results] | No | same repo as A1 |
| B1 | danijar DreamerV3 JAX + demo replay pre-fill | Uncertain (#214 negative; RLC'24 positive on nav) | **Yes** | ~2-3 pd |
| B2 | r2dreamer w/ replay critic loss, no clamp, standard entropy | Uncertain (project data: needs dense) | Yes | ~0.5 pd (already ported) |
| — | TD-MPC2 vanilla | Unlikely at 3M sparse [I] | Yes (demo-free = not H4) | ~2 pd |

- **If H4 must be "demos as dynamics/reward data only"**: B1 is the only published, unmodified pipeline that does
  this (Staley et al. 2024 is the citation). Effort ~2-3 person-days: embodied env adapter for `FullTaskEnv`
  (gymnasium -> embodied, 64x64 two-camera image key + 17-d vector key), npz -> Dreamer replay chunk writer,
  JAX/CUDA env on the cluster (separate from the torch env; NEVER conda into the existing one), eval hook to
  wandb_eval. Risk: JAX install + #214-style null. Run at defaults (train_ratio 32-64 given CPU-bound sim,
  reward scale 1, size 12M or 25M). Keep this arm's negative result publishable: "official DV3 + demo replay".
- **If the paper can accept a demo-augmented MBRL arm** (state the confound: BC init imitates demonstrator actions,
  so source differences may propagate): A1 is the strongest bet. Effort ~4-6 person-days: Genesis task in
  their env registry (their wrappers assume ManiSkill3/Meta-World/Robosuite gymnasium APIs), demo `.pkl`
  conversion from our npz (states, actions, rewards, rgb), 64px encoder config, action_repeat 1 on their side (ours
  is inside the env), and a wall-clock check: their 5.19 h/100k on a 3090 plus our sim => ~1M sim steps
  (250k decisions) is ~15-20 h, fits the 2-day cap; 3M does not without multi-env [I].
- **Cheapest of all** is B2 (r2dreamer, drop the clamp, keep replay critic loss at the published 0.3 scale, entropy
  3e-4, sparse) — but project evidence says it never learned sparse; it is a cheap negative control, not a bet.

## 4. Evidence ledger (what I actually read)
- NM512/dreamerv3-torch README (stale notice), issues #16/#6 (indexing/terminal bugs); issue-title search for
  critic/value/diverge/demo on NM512 and danijar repos (no runaway report found).
- danijar/dreamerv3 configs.yaml (`retnorm`, `valnorm`, `slowvalue`, `imag_loss`/`repl_loss`, `horizon: 333`,
  `actent 3e-4`, no demo keys) and agent.py (repl_loss present, no lambda-return clamp); issues #214, #181, #190.
- arXiv 2301.04104v2 (beta_repval 0.3, percentile 5-95 EMA 0.99, L=1 floor, zero-init rationale).
- MoDem arXiv 2212.05698 (ar5iv) + facebookresearch/modem README; modemv2 README (archived, CC-BY-NC).
- TD-MPC2 arXiv 2310.16828v2 (pixels 64px DMC only, budgets 2-14M, no demos) + repo README/config.yaml.
- DEMO3 arXiv 2503.01837v1 + adrialopezescoriza/demo3 README (flags, pkl demos, baselines by flag, 5.19 h/100k).
- Staley & Sinapov RLC 2024 PDF (Alg. 1: demos -> replay only; PinPad5/MemoryMaze; no code link).
- NM512/r2dreamer README (5x faster DV3 baseline; no demo support; recipe version unstated).
- Local: baselines/rl/full_env.py (gymnasium, action_repeat in env), R2DREAMER_V2_FIXES_EXPLAINED.md, PAPER_PLAN.md
  2026-08-10/11 entries (no-actor-BC rule; hardened-predicate nulls).
Not found / unverified: any GitHub issue matching our runaway; whether NM512/r2dreamer's DV3 baseline includes the
replay critic loss; DEMO3's exact torch/gymnasium pins; the 7 h/1M DreamerV3 wall-clock source.
