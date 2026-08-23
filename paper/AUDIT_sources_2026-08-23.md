# Source-fidelity audit: our learners vs their published / reference implementations

Date: 2026-08-23. Auditor: read-only (no files changed, nothing launched).
Trees read: `/home/travel/workspace/genesis_pickaplace` (sparse checkout, *.py/*.md/*.sh/*.json
only) and `/home/travel/workspace/dreamerv3-torch-genesis` (local copy of the dv3 fork — NOTE: it
is NOT the cluster tree; it lacks the `genesis_pick_msrecipe_shaped` overlay the round robin used).
References were fetched live (URLs given per section). Prior internal audits
(`paper/rlpd_literature_comparison_2026-08-13.md`, `paper/rlpd_audit_2026-08-14.md`,
`paper/AUDIT_impl_2026-08-22.md`) were read and then re-checked against the sources; where I
agree I say so briefly and add only what is new, where I disagree or can sharpen I say why.

Classification key: **ID** = intentional & documented in repo; **IU** = intentional & undocumented
(or documented only as "our choice" without the source value); **PB** = probable bug / oversight.
"Observed behaviour" targets: RLPD ignites ~50 % of seeds on human demos and 0/6 on DP-harvested
demos with critic divergence; DreamerV3 rarely ignites within 300k env steps; r2dreamer ignites
8/34 seeds sparse and 4/4 with dense shaping.

---

## 1. RLPD (Ball, Smith, Kostrikov, Levine, ICML 2023)

### (a) Sources consulted
- Paper: https://arxiv.org/abs/2302.02948 (Tables 1–2 via https://ar5iv.labs.arxiv.org/abs/2302.02948).
- Reference code: https://github.com/ikostrikov/rlpd — `README.md` (launch commands),
  `configs/td_config.py`, `configs/sac_config.py`, `configs/rlpd_config.py`,
  `rlpd/agents/sac/sac_learner.py`, `train_finetuning.py`, `rlpd/data/binary_datasets.py`
  (all fetched raw from `main`).
- Ours: `baselines/rl/rlpd_sac.py`, `baselines/rl/train_rlpd.py`, `baselines/rl/train_sacfd_full.py`,
  `baselines/rl/full_env.py`, `cluster/sbatch_rlpd.sh`.

Reference Adroit command (the sparse-manipulation setting of the paper):
`train_finetuning.py --env_name=pen-binary-v0 --utd_ratio=20 --start_training 5000 --max_steps 1000000
--config=configs/rlpd_config.py --config.backup_entropy=False --config.hidden_dims="(256, 256, 256)"`.
`rlpd_config` = `sac_config` + `num_qs=10, num_min_qs=2, critic_layer_norm=True`; `sac_config` =
`td_config` (actor/critic lr 3e-4, hidden (256,256), discount 0.99, tau 0.005) + `temp_lr 3e-4,
init_temperature 1.0, target_entropy=None (→ −dim/2), backup_entropy=True (overridden False on CLI)`.
`train_finetuning.py` flags: `batch_size 256, offline_ratio 0.5, start_training 1e4 (README uses 5000),
utd_ratio 1 (README 20)`; offline half interleaved `[0::2]`, `update()` slices `utd_ratio`
mini-batches, critic per slice, actor + temperature once on the last slice.

### (b) Ours vs reference

| knob | ours (cluster, `sbatch_rlpd.sh:199-205`) | RLPD ref (Adroit) | class |
|---|---|---|---|
| offline/online batch | 128/128, fresh draw per grad step (`rlpd_sac.py:243-249`) | 128/128, sliced from one interleaved 256·UTD batch | match |
| ensemble E / min-subset Z | 10 / 2, target critics, redrawn per step (`:202-203,:258-260`) | 10 / 2 (Adroit) | match |
| UTD (critic) | **10** (`train_rlpd.py:55`, `sbatch_rlpd.sh:202`) | **20** | ID (RLPD_PLAN.md §Compute: budget) |
| actor/α update cadence | once per env step, last grad step (`rlpd_sac.py:283-301`) | once per `update()` on last slice | match |
| entropy backup | OFF (`:269-271`, default False) | OFF for Adroit/AntMaze/pixels | match (fixed after 08-14 audit) |
| γ | **0.998** (`train_rlpd.py:99`) | **0.99** | ID (horizon argument, `train_rlpd.py:99-102`) |
| critic hidden | **(256,256) + LN** (`rlpd_sac.py:345`) | **(256,256,256) + LN** (Adroit/AntMaze) | IU (PLAN says "256x256", no source value) |
| actor hidden | **(256,256)** | **(256,256,256)** (same `hidden_dims`) | IU — prior audit listed only the critic |
| LayerNorm affine | **shared across the 10 members** (`:114-115`, `--per-member-ln off` in `sbatch_rlpd.sh:201`) | per-member (independent param trees) | IU — known since 08-14, deliberately left off for checkpoint byte-identity; the round robin still ran with it |
| learning_starts / random warm-up | **1000** (`rlpd_sac.py:359`) | 5000 (README) / 1e4 (flag default) | IU (PLAN lists 1000, no source value) |
| lr (actor/critic/α) | 3e-4 / 3e-4 / 3e-4 | 3e-4 ×3 | match |
| α₀ / target entropy | 1.0 / −dim/2 = −3.5 (`:351`) | 1.0 / −dim/2 | match |
| τ / target update | 0.005 every grad step (`:303`) | 0.005 inside each critic update | match |
| critic loss reduction | Σ_E mean_B (`:275-276`) | mean over E,B | benign (Adam) |
| actor objective | ensemble-mean Q over the MIXED batch (`:294-295`) | same | match |
| online buffer | 300k (`:358`), 100k steps | capacity = max_steps | match |
| online step budget | **100k** (cluster), 200k pilot | **1e6** (Adroit README), 300k AntMaze | IU — budget is 10× below the reference Adroit budget |
| offline data | 91 human tapes (dH) / 93 DP tapes incl. 30 fails (dDP) / 66 successes (dR2D); terminals at pick `done=True`; fail tapes whole, `done=False` | Adroit: human demos + **BC-policy rollouts incl. failures**, `remove_terminals=True` → every offline transition `mask=1` (`binary_datasets.py`) | see §1c |
| offline action clip | clip to ±1 at encode (`train_sacfd_full.py:110-115`) | clip to ±(1−1e-5) | n/a (SB3 never takes log-prob of buffer actions) |
| episode horizon / reward | 900 sim steps train, 400 eval; +1 terminal on pick, terminate | Adroit: 100–200 steps, +1 **per solved step**, no termination on success | task design (ID in lit doc) |
| action repeat / control rate | 1 @ 30 Hz joint-delta (cap 0.025) | n/a (Adroit 1) | ID |

### (c) Deviations that matter, classified

1. **γ = 0.998 + unterminated 1200-frame fail tapes (dDP) — IU (γ documented; the combination is not).**
   The reference also feeds *failed* rollouts into the prior buffer (Adroit BC data, masks all 1), so
   "failure tapes in the offline half" is not itself a departure from RLPD. What is different is the
   loop gain: a bootstrapped zero-reward chain of length L with no terminal anchor has spectral radius
   γ; at γ=0.99 and L≈100–200 (Adroit) the chain cannot sustain overestimation, at γ=0.998 and L=1200
   (`m1all_harvest` fails kept at the cap, 51 % of the buffer) it can (0.998^1200 = 0.09, vs
   0.99^200 = 0.13 but with 6× more steps to compound). `train_sacfd_full.py:89-91` emits the whole
   tape with `done=False`; `:286-288` flips `done` only on rewarded frames. This is the same
   mechanism AUDIT_impl F1 describes; the source comparison adds that the *reference tolerates failure
   data* — the paper's design assumption is short horizon + γ=0.99, which we violated, not "no fails".
   Plausibly explains the dDP critic divergence (Q 12–26k at 100k) and ordering dR2D < dH < dDP.
2. **True Q is bounded in [0,1] on this task and nothing uses that.** With a single +1 terminal and no
   other reward, the discounted return ∈ [0,1], so a target clamp `target_q.clamp_(0, 1)` (or
   `[0, 1/(1−γ)]` for the hold/shaped variants) is sound (does not move the fixed point) and would make
   the observed Q=12,900 impossible. Neither SB3 nor the RLPD reference does this, so it is **not a
   fidelity bug**, but it is the cheapest principled guard against exactly the failure seen (PB-adjacent
   omission). The one-shot/10k-throttled watchdog (`rlpd_sac.py:318-324`) only prints.
3. **2-layer actor AND critic vs the reference's 3-layer for sparse domains — IU.** `make_rlpd`
   `net_arch=(256,256)` feeds both `pi` and `qf` (`rlpd_sac.py:370-373`). Paper Table 2: Adroit and
   AntMaze use 3 layers. Low expected effect; free to align.
4. **Shared LayerNorm affine — IU, known, not fixed in the round robin.** `sbatch_rlpd.sh:201` passes
   `--per-member-ln off`. The reference's 10 critics are independent parameter trees. Effect on
   ensemble diversity (the min-of-2 pessimism that is supposed to stop divergence) is unmeasured; the
   divergence in dDP happened with this on. Worth an A/B before any "RLPD fails on model demos" claim.
5. **UTD 10 vs 20, budget 100k vs 1e6, learning_starts 1000 vs 5000 — ID/IU.** Each halves or
   decimates the amount of critic fitting relative to the reference; combined with γ=0.998 (5× the
   reference's effective horizon) the value propagation per unit compute is ≥20× slower than the
   published setting. This is consistent with "~50 % of seeds ignite by 100k" rather than anomalous.
6. **Demo terminal predicate weaker than env and one frame early** (`train_sacfd_full.py:58`
   `picked_f = (can_z[:-1] > pick_z) & (grip[:-1] > GRIP_CLOSED_FRAC)` vs env's sustained/EEF-distance
   pick) — PB, already AUDIT_impl F2; arm-independent. The reference labels offline reward from the
   dataset's own per-step reward, so there is no analogue — but it means our demo +1 sits at a state
   the env never rewards.
7. **Offline half sampled uniformly per transition** (`rlpd_sac.py:185`) — matches the reference
   (`Dataset.sample`), and it is why the 30 long dDP fail tapes dominate the offline half; stating
   "fail share of buffer" per arm in the paper is the right disclosure (already done in RESULTS).
8. Eval horizon 400 vs train 900 vs DP eval 1200 (`train_rlpd.py:109,148`, `sbatch_rlpd.sh:225,232`,
   `sbatch_dp.sh:384-386`) — F13/F14 of AUDIT_impl; not a source issue but must be captioned.

### (d) Explanatory power for observed behaviour
- "~50 % of human-demo seeds ignite": in-family given UTD/budget/γ deviations (#5) and per-step vs
  terminal reward (task); nothing in the port contradicts the paper's algorithm.
- "0/6 on DP demos with critic divergence": #1 (+ F1/F4) is the primary, source-grounded mechanism;
  #4 is a plausible amplifier; #2 is the cheapest fix. The reference would have run this data at γ=0.99
  on 100–200-step tapes, where the same tapes are harmless.

---

## 2. SACfD-style demo relabel (`train_sacfd_full.py`, `train_sacfd.py`, `demo_buffer.py`)

### (a) Sources
- SAC: Haarnoja et al. 2018, https://arxiv.org/abs/1801.01290 (and SB3 2.8 defaults).
- DDPGfD: Vecerik et al. 2017, https://arxiv.org/abs/1707.08817 (demos kept **permanently** in replay,
  prioritized replay with demo bonus, 1-step + n-step (n=10) mixed targets, L2 regularisation,
  multiple learner steps per env step).
- RLPD (above) for the "demo half of every batch" contrast.

### (b) Ours vs sources

| knob | ours | source | class |
|---|---|---|---|
| demo persistence | demos injected ×3 into a 300k FIFO (`demo_buffer.py:96-128`, `train_sacfd.py:43-64`); all copies evicted after 300k online steps (`--steps 400000` default in `train_sacfd_full.py:410`) | DDPGfD: demos never evicted; RLPD: immutable 50 % half | IU — docstring says "diluted as fresh experience streams in" but not that they vanish entirely |
| prioritisation | none (uniform); ×3 duplication only | DDPGfD: PER with ε_D demo bonus | ID (`train_sacfd.py:10-13`) |
| n-step | 1-step only | DDPGfD: mix 1-step + 10-step | IU |
| γ | 0.98 in `build_model` default (`train_sacfd.py:43`), 0.998 via `train_sacfd_full --gamma` | SAC 0.99 | ID (documented "silent killer"; every early SACfD wave ran 0.98) |
| learning_starts | 100 | SAC 1e4 (SB3 100) | ID |
| target entropy | SB3 'auto' → −dim = −7 | SAC −dim | match (RLPD uses −dim/2; the two arms differ here — IU) |
| demo terminal | `done=True` at relabelled pick, no post-terminal drop (`:286-291`) | n/a | PB (F3) |
| stage gating | reward only if env-measured episode stage ≥ stage (`:53-88`) | n/a | ID, sound |

### (c) Notes
- The "SACfD = 0 in ~30 runs" baseline is not a faithful DDPGfD/SACfD baseline: no permanence, no
  PER, no n-step, γ=0.98 for most of those runs. The RLPD > SACfD contrast should be captioned as
  "RLPD > naive demo-seeded SAC", not "RLPD > SACfD".
- `demo_buffer.load_demo_transitions` (pick-scope legacy) uses the *state* gripper (`s[:,6]`) while
  `relabel_full` uses the *commanded* gripper (`a[:,6]`) — two predicates for one event (the repo's own
  "one definition" rule, `full_env.py:60-75`). PB, low impact today (legacy path).

---

## 3. Diffusion Policy (Chi et al. 2023) via lerobot

### (a) Sources
- Paper: https://arxiv.org/abs/2303.04137 (PDF v5; §5.2: "All state-based tasks are trained for 4500
  epochs", position control, action horizon 8 "optimal for most tasks").
- lerobot fork used (`dirkmcpherson/lerobot@genesis-fixes`):
  `src/lerobot/policies/diffusion/configuration_diffusion.py` (fetched), `modeling_diffusion.py`
  (fetched: no EMA anywhere; obs/action deques; `num_inference_steps=None → num_train_timesteps`).
  Upstream `main` has since moved to `horizon 64 / n_action_steps 32` — the fork is on the classic
  16/8 defaults, which is what our runs used.
- Ours: `cluster/sbatch_dp.sh:348-353` (train flags), `:384-386` (eval), `baselines/dp_runner.py`,
  `baselines/convert_to_lerobot.py`.

### (b) Ours vs sources

| knob | ours | DP paper (state-based) | lerobot fork default | class |
|---|---|---|---|---|
| n_obs_steps / horizon / n_action_steps | 2 / 16 / 8 (defaults, nothing overridden) | 2 / 16 / 8 | same | match |
| control rate → time horizon | **30 Hz** → 0.53 s prediction, 0.27 s replan | 10 Hz → 1.6 s / 0.8 s | — | IU (only the dHpruned "idle collapse" hints at it) |
| diffusion | DDPM 100 train / 100 inference, squaredcos, ε-pred, clip ±1 | 100/100 (sim) | same | match |
| normalisation | MIN_MAX state+action, env_state | min-max to [−1,1] | same | match |
| optimiser | AdamW 1e-4, betas (0.95,0.999), wd 1e-6, cosine 500 warm-up | same | same | match |
| **EMA** | **none** (fork has no EMA) | EMA 0.9999/power 0.75 on weights | none | IU (lerobot-inherited; not stated anywhere in repo) |
| batch size | **64** (`sbatch_dp.sh:353`) | 256 (low-dim) | 8 | ID (PAPER_PLAN) |
| training length | **100k steps** → dR2D ≈ 700 epochs, dHpruned ≈ 180, dH unpruned ≈ 130 | **4500 epochs** | 100k | IU — dH/dHpruned are 25–35× fewer epochs than the paper |
| checkpoint | last | best of periodic evals (robomimic protocol) | last | ID-ish (honest) |
| vision | none (state + privileged env_state) | state-based variant | — | ID |
| action space | absolute joint targets | position control (paper's best) | — | match |
| eval | 1200 steps, 30 eps one process (vs RLPD 400, fresh process) | — | — | PB in protocol (F13) |

### (c) Deviations and what they could explain
- Under-training vs the paper (epochs) is the biggest unstated gap, and it is *source-dependent*:
  the short dR2D tapes (9,184 frames) get ~700 epochs while unpruned human tapes get ~130. Part of
  "dR2D_DP 0.96 vs dH_DP 0.27" may be epoch count, not demo quality. Cheap test: train dH for
  5× steps (or match epochs across arms) before claiming a source effect for BC.
- No EMA: DP's clean curves in the paper rely on EMA weights; without it, the `last` checkpoint is a
  noisier draw. State it.
- 30 Hz horizons: the paper's receding-horizon argument is in seconds; at 30 Hz our 8-step chunk is
  0.27 s, short enough that idle teleop frames dominate the human chunks (which is why pruning helped).
  Worth one sentence in the paper.

---

## 4. DreamerV3 (Hafner et al. 2023) — `dreamerv3-torch-genesis` (fork of NM512/dreamerv3-torch)

### (a) Sources
- Paper: https://arxiv.org/abs/2301.04104 (v2, 2024-04); HTML https://arxiv.org/html/2301.04104v2
  (γ 0.997, imag horizon 15/16, η 3e-4, free nats 1, β_rep 0.1, return-norm 5–95 pct EMA 0.99;
  note v2 changed several values (β_dyn 1, replay critic loss 0.3, lr 4e-5) that NM512 does not have).
- Upstream defaults: https://raw.githubusercontent.com/NM512/dreamerv3-torch/main/configs.yaml
  (batch 16×64, train_ratio 512, precision 32, no `dyn_gru_blocks`, discount 0.997, kl_free 1.0,
  dyn_scale 0.5, rep_scale 0.1, unimix 0.01, actor entropy 3e-4, model lr 1e-4, actor/critic 3e-5).
- Ours: `configs.yaml` (defaults :1-135, `genesis_pixels` :370-421, `genesis_pick_msrecipe` :423-446),
  `dreamer.py:51-52,430-433,486-599`, `tools.py:495-540` (sampler), `envs/genesis.py`,
  `convert_genesis_demos_repeat.py`.
- No published DreamerV3-with-demonstrations recipe exists; the demo injection is the project's own
  (MS recipe). The closest published analogue for "demos as replay data" is RLPD's symmetric sampling.

### (b) Ours (genesis_pixels + genesis_pick_msrecipe) vs upstream/paper

| knob | ours | upstream NM512 / paper | class |
|---|---|---|---|
| discount | 0.997 | 0.997 | match |
| imag_horizon / λ | 15 / 0.95 | 15 / 0.95 | match |
| unimix / free nats / KL scales | 0.01 / 1.0 / 0.5, 0.1 | same | match |
| actor entropy / lr | 3e-4 / 3e-5 | same | match |
| encoder symlog_inputs, critic symlog_disc, reward symlog_disc | same | same | match |
| batch | 16×64 (msrecipe overlay; fork *defaults* changed to 32×96, `configs.yaml:78-79`) | 16×64 | match (overlay) |
| train_ratio | **256** (`:439`) → one update per 4 decisions | 512 (DMC), 1024 (Atari100k) | ID ("MS update intensity") |
| precision | **16** (fork default, `configs.yaml:22`) | 32 | IU — AMP in the whole WM/AC (`models.py:33,260`) |
| RSSM GRU | **BlockGRU, 8 blocks** (`configs.yaml:46`, `networks.py:67-75` "Experimental") | single GRU (NM512); block-diagonal GRU is a DreamerV3-v2 feature | IU |
| action_repeat / time_limit | 4 / 600 sim = 150 decisions | DMC 2 / 1000 | ID |
| env steps | 3e5 → **75k decisions → ≈18.75k gradient updates** (+100 pretrain) | DMC 1e6/rep2/ratio512 ≈ 250k updates; Atari100k ≈ 100k | ID in steps, **IU in updates** |
| reward | +100 terminal (`envs/genesis.py:185`, `genesis_reward_scale`), demos carry +100 | symlog/twohot are scale-robust; paper rewards are native | ID (MS parity) |
| demos | loaded into `train_eps` (`dreamer.py:566-568`), sampled **length-weighted uniform** with everything else (`tools.py:502-507`), no duplicate / reinject / fixed ratio | no demos in paper; RLPD: fixed 50 % | IU — demo share decays to ≈10 % of samples by 300k (67×~130 decisions vs 75k online) |
| demo terminal placement | +100 and `is_terminal` **12 decisions (48 frames) after** the relabelled pick grant (`convert_genesis_demos_repeat.py:118-126,185-193`) | env terminates at the hardened pick (≈10 frames after grant) | IU — reward/cont labels for "lifted, holding" states differ between demo and online data by ~38 frames |
| truncation | `is_terminal=False` at time limit, cont head bootstraps | same | match |
| eval | out-of-process, `eval_action_mode` sample/mode (fork-only knob) | mode | ID |

### (c) Deviations and what they could explain ("rarely ignites within 300k")
1. **Update budget is ≈0.6× the project's own ManiSkill takeoff.** The dv3 MS reference took off at
   110–137k env steps at action_repeat 1 and train_ratio 256 → 27–34k updates
   (`paper/r2d_ms_control_2026-08-15.md:158-168`); genesis 300k env steps at repeat 4 = 75k decisions
   → 18.75k updates. ROUND_ROBIN_RESULTS §6 compares *env steps* ("2.2–2.7× the MS takeoff window");
   in gradient updates the genesis budget is below the takeoff point. This alone predicts "rarely
   ignites by 300k" and is the first thing to state (and to extend: 600k env steps ≈ MS takeoff).
2. Demo dilution with no counter-measure: RLPD's central empirical claim is that a *fixed* demo share
   matters; r2dreamer got `demo_duplicate 4 + reinject` for the same reason; dv3 has neither. Cheap
   lever: up-weight demo episodes in `sample_episodes` (or duplicate keys) — IU today.
3. Demo/online label inconsistency at the pick (grant slack 12 decisions): the reward head sees
   "lifted+closed → 0 for 12 decisions then +100, cont=0" from demos and "lifted → +100, cont=0"
   online. Not fatal (value of the lifted state is ≈0.997^12·100 either way) but it is a clean,
   unnecessary inconsistency; an honest sentence or a re-encode with the env predicate fixes it.
4. precision 16 / BlockGRU-8: both undocumented departures from NM512; unknown effect; the MS
   positive control ran with the same fork so they are not *the* cause, but they belong in the
   appendix config table.

---

## 5. r2dreamer (R2-Dreamer, Morihira et al., ICLR 2026) — cluster-only tree

### (a) Sources
- Paper: https://arxiv.org/html/2603.18202v1 (defaults: batch 16×64, lr 4e-5, imag horizon 15,
  **discount horizon 333 (γ=0.997)**, λ 0.95, actor entropy 3e-4, free nats 1, unimix 1 %, replay
  5e6, **no demonstrations, dense-reward benchmarks only**).
- Repo: https://github.com/NM512/r2dreamer (our port = tarball `cluster/r2dreamer_port.tar.gz`
  against commit 546e4fa — **not in this sparse checkout**, so only documentation could be audited:
  `cluster/R2DREAMER_CLUSTER.md`, `R2DREAMER_V2_FIXES_EXPLAINED.md`, `cluster/sbatch_r2dreamer.sh`,
  `paper/r2d_ms_control_2026-08-15.md` §5, `paper/ROUND_ROBIN_RUNNING_2026-08-19.md`).

### (b) Documented genesis champion (`genesis_pick_v5d4c_delta[_shaped]`) vs R2-Dreamer defaults

| knob | ours (as documented) | R2-Dreamer default | class |
|---|---|---|---|
| rep loss | r2dreamer (decoder-free BT) | same | match |
| batch / lr / optimiser | 16×64 / LaProp 4e-5 | 16×64 / 4e-5 | match |
| discount | **conflicting docs**: `horizon 1000→333` (V2_FIXES:21, `_base_` 333 per r2d_ms_control:162) vs "discount 0.999" (ROUND_ROBIN_RUNNING:75,103; shaping hard-codes 0.999) | 333 (0.997) | **unverifiable here; PB if 0.997 vs 0.999 mismatch** (AUDIT_impl F8) |
| actor entropy | **3e-5** | 3e-4 | ID (V2_FIXES §1 "entropy ratchet") |
| actor dist | `bounded_normal_clipped` | `bounded_normal` | ID (GENESIS_PORT_STATUS) |
| reward_scale / return_clamp | 100 / 100 | 1 / off | ID (MS parity) |
| replay | 450k FIFO, demo_duplicate 4, reinject every 150k, pretrain 1000 | 5e6, no demos | ID |
| action_repeat / time_limit | 4 / 400 sim = 100 decisions | 1–2 | ID |
| budget | 3M env steps = 750k decisions | 1M–? | ID |
| eval protocol | best-of-~30 checkpoints, 3 confirm seeds + 1 mode | last checkpoint | ID but mixed with other algos' single-checkpoint numbers (F19/F22) |

### (c) What can/cannot be audited from this box
- CAN: launch flags (`sbatch_r2dreamer.sh:234-240`), the adapter-boundary shaping harness
  (`paper/shaping_livefire_2026-08-19/record_r2d_trace.py`: `GenesisPick(... reward_scale=100,
  pick_shaping=...)` and `phi = env._env._pick_phi()` — i.e. the adapter reuses FullTaskEnv's φ but
  applies its own `0.999·φ' − φ`).
- CANNOT: `configs/env/genesis_pick_v5d4c_delta*.yaml` (discount!), `envs/genesis.py` of the port,
  `replay_gate.py`, the `add_chunk` demo packing, `train_ratio`, whether `is_terminal` on the demo
  tapes uses the same 48-frame grant slack (`to_dreamer_demos.py --grant-slack 48`, R2DREAMER_CLUSTER
  :143), and `eval_genesis.py` defaults (F21). **The γ question decides whether "policy-invariant" may
  be said for the headline dH_R2D dense 4/4**: one grep on the cluster tree.
- Observed 8/34 sparse vs 4/4 dense: all documented levers (entropy 3e-5, clipped actor, FIFO with
  reinject, +100 scale) are project-specific; nothing in the R2-Dreamer paper covers sparse reward or
  demos, so "r2dreamer on sparse pick" is an out-of-distribution use of the method and the ignition
  lottery is not evidence about R2-Dreamer per se. The dense arm removes the exploration lottery; if
  γ is mismatched the shaping also adds a distance-proportional per-step penalty (≈ −0.4·d per agent
  step at scale 100, see §6), which is *more* than a pure potential and may itself help ignition —
  another reason the invariance wording must be verified before it is printed.

---

## 6. Potential-based reward shaping (Ng, Harada, Russell 1999)

### (a) Sources
- Ng et al. 1999 (ICML), theorem: F(s,a,s') = γΦ(s') − Φ(s) is necessary and sufficient for policy
  invariance; the proof treats episodic tasks via an absorbing state.
- Grzes 2017, "Reward Shaping in Episodic Reinforcement Learning", AAMAS
  (https://www.ifaamas.org/Proceedings/aamas2017/pdfs/p565.pdf): with the usual V(terminal)=0
  convention, invariance additionally requires Φ(terminal)=0 (the shaped return differs from the
  original by γ^T Φ(s_T) − Φ(s_0)).
- Ours: `full_env.py:117-130` (constants, γ 0.998 / SCALE 2), `:276-280` (`_pick_phi`),
  `:337,:387` (φ reset), `:403-415` (applied once per `step()`), `:486-490` (terminate on pick),
  `:530-546` (tip termination); live-fire `paper/shaping_livefire_2026-08-19/VERDICT.md`.

### (b) Check against the theorem

| item | ours | theorem | class |
|---|---|---|---|
| form | `total_reward += γ·φ(s') − φ_prev` (`:413`) | γΦ(s') − Φ(s) | match |
| φ | −2·‖eef − can‖ (`:280`) | any bounded Φ | ok |
| γ matching | RLPD 0.998 = `PICK_SHAPING_GAMMA` (`:129`) = `train_rlpd --gamma` default; dv3 0.997 passed via `pick_shaping_gamma` (cluster tree, not here); r2d hard-coded 0.999 at adapter vs documented agent discount 333/0.997 | γ must equal the learner's discount | RLPD match; dv3 unverifiable locally; **r2d PB/unknown** |
| cadence | once per decision (post-fix), live-fire max_err 0 / 2.4e-7 | once per agent transition | match |
| truncation | SB3 bootstraps timeouts; dv3/r2d `is_terminal=False` | infinite-horizon form | match |
| **terminal potential** | φ(s_T) = −2·d_T ≠ 0 at pick (d_T = tool-to-can-centre at grasp, a few cm → bias −0.06…−0.2 on a +1 reward) and at tip (grip open, can far → up to −1) | Φ(terminal)=0 for exact invariance under V(terminal)=0 | **PB (IU at best)** — "exactly policy-invariant" is false; bias is *against* terminating and scales with eef–can distance, so it mildly penalises picks and strongly penalises tipping-while-far (a change of optimal policy in principle; benign direction in practice) |
| demo half | demos keep sparse rewards while online rewards are shaped (`train_rlpd.py:74-79`, `rlpd_sac.py:249`) | shaped MDP must be one MDP | IU, registered caveat; per-transition target error Φ(s)−γΦ(s') (O(1e-2)) with per-state offset up to |Φ|≈1 — same order as the task reward. FK of the recorded joints would let the demos be shaped consistently |
| hover math in comment | "any 400-step episode" (`:122-125`) | training episodes are 900 | comment error (F12) |

### (c) Consequence
The RLPD dense arm (4/6 ignite) and the r2d dense arm (4/4) should be described as "potential-based
shaping with a non-zero terminal potential (≤ a few % bias at the pick, larger at tip terminations)
and, for RLPD, sparse-labelled demos in half of each batch"; "exactly policy-invariant (Ng 1999)" is
not supportable as written, and for r2d not even the γ match is established.

---

## 7. Cross-cutting observations (source-grounded)

- Every learner here is run far below its reference compute in the unit that matters to that
  learner: RLPD 100k steps × UTD 10 (ref 1e6 × 20), DP 130–700 epochs (ref 4500), dv3 ≈19k updates
  (MS takeoff ≈30k, DMC ref 250k). "Source effects" measured at these budgets are effects on
  *early-training* behaviour; the paper should say so or show a budget sweep on one cell per learner.
- The reference RLPD prior data *contains failures* (BC rollouts); our dDP result therefore does not
  contradict Ball et al. — it extends their recipe to γ=0.998 / 1200-step unterminated tapes, where
  it breaks. That is a finding about horizon/γ × tape consistency, not about "model demos".
- Terminal handling differs from every reference in the same direction: the env terminates on success
  (Adroit/MS-like reference envs used here do not, RLPD's offline loader strips terminals), demos carry
  `done=True` at a weaker/earlier predicate (RLPD) or 48 frames late (dv3/r2d). Unify the pick
  predicate and terminal placement across the three demo encoders or table the differences.

---

## Ranked top-10 (change, or state in the paper)

1. **State (and ideally fix) the γ=0.998 × unterminated-fail-tape mechanism for dDP_RLPD**: Ball et
   al.'s own prior data includes failures but at γ=0.99 on ≤200-step tapes; ours bootstraps 1200-step
   fail chains at 0.998. Add an env-terminal guard (tip predicate → `done=True`) in
   `delta_encode_transitions` and clamp the TD target to the task's true range [0,1]
   (`rlpd_sac.py:271`) — both sound, cheap, and would have made Q=12,900 impossible.
2. **Re-run or caption dv3 in gradient updates, not env steps**: 300k env steps at repeat 4 /
   train_ratio 256 = 18.75k updates ≈ 0.6× the MS takeoff count; the "rarely ignites by 300k" result
   is budget-limited by construction. Extend to ≥600k or raise train_ratio; say so in the paper.
3. **Verify the r2dreamer discount (333/0.997 vs 0.999) on the cluster tree before printing
   "policy-invariant" for dH_R2D dense 4/4**; if mismatched, the dense arm also carries a
   distance-proportional per-step penalty (≈ −0.4·d/step at scale 100).
4. **Drop "exactly policy-invariant" everywhere**: φ(terminal) ≠ 0 (`full_env.py:403-415`) and the
   RLPD demo half is unshaped. Either zero φ on terminal transitions (`if terminated: phi = 0`) and
   shape the demos via FK, or state the biases (≤ a few % at the pick, up to −1 at far tips).
5. **Match DP training epochs across demo sources (or report them)**: 100k steps × 64 is ≈700
   epochs for dR2D but ≈130 for unpruned dH (paper: 4500). The BC "model > human" ordering is
   confounded with epochs per frame; also state the absence of EMA (lerobot fork) and the 30 Hz
   chunk horizon (0.27 s vs the paper's 0.8 s).
6. **Turn on per-member LayerNorm and 3-layer nets for RLPD (or justify)**: both are the reference's
   sparse-domain setting; the round robin ran the known-deviating shared-affine LN
   (`sbatch_rlpd.sh:201`), which is exactly the component meant to prevent the divergence observed.
7. **Give dv3 a fixed demo share** (duplicate/up-weight demo episodes in `tools.sample_episodes`) —
   RLPD's headline ablation says the 50/50 share is what makes offline data useful; dv3's demo share
   decays to ~10 % with no counter-measure while r2dreamer got duplicate+reinject.
8. **Unify the pick-terminal definition across encoders**: RLPD demo +1 at z+grip proxy one frame
   early (`train_sacfd_full.py:58`), dv3/r2d demo +100 twelve decisions *after* it
   (`convert_genesis_demos_repeat.py:118-126`), env at the hardened sustained pick. Three
   definitions of one event; pick the env's and re-encode, or list them in the appendix.
9. **Rename/caption the SACfD baseline**: it is demo-seeded SAC with FIFO eviction, no PER, no n-step,
   γ=0.98 for the historic runs — not DDPGfD/SACfD as published; "RLPD ≫ SACfD" must be worded
   accordingly.
10. **Per-learner budget table in the appendix** (env steps, decisions, gradient updates, epochs,
    wall-clock, eval horizon 400/1200/400/600, eval protocol last/best/periodic) — every cross-algorithm
    sentence currently mixes these; the table makes the comparisons honest without re-running anything.

### Verified-clean items (no action)
RLPD 50/50 per-grad-step batches, E=10/Z=2 on target critics, actor on ensemble mean, α₀=1,
target entropy −dim/2, τ 0.005, backup_entropy off, truncation bootstrapping (online) — all match
`ikostrikov/rlpd`. DP horizon/obs/action-steps/scheduler/normalisation match the fork and the paper.
dv3 core hyperparameters (γ, λ, imag horizon, unimix, free nats, KL scales, entropy, lrs, symlog heads)
match NM512; cont = 1 − is_terminal; shaping cadence once per decision is exact (live-fire 0.0e0).
