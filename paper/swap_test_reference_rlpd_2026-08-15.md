# SWAP TEST (cell A'): reference RLPD on OUR ManiSkill wrapper + OUR demos

Date: 2026-08-15. Status: BUILT + GATED, launch-ready. No long run launched.

## Why

Our in-house RLPD trainer (baselines/rl/train_rlpd.py, SB3-SAC machinery in
baselines/rl/rlpd_sac.py) FAILED its positive control: 0/3 seeds, flat 0.00
success through 300k steps on ManiSkill PickCube-v1 with 50 motion-planning
demos (runs rlpd_msctl_s{0,1,2}; handoff §27). Two suspects survive:

1. our trainer (rlpd_sac.py) is defective, or
2. our wrapper / demo-relabel path (ms_env.py) mis-specifies the MDP.

The swap test separates them: run the AUTHORS' reference implementation
(github.com/ikostrikov/rlpd, JAX) on OUR EXACT wrapper and OUR EXACT demo
data. Reference succeeds -> our trainer is definitively defective. Reference
also fails -> the wrapper/demo path is the prime suspect (and the published
implementation's credibility shields the trainer).

## PRE-REGISTERED BAR (written before any long run existed)

Reference RLPD is VALIDATED on this setup iff **>=2/3 seeds reach eval
success >= 0.50 by 300k env steps, n=10 episodes, decision points at 50k
multiples only**. Identical to the failed control's bar, for symmetry.
10k-interval evals are monitoring only and carry no decision weight.

## Setup

| piece | value |
|---|---|
| reference code | ~/workspace/rlpd_ref @ ikostrikov/rlpd HEAD (clone 2026-08-15) |
| venv | ~/workspace/rlpd_ref/.venv (NEW, pip-only, python 3.10) |
| stack | jax[cuda12] 0.4.35, flax 0.10.2, optax 0.2.4, tfp 0.25.0, gym 0.23.1, gymnasium 0.29.1, mani_skill editable -e ~/workspace/ManiSkill (same checkout as the control), sapien 3.0.2, numpy 1.26.4, torch CPU-only |
| env | swap_test/ms_adapter.py: old-gym API over ms_env.MSPickCubeEnv, IMPORTED unmodified from genesis_pickaplace |
| demos | ms_env.load_ms_demos (the control's own loader) -> reference Dataset; 50 motionplanning episodes, cut at first relaxed success, +1 terminal, mask=0 at terminal |
| trainer | swap_test/train_swap.py: mirrors train_finetuning.py; reference Adroit-sparse recipe |
| launch | swap_test/run_seeds.sh |

Reference hyperparameters (their README Adroit Binary sparse recipe, i.e.
`--config=configs/rlpd_config.py --config.backup_entropy=False
--config.hidden_dims="(256,256,256)" --utd_ratio=20 --start_training 5000`):
num_qs=10, num_min_qs=2, LayerNorm critics, hidden (256,256,256), discount
0.99, tau 0.005, all lrs 3e-4, init_temperature 1.0, target_entropy
-action_dim/2 = -2.0, backup_entropy False, batch 256, offline_ratio 0.5
(128 demo + 128 online per minibatch), UTD 20 with one actor update per env
step, random-action warmup 5000.

Notable contrasts vs our failed control's config (same intent, different
knobs -- these are the reference's own choices, deliberately NOT ported):
UTD 20 vs our 10; hidden (256,256,256) vs our (256,256); warmup 5000 vs our
1000; JAX/tanh-Normal actor vs SB3's squashed Gaussian; no Q-watchdog.

## Documented deviations from stock train_finetuning.py

| id | deviation | justification |
|---|---|---|
| D1 | env + dataset swapped for our adapter/loader | the point of the test |
| D2 | max_steps 300k (their Adroit runs use 1M), decision evals at 50k multiples | bar symmetry with the failed control; 300k is inside their budget, and the bar only claims validation if it PASSES there |
| D3 | offline-batch rewarded-frame counter | instrumentation only (gate c); no behavioral change |
| D4 | eval protocol = the failed control's (fresh env, seeds 1e6+i, deterministic mode actions, n=10; success/native/grasp logged) | symmetry; the reference's own evaluate() computes the same headline number here (episode return == success under +1-terminal) |
| D5 | checkpoint at every 50k decision eval | reference gates checkpointing behind a flag; needed for the round-trip gate + post-hoc eval |
| D6 | d4rl/dmcgym/mj_envs imports dropped | those benchmarks are not installed; nothing in the SACLearner path uses them |
| D7 | torch installed CPU-only in the venv | mani_skill needs torch for tensor plumbing only (sim_backend=physx_cpu); keeps torch's cu126 nvidia pip libs from colliding with jax[cuda12]'s |
| D8 | jax 0.4.35 / flax 0.10.2 / tfp 0.25.0 instead of the repo's 2023-era `jax[cuda]>=0.3.13` pins | repo pins predate CUDA 12 wheels; this trio is the newest set whose APIs the repo code still uses (verified: no removed-API usage in the imported paths) |
| D9 | reward relabel is +1 terminal, NOT +100 | the failed control's actual relabel was +1 (demo_census.json: "reward = +1 on it, then done"), and the genesis RLPD arm is also +1 (train_rlpd.py:81); the swap must ingest byte-identical data. +100 terminal exists only in the r2dreamer arm's loader. NOTE for the record: handoff §27's cell-A diagnostic "actor_q_mean 0.39 vs +100 terminal" misstates the MS control's scale -- its max return is 1.0, so 0.39 is not the anomaly that line implies |
| D10 | `import tensorflow_probability.substrates.jax` pre-import shim in swap_test/{train_swap,gates}.py | tfp lazy-loads substrates; without full tensorflow installed, the reference's attribute-access pattern (`tensorflow_probability.substrates.jax`) raises AttributeError unless the submodule was imported first. Zero reference-code modification, zero numerics |
| D11 | nvidia-cuda-nvcc-cu12 pinned 12.6.85 | the 12.9.86 wheel is a namespace package (`__file__ is None`), which crashes jax 0.4.35's `_cuda_path()` at import; 12.6.85 is the last regular-package layout |

## Gates -- ALL PASS (2026-08-15, this box, cell B concurrently running)

| gate | requirement | result |
|---|---|---|
| (a) adapter smoke | obs (42,) f32 / act (4,) in [-1,1] as real gym Boxes; episodes truncate at exactly 100 with TimeLimit.truncated; random policy 0 success over >=3 eps; demo tapes terminate in success through the wrapper | **PASS**: Box(-inf,inf,(42,),f32) / Box(-1,1,(4,),f32); 3 random episodes all length 100, success 0; demo replay 3/3 solved (traj_0 seed0 @69, traj_1 seed1 @70, traj_2 seed2 @46, reward_sum 1.0 each -- frames consistent with the r2d-MS control's independent measurement) |
| (b) demo census through reference loader | episodes 50 / transitions 3440 / rewarded 50 / density 1.453488% -- EXACT match to rlpd_msctl_s0/demo_census.json | **PASS**: 50/50, 3440/3440, 50/50, 1.453488/1.453488 all MATCH; reward values exactly {0.0, 1.0}; mask==0 rows = 50 = episode count |
| (c) train smoke | 2-5k steps, no NaN, losses move, rewarded demo frames counted into training batches (>0), checkpoint save/load round-trip in a fresh process | **PASS**: 3000 steps (1000 warmup + 2000 trained), no NaN; critic_loss 0.40->0.005, temperature 1.0->0.58, entropy ~2.7; rewarded demo frames in batches: 45 in the FIRST update, 74,500 cumulative by step 3000 (expectation 2560 offline samples/step x 1.453% = 37.2/step -- on the nose); fresh-process restore of checkpoint_3000: actor param checksum differs from init (8001.31 vs 7997.96), restored agent evals cleanly |
| (d) negative control | untrained policy evals 0.00 | **PASS**: eval @0 success=0.00 native=0.00 grasp=0.00 mean_len=100 (n=10); post-smoke @2000 also 0.00 (expected at 2k steps) |

Smoke wall time: 3000 steps in ~3 min => trained throughput ~17 env steps/s at
UTD 20 => a 300k-step seed is ~5h. Smoke logs: rlpd_ref/swap_test/smoke.log;
census artifact: swap_test/checkpoints/smoke/demo_census_through_reference_loader.json.

## Launch commands (3 seeds)

```bash
cd /home/j/workspace/rlpd_ref
# sequential (default; safest while cell B owns the card):
bash swap_test/run_seeds.sh
# parallel (all 3 at once):
bash swap_test/run_seeds.sh --parallel
```

GPU-memory guidance: cell B = 3x train_rlpd.py at ~380 MiB each until
~morning; total GPU ~1.5/12 GiB used at build time. Each swap seed runs with
XLA_PYTHON_CLIENT_PREALLOCATE=false (set inside run_seeds.sh) and small MLPs
(a few hundred MiB per JAX process under on-demand allocation). Parallel-3
fits the card; the binding constraint is CPU (3 genesis sims + 3 physx_cpu
sims + JAX host loops). Sequential (~5h/seed, ~15h total) is the safe default
while cell B runs; parallel (~5-7h total) once cell B exits.

## Second-eyes observations on our wrapper/demos (part of the point)

1. Nothing in ms_env.py smelled wrong en route. The demo tapes replay to
   success through the wrapper (3/3 here, 5/5 in the r2d-MS gate), the
   predicate is single-sourced, the index map is derived live and asserted
   against recorded success flags. If the reference also fails, the next
   suspects inside the wrapper are semantic, not mechanical: terminate-on-
   relaxed (mid-motion terminal states -- only 11/50 demo cut frames are also
   native-success) and the 100-step horizon.
2. Handoff §27's "actor_q_mean 0.39 vs +100 terminal" is a units error: the
   MS control's max return is 1.0 (one +1 then terminate). Q=0.39 against a
   solved-policy value of ~0.5-0.6 (gamma 0.99, ~50-70 steps to reward) is
   NOT the smoking gun that line reads as. The flat grasp~0 is the real
   symptom.
3. If the reference PASSES, note it does so with UTD 20 (ours 10), 3-layer
   critics (ours 2), and warmup 5000 (ours 1000). "Trainer defective" then
   still spans code bug vs config gap; a follow-up ablation (reference at
   UTD 10 / 2-layer) would separate those cheaply with the working oracle.

## Interpretation table (pre-registered)

| reference result | reading |
|---|---|
| >=2/3 seeds >=0.50 at a 50k decision point | our trainer is definitively defective; genesis RLPD rows inherit the trainer caveat; bug hunt moves inside rlpd_sac.py with a working oracle to diff against |
| <=1/3 seeds, same flat-0 signature | wrapper/demo-relabel becomes prime suspect; next probe is the reference repo on ITS OWN benchmark (adroit) in this venv, to rule the venv in/out |
| mixed (1/3 with real success curve) | rerun-with-more-seeds decision goes to the user; do not silently extend the bar |
