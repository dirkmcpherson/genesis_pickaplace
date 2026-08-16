# MS DENSE PAIR (cell A''): dense-reward discriminator, both trainers

Date: 2026-08-16. Written and committed BEFORE launch (pre-registration).

## Context

Both RLPD implementations have now FAILED sparse PickCube identically:
- OUR trainer (SB3, rlpd_msctl_s{0,1,2}): 0/3, 0.00 at every 50k point to 300k.
- REFERENCE trainer (ikostrikov/rlpd on our wrapper/demos, RLPD-ref-MS_s{0,1,2}):
  0/3, success AND grasp 0.00 at every 50k decision point to 300k
  (verified in rlpd_ref/swap_test/checkpoints/RLPD-ref-MS_s*/train.log).

The only ManiSkill positive we own (March dv3, ~1.0) trained on
NORMALIZED_DENSE online reward with success termination. Nothing has ever
passed sparse-PickCube-at-300k in this stack. The dense pair tests whether
reward mode, not trainer code, is the load-bearing variable.

## The change (new-param-only, shared-tree discipline)

`baselines/rl/ms_env.py` gains `reward_mode` on MSPickCubeEnv
('sparse' default | 'normalized_dense' = ManiSkill's own shaping passed
through; termination/truncation/success logic identical in both modes).
Plumbed as `--reward-mode` (train_rlpd_ms.py) and `--reward_mode`
(rlpd_ref/swap_test/train_swap.py). Demo tapes stay SPARSE-relabeled in both
modes -- dense-online + sparse-demo-tape is exactly the March dv3 positive's
mixture, so the mixture is proven workable. Sidecars/wandb config record
reward_mode. One knob follows the mode: our trainer's warn-only Q-watchdog
threshold goes 2.0 -> 2*horizon=200 under dense (2x max task return in both
cases; log-hygiene only, no behavior).

## Gates -- ALL PASS (2026-08-16, pre-launch)

| gate | result |
|---|---|
| (a) default-path regression | PASS: pre-edit file (git HEAD) vs edited file, same seed + same 120-action tape: reward/term/trunc streams IDENTICAL over the 100-step episode, max |obs diff| = 0.0. Dense mode on the same tape: rewards nonzero from step 1 (0.0471, 0.0429, ...; min 0.0112, max 0.0575), episode length and term/trunc flags identical, obs stream max diff 0.0 (reward mode does not perturb dynamics). Script: scratchpad/gate_a_regression.py |
| (b) dense smoke, OUR trainer | PASS: 2k steps, ep_rew_mean 4.14 -> 4.35 (dense stream live), actor_q_mean 0.45 -> 1.15, no NaN, negctl eval 0.00 |
| (b) dense smoke, REFERENCE | PASS: 2k steps, critic_loss 0.37 -> 0.005, q -0.36 -> +1.02 (dense scale), temperature 1.0 -> 0.75, no NaN, 45 rewarded demo frames in first update, negctl 0.00 |
| (c) demo census unchanged | PASS: both loaders: 50 eps / 3440 transitions / 50 rewarded / 1.453488% -- exact match to rlpd_msctl_s0/demo_census.json (demos untouched by the edit) |

## PRE-REGISTERED BAR (per trainer)

**>= 2/3 seeds reach eval success >= 0.50 by 300k env steps, n=10,
decision points at 50k multiples ONLY.** Same bar as both sparse controls.

Reference points: dv3 dense takeoff was ~110-137k steps; ManiSkill's own
dense-SAC baselines learn PickCube well under 1M steps.

## Interpretation table (pre-registered)

| outcome | reading |
|---|---|
| ref learns, ours doesn't | our trainer indicted (with a working same-MDP oracle to diff against) |
| both learn | our core machinery functional; cell A collapses to "sparse PickCube at 300k is too hard for every RLPD-class trainer we ran"; the genesis rows' caveat becomes about reward sparsity, not code |
| both fail | the dense wrapper path is suspect -- check our env config against ManiSkill's own SAC baseline config (control_mode, horizon, robot_uids) before concluding anything |
| ours learns, ref doesn't | unexpected; investigate reference-side adapter first (it is the newest code) |

## Launch (2026-08-16, GPU free -- sparse pair done, cell B finished)

3 + 3 seeds, all parallel. Memory precedent: ours ~444 MiB/proc, ref
~670 MiB/proc => ~3.4 GiB total, fits the 12 GiB card.
XLA_PYTHON_CLIENT_PREALLOCATE=false on the jax side.

```bash
# OURS (dv3 venv interpreter, as the sparse control):
cd /home/j/workspace/genesis_pickaplace
for s in 0 1 2; do
  out=baselines/rl/checkpoints/rlpd_msdense_s$s; mkdir -p $out
  setsid nohup /home/j/workspace/dreamerv3-torch/venv/bin/python \
    baselines/rl/train_rlpd_ms.py --steps 300000 --seed $s \
    --reward-mode normalized_dense --run-name RLPD-ours-MSdense_s$s \
    --out-dir $out --project genesis_paper > $out/train.log 2>&1 &
done
# REFERENCE:
cd /home/j/workspace/rlpd_ref
for s in 0 1 2; do
  out=swap_test/checkpoints/RLPD-ref-MSdense_s$s; mkdir -p $out
  XLA_PYTHON_CLIENT_PREALLOCATE=false setsid nohup .venv/bin/python \
    swap_test/train_swap.py --seed $s --max_steps 300000 \
    --reward_mode normalized_dense --run_name RLPD-ref-MSdense_s$s \
    --log_dir $out --notqdm > $out/train.log 2>&1 &
done
```

Launched PIDs + log paths recorded below after launch (the only post-launch
edit to this file).

## Launch record (2026-08-16, all six live)

Startup verified per seed: correct `reward=normalized_dense/relaxed` env line,
step-0 negative control 0.00, census gate MATCH (ref side), no tracebacks.
GPU total ~2.4 GiB (ours 444 MiB/proc, ref 354 MiB/proc under
XLA_PYTHON_CLIENT_PREALLOCATE=false).

| run | PID | log |
|---|---|---|
| RLPD-ours-MSdense_s0 | 2953049 | baselines/rl/checkpoints/rlpd_msdense_s0/train.log |
| RLPD-ours-MSdense_s1 | 2953052 | baselines/rl/checkpoints/rlpd_msdense_s1/train.log |
| RLPD-ours-MSdense_s2 | 2953054 | baselines/rl/checkpoints/rlpd_msdense_s2/train.log |
| RLPD-ref-MSdense_s0 | 2953180 | ~/workspace/rlpd_ref/swap_test/checkpoints/RLPD-ref-MSdense_s0/train.log |
| RLPD-ref-MSdense_s1 | 2953183 | ~/workspace/rlpd_ref/swap_test/checkpoints/RLPD-ref-MSdense_s1/train.log |
| RLPD-ref-MSdense_s2 | 2953185 | ~/workspace/rlpd_ref/swap_test/checkpoints/RLPD-ref-MSdense_s2/train.log |

wandb: project genesis_paper, run names as above. Decision reads at 50k
multiples only, from the train.log `[eval @...]`/DECISION-POINT lines.
