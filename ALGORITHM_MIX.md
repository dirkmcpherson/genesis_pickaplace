# Algorithm Mix: BC + RL-from-Demonstrations

Plan of record for the learning-algorithm portfolio on the genesis pick-place-slide
task, and how each feeds the **ouroboros experiment** (human demos → policy →
model-demos → policy-of-model-demos; do differences emerge across generations?).

## Wave 1 — get results FIRST (running / next)

| algorithm | family | status | infra |
|---|---|---|---|
| **DP** (Diffusion Policy) | BC | state leg: picked 0.33 / contact 0.11 / nested 0.07. Image legs (top / top+wrist) evaluated 2026-07-25: img2 picked 0.20 w/ 100% pick→place conversion; img1 weaker | lerobot; `run_dp_img.sh`; datasets `lerobot_dataset_*` |
| **SACfD** | RLfD (off-policy) | JOINT pick champion: 0.55 mean / 0.93 best seed (ALL 91 demos incl. failures). **CARTESIAN: seed-once recipe FLAT 0.00** (h900 through 125k AND h1800 through 100k, 2026-07-26/27) -> RLPD-style variant (`train_rlpd.py`: immutable 50/50 demo buffer, LayerNorm critics, UTD 4) is the live cartesian RLfD leg | SB3; `train_sacfd_full.py` / `train_rlpd.py` |
| **DV3fD** (DreamerV3 + demo prefill) | RLfD (model-based) | plumbing audited clean (reward census 272 ✓, negative control 0); 5M-step / ~104k-update run on cluster (TAG-resume across 48h allocations) | `dreamerv3-torch` branch `genesis`; batched vec env 23 FPS |

Wave-1 deliverable: same-protocol comparison (15+ random ICs, x3 seeds where
feasible, stage metrics picked/placed/contact/nested, videos) of the three families
on human demos — then pick the ouroboros teacher(s).

## Wave 2 — queued behind wave-1 results

| algorithm | family | why | cost |
|---|---|---|---|
| **ACT** | BC | strongest DP competitor on manipulation; different inductive bias (chunking transformer + CVAE vs diffusion). Trains on the SAME converted datasets | ~zero: `--policy.type=act` in lerobot |
| **RLPD-style SACfD** | RLfD | fixes demo dilution (measured ~18% late-buffer in dv3; SACfD seeds once then demos wash out): separate demo buffer sampled 50/50 every batch + LayerNorm critics + UTD>1 | ~1 day mod to `train_sacfd_full.py` |
| plain MLP BC | BC | floor baseline only (exists in `bc/`) | free |

**Deliberately skipped:** GAIL/adversarial IL (unstable, sample-hungry, adds no
question RLPD doesn't), IQL/CQL offline RL (91 demos is too few), DDPGfD
(superseded by SACfD).

## Ouroboros integration

The harvest loop is teacher-agnostic (`harvest_ai_demos.py --teacher-type dp|sac`,
image teachers supported via rig probe, `--images` records camera obs for image
students). Cluster chain: `cluster/sbatch_ouro_train.sh` ⇄ `sbatch_ouro_harvest.sh`
(train+eval → negative-control-gated verified harvest → next-gen dataset → resubmit,
up to MAXGEN).

Matrix once wave 1 picks teachers:

- per-algorithm chains: DP→DP→…, SACfD→(BC of its demos)→…
- **cross-family**: BC-of-RL-demos vs BC-of-human-demos — RL demos are typically
  lower-entropy; this is the distribution-narrowing question the experiment probes.

## Honesty protocol (applies to every cell)

- identical eval: `eval_core` ICs, honest settled nested, full-length horizon
- x3 seeds for headline numbers; same-machine baselines only; idle box for official runs
- negative controls: random-teacher harvest (~0 kept), fail-demo stage rates
- every trained policy → eval videos to the user
- pre-register the wave-1 prediction: SACfD leads pick rate, DP leads
  downstream-stage conversion, DV3fD unknown pending 5M run (prior runs: world
  model converges, policy flat at ~21k updates)
