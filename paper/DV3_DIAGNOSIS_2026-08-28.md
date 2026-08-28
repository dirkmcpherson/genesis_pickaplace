# Why dv3 does not ignite where r2dreamer does — diagnosis 2026-08-28

Fresh-context agent, read-only over both cluster trees (`$LAB/dreamerv3-torch`, `$LAB/r2dreamer`) and
every existing dv3 genesis log. User: "we really do expect dv3 to work and I'm consistently confused why
it does not." Verified non-bugs first, then ranked suspects, then what was launched.

## 0. Verified NOT the problem
Reward path is consistent (demo terminal 100 == online `genesis_reward_scale` 100; `run.log` "Total reward
of demos: 6700"); action convention consistent (backward row in `simulate` and `to_dreamer_native.py:17`);
is_terminal/TimeLimit handling correct (`wrappers.py:13-22`); shaping applied per decision in both
learners; no observation staleness. The world model itself trains fine (image loss 1800→49, kl→19,
reward_loss→0.002).

## 1. Side-by-side (the differences that matter)

| knob | r2dreamer (works, dense) | dv3 (all existing runs) |
|---|---|---|
| critic target | imagined λ-return **clamped at 100** (`return_clamp: 100`, `dreamer.py:502-503,552-553`) **+ replay λ-return loss** (`repval` 0.3) | imagined λ-return only, **no clamp, no replay anchor** (`models.py:449-479`) |
| gradient updates per run | 1 update / 2 decisions → **373.5k @3M sim** | 1 / 4 decisions (train_ratio 256) → **19.5k @300k, 62.5k @1e6**; 5M runs 58k |
| training horizon | **400 sim = 100 decisions** (TL-1200 pilot went 0/2) | 600 sim (08-19); final-RR overlay **1200 = 300 decisions** |
| demo terminal | on the pick row | **12 decisions (48 sim steps) late** (`grant_slack_decisions: 12`) in every run that exists; fixed in the final-RR converter, which has never run |
| demo exposure | prefill ×4 dup + re-inject every 150k (≈10 % of ring, constant) | loaded once, sampled ∝ length: ~12 % → ~4 % share, no dup, no re-injection |
| actor gradient / entropy | REINFORCE, entropy 3e-5 | dynamics backprop, entropy 3e-4 |
| model | deter 2048, hidden 256, depth 16, lr 4e-5 | deter 512, hidden 512, depth 32, lr 1e-4/3e-5 |
| envs | 6 CPU worlds | 1 |
| eval | `actor.mode`, K-checkpoint best-of | `actor.sample()` labelled "deterministic" (mislabel) |

## 2. Ranked suspects
1. **Unbounded critic + reward-head leak at ×100 (most likely).** In every ×100 dv3 run `value_mean`
   climbs monotonically past the maximum attainable return: sparse 49–67 (dH), 152–167 (dR2D);
   dense **415–515** — with realized reward ≈ 0 and `imag_reward_mean` 0.1–1.0/step (leak fixed point
   ≈ 0.2/(1−0.997) ≈ 67). `EMA_095` follows, so a real +100 terminal is a blip in the normalized
   advantage; actor entropy collapses 8.8 → −1..−4. r2dreamer's own yaml header records the identical
   "lambda-return explosion past the 100 max" and its fix (clamp + repval); its `train/val` pins at 97–99.
   Discriminating experiment: clamp the target at 100 (3 lines) and rerun dense dH at 300k–1M.
2. **Update budget.** dv3 has never exceeded ~62k updates; r2dreamer's first picks appear at ~11–53k
   updates and are just as transient (0.20 → 0.00 → 0.33 across checkpoints). dv3's "transients" (N4)
   are the same phenomenon cut off. Experiment: train_ratio 512.
3. **Demo terminal 48 sim steps late** in all existing runs → the rewarded demo state is one the online
   agent can never visit. Experiment: the patched (grant-slack 0) sets — first use is the block queued today.
4. **Training horizon.** 24/67 dH demos exceed 150 decisions; r2dreamer needed 100. Experiment: TL 400.
5. Demo density decay (no dup/reinject). 6. Entropy 3e-4. 7. Actor-gradient style / model shape /
   shift-aug / 1 env. 8. Eval mislabel (measurement only).

Reframing that matters for the paper: **r2dreamer sparse never learned either** (s40/s41: 0.00 at every
checkpoint, val 0.2–0.9, entropy pinned). "r2dreamer works" = dense + clamp + repval + 373k updates +
best-of-K selection.

## 3. Launched 2026-08-28 (dreamerv3-torch tree; `cluster/patches/dreamerv3-torch-genesis_final_rr.patch` regenerated)
- `models.py._compute_target`: flag-gated `return_clamp` (default 0 = unchanged behaviour).
- `configs.yaml`: `genesis_dv3clamp` (clamp 100) and `genesis_dv3fix` (clamp 100 + train_ratio 512 + time_limit 400).
- `sbatch_genesis_final_rr.sh`: `EXTRA_CONFIGS`, `TAGSUF`, registry knob `extra_configs`.
- Jobs: dH dense × seeds 20–22 × {clamp, fix} at 1e6 env steps (6), plus the 3 untouched dH dense
  baselines at 5M (the dDP/dR2D baselines were cancelled until a recipe ignites).
- Readout: `value_mean` ≤ 100 and sustained `train_success_rate`, then the protocol eval. If `fix`
  ignites and `clamp` does not, suspects 2/4 carry weight; if both ignite, the clamp alone is the story.

## 4. Disclosure (added to AUDIT_INDEX as E8)
The matrix's world-model arm (r2dreamer) trains with a return-target clamp and a replay critic loss;
RLPD and dv3 do not. The "no TD-target clamp in the matrix" decision (08-23) was applied to RLPD only.
Any cross-learner mechanism claim must state this.

## 5. Touch/reach probe (user suggestion) — not useful here
`FullTaskEnv(scope='reach')` was added (+1/terminate at tool–can distance < 0.17 m). At reset the tool
frame is already 0.09–0.18 m from the can and a random policy triggers it in 1 step; finger–can
*contact* is reached by a random policy in 4/6 episodes. The arm starts adjacent to the can: grasp + lift
IS the task, there is no easier sub-goal with the same structure. The scope stays in the code, documented
as trivially satisfied.
