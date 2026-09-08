# What survives open-loop replay: demonstration re-execution is not free (2026-09-08)

*A standalone, citable result from the robomimic Can leg (`paper/ROBOMIMIC_PLAN_2026-09-05.md` amendment A4). It is
reported separately because it is a fact about **replaying recorded demonstrations**, not about human-vs-machine data,
and it bears directly on any design that plans to replay, retime, smooth, or discretise recorded actions — including this
project's own discrete-action line (`paper/DISCRETE_ACTION_REPLAY_2026-09-06.md`), which assumed re-execution was
approximately free. Every number below is from tool output; nothing is estimated.*

## 1. Setup
Environment: robosuite 1.5.1 / robomimic 0.5.0 / MuJoCo 3.3.7 (`$LAB/robo_venv`), the PickPlaceCan env built from each
dataset's own `env_args` (OSC_POSE, 20 Hz, `control_delta`, output caps 0.05 m / 0.5 rad). Data: robomimic v1.5 Can,
`low_dim_v15.hdf5` (PH/MH) and `low_dim_sparse_v15.hdf5` (MG, SAC-generated), sha256 recorded in
`$LAB/robomimic_data/v1.5/can/sha256.txt`.

**Replay protocol.** For each tape: `reset_to({"model": <the tape's model xml>, "states": states[0]})`, then step the
tape's recorded actions open-loop for the tape's own length T, re-reading the observation each step
(`env.get_observation()`); the tape counts as reproduced iff `env.is_success()["task"]` fires within T. Builder:
`baselines/robomimic/build_reexec_arms.py`; the same code, env instance and seed for every condition below.

## 2. Result 1 — reproduction rates differ sharply by generator (unmodified actions)
| source | tapes replayed | reproduced | rate |
|---|---|---|---|
| human, multi-operator (MH200) | 200 | **185** | **93 %** |
| machine, SAC checkpoints (MG200s, all successful rollouts) | 200 | **114** | **57 %** |

Same environment, same replay code, same success predicate. A separate check with the G0 gate protocol on 10 PH tapes
(single operator) reproduced the outcome flag on 10/10 with the world can position within 1 cm on 7/10.

**Why the machine tapes fare worse is measurable, not speculative:** MG actions command 4.6× larger OSC deltas than human
ones (mean |a| over the six arm dims 0.664 v 0.144), are 6.2× less smooth step to step (mean |a_t − a_{t−1}| 0.274 v
0.043), saturate 5× more often (12.2 % v 2.4 % of rows at |a| ≥ 0.99), and use a continuous gripper channel (1,903
distinct values; 1 % exactly ±1) where every human row is a binary ±1 toggle
(`$LAB/robomimic_data/arms/action_stats.json`). Large, saturated, high-frequency commands leave no margin: the
un-restorable controller state that open-loop replay cannot reproduce is a fixed *relative* error for every source
(one-step end-effector error / commanded step: median 0.57 PH, 0.58 MH, 0.56 MG), so the *absolute* divergence scales
with the commanded step (2.5 / 2.8 / 6.9 mm for file steps of 8.2 / 8.9 / 14.8 mm) and accumulates fastest in the
machine tapes.

## 3. Result 2 — any action edit collapses reproduction
Same protocol, actions modified before replay (arm-level parameter, gripper as stated):

| condition | modification | reproduced |
|---|---|---|
| MG200s + causal EMA, β 0.87 | smoothed to human roughness (\|Δa\| 0.043), gripper binarised at 0 | **0 / 200** |
| MG200s + causal EMA, β 0.87, magnitude restored | as above, per-tape rescale to the original mean \|a\| | **0 / 200** |
| MG200s + causal EMA, β 0.30 (mild) | \|Δa\| 0.139 (still 51 % of the original roughness) | **3 / 40** |
| MG200s + causal EMA, β 0.50 / 0.70 / 0.86 | \|Δa\| 0.103 / 0.069 / 0.040 | **0 / 40** each |
| MH200 + uniform noise, ε 0.406 | roughened to MG's \|Δa\| 0.274 | **5 / 200** |
| MH200 + uniform noise, ε 0.20 | \|Δa\| 0.146 (53 % of MG's) | **63 / 200** |
| MH200 + uniform noise, ε 0.15 | \|Δa\| 0.116 (43 % of MG's) | **98 / 200** |

Note the asymmetry: *smoothing* machine actions destroys them as completely as roughening does, and more completely than
roughening human actions by a comparable factor. A modified action sequence is a different trajectory, and an open-loop
replay has no mechanism to correct back onto the recorded states.

## 4. Result 3 — the learner cost of training on a re-executed arm (upper bound)
An RLPD arm trained on the re-executed, success-filtered human tapes (`MH200_re15`: 95 tapes / 15,702 rows,
**unmodified** actions) scores **0.080** on the 50-state bank (mode, 8 seeds: 2, 8, 2, 5, 2, 0, 4, 9 / 50) against
**0.455** for the natively recorded MH200 (200 tapes / 41,134 rows). This falsified the pre-registered prediction that
re-executed control arms would sit within 0.10 of the native arm (`ROBOMIMIC_PLAN` A4 addendum, P-A4-3′).

**This −0.375 is an upper bound on the cost of re-execution, not an estimate of it:** the re-executed arm is also 62 %
smaller, and demonstration row count is itself a strong driver in this setting (machine arms: 16.5k rows → 0.147,
59.2k → 0.475, 536.5k → 0.610). Amendment A6 separates the two by adding a natively collected human arm at matched size
(`MH80`, 80 tapes / 16,406 rows, running at the time of writing); `MH80` − `MH200_re15` is then a clean estimate of the
re-execution cost at matched scale.

## 5. What to take from this into other work
1. **Budget for the build, not just the training.** A "replay the recorded actions with a small modification" design
   loses most of its data before any learning starts — here, between 51 % and 100 % of it depending on the edit.
2. **What survives is selected and smaller**, and both properties independently depress downstream performance; a
   re-executed arm is not a drop-in substitute for the arm it came from, and comparisons against native arms are
   confounded unless a size-matched native control is included.
3. **Reproduction rate depends on the generator's action statistics**, so the same replay design can be cheap for human
   teleoperation and near-impossible for policy-generated data recorded in the same environment.
4. **Test the yield first.** A 200-tape replay costs minutes of CPU here (≈ 3 s/tape) and answers whether the design is
   viable before any GPU time is spent.

## 6. Provenance
Builder and yields: `baselines/robomimic/build_reexec_arms.py`; logs `$LAB/robomimic_data/logs/build_A4_*.log`,
`ladder_A4_{MG,MH}.log`. Action statistics: `$LAB/robomimic_data/arms/action_stats.json`
(`baselines/robomimic/robo_common.py` layout constants). One-step error probe: `$LAB/robomimic_data/step_rel_err.py`.
Learner cells: `$LAB/robomimic_runs/rlpd/rlpd_MH200_re15_a4_s{0..7}/eval_bank50_mode/metrics.json` (bank sha
`72b75550…`, LAST checkpoint, 100k decisions). Registrations: `paper/ROBOMIMIC_PLAN_2026-09-05.md` §A4 and its addendum
(predictions written before the builds), §A6 (predictions written before the runs).
