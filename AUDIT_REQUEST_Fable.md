# Audit request — work of 2026-07-28 → 07-30

Written by the assistant that did the work, for an independent audit. I have a
documented tendency this session to (a) launch parallel jobs on unverified edits,
(b) assert mechanisms before measuring them, and (c) add parameters whose defaults
silently encode semantics. Weight the audit accordingly: **the claims below marked
ASSERTED are the ones most likely to be wrong.**

Repos/branches: `genesis_pickaplace@4dof-cartesian`, `dreamerv3-torch@genesis`.
Everything reproducible on the dev box with `.venv-eval` unless noted.

---

## A. Claims I believe are VERIFIED — spot-check these

Each has evidence; the audit task is to confirm the evidence supports the claim.

| # | claim | evidence I have |
|---|---|---|
| A1 | Joint DP reaches **0.67 picked** in-distribution, 0.13 on random ICs | `baselines/eval_indist_joint.log`, 15 rollouts each |
| A2 | Every EEF encoding reaches ~0.00 in-distribution (vel/delta/abs/abs6/delta6, + ACT) | per-leg `eval_*.log`; table in `July30th_Fable.md` §2 |
| A3 | Batched cartesian == single-env cartesian | parity probe: mean 0.1 mm / max 0.7 mm tool-trajectory deviation, identical stage outcomes, 0.00 mm cross-env spread |
| A4 | Tip predicate fires correctly in both envs | unit tests: force-tipped can + open gripper → reward −0.5 (pre-change), `terminated=True`; upright unaffected |
| A5 | The real teleop commanded **only pitch** | `wx=wz=0` in 25/25 demos; roll/yaw drift is null-space, up to 1.03/1.34 rad |
| A6 | 4 datasets were metadata-truncated (pre-`finalize()`), rest clean | sweep in the 07-30 log; `pick_pruned` rebuilt 66/66 |
| A7 | dv3 `train_success_rate` counted **tipped** cans as successes | `tools.py` used `success = (length < time_limit)`; our env terminates on tip |

**A1 has a known defect:** that policy was trained on the **truncated** dataset (60
of 66 episodes). The number is real but the training set was ~9% smaller than
labelled. **Audit task: retrain on the rebuilt dataset and confirm 0.67 survives.**
It is the positive control the whole project now leans on.

---

## B. ASSERTED but NOT verified — highest audit value

### B1. The obs × action 2×2 (built today, never run)
`baselines/build_obs_action_2x2.py` emits 4 cells from one dual-representation
source. **I have not verified that any cell trains, or that the cross cells are
semantically correct.** Specific risks:
- `jobs_eact` pairs a 17-dim joint obs with abs6 actions — is `PROPRIO=8` the right
  split for that obs? (joint obs = 6 joints + grip + effort = 8 proprio, 9 world)
- `eobs_jact` pairs an 18-dim ee obs with joint targets — `PROPRIO=9`.
- abs6 actions are re-derived inside the builder rather than reused. **Check the
  derivation matches `derive_cartesian_realized.py --mode abs6`** (tool pose +
  rotvec relative to reset, action_i = pose reached at i+1).
- All four cells report identical frame counts (148,688). That is expected (same
  source, same pruning) but is also what a copy-paste bug would look like —
  **confirm the four datasets actually differ** in the intended dimensions.

### B2. `TIP_PENALTY = 0.0` reasoning
I argued termination alone provides a self-scaling implicit penalty (forfeited
future value), so a flat −0.5 was suppressing exploration when the agent was worst.
**This is an argument, not a measurement.** The overnight tip0 runs test it.
Audit: confirm all four sites changed together (`full_env.py`,
`genesis_vec_env.py`, `to_dreamer_demos_cartesian.py`, `relabel_cartesian.py`) and
that no demo carries a negative reward (I checked abs6/delta6 → min 0.0; **the
joint demo set and `episodes_cartesian_delta` were NOT rebuilt** — check whether
they are still stamped −0.5 and whether anything consumes them).

### B3. Dual-representation harvesting
Harvested demos now carry `states_joint`/`actions_joint`. Verified on **one
40-step rollout**, not on a full harvest or a downstream training run. Audit: is
`actions_joint` the command actually sent (`_last_joint_cmd`), and is it aligned
with `states_joint[i]` rather than off by one?

### B4. The 8-GPU night launcher
`cluster/launch_8gpu_night.sh` was dry-run only. Per-run `SEED` support in the dv3
sbatches is **new and unexercised** — confirm the three seeds in each multi job
actually diverge (compare their `train_return` curves; identical curves ⇒ the seed
never reached dreamer).

### B5. Evaluation-distribution change
`eval/*` changed meaning on 07-30 from random-IC to demo-IC. Audit: any comparison
in wandb spanning that date is invalid. Confirm `eval_random/*` and
`eval/gen_gap_picked` are being logged so nothing was actually lost.

---

## C. Areas where I would look for bugs I have not found

1. **Silent defaults.** Five found this week (see `July30th_Fable.md` §4). I found
   the last one by audit, not by failure — so the class is probably not exhausted.
   Suggested sweep: every function with a mutable-semantic default
   (`control=`, `scope=`, `max_steps=`, `cap=`, `ic_mode=`, `layout=`) and every
   call site that omits it.
2. **Subprocess argument drift.** Three separate bugs came from a parent process
   not passing a mode to a child (`wandb_eval`, `genesis_eval`, the harvester).
   Any `subprocess`/`sbatch --export` boundary is suspect.
3. **Episode-length / off-by-one.** Demo npz now come in two shapes (n/n and
   n+1/n). Four scripts normalise this independently
   (`make_dp_pruned`, `relabel_cartesian`, `to_dreamer_demos_cartesian`,
   `build_obs_action_2x2`). **They should agree; I have not cross-checked them.**
4. **The `no-pick` early abort** (`NO_PICK_ABORT = 700`) predates pick-scope work
   and may truncate slow-but-real picks in `--scope full` harvests.
5. **`cartdv`/`cartvv` fail ~50% of periodic evals** (11 OK / 10 FAILED each).
   Unexplained. Probably GPU contention, but unconfirmed.

---

## D. Specific things I got wrong this session (calibration for the auditor)

Not self-flagellation — a map of where my reasoning fails.

1. Claimed delta control "self-corrects". It does not; relative commands integrate
   error. Refuted by a rollout drift measurement (15 cm by t=150).
2. Claimed "4-DOF cannot express the demos' wrist". Wrong framing: 4 commanded DOF
   is sufficient; we *pinned* two DOF the real controller left free.
3. Predicted abs6 (FK-equivalent to joint) would reach ~0.67. It reached 0.00.
4. Reported "policy never closes the gripper" from a 120-frame window that was
   entirely pre-grasp. Artifact of my sampling.
5. Launched 8 parallel collectors on an edit that had never run → `NameError`,
   zero output. Twice in one day (also a derived script with no eval step).
6. Told the user three times that dv3 eval videos "should" work, reasoning from
   code, before reading the log that showed 100% failure.

Pattern: **I reason from code and declare, instead of measuring and reporting.**
Where this audit finds a claim with no attached measurement, assume it is unverified.

---

## E. Highest-value audit outputs, ranked

1. Confirm or refute **A1 on the rebuilt dataset** — everything else is calibrated
   against it.
2. Validate **B1** before its results are interpreted; a broken cross cell would
   produce a confident wrong answer to the central open question.
3. Cross-check the four **length-normalisation** implementations (C3).
4. Run the **silent-default sweep** (C1).

---
---

# AUDIT REPORT — completed 2026-07-31

Method: every check below is a measurement, not a re-reading. Fixes were applied
and pushed as they were found.

## Verdicts

| item | verdict |
|---|---|
| **A1 positive control** | **CONFIRMED — picked 0.67 in-dist, exactly replicated** on the rebuilt 66/66 dataset (was trained on 60/66). Downstream improved: placed 0.53 / contact 0.20 / nested 0.07; random-IC 0.27 (vs 0.13 historical — better, within known seed spread). The number the project calibrates against is real. |
| **B1 (2×2 datasets)** | VALID. Hash-verified: states identical within obs rows, actions identical within action columns, differ across; builder abs6 == reference derivation to 0.0e+00; proprio splits 8/9 and 9/9 correct. Identical frame counts are the shared-source construction, not a bug. |
| **B3 (dual representation)** | VALID. Command leads measurement (best lag 2, not 0) — `actions_joint` is the command, not a stale pose copy. |
| **C3 (length normalizers)** | **BUG FOUND AND FIXED** (`afb45a8`): `relabel_cartesian` read grip from `a[:,4]` unconditionally — a ROTATION axis in 7-dim abs6/delta6 actions. Tip truncation off by 8 frames on the probe uid; pick predicate corrupted. Same grip-column bug fixed twice before in sibling files; the doc's §C3 named exactly this location. After fix: grants match the reference converter exactly (66/63/19/12/16). Blast radius: zero completed runs (masked by the `control='vel'` default — two silent-default bugs cancelled), would have corrupted the first real 6-DOF RL run. |
| **C1 (silent-default sweep)** | Clean. 1 regex false positive (nested paren), 1 benign (`validate_cartesian`, standalone dev tool). Every live construction passes `control=` explicitly. |
| **B2 (−0.5 remnants)** | **FOUND AND FIXED**: `genesis_cartesian` and `genesis_cartesian_delta` dreamer demos still carried −0.5; rebuilt, all five demo sets now min-reward 0.0. NB the CLUSTER copies of abs6/delta6 demos are still the −0.5 versions — re-rsync before the tip0 experiment. |
| **BC tip-truncation asymmetry** | Documented as a deliberate decision in both BC dataset builders, with a revisit trigger. |

## Meta-observation

The audit found bugs precisely where the request predicted (§C3's independent
normalizers, §B2's unrebuilt demo sets) and found the request's verified claims
sound. The two-bugs-masking pattern (grip column × control default) is worth
remembering: fixing one silent default can ARM a previously-dormant bug.

## Still open after this audit

- ~~Cluster dv3 segfault~~ RESOLVED 07-31 (poisoned conda env; verify_env.sh).
  A100 retest PASSED 07-31 (user-run): full world + 16-env batched/camera
  build/reset/render clean on pax A100s — constraint widened to include a100.
- abs6-for-RL degeneracy (13-33 step episodes from unbounded pose commands): real,
  needs a design decision (per-step target clamp?) before abs6 RL is meaningful.
- Cluster demo rsync for tip0 (above).
