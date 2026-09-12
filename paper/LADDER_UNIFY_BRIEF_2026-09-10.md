# Ladder unification: design contract (2026-09-10, coordinator)

Read first: `paper/E2E_TRAINING_PROBLEMS_2026-09-10.md` §0 and `paper/E2E_AUDIT_BRIEF_2026-09-10.md`
§2, §4, §8 (defects 1–5). This document fixes the DESIGN. Lanes implement it. Do not widen it.

## Why

1. {RLPD} and {r2dreamer} trained on different ladders (audit brief §2).
2. The (x) ladder's top rung cannot be paid in-episode; the episode terminates on an unpaid
   event; the demo buffer pays a rung the env cannot (brief §8, defect 5).
3. `nested_proxy` (contact ∧ grip cmd < 0.3 ∧ both upright, instantaneous) has precision 0.114
   (human) / 0.029 (machine) against the settled predicate and reverses the arm ordering (§4a).
4. Both ladders pay a contact rung that fires while the robot still holds the can, so policies
   press the held can and run to the horizon.
5. `slide_success` in the env is still amendment (l) (grip < 0.3, withdrawn by (p)); amendment (x)
   registered a three-condition definition that exists only as a tape classifier.

## Decisions (the contract)

**D1. One code tree.** Both learners import the same `baselines/rl/full_env.py`,
`baselines/genesis_can_env.py` and a NEW shared module `baselines/stage_predicates.py`. No
environment-variable reward gates. `FULLENV_REWARD_X` is removed; launchers REFUSE to start if it
is set ("legacy gate set; this tree has no gates").

**D2. One ladder, sticky, paid once, in-episode:**

| rung | reward | definition | terminal |
|---|---|---|---|
| `picked` | 1 | unchanged (genv hardened flag) | no |
| `placed_v2` | 1 | unchanged (grip cmd < 0.45 ∧ shelf footprint ∧ z-band ∧ tilt < 20°, sustained 10 frames) | no |
| `contact_push` | 2 | **`placed_v2` already granted** ∧ can↔goal solver contact ∧ tool on the far side ∧ no gripper↔goal contact | no |
| `slide_success` | 4 | in-episode (x): `placed_v2` granted ∧ **pushed** ∧ `nested_v2` (below) | **yes** |

Max return 8 (return clamp unchanged). `tipped` terminates with its existing penalty. **Nothing
else terminates.** In particular `nested_v2` is LOGGED, never paid, never terminal, and the old
`nested` proxy no longer terminates anything.

**D3. `nested_v2` replaces `nested_proxy` in every log, table and figure.** No settle simulation;
reads state only:
`picked ∧ placed_v2-granted ∧ dist_xy(can, goal) ≤ NESTED_TOUCH_DIST (0.081) ∧ tilt(can) < 20° ∧
tilt(goal) < 20° ∧ can z in the shelf resting band ∧ not in_hand ∧ at rest`.
- `at rest`: can xy displacement ≤ 2 mm over the last 12 env frames (3 decisions).
- `in_hand`: |tool_xy − can_xy| < HELD_LEVER. Calibrate HELD_LEVER on data: held frames in the
  human tapes sit at ~1.5 cm lever (SLIDE_ANATOMY); a fist push contacts the can surface at
  ≥ 3.3 cm (can radius). Start at 2.5 cm; Lane 1 reports the measured separation. No gripper term.
- `pushed`: after the FIRST `placed_v2` grant, the can's xy distance to the goal decreased by
  ≥ 10 mm cumulatively while not in_hand.
- Legacy `nested` (proxy), `placed`, bare `contact` stay computed for continuity; pay nothing.

**D4. One `slide_success` definition** in the env reward, the demo relabel and BOTH evaluators
(`baselines/eval_e2e.py`, r2dreamer `eval_genesis.py`). The settled `nested_honest` stays as the
post-hoc reference column (it is what `nested_v2` is validated against) but is no longer needed
for the statistic of record.

**D5. Demo relabel by re-execution.** New sets `<set>_rz` are produced by re-executing each tape
through `FullTaskEnv` (the training code path), so tape reward == env reward by construction.
Action streams must be byte-identical to the source set (assert sha256), only the reward column
changes; the manifest records the ladder provenance (D6). Report the per-rung tape counts for the
human set (the (x) tape classifier gave released 64 / pushed 64 / arrived 15; explain any
difference).

**D6. Provenance stamp.** `full_env.ladder_provenance()` → `{stage_reward, terminal_stages,
shaping, sha256 of full_env.py / genesis_can_env.py / stage_predicates.py, git describe}`. Printed
at env construction (`[ladder] …`) and written by every trainer/launcher to
`<logdir>/ladder_provenance.json` and by every evaluator into `metrics.json`. A table builder
refuses to merge rows with different stamps.

**D7. Sparsity fallback, built now, OFF by default.** Optional potential-based shaping
`goalward` = γ·φ(s′) − φ(s) with φ = −dist_xy(can, goal), active only while
`placed_v2`-granted ∧ not in_hand ∧ can↔goal contact. Constructor argument, never an env var,
included in the stamp. Registered trigger (Lane 4): if by 50 % of budget no seed of an arm has
any `contact_push` in training rollouts, the arm is rerun with shaping on — disclosed.

**D8. Episode record (amendment (w)) is always on.** No gate. Logging only.

### Lane 1 outcomes that bind the merge (2026-09-11 02:40)

- **HELD_LEVER stays 0.025.** On the 74 tapes of record the held tail reaches 3.23 cm (p99) and
  fist contact starts at 3.43 cm; raising the lever triples a second error (a withdrawn tool
  10 cm BELOW the can vetoing a good nest, since `in_hand` is xy-only). Every count is identical
  from 1.5 to 4.0 cm; with the clause disabled all collapse to 0 (the control that it is live).
- **`nested_v2` on the human tapes: precision 1.000, recall 0.786 vs the settle (11 of 14).**
  Misses: 255/305 (the recording ENDS while the can is still moving, so `at_rest` cannot hold)
  and 308 (`placed_v2` never granted). Amendment (x) on the same tapes: arrived 15, exactly its
  registered 15 (the 21 was the reconstruction cohort — corroboration, never a count of record).
  So P3 expects the `_rs` human set to pay on **11** tapes and `_rz`'s top rung on **≤ 11**
  (those 11 that also satisfy `pushed`); Lane 4 records the actual counts and lists the uids.
  Do NOT extend tapes to rescue 255/305 — disclose instead.
- **(x)'s `released` fires before the pick** (median decision 7 of ~389): the tape classifier's
  push gain includes the carry. The relabel MUST use the tracker through the env (D5), never
  `slide_predicate.py`.
- `nested_proxy` recall on human demonstrations is 0.143 (misses 12 of 14 real nests); with
  precision 0.000 on the machine policy arm it fails in both directions.
- Replay fidelity of `dHfull_w3` tapes through `FullTaskEnv`: 68/74 identical in decisions and
  summed reward (the known non-determinism of that lineage, SLIDE_CLAUSE5_LINEAGE §7). The
  `_rz`/`_rs` builds must record per-tape whether the re-execution matched the source outcome.

## Lanes

- **Lane 1 — predicate** (worktree): `baselines/stage_predicates.py` (pure functions over
  can/goal/tool poses, contacts, and a short history; no Genesis import), unit tests on synthetic
  histories, calibration of HELD_LEVER on human census tapes, validation of `nested_v2` and
  in-episode `slide_success` against `nested_honest` / the (l) predicate on (a) the 74 human
  census tapes (honest json exists locally) and (b) ≥ 60 policy episodes rolled out locally from
  1–2 {RLPD} checkpoints (copy from the cluster; CPU only). Deliverable:
  `paper/NESTED_V2_PREDICATE_2026-09-10.md` with confusion matrices and the chosen constants.
- **Lane 2 — ladder** (worktree): `full_env.py` per D1/D2/D6/D7/D8; `relabel_reward.py` per D5
  (code + dry run on ≥ 5 local tapes; no cluster set builds); `eval_e2e.py`, `eval_e2e_annot.py`,
  `cluster/make_r2_annotator.py`, and the r2dreamer adapter/evaluator (local
  `~/workspace/r2dreamer/envs/genesis.py`, `eval_genesis.py`) to consume the new stage names;
  `cluster/sbatch_rlpd_e2e.sh` and a ported `cluster/wmfix_full.sbatch` to stamp and to refuse
  legacy gates. Code against the Lane-1 interface below; a stub is acceptable until merge.
- **Lane 3 — one tree** (read-only on the cluster, writes one doc): diff `$LAB/gp_e2e`,
  `$W/gp_root`, `$W/long_run_candidate_2026-09-08/release_v4` and `$W/r2dreamer_fix` against the
  repo and `~/workspace/r2dreamer`; list every BEHAVIOURAL difference (not comments); write
  `paper/TREE_RECONCILIATION_2026-09-10.md` with the merge plan and the exact procedure to create
  `$LAB/gp_unified` for the next batch. Never modify `gp_root`/`gp_e2e` (in-flight jobs import
  them at runtime). Never submit jobs.
- **Lane 4 — smoke + registration** (after 1–2 merge): cluster clone, short smokes of both
  learners with identical stamps in both logs, `_rz` set builds, PHASE_PLAN amendment (z) with
  predictions and disconfirm branches BEFORE any training job.
- **Lane 5 — annotated DEMONSTRATIONS for the user** (user request 2026-09-11 00:40, priority):
  the user wants to check the phase annotations on the demonstrations themselves. Re-execute
  human tapes (`$W/demos_state_full/dHfull_all`) and machine tapes (`dDPfull_first`) through the
  unified `FullTaskEnv` (the D5 relabel path) and render each with stage chips that light on the
  frame each stage is granted (`picked`, `placed_v2`, `contact_push` release-gated, `nested_v2`,
  in-episode `slide_success`) beside the legacy `nested_proxy`, the tracker diagnostics (lever,
  in_hand, at_rest, goalward gain) and the settled `nested_honest` verdict. Stratify: ≥ 6 human
  (2 slides, 2 nested-by-drop, 1 carry-in, 1 no-pick) and ≥ 6 machine (same classes where they
  exist; machine tapes mostly run to the 601 cap). ≤ 400 frames per clip, small resolution.
  Optional after that: ≥ 4 policy episodes from existing {RLPD} checkpoints (human s901, machine
  s920) over the disputed classes. Local CPU render; write to
  `can_pos_recovery/videos_ladder_2026-09-11/` with an `INDEX.md` saying what each clip should
  show and what the user is checking; the coordinator sends them with SendUserFile.

## Lane 4 pilot design (user 2026-09-11 00:40: "start a set of shorter runs to verify")

Registered as PHASE_PLAN amendment (z) BEFORE submission. Both learners from `$LAB/gp_unified`
(RLPD via `GENESIS_PICKAPLACE_ROOT`, r2dreamer via `GP_ROOT`), QOS **normal** (separate 10-GPU
cap; the old batch occupies the preempt cap), demo sets `dHfull_all_rz` / `dDPfull_first_rz`
built by D5 on the cluster (action sha256 identical to the sources; only reward differs).

**Two ladders, one env, selected by constructor argument and stamped** (user addition 01:00,
"consider the settled-contact — that is what we want to induce; humans do it with a slide"):

| ladder | pays | terminal | max return / r2d clamp | demo sets |
|---|---|---|---|---|
| `staged` (D2) | picked 1 / placed_v2 1 / contact_push 2 / slide_success 4 | slide_success | 8 / 8.0 | `_rz` |
| `sparse` | **nested_v2 1.0 only** (settled contact: released, at rest, within 0.081 m, both upright, not in hand) | nested_v2 | 1 / 1.0 | `_rs` |

Everything else (tip rule, every logged stage, `pushed`, tracker diagnostics) is identical, so
the arms differ only in reward and terminal. The sparse arm logs whether each settled contact
came by the slide route (`pushed` = True) or by a drop.

| learner | ladder | arms | seeds | budget | checkpoints / milestones |
|---|---|---|---|---|---|
| {RLPD} | staged (verification pilot) | human / machine-first | 2 v 2 (s940–941 / s960–961) | 100k decisions | 40k / 100k |
| {r2dreamer} | staged (verification pilot) | same | 2 v 2 | 1M online steps | 0.5M / 1M |
| {RLPD} | **sparse** (sample batch) | same | 2 v 2 (s945–946 / s965–966) | **250k decisions** (full) | 40k / 100k / 250k |
| {r2dreamer} | **sparse** (sample batch) | same | 2 v 2 | **4M online steps** (full) | 0.5M / 1M / 2M / 4M |

16 jobs on QOS normal (9 free slots at 01:00; the short staged pilots free slots within ~4 h,
so the sparse batch finishes by ~2026-09-11 evening). The sparse arm runs at the FULL budget
because a sparse null at a short budget is uninformative; its milestones at 100k / 1M give the
like-for-like read against the staged pilot at the same step.

Predictions (each with its check): **P1** every run's `ladder_provenance.json` and Slurm
`[ladder]` stamp are identical within a ladder and differ between ladders ONLY in `ladder`,
`stage_reward`, `terminal_stages`, `return_clamp` (assert; a mismatch aborts the pilot). **P2**
zero `contact_push` grants without a prior `placed_v2` in every episode record (structural;
count = 0). **P3** the `_rz` and `_rs` human sets pay their top rung on the same tapes the (x)
classifier selects (Lane 1: 21/74 on the reconstruction cohort; record the re-execution count and
explain any difference); `_rs` pays exactly one +1 per paying tape at the nested_v2 frame. **P4**
by the end of the staged budget ≥ 1 seed per arm shows `contact_push` in training rollouts;
disconfirm → rerun that arm with D7 shaping ON, disclosed. **P5** max episode return ≤ 8 (staged)
/ ≤ 1 (sparse) and no negative drift in `picked` versus the (x)-batch curves at the same step.
**P6** no job ends `FAILED 2:0 00:00:00` (requeue guard). **P7 (the user's question, sparse
arm):** settled contact is reached by ≥ 1 seed per arm within the full budget; **P8** the route
census — prediction: under BOTH ladders the majority of `nested_v2` events have `pushed` = False
(drop route); disconfirm = slide route ≥ 50 % in any arm, which would mean paying the outcome
alone induces the slide. Decision rule registered now: if sparse reaches `nested_v2` at ≥ the
staged rate at the matched milestone, the intermediate rungs are unnecessary and sparse becomes
the primary for the 16v16; if sparse is 0 in all seeds at the full budget while staged > 0, the
shaping is necessary. Readout: rnd30 mode cells at every milestone with `nested_v2`,
`slide_success`, `pushed`, `nested_honest`; NOT a source comparison (n=2).

## Lane-1 interface (Lane 2 codes against this)

```python
# baselines/stage_predicates.py
NESTED_TOUCH_DIST = 0.081; AT_REST_MM = 2.0; AT_REST_FRAMES = 12; PUSH_GAIN_MM = 10.0; HELD_LEVER_M = 0.025
class StageTracker:
    def __init__(self, goal_xy, shelf_top_z): ...
    def reset(self): ...
    def update(self, *, can_pos, can_quat, goal_pos, goal_quat, tool_xy, grip_cmd,
               picked, can_goal_contact, gripper_goal_contact, placed_v2) -> dict:
        """Call once per env frame AFTER the sim step. Returns sticky flags:
        {'in_hand','at_rest','released','pushed','contact_push','nested_v2','slide_success'}
        plus diagnostics {'goalward_gain_m','lever_m'}."""
```
`full_env.py` owns `picked` / `placed_v2` / `tipped` and the reward loop; it feeds the tracker
and reads the flags. Evaluators and the relabel call the same tracker through the env.

## Rules for every lane

- In-flight jobs read `$W/gp_root`, `$LAB/gp_e2e` and `$W/r2dreamer_fix` at runtime. Do not
  touch them. Do not submit Slurm jobs. Keep the local GPU free (CPU runs only).
- Every number carries an `{algorithm}` header and the command that produced it.
- An absent value is not a zero. Before reporting any 0, show the test that could have made it
  non-zero. Read code in the tree that ran, not the tree you assume ran.
- Commit in your worktree with clear messages; do not rebase or touch other branches.
- Report back: what changed (files), what was measured (numbers + commands), what is unverified.

## Ladder N (NESTED) — design for the batch AFTER the pilot (user, 2026-09-11 ~16:00; PROPOSED, not registered)

User's words: "nest such that max reward (or sparse reward) comes from release, moving the
gripper to the opposite side of the can, and sliding it towards the goal can. If we didn't
think sparse was going to work we could ramp up reward for the slide and give a big boost for
contact." The pilot runs out first; its sparse arm (P7/P8) decides between variants A and B.

**Tracker additions (`stage_predicates.py`):**
- `released` now requires `picked` (closes the reset artefact: 4 of 30 `rnd30` starts fire
  `placed_v2` with the arm at home).
- `farside` (sticky): `released ∧ not in_hand ∧ dot(tool−can, goal−can) < 0 (xy) ∧
  |tool_xy − can_xy| ≤ 0.08 m` — "the gripper is on the opposite side of the can, close enough
  to push". No solver-contact term (the (g) `contact_push` needs solver contact, which 10/13
  human slides never make in sim).
- `slide_gain` (m, monotone): the decrease of dist_xy(can, goal) below its running MINIMUM since
  release, accumulated only on frames where `farside` holds THIS frame and the can is not in
  hand. Paying on new minima only means oscillation cannot farm it.
- `home` := `nested_v2 ∧ farside-granted ∧ slide_gain ≥ 0.01` (settled arrival by a push from
  the far side). Terminal.
- Optional switch `far_release`: the release that counts for `farside`/`home` must happen at
  dist_xy(can, goal) ≥ 0.10 m, so a drop-and-nudge cannot pay (human set-down remaining
  distance 10.7–12.3 cm, SLIDE_ANATOMY; count the machine tapes that pass before adopting).

**Variant A — `nested_sparse`:** only `home` pays, +1, terminal. Everything else logged.
**Variant B — `nested_ramp`** (each rung REQUIRES the previous one):

| rung | pays | requires |
|---|---|---|
| `picked` | +1 | — |
| `placed_v2` | +1 | picked |
| `farside` | +1 | placed_v2 |
| slide ramp | dense: +2 × min(1, slide_gain / 0.10 m), paid incrementally on new minima | farside |
| `home` | **+4** ("the big boost for contact"), terminal | farside ∧ slide_gain ≥ 0.01 ∧ nested_v2 |

Max return 9 (r2dreamer `return_clamp` 9.0). Tip rule unchanged (Lane 6 may move its threshold
by a separate amendment). `nested_v2` alone pays nothing and terminates nothing in either variant.

**Decision rule (extends the pilot's):** if the pilot's sparse arm reaches `nested_v2` in ≥ 1
seed per arm, the 16v16 runs Variant A; otherwise Variant B. Either way the ladder is one of
these two, registered as amendment (aa) with the demo-side counts below, BEFORE submission.

**Demo-side check to run now (Lane 7, no cluster, no jobs):** relabel all 146 tapes by
re-execution under A and B (and with `far_release` on/off): per-arm reward totals, how many
human/machine tapes reach `farside`, `slide_gain` distribution, how many reach `home`. The human
set should earn near the maximum on its 13 sim-slides; if it does not, the definition is wrong,
not the humans.

## Decision rule for the ≥ 16-seed batch (user, 2026-09-11 ~21:30: "soon we have to decide which of these reward variants has the best chance and then run ≥ 16")

**Candidates (all with `tip_guard=not_in_hand`, `far_release` off):** `nested_ramp` v2, `nested_sparse`,
and the pilot's plain `sparse` (settled contact by any route), each per learner. The staged ladders
are out (their top rung was a set-down-transient detector; pilot policies do not slide under the
calibrated predicate).

**Primary statistic:** the `home` rate — settled contact reached BY A SLIDE — in the deterministic
`rnd30` + `hold15` cells at the largest milestone every candidate has reached, plus the
training-rollout `home` rate over the last 200 episodes at that step. **Secondary:** `slide_event`
rate (pushes that did not arrive), number of seeds that ignited (≥ 1 `home`), and the P8 route
split for the plain-sparse arm (a settled contact reached by a DROP does not count).

**Rule:** the variant with the highest `home` rate at the matched milestone goes to 16 v 16; a tie
is broken by ignition count, then by `slide_event`. If NO variant shows `home` in ≥ 2 seeds per
learner by the readout, the answer is "not yet decidable" and the 16-seed batch waits for the 4M /
500k extensions rather than being spent on a coin flip.

**Readouts that decide, in time order:** the route census of the pilot's sparse checkpoints (hours);
{RLPD} 100k checkpoints of the batch (~2–3 h from 21:30) and 250k (~10 h); {r2dreamer} 2M milestones
of the ramp arm (~7 h) and the 4M extension seeds (~14 h); the local dv3 ramp run at 2M (~6 h).

**Logistics for 16 v 16 per learner (32 runs each):** GPU ceiling 30 concurrent (normal 10 +
preempt 20); {r2dreamer} 4M ≈ 14 h per run → two waves ≈ 30 h; {RLPD} 250–500k ≈ 10–20 h per run.
The sets exist (`_rnrh`/`_rnsh` of record); the launch scripts exist (`cluster/submit_ln_batch.sh`);
the only new artefacts are the registration (amendment (ab′)/(ac) with the chosen variant, the
seed ranges, and the predictions) and a fresh clone if the tree moves. Submission is one command
per learner once the variant is named.
