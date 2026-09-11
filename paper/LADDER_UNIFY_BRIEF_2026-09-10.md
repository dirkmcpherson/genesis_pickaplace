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
