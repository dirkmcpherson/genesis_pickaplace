# `contact_push`: a stricter contact predicate, logged alongside `contact`, and a post-hoc re-score of the world-model cells (2026-09-07)

Registered as PHASE_PLAN_2026-09-04.md amendment (g) (2026-09-07 14:47, before any re-score job) and corrected by amendment (g′) (15:08: the projection point moved from the WRIST link to the TOOL point after ADVERSARIAL_REVIEW_eval_env_2026-09-07 S2-5; the first re-score submission was cancelled before any cell finished, so nothing wrist-based was ever read out). Delegated brief: `~/wm_fix_2026-09-03/agent_brief_contact_push.md`. Numbers below are copied from tool output (`contact_push_table.py`, `contact_push_check.py`, `contact_push_bank_geometry.py`).

## 1. Why

User: "is contact getting credit for touching the gripper to the goal can? contact is ideally through a slide where the gripper and the goal can are on opposite sides of the pick-can." The predicate of record (`baselines/genesis_can_env.py` `step()`):

    contact  =  picked  ∧  solver contact between the pick-can (`bottle`) and the goal can (`goal`)  ∧  ee_x < can_x

The gripper alone touching the goal never counts (the pair tested is pick-can ↔ goal), but the far-side test is along robot-x only, not along the pick-can→goal direction, and gripper–goal contact is not excluded. Worse (S2-5, reproduced in §3b): `ee` is the WRIST link, ~0.145 m behind the tool, and `ee_x < can_x` holds on 332/332 banked states of both arms — operationally `contact ≡ picked ∧ solver contact pick-can↔goal`.

## 2. The predicate (exact)

Per sim step, with `tool` = `GenesisCanEnv.tool_pos()` (the gripper tip reconstructed from the wrist link with the wrist-frame offset calibrated against the real robot's tool pose at HARDCODED_START — now calibrated on every reset for every env, not only workspace-limited ones), `can` = pick-can position, `goal` = goal-can position, all in the world frame:

    contact_push  =  picked earlier in the episode
                  ∧  pick-can ↔ goal solver contact this step            (bottle.get_contacts(goal) non-empty)
                  ∧  (tool_x − can_x)(goal_x − can_x) + (tool_y − can_y)(goal_y − can_y)  <  0
                  ∧  no gripper ↔ goal solver contact this step           (goal.get_contacts(kinova) empty)

Sticky once true (like `contact`). It does NOT require `contact`'s `ee_x < can_x`, so the two can disagree in both directions; both are reported. Logged with it: `contact_frame`, `contact_push_frame` (sim step of the first grant), `contact_gripper_goal` (any gripper–goal contact on a post-pick pick-can/goal contact frame), `contact_farside` (any such frame with the TOOL dot product < 0), `contact_farside_wrist` (the same with the wrist link — the withdrawn first definition, kept only for comparison). A `contact` episode that is not `contact_push` fails for exactly one of two reasons: **wrong side** (`contact_farside` false: on no contact frame was the tool on the far side) or **gripper–goal** (`contact_farside` true, so every far-side contact frame also had gripper–goal contact).

Why the tool point and not the wrist (amendment (g′)): the wrist sits ~0.145 m behind the tool, so a wrist-based far-side test is satisfied by almost any grasp that approaches from the robot's side — on the banked HELD pick-grant states it fires 132/148 times (§3b), i.e. on the pure carry the predicate exists to exclude. With the tool point a held can's centre is ~2.3 cm BEHIND the tool (§3: `tool_fwd` +0.023 at gap 0), so a can still in the grasp is on the wrong side by construction and the far-side clause can only be earned once the can is ahead of the finger tips.

`contact` and every reward/termination rule are unchanged; `contact_push` is never rewarded and never terminates. Where it lives: repo `baselines/genesis_can_env.py` (predicate + info keys), `baselines/rl/full_env.py` (`_granted` bookkeeping only), the private cluster copy `$W/gp_root/baselines/{genesis_can_env.py,rl/full_env.py}` (byte-identical patch, `~/wm_fix_2026-09-03/contact_push_patch.py`), the r2dreamer adapter `envs/genesis.py` (`log_contact_push` next to `log_contact`; `log_*` keys are excluded from the encoder and unmatched by `mlp_keys: 'state'`, so every existing checkpoint loads with 0 missing / 0 unexpected keys — verified in every re-score log) and the evaluator `eval_genesis.py` (`stages` gains `contact_push`; per-episode `contact_diag`; summary `contact_push_diag`; `~/wm_fix_2026-09-03/contact_push_r2d_patch.py`; local mirror `cluster_r2d/` in sync, md5-verified). Side fix in the evaluator while there: a `--ic-skip` (hang) episode now records all-False stages instead of the previous episode's dict (only the rnd300 dDP s1 cell of record ever used `--ic-skip`; its ep 269 `stages` entry carried ep 268's flags — the hang was already counted as a failure in the success column, which is unaffected).

## 3. Unit check (scripted states, CPU, `baselines/diagnostics/contact_push_check.py --bank holdE_place.json --uid 252`)

One world (`gc_kp4_riser3_shelf6`); the held-can pick-grant entry of uid 252 restored exactly as `full_env._restore_place_entry` does (grip closed on the can), the pick-can then shifted `gap` m forward out of the fingers, the goal teleported to touch it (centre distance 0.068 m) at angle θ from the eef→can direction; flags read on the next `env.step`. `dot_tool` = the predicate's projection, `dot_wrist` = the withdrawn wrist-based one, `tool_fwd` = tool point ahead of the can centre (m), `bg`/`gg` = pick-can–goal / gripper–goal solver contact counts.

| gap | θ | dot_tool | dot_wrist | tool_fwd | bg | gg | contact | push | note |
|---|---|---|---|---|---|---|---|---|---|
| — | far (0.5 m) | +0.0114 | −0.0526 | +0.023 | 0 | 0 | **F** | **F** | control |
| 0.00 | 0 | +0.0015 | −0.0068 | +0.023 | 1 | 2 | **T** | **F** | (c) HELD can pushed straight in: wrong side (tool ahead of the can) AND finger tips at (+0.056, ±0.03) touch the goal; the wrist test would call it far-side |
| 0.03 | 0 | −0.0005 | −0.0079 | −0.009 | 1 | 0 | **T** | **T** | (a) can 3 cm out of the fingers, gripper behind it, no finger–goal contact |
| 0.06 | 0 / 30 / 60 | < 0 | < 0 | −0.037 | 1–3 | 0 | T | T | pushes from behind |
| 0.09 | 100 | +0.0003 | +0.0010 | −0.061 | 1 | 0 | **T** | **F** | (b) goal on the gripper's side: wrong side, no finger contact; `contact` still T (ee_x < can_x holds everywhere) |
| 0.12 | 100 | −0.0005 | −0.0011 | −0.076 | 1 | 0 | T | T | with the can far enough ahead the same angle becomes a push |
| 0.03 | 90 | +0.0001 | +0.0002 | −0.010 | 1 | 1 | T | F | side push brushing a finger |
| 0.00 | 180 | −0.0013 | +0.0065 | +0.022 | 2 | 6 | T | F | goal inside the open hand: finger–goal contact (the tool test even calls it far-side; the contact clause rejects it) |

Full 41-row scan in the script output. Canonical cases: **(a)** T/T, **(b)** T/F (wrong side), **(c)** T/F (gripper–goal AND wrong side under the tool point). Control F/F. UNIT CHECK PASS (both the wrist-based first version and the tool-point version pass the three canonical cases; they differ on the held can, which only the tool point puts on the wrong side).

Case (c) is the physically important one: with the can still IN the grasp, a straight push registers finger–goal contact AND the tool point is ahead of the can, so `contact_push` can only be earned after the can has left the finger tips (release, or a push with the can ≥ 3 cm ahead of the fingers). The re-score measures how much of each arm's `contact` credit is of that kind.

### 3b. Tool vs wrist on every banked state (`baselines/diagnostics/contact_push_bank_geometry.py`, pure FK at the banked qpos against the recorded can/goal)

| bank (n) | source | `ee_x < can_x` (`contact`'s clause, wrist) | `dot < 0` WRIST (withdrawn) | `dot < 0` TOOL (**contact_push**) | `tool_x < can_x` |
|---|---|---|---|---|---|
| polE_place (148, can HELD at the pick grant) | machine (`--dump-entries`) | 148/148 | 132/148 | **100/148** | 69/148 |
| polE_contact (160, can just released on the shelf) | machine (`--dump-entries`) | 160/160 | 123/160 | **123/160** | 126/160 |
| holdE_place (13, held) | human | 13/13 | 13/13 | **11/13** | 11/13 |
| holdE_contact (11, released) | human | 11/11 | 7/11 | **9/11** | 9/11 |

Reproduces the review's table exactly. Two readings. (i) `contact`'s directional clause rejects nothing (332/332). (ii) The far-side clause ALONE does not exclude a carry even with the tool point: on 100/148 held pick-grant states the goal happens to lie on the far side of the can from the tool (the can is grasped from the robot's side and the goal is further out), so the tool-point dot is negative before anything has been pushed. (The coordinator's "69/148" is the `tool_x < can_x` column, a different test; the registered predicate is the dot.) What makes `contact_push` exclude the carry-into-goal is the conjunction with the gripper–goal clause: a held can reaches the goal with the finger tips 5.6 cm ahead of its centre (§3 row gap 0), so the fingers touch the goal on the same frame. The re-score's failure-reason split (§5) shows how often each clause does the work.

### 3c. S2-4 — bank entries whose restore does not reproduce the intended grip (recorded here; the banks are NOT rebuilt by this task — another agent owns the bank fix)

`--dump-entries` stores the agent's raw action `a[6] ∈ [−1,1]` as `grip_cmd`; the restore reads it as a physical 0..1 command (`gripper_targets(grip_cmd·100)`, clipped to [0,1]); the intended physical command is `(a[6]+1)/2`. Human banks (`make_phase_banks.py`) store physical commands and are unaffected.

| bank (n) | grip_cmd range (mean) | intended physical mean | affected | subset used for the "intended grip" tables |
|---|---|---|---|---|
| polE_place (148) | −0.032 … 1.000 (0.823) | 0.911 | **35/148** restore with a command that OPENS relative to the measured closure (`grip_cmd < grip_obs`; with the correct scale 5/148 would) | the other **113/148** (`grip_cmd ≥ grip_obs`); strict \|restored − intended\| ≤ 0.05: 80/148 (sensitivity row) |
| polE_contact (160) | −0.991 … −0.127 (−0.747) | 0.127 | **160/160** restore fully OPEN (clipped at 0) instead of the intended 0.005–0.44; 122/160 differ by > 0.05 | qualitative (intended and restored both "released", < PLACE_RELEASE 0.45): **160/160**; strict ≤ 0.05: 38/160 (sensitivity row) |
| holdE_place (13) / holdE_contact (11) | 0.674 … 0.915 / 0.000 … 0.450 | = grip_cmd | 0 | all |

Affected pseudo-uids — polE_place (35): 900002, 900005, 900007, 900022, 900034–900052 (19 consecutive), 900072, 900075, 900076, 900079, 900080, 900081, 900084, 900085, 900086, 900089, 900091, 900118. polE_contact strict-affected (122): all except the 38 with `grip_cmd < −0.9` (list in `~/wm_fix_2026-09-03/contact_push_bank_geometry.json`, key `strict_ok_uids`). The affected polE_place entries are those most likely to drop the can at restore (the record's `restore_failed`), so the cells of record already carry them as failures; the split tables below quantify the rest.

## 4. Re-score protocol

CPU only, 8 jobs `cpsc_rescore_L0-7` (**3350666–3350674**, submitted 15:08 with the tool-point predicate; the wrist-based submission 3350229–36 of 14:47 was cancelled at 15:05 with 0 cells finished; 16 cores each, 4 concurrent evals per job = the launcher of record's 4 cores per eval), `~/wm_fix_2026-09-03/cpsc_rescore.sbatch` + `cpsc_make_cells.sh` (192 cells). Same checkpoints (`latest.pt`), same seed 0, same banks, same evaluator flags as the cells of record (`--episodes 1 --ic-set all --max-steps 600 --entry-bank <bank>` for the phase cells; `--ic-file eval_ics.json --ic-set rnd|hold --episodes 30|15 --max-steps 1200` for the end-to-end cells), written to NEW dirs `fresh_eval_<tag>_<mode>_cp`. Cells: contact-after-release human n11 / machine n11 × {polE_contact (160), holdE (11)}; carrycontact human / machine n21 × {polE_place (148), holdE_place (13)}; end-to-end dHfull_all / dDPfull × {rnd30, hold15}; mode and sample everywhere. Reproduction check: per-episode `contact`, `outcome` and `steps` of every re-scored cell compared with the cell of record (`contact_push_table.py`); every mismatch is listed in §7. Every table is given for all bank entries and for the §3c "intended grip" subset (post-hoc filter on the pseudo-uid; no bank is rebuilt).

RESULTS-PENDING
