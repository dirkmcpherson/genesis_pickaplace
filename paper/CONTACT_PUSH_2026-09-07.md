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

CPU only, 8 jobs `cpsc_rescore_L0-7` (**3354669–76**, submitted 20:25 on the re-frozen code copy `$W/cpsc_frozen` — amendments (g'') and (g'''); the 20:15 submission 3354503+3354516–22 was cancelled 10 minutes in, with 0 cells finished, to adopt the landed (l′) `slide_success`; the wrist-based submission 3350229–36 of 14:47 was cancelled at 15:05 with 0 cells finished, and the tool-point submission 3350666–74 of 15:08 was killed by the cluster-wide disk-full outage between 16:00 and 17:00 with 85 of 192 cells done, preserved under `$W/cpsc_v1_backup/` and used for no reported number; 16 cores each, 4 concurrent evals per job = the launcher of record's 4 cores per eval), `~/wm_fix_2026-09-03/cpsc_rescore.sbatch` + `cpsc_make_cells.sh` (192 cells). Same checkpoints (`latest.pt`), same seed 0, same banks, same evaluator flags as the cells of record (`--episodes 1 --ic-set all --max-steps 600 --entry-bank <bank>` for the phase cells; `--ic-file eval_ics.json --ic-set rnd|hold --episodes 30|15 --max-steps 1200` for the end-to-end cells), written to NEW dirs `fresh_eval_<tag>_<mode>_cp`. Cells: contact-after-release human n11 / machine n11 × {polE_contact (160), holdE (11)}; carrycontact human / machine n21 × {polE_place (148), holdE_place (13)}; end-to-end dHfull_all / dDPfull × {rnd30, hold15}; mode and sample everywhere. Reproduction check: per-episode `contact`, `outcome` and `steps` of every re-scored cell compared with the cell of record (`contact_push_table.py`); every mismatch is listed in §7. Every table is given for all bank entries and for the §3c "intended grip" subset (post-hoc filter on the pseudo-uid; no bank is rebuilt).

Two protocol notes carried by amendment (g''): (i) the sweep runs a FROZEN snapshot of the shared trees taken at 20:14 (it includes the eval-fixes agent's amendment-(j) patch: pinned bank entries, `nested_honest`/`nested_proxy`, `placed_v2` logged in the full scope, the shelf-band assertion), so all 192 cells come from one code version while other agents keep editing the shared copies; (ii) the banks are the `*_rawgrip` copies — byte-identical to what the cells of record used — because the (g)/(g′) protocol is "banks as they are"; the physical-grip rebuild is the (j) `_v2` sweep, not this one. A third column, **`slide_success`** (amendment (l′), the user's settled definition of slide/end-to-end success: picked ∧ pick-can↔goal solver contact ∧ gripper commanded open < 0.3 ∧ can in the shelf footprint with tilt < 20°, sustained 3 decisions = 12 env frames), comes from the eval-fixes agent's landed implementation, which the frozen copy carries verbatim. Because both phase scopes terminate on the first contact frame, (l′) evaluates that window over the first 12 frames of the post-episode settle with the last command held and records the route per episode (`sustained` = the window closed inside the episode, `settle` = only in the held continuation). My own reading of the earlier (l) wording was withdrawn before any cell of the restarted sweep finished (amendment (g''')).

Disclosure carried by (g'''): the (j)/(l′) evaluator runs ONE post-episode settle (100 scene steps) in the phase scopes, which the cells of record did not. Episode 0 of a re-scored cell must still match the record bit-for-bit on `contact`, `outcome` and `steps` (verified on the smoke cell: uid 252, 12 steps, contact, identical); from episode 1 the settle changes the world state carried into the next restore, so later-episode divergence is expected and is not evidence about `contact_push`. The reproduction verdict is therefore reported as episode-0 identity plus an aggregate-rate comparison (§7).

## 5. Results (PROVISIONAL — unpinned cells; see §7 before quoting)

Two of the three primary cells are complete at 8 seeds per arm; the end-to-end cells and the sample/holdE companions were still running when the cluster link dropped. Every number below comes from `~/wm_fix_2026-09-03/contact_push_table.py` over the frozen sweep 3354669–76 (raw banks, (j)+(l′) evaluator). **These cells are unpinned** (`TI_NUM_THREADS` not set), so per-episode traces are reproducible only on a machine with the same physical core count; if the pinning sweep succeeds, this section is regenerated from pinned cells.

### 5.1 Contact after release (`scope='contact'`, polE_contact 160, MODE — the statistic of record for that phase)

| arm | seeds | episodes | `contact` (record) | `contact` (re-score) | `contact_push` | `slide_success` (l′) | failing = contact ∧ ¬push | gripper–goal | wrong side |
|---|---|---|---|---|---|---|---|---|---|
| human n11 (sub-floor) | 8 | 1280 | 759 (0.593) | 764 (0.597) | 443 (**0.346**) | 0 (0.000) | 321/764 (0.420) | 51 | 270 |
| machine n11 | 8 | 1280 | 770 (0.602) | 770 (0.602) | 468 (**0.366**) | 2 (0.002) | 302/770 (0.392) | 49 | 253 |

`contact_push` human − machine **Δ = −0.020, exact two-sided perm p = 0.308** (n = 8 v 8); `contact` Δ = −0.005, p = 0.918; `slide_success` Δ = −0.002, p = 0.467. Per-seed `contact_push`: human [46, 49, 54, 61, 59, 60, 52, 62] vs machine [64, 62, 57, 60, 61, 63, 49, 52].

### 5.2 Carrycontact (`scope='carrycontact'`, polE_place 148, MODE)

| arm | seeds | episodes | `contact` (record) | `contact` (re-score) | `contact_push` | `slide_success` (l′) | failing = contact ∧ ¬push | gripper–goal | wrong side |
|---|---|---|---|---|---|---|---|---|---|
| human 21 | 8 | 1184 | 955 (0.807) | 955 (0.807) | 338 (**0.285**) | 1 (0.001) | 617/955 (0.646) | 87 | 530 |
| machine n21 | 8 | 1184 | 942 (0.796) | 942 (0.796) | 296 (**0.250**) | 2 (0.002) | 646/942 (0.686) | 74 | 572 |

`contact_push` human − machine **Δ = +0.035, p = 0.546**; `contact` Δ = +0.011, p = 0.348; `slide_success` Δ = −0.001, p = 1.000. Per-seed `contact_push`: human [81, 27, 38, 35, 46, 43, 45, 23] vs machine [24, 48, 32, 33, 50, 20, 50, 39].

### 5.3 End-to-end (rnd30 / hold15) and the sample + holdE companions — PENDING (cells still running)

### 5.4 S2-4 subsets (bank entries whose restore reproduces the intended grip)

The split tables move nothing. Carrycontact polE MODE on the 113/148 intended-grip subset: human 0.250 vs machine 0.264 `contact_push` (Δ −0.013, p 0.773); on the strict 80/148 subset 0.229 vs 0.258 (Δ −0.029, p 0.580). Contact-after-release on the strict 38/160 subset: 0.253 vs 0.305. The failing fraction is stable across subsets (0.69/0.67 → 0.69/0.67 → 0.71/0.68), so the grip-scale defect is not what drives the geometry result.

## 6. Verdict (provisional, on the two complete cells)

1. **The user's question is answered: no, `contact` is not being earned by touching the gripper to the goal can — it is being earned by carrying the can into it.** Gripper–goal contact explains only 51/321 (16%) of the human and 49/302 (16%) of the machine failing episodes at the contact phase, and 87/617 (14%) / 74/646 (11%) at carrycontact. The dominant failure of the stricter test is **wrong side**: at the moment the pick-can touches the goal, the tool point is not behind the can along the can→goal line — the signature of a can still in the grasp being carried into the goal, not a slide.
2. **The size of that credit is large.** At carrycontact 65–69% of `contact` credit fails `contact_push`; at contact-after-release 39–42%. The difference between the two phases is itself the mechanism: the contact-phase entry states start with the can already released on the shelf, so more of its contacts are genuine pushes.
3. **`slide_success` (l′) is ~0 in both phase families** (1–2 episodes in 1184–1280 per arm, every one earned only during the post-episode settle, none sustained in-episode). Under the release-based definition the phase policies do not solve the task at all — consistent with (1) and with the user's decision to make release load-bearing.
4. **The source null survives the stricter predicate.** Human − machine on `contact_push` is −0.020 (p 0.31) at the contact phase and +0.035 (p 0.55) at carrycontact; both inside the registered |Δ| < 0.10 (P1 met on the two complete cells). The failing fraction differs by arm by +0.028 (contact) and −0.040 (carrycontact) — inside the registered 0.05 band, so P2's second clause is met, while **P2's first clause (failing fraction ≤ 0.15) is decisively violated**, at 0.39–0.69. The registered disconfirm branch that applies is the wrong-side one: the contact credit of record includes non-slide contacts, in both arms, and the paper must say so with the fraction per arm.
5. `contact_push` is not proposed as a statistic of record; (l′) `slide_success` is the user's settled success definition, and `contact_push` is the geometric diagnostic that shows *why* the old predicate needed replacing.

## 7. Determinism note — RESOLVED: the axis is the machine's physical core count, not the node and not the CPU family

**Provenance of my original figure.** It came from the first tool-point sweep, jobs **3350666–74** (killed by the disk-full outage; 85 cells kept as `$W/cpsc_v1_backup/`), NOT from the withdrawn wrist version (submission 3350229–36 wrote zero cells). The comparison was cell of record vs `_cp` re-score on per-episode `steps`, `outcome`, `stages.contact` — the same three fields as the eval-fixes agent's `$W/repro_check.py`. Same quantities, same direction.

**Withdrawn:** the attribution to *node identity*, and the framing that the job's allocation geometry (`-n 4` vs 4 evals on a 16-core allocation) was the driver. Both are wrong.

**The resolved account** (`$W/core_audit2.py`, every comparison joined to the SLURM node that produced each cell of record and each re-run, with `CPUTot` and CPU family per node; 128 comparisons across both sweeps, all with a resolved record node):

| record vs re-run | comparisons | bit-identical | differing |
|---|---|---|---|
| **same physical core count** | 53 | **53** | **0** |
| different physical core count | 75 | 56 | 19 |

- **Same core count reproduces exactly, without exception** (53/53), across different nodes, different CPU families and different code versions ((g) vs (g)+(j)+(l′)).
- **All 19 differing comparisons have a 36-core machine on exactly one side** — record `pax070`/`pax109` (36c, broadwell) re-run on 48c/64c, or 48c/64c records re-run on `pax070`. No pair of non-36-core machines ever disagreed: broadwell-40c, cascadelake-48c, sapphirerapids-64c and emeraldrapids-64c all agree with each other bit-for-bit. So it is thread count, not microarchitecture: `pax069` (broadwell, 40c) agrees with `pax012` (sapphirerapids, 64c) and both disagree with `pax070` (broadwell, 36c).
- **A core-count change does not force divergence** (56 of 75 unequal-count comparisons are still bit-identical): only cells with episodes near a decision boundary amplify it, which is why the free-standing-can contact scope shows it and carrycontact (short episodes from a stable grasp) never does.
- **The two sweeps agree with each other**: `dH_..._subfloor` s0/s4/s7 give identical difference triples in v1 (re-run on 48c/64c) and v3 (re-run on 64c/48c) — (71,40,31), (63,31,23), (52,16,12) — while their record was made on the 36-core `pax070`. The re-score is deterministic; the 36-core cell of record is the outlier.
- **The `pax053` "three of four" anomaly is resolved and was never anomalous**: those four cells' *records* came from two different machines — s1/s5/s7 from `pax070` (36c, differ) and s3 from `pax119` (48c, identical to the 64c re-run). Same-core-count identity holds there too.
- **Effect size**: aggregate rates move by ≤ 0.011 (contact after release, human 0.609 → 0.613; machine 0.605 → 0.602); the arm contrast is a difference of two arms evaluated in the same sweep on the same machine mix, so it is far less sensitive than per-episode identity.

**Consequence for these tables (open at the time of writing).** Every cell of this sweep is *unpinned*: the Genesis/taichi CPU thread count follows the machine's physical cores, so a cell's per-episode trace is reproducible only on a machine with the same core count. The eval-fixes agent is sweeping `TI_NUM_THREADS` ∈ {4, 8, 36} on 40- and 64-core nodes against the 36-core baseline; if pinning makes results hardware-independent, this sweep should be re-run pinned before its numbers are quoted, and §5/§6 will be regenerated from the pinned cells. Nothing is re-run until that verdict arrives.

### 7b. Per-cell reproduction of the current sweep (as of the 64 cells finished before the link dropped)

9 of 64 cells differ from their record; all 9 are exactly the cells whose record was produced on a 36-core machine:
5 contact-phase cells `dH_..._subfloor_s{0,1,4,5,7}` polE mode (record `pax070`, 36c) and the 4 end-to-end cells of
`dHfull_all_s2` (record `pax109`, 36c, post-hoc job 3302999). Every other cell — including all 16 carrycontact polE
cells, both modes, and every contact-phase cell whose record came from a 48c or 64c machine — is bit-identical.
Aggregate effect on the two complete primary cells: contact after release, human 0.593 → 0.597 (5 episodes of 1280),
machine 0.602 → 0.602 (0); carrycontact, both arms unchanged (955/955 and 942/942).

### 7c. The v1 observation as originally written (superseded by §7, kept for the record)

(Original wording, superseded by §7 above; the node attribution in it is withdrawn.) Pre-outage evidence from the 85 cells of the killed sweep suggested the contact-after-release cells were not bit-reproducible across compute nodes: `dH_subfloor_s3` 0/160 episodes differed while `dH_subfloor_s1` differed on 65/160 and `dDP_n11_s7` on 99/160, with the pattern appearing to track the node (pax070 3/3, pax012 0/4, pax053 3/4, pax097 0/1) and aggregate contact moving by ≤ 0.011. The re-audit in §7 shows the node correlation does not hold up (same node, same second, 3 of 4 cells diverged) and withdraws it.
