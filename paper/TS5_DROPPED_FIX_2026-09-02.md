# The validated gripper fix is not in the frozen corrected-world sets (2026-09-02)

**One-line:** the paper's corrected world `gc_kp4_riser3_shelf6` was built WITHOUT
`grasp_timeconst 0.005`; the 08-25 gripper lab's defect A (soft finger–can contact →
8–10 mm penetration at ~150 N → over-squeeze) is therefore live in every corrected-world
arm; this session's inference (unconfirmed, §4.3) is that the user's 09-02 observation on
`v2_w3_dHv2` ("real fingers close in, sim fingers stay straight or bend out; failures are the
gripper pinching the can away") is that defect.

## 1. Evidence chain

| # | fact | source |
|---|---|---|
| 1 | frozen w3 sets are stamped `sim_variant=gc_kp4_riser3_shelf6` (sha aaa88a8185f46f0e) | `paper/V2_BUILD_2026-09-01.md`; **user-confirmed from the cluster session 09-02** |
| 2 | `gc_kp4_riser3_shelf6` has no `grasp_timeconst`; `post_build` only sets contact timeconst when the variant dict carries the key | `baselines/sim_variants.py` (VARIANTS + `post_build`) |
| 3 | the fix exists as `gc_kp4_riser3_shelf6_ts5`, labelled "WORLD OF RECORD from 2026-08-26 (user decision)" | `baselines/sim_variants.py` |
| 4 | AUDIT_INDEX E5 (08-29) calls `grasp_timeconst` "adopted" | `paper/AUDIT_INDEX.md` |
| 5 | no `_ts5`-stamped tape/manifest/checkpoint exists on this box | `find` 09-02 |
| 6 | the lab validated the fix ON `gc_kp4_riser3_shelf6`, same machine, 75 demos: penetration 9.0 → 0.9 mm (p95 of carry frames), tip-overs 38 → 23, free-tips 20 → 12, nested 17 → 20, placed 55 → 60, pick-scope 57 → 58/66, random-teacher negctl 0/90; cost: carry force 147 → 245 N; "adopt NOW … but the cluster must re-run §8 before anything is adopted for the paper" | `paper/gripper_lab_2026-08-25.md` §5.3, §6 |
| 7 | this box (pop-os, AVX2, 3rd machine) reproduces the lab's per-uid flips in open-loop replay: 261 and 301 end tipped (tilt 90°) in the paper world and upright with ts5 (261 nested, 301 placed); 233/242 nested in both | `can_pos_recovery/videos_sbs/*.mp4` (sent to user 09-02 19:50) |
| 8 | finger–can friction is `max(URDF 1.0, can_friction)`: can_friction < 1 cannot touch the grasp | gripper_lab §1.3 |

So between 08-26 ("adopt") and 09-01 (v2 build) the fix fell out of the built world. Answered
in §4: omission (one pilot recording on ts5, never a matched set); the cluster §8 re-run was
pick-only and a wash.

## 2. What it explains

- The pinch-away failure mode in the dHv2 side-by-sides (user, 09-02): the sim grasp is a
  penetration grasp; the closed fingers sit ~18 mm inside the can's surface and the PD error
  keeps pushing → the can squirts out or tips on release. The lab's tip-over count halves
  with ts5 for exactly this reason.
- Where the 09-02 world-parameter search's gains came from (`paper/WORLD_SEARCH_PREREG_2026-09-02.md`
  §4): finger_force ↑, goal/table friction ↓, all gains downstream of pick — knobs compensating
  a soft-contact grasp on the legacy world. That search is a pilot on a world the paper does not
  use; it is not the fix.
- "Fingers stay straight / bend out": defect A (fingertips inside the can cannot wrap it).
  Mimic-fight from the policy side is ruled out (§4.2: single grip scalar everywhere).

## 3. Same-box replication of the lab's full-task table (this box, 6-way, 09-02 evening)

> **09-03 caveat (see §6):** these tables ran on the replay harness *before* the history-leak fix —
> a demo's stage depended on which demo ran before it in the same shard. Directions/aggregates
> are corroborated by the fixed-harness re-score (§6: θ0-w3 vs θ0-ts5 c+n 50 vs 50, nested 19 → 22),
> the per-uid flip lists are partition-specific and should not be cited individually.

_Chain `gripper_lab.py full --cfg g_base|g_stiff5 --label success|fail --parallel 6`
(outputs `baselines/demos_v1/_fulltask/g_{base,stiff5}/manifest_{success,fail}.json`);
identical code path to the 08-25 lab, third machine (pop-os i5-10600K, 6 physical cores), box
otherwise idle, single rep each (~14 min per cfg×label)._

**75 success-labelled demos, open-loop replay of the real streams** (raw manifest flags; the
lab's 08-25 laptop numbers in parentheses):

| cfg | picked | placed | contact | nested | tipped | tipped_free | pen_closed med / max (mm) | F_closed med (N) |
|---|---|---|---|---|---|---|---|---|
| `g_base` (= paper world, ts 0.02) | 70 (71) | 58 (55) | 22 (22) | 17 (17) | 37 (38) | 19 (20) | 9.9 / 16.4 (9.9 / —) | 147 (147) |
| `g_stiff5` (= `_ts5`) | 69 (69) | 57 (60) | 21 (20) | 19 (20) | **25** (23) | **12** (12) | **1.5 / 3.2** (1.5 / 3.1) | 237 (245) |

Same story on all three machines: penetration ÷6, tip-overs −12, free tips −7, nested +2,
picked −1, carry force ×1.6. Aggregate contact/placed are flat (within ±1 / ±3) — the fix
moves *which* demos succeed, not how many, except for tipping.

Per-uid flips (this box), vs the lab's Appendix A lists:

| predicate | lost by ts5 | gained by ts5 | overlap with lab |
|---|---|---|---|
| picked | 303, 316 | 321 | lost 2/3 (lab also lost 255), gained 0/1 (lab: 331) |
| contact | 233, 236, 259, 297, 298, 325, 326, 328 | 235, 237, 261, 262, 293, 311, 315 | lost 5/9, gained 5/7 |
| nested | 247, 317, 328, 330 | 237, 261, 262, 267, 293, 309 | lost 2/5, gained 5/8 |
| tipped_free (fewer = better) | stops: 245, 246, 258, 261, 267, 287, 303, 320, 331, 333, 335 (11) | starts: 244, 247, 256, 321 (4) | stops 6/10, starts 2/2 |

Reading: the *direction* and *size* of every effect reproduce; about half the individual flips
do (open-loop replay is chaotic at the margin — this is the same-shard/same-machine caveat the
paper already carries). 261 is the cleanest single case (paper world: placed then tips free;
ts5: nested, upright) and matches the side-by-side video. **Caveat on 301:** the sbs render
(`render_grasp_sbs.py`, its own stepping loop) ends 301 upright under ts5, the lab harness ends
it tipped under both — 301 is borderline and should not be cited as a ts5 win.

**Fail-labelled negative controls (16 demos, same replay):**

| cfg | picked | placed | contact | nested | tipped_free | pen_closed med (mm) |
|---|---|---|---|---|---|---|
| `g_base` | 10 | 5 | 3 (270, 307, 312) | 1 (307) | 9 | 6.7 |
| `g_stiff5` | 9 | 6 | 1 (307) | 1 (307) | 8 | 0.9 |

ts5 does not manufacture success on fail-labelled tapes: the single nested FP (307) is the same
uid under both worlds, contact FPs drop 3 → 1. The nested false-positive floor of the metric
(1/16 ≈ 6%, vs the 2/16 measured on 07-20 on the legacy world) is not raised by the world change.

## 4. Answers from the cluster/paper session (relayed by the user, 09-02 ~22:00)

1. **Omission, not decision.** Commit `7c8195d` (08-26 07:48) declared
   `gc_kp4_riser3_shelf6_ts5` the world of record and started one human recording on it
   (`dH_w4`, SESSION_LOG row 154) → `matched_w4_pilot/dH` + one DP pilot (`dp_pilotw4`);
   `matched_w4` was never built and no PREREG entry reverts the declaration. Every frozen w3
   set, every model harvest and every reported arm was built on `gc_kp4_riser3_shelf6`
   without ts5 → **no reported arm sits on a ts5 world**; the "world of record from 08-26"
   comment in `sim_variants.py` is wrong as a statement of what ran. The gripper_lab §8
   cluster re-run was done for pick recreation only: **57/66 base vs 58/66 ts5** (ts5
   rescues 246, loses 316 and 254) — a wash, which is the most likely reason it was never
   pursued. **E5 caveat (important for §1 row 6 and for the search):** because Genesis
   averages the two geoms' `sol_params`, setting the can geoms to 0.005 also stiffens the
   can–goal and can–shelf contacts 1.6× (0.02 → 0.0125) — those are the metric-bearing
   contacts for `contact` and `nested`. So the ts5 nested/tip-over gains are not purely a
   grasp effect; part may be a stiffer can–goal contact.
2. **Single grip scalar.** Every learner acts through `env.step`, whose gripper component is
   one scalar mapped by `gripper_targets` to all finger dofs (`genesis_can_env.py:244`);
   the cartesian env likewise. Mimic-fight cannot come from the policy side. (Q3 closed.)
3. **Pinch-away in the composites: unverified.** The cluster composites are w3 raw-human
   replays only, no ts5 counterpart, and pinch-away has not been scored on them. The
   attribution in this doc's one-liner (user's observation = defect A) is THIS session's
   inference from the lab numbers, not a scored result — treat as unconfirmed.
4. **Recommendation from the cluster session: B** (keep w3, disclose defect A with §3 as
   the measured cost). Reasons: A31–A33 mid-run on w3 with 2.5 weeks left; the only
   policy-relevant ts5 evidence is the 57-vs-58 pick wash; ts5 changes the metric-bearing
   contacts (E5); **open-loop replay improvements have not yet been shown to move
   closed-loop policy numbers.** A becomes worth it only if the CEM champion shows a
   large closed-loop gain. If a rebuild happens, the target is θ0-ts5.
5. Extending `sim_variants.install/post_build` to carry finger_kp/can_rho/frictions is
   acceptable on three conditions: a NEW variant name only (no edits to existing dicts);
   `post_build` asserts the geom count it touched (METHODS open item 8); a selftest. The
   cluster pulls on demand — a break hits launchers at their next start.
6. `episodes_pick_phase_dppruned` (72 tapes) will be rsynced here from
   `$LAB/genesis_pickaplace/baselines/episodes_pick_phase_dppruned` when the network is back.
7. Cluster headline numbers are single-pipeline (eval_core.run_eval via wandb_eval.py: one
   process per eval block, env built once, reset per IC = scene reuse; WM via r2dreamer's
   eval_genesis.py, same shape; demo recording = sharded scene reuse). The
   fresh-process-per-episode pipeline produced no headline number. The 12/74 cross-pipeline
   disagreement is new to them and goes into the CONFOUNDS ledger. **Their question — how
   many of the 12 disagree at the pick stage: 0.** Per-uid no-pick sets are identical
   ({234, 278, 316, 318, 319} in both pipelines); all 12 disagreements are placed/contact/
   nested rungs.

## 4b. What this changes here

- §1 row 6 / §3 must be read with the E5 caveat: "penetration ÷6 and tip-overs −12" is
  solid (finger–can contact), but "nested +2" and any search gain at contact/nested on the
  ts5 world are on a world whose can–goal contact is also 1.6× stiffer.
- The bar for option A is CLOSED-LOOP, not replay. The cheapest closed-loop test that needs
  no retraining: evaluate an existing w3-trained checkpoint (e.g. the DP on
  `matched_w3/dHv2raw`) on w3 vs w3+ts5 here with the same harness and ICs — a policy
  trained on the soft-contact world may lose on ts5 (grip-effort obs shifts 18 → 26–28
  raw), which would itself be a result. Needs a checkpoint rsynced from the cluster.

## 5. Options (user decision; costs from gripper_lab §6.4)

- **A. Rebuild v2/w3 on `gc_kp4_riser3_shelf6_ts5`.** World change, not MDP change:
  re-collect stride-1 human tapes (cheap, minutes at 6-way), re-harvest dDP/dR2D from
  re-trained teachers (the expensive part — cluster days), re-run positive/negative controls,
  refit `obs[7]` normalisation (grip effort shifts 18 → 26–28 raw), extend the `sim_variant`
  assertion to the new name, PREREG amendment + same-machine re-baselines. Placements and
  predicate constants untouched.
- **B. Keep w3 as-is and DISCLOSE defect A** as a known sim limitation, citing the lab's
  penetration number and the tip-over halving as the measured cost. Zero compute; weakens
  every sim-fidelity claim and leaves the pinch-away in every video.
- **C. Do A for the human arm only** (cheap re-collect) and report the model arms on w3 with
  the mismatch disclosed. Not recommended: mixed-world comparison contradicts the matched-set
  design.

Recommendation (this session, before §4): A if the paper still has cluster-days; otherwise B with
the §3 numbers as the disclosed cost. **Cluster session (§4.4): B, A only on a large closed-loop
gain.** Either way, no further physics search on a no-ts5 world.

## 6. 09-03: the replay harness was history-dependent; fixed-harness re-score (Amendment 3)

**Defect.** Every replay-path reset (`replay_harness.rollout`, `world_search.score_variant`,
`gripper_lab`, `fulltask_fidelity_lab`, `render_*`, `remeasure_contact`, hence the `cpu_research`
placements) teleported the arm with `set_dofs_position` and never re-issued the PD target, so the
reset step ran under the *previous* tape's last command (fresh process: FORCE mode, zero torque).
`GenesisCanEnv.reset` was fixed for this on 08-14 (P2 FIX); the replay path was not. Probe (uid 242,
same θ, `_ts5`): solo nested 0.068 / after 305 nested 0.069 / after 263 **placed** 0.081; with the
fix all three bit-identical 0.0756 (`can_pos_recovery/leak_probe.py`). Scale: same θ, idle box, two
shard partitions of the fit-50 differed on 7/50 stages (−9 c+n) — the size of the headline gain.
Fix: `replay_harness.reset_arm()` at all seven sites. **The recorder/env path
(`record_demos`, `eval_core`/`wandb_eval`, r2dreamer eval) never had the defect — no trained-policy
number is touched.** Ledger: CONFOUNDS row 45; it is the mechanism behind rows 14 and 40.

**Re-score on the fixed harness** (74 search uids = fit 50 + sealed holdout 24, nominal placements,
6-way, idle box; noise floor = 3 extra runs with the can offset by 1 mm; `rescore.json`):

| all-74 picked/placed/contact/nested (c+n) | paper world `gc_kp4_riser3_shelf6` | `_ts5` |
|---|---|---|
| θ0 (kp 40 / force 50 / ρ 1000 / fric 0.2 / 0.5 / 2.0) | 69/62/31/19 (**50**) | 69/57/28/22 (**50**) — jitter range 48–50 |
| θ_w4 (kp 64.3 / force 59.3 / ρ 852.5 / fric 0.2 / 0.443 / 2.118) | 66/53/22/15 (**37**, post-hoc) | **69/63/36/29 (65)** — jitter range 64–67 |

- **Sealed holdout (24):** θ_w4 22/21/13/9 vs θ0 22/16/8/6 → c+n **+8**, picked equal → pre-registered
  R1 CONFIRM. Noise floor R2: min θ_w4 c+n 64 > max θ0 50 (θ0's own 1 mm spread is 2) → above floor.
- **The gain is an interaction.** ts5 alone at θ0 is c+n-neutral (nested +3, placed −5); the re-fit
  alone on the paper world is harmful (−13); together +15. Nothing here ports to w3 as a parameter
  tweak — the proposal is exactly `gc_kp4_riser3_shelf6_ts5_w4` (caveat: θ_w4 was fit on ts5, so 37 is
  not "the best θ on w3"; it only shows the changes are not separable).
- Per-uid (fixed harness, θ_w4 vs θ0 on `_ts5`, * = holdout): UP 13 — 244* 246* 247 333* picked→nested,
  245* picked→placed, 297* picked→contact, 295 317 320 contact→nested, 306 placed→nested, 298 311* 328
  placed→contact. DOWN 2 — 235 contact→placed, 330 nested→contact.
- Still replay evidence. The recorder-path census (the paper's own recording pipeline, pick and full
  scope, w3 vs ts5 vs ts5_w4 on the same 74 uids; gate pre-stated: ts5_w4 loses ≥ 2 picks vs ts5 or
  shows no full-scope c+n gain → NOT proposed) is running as of 10:03 (`baselines/demos_v2/census_ts5w4_0903/`).

## 7. 09-03 11:45: recorder-path census + real-ground-truth fidelity — the replay gain does not transfer

Full readout in `WORLD_SEARCH_PREREG_2026-09-02.md` (A2.3 recorder-path census readout). Headline,
recorder path (`record_demos.py --teacher human`, 74 uids, full scope, cumulative): w3 69/26/5,
ts5 67/22/5, **ts5_w4 69/21/7** (c+n 31 / 27 / 28); pick scope picks 68 / 66 / 67. The +15 c+n
replay-path gain of Amendment 3 becomes +1 vs ts5 and −3 vs w3 in the paper's own recording
pipeline — the CEM fit lives in the replay basin (open-loop PD at tape rate), not the recorder's
closed-loop follower basin. Against the bag ground truth (`can_pos_recovery/real2sim_fidelity.py`):
all three worlds track the human tool to the same 1.6–1.7 cm same-instant median (0.3 cm after
time-warp), same 0.8–0.9 s mean lag, same 0.97–0.98 duration ratio, same ±0.2 s closure/lift/release
offsets, and the can comes up within ~0.1 s of the human's lift in all three; paired deltas 0.00.
**Not proposed for retraining** (no better real2sim shown). What ts5 does change in the recorder
path: fewer picks (66–67 vs 68) and fewer contacts (21–22 vs 26), nested 5–7 vs 5 — i.e. defect A's
fix costs pick recreation on this path, which is the head agent's option B (keep w3, disclose) in
numbers. Real can trajectory remains unmeasured (camera click pass never done).
