# World-parameter search — adversarial audit + pre-registration (2026-09-02)

User request: "train an RL policy to adjust the physics parameters to increase demo
performance... sparse reward of the performance of the full set for each phase", then
"adversarial audit your plan, and then execute the synthesis of the two."

This doc is §1 the audit of the chat-proposed plan, §2 the synthesized protocol
(pre-registered BEFORE the search runs), §3 measured instrument facts. Written before
launch; only §3's baseline table is filled in by the driver afterward.

## §1 Adversarial audit of the chat plan (each finding checked against repo/machine)

1. **Wrong machine.** The plan priced evals from "~25 s/episode, 16-way sweet spot" —
   both numbers are OLD-BOX (BB, 32 cores) lore. This box: `nproc`=12; measured
   19.5 ms/cmd → ~40 s for a mean (1976-cmd) tape, idle, single process. The validated
   operating point here is ~6-way (yesterday's recovery rounds). Eval cost is ~2.5×
   the plan's estimate → the "overnight ~300–400 evals" claim was fantasy; realistic
   is ~75–90 candidates in an 11 h window.
2. **`constraint_timeconst` is not a dimension.** Genesis 0.2.1 `RigidOptions` rejects
   it (measured: `GenesisException: Unrecognized attribute`). The kwarg in
   `build_world` exists for 1.2.x comparison work only. 6 dims, not 7.
3. **World-build overhead was unbudgeted — and is fine.** 135 s cold, **14.3 s warm**
   (taichi offline cache; verified the cache hits across different parameter VALUES).
   Fresh-process-per-candidate is therefore safe AND cheap. The tempting alternative
   (persistent workers mutating kp/force/friction/mass at runtime) is rejected: any
   silently-no-op setter would corrupt the whole search — this repo's six
   silent-default bugs are exactly that failure class.
4. **Geometry must not be searched.** can_height 0.101 / can_radius 0.033 are measured
   reality (FK grasp heights; a real 66 mm can). Letting the optimizer deform them is
   fitting sim to demos by changing the object — the 07-08 panel's sin. Excluded.
   can_rho bounded tight [700, 1400] around the physically computed ~1010 kg/m³
   (0.35 kg soup can / πr²h).
5. **Scalarization trap.** A naive weighted sum will happily trade picked DOWN for
   nested UP (sticky-fingers world). Guard: hard reject any candidate with
   picked < baseline − 2; full per-phase vector recorded for every candidate so the
   scalarization hides nothing.
6. **The plan conflated proxy-set and fit-set.** Eliminated: the search objective IS
   the fit set (~50 demos); no 25-demo proxy. Holdout (~24) is never seen by the
   optimizer and is scored once, at the end, champion + baseline only.
7. **No baseline denominator.** The env-basin funnel (73/67/33/25) is the WRONG
   baseline for a replay-basin search. θ0 is evaluated ×2 under search-identical load
   BEFORE the search; its rep-to-rep stage flips are the live noise floor. Headline
   comparison is SOLO (idle) holdout, champion vs θ0.
8. **Universe was wrong.** 77 success-labeled trials include stubs 290/322 (gripper
   never closes; excluded since 07-06) → 75 real successes, 74 solved (303 unsolved,
   no video, control-limited). 237 is still `ok_batch` (its placement CPU-refused in
   round 3) — included with disclosure: a world that lifts it counts honestly, but it
   is 1-demo noise risk. Old JSON `stage` labels are stale/absent → baseline measures
   all 74 fresh; the split stratifies on MEASURED baseline stage, not labels.
9. **RL framing.** Already rejected in chat (correctly): stateless θ→score is a
   black-box problem. Synthesis keeps CEM — hand-rolled (~40 lines), no new pip
   dependency (a 6 am `pip install cma` is its own failure mode), rank-based (robust
   to the integer+flip noise), resumable.
10. **Thread config is untouchable.** Load flips are plausibly fp-reduction-order
    noise; changing taichi/OMP thread counts would change the fp stream vs every
    historical number. Candidates are compared under IDENTICAL 6-way load; official
    numbers come from solo runs. No thread tuning.
11. **Two basins.** Search scores the REPLAY basin (cheap; the basin all placements
    were validated in). The training payoff lives in the ENV basin. A champion must
    get an env-basin spot-check before any training-use claim. Post-hoc, disclosed.
12. **GPU stays idle deliberately** (user preference to explain): GPU-batch scoring is
    convicted twice (ok_batch transfer cliff), and one scene = one world config, so
    per-candidate physics can't batch anyway. CPU replay is the basin of record.
13. **Placement coupling (kept from chat plan, sharpened).** All 74 placements were
    fit UNDER θ0. Gains may partly be placement re-fitting in disguise; losses may be
    placement-limited. Fixed placements → result is a LOWER BOUND on what a jointly
    re-fit world could do. Disclosed; joint search is out of budget.
14. **This is a pilot.** ~7–8 CEM generations detects headroom; it does not converge.
    Framed and reported as such. State checkpoints every generation; resumable.

## §2 Pre-registered protocol

- **Universe:** the 74 solved successes (label=success, status ok|ok_batch), stored
  `can_pos`/`can_quat` verbatim (incl. the two z=0.085 lying entries 234/318).
- **Fixed:** can_height 0.101, can_radius 0.033, substeps 8, table=True, static goal.
  `trial_placements.json` is NEVER written by this pipeline.
- **Dims (6), searched in ln-space, θ0 = current world:**
  finger_kp 40 [10,120] σ.45 · finger_force 50 [10,150] σ.45 · can_rho 1000
  [700,1400] σ.15 · can_friction 0.2 [0.05,1.2] σ.55 · table_friction 0.5 [0.1,1.5]
  σ.45 · goal_friction 2.0 [0.4,4.0] σ.45. (σ = initial CEM std in ln units.)
- **Per-demo stage** (exact recovery-campaign rule, incl. proximity nested ≤ 0.081):
  nested > contact > placed > picked > none. **Score = Σ stage value**
  (0/1/2/3/5). Tiebreak: lower mean final-can→goal distance over picked demos.
  Regression guard: score := −∞ if picked_count < baseline_picked(fit) − 2.
  Rollout exception → demo scored none (logged).
- **Baseline first:** θ0 on all 74, ×2, 6-way load. Flip count recorded = noise floor.
- **Split:** stratified by baseline-rep1 stage, seed 0, ~1/3 per stratum → holdout
  (~24), rest fit (~50). Frozen to `split.json` before the first search candidate.
- **Optimizer:** CEM pop 10 = 8 sampled + θ0 + best-so-far; elites top-3; smoothing
  α 0.6 on mean and σ; σ floor 0.04; clip to bounds. Deadline-bounded (17:00 EDT);
  checkpoint per generation.
- **Validation:** top-2 distinct candidates → ×2 re-evals at 6-way (fit set);
  champion = best mean. Then holdout, SOLO runs: θ0 ×1 (solo determinism proven
  07-20 ×3), champion ×2 (if the two reps disagree, the WORSE one is used for the
  decision rule; both reported).
- **Decision rule (holdout, solo):**
  - **CONFIRM headroom:** champion (contact+nested) ≥ θ0 + 3 demos AND picked ≥
    θ0 − 1 AND placed ≥ θ0 − 1 → earns the cluster-scale run / world-v3 discussion
    (user decision; PREREG amendment + full re-collect + same-machine re-baselines
    would all be required before any training claim).
  - **EDGE:** +1..+2 → headroom at the noise edge; reported, no world-v3 claim.
  - **NULL:** ≤ 0 → physics knobs at fixed placements don't lift the funnel;
    reportable null (supports "control/placement-limited" framing).
- **Non-actions:** frozen matched sets untouched; no dataset collection; no
  trial_placements world-block change; env-basin spot-check required before any
  training use of a winner.

## §3 Measured instrument facts (pre-launch)

- Box: 12 logical cores, idle (only desktop daemons). GPU idle by design (§1.12).
- Sim: 19.5 ms/cmd (θ0 world, solo). Mean tape 1976 cmds ≈ 39 s + 100-step settle.
- Build: 135 s cold / 14.3 s warm across param values (taichi offline cache, 6.1 MB).
- `constraint_timeconst` rejected by 0.2.1 RigidOptions (probe log
  `wsearch/probe_ct.log`).
- Census: 77 success-labeled = 75 real + stubs 290/322; solved 74 (42 ok + 32
  ok_batch); unsolved 303. Non-identity quats: 234, 318 (z 0.085). 72 entries z 0.113.
- Expected throughput: ~8.3 min/candidate (50 fit demos, 6 workers) → ~7–8
  generations by deadline.
- Baseline + flips: filled in by driver (`world_search/baseline.json`).

Outputs: `can_pos_recovery/world_search/` (split.json, baseline.json, gen_*.json,
state.json, validation.json, results.json, drive.log). Code:
`can_pos_recovery/world_search.py`.

## Amendment 1 (2026-09-02 ~14:30 — BEFORE any holdout contact; user-directed)

User extended the budget to tomorrow morning and asked whether placement re-fit
should interleave with the physics search. Adversarial critique of the sequenced
plan (in-session) produced this synthesis. The holdout remains sealed; nothing here
touches the verdict's inferential core (fixed placements, sealed holdout, solo runs,
decision rule unchanged).

1. **Stall rule (new, procedural):** search stops at the FIRST of (a) best-ever
   unimproved for 3 consecutive generations, (b) 21:30 hard cap. Rationale:
   fit-score curve decelerating (143-ref: 150/151/151/156/156/161/161 through
   gen 6); marginal generations mostly buy winner's-curse exposure. The old 17:00
   deadline replaced at the gen-6/7 boundary (drive restarted from checkpoint;
   gen 7's first evaluation, a θ0 re-eval scoring exactly 143 again, was discarded
   by the restart).
2. **Validation widened** top-2×2 → top-3×2 (selection from ~100+ noisy evals needs
   more defense); champion per-demo stages now persisted (they were discarded —
   phase 2 needs them for targeting).
3. **Phase 2 (tonight, AFTER results.json exists): placement re-fit under the
   validated champion.** `can_pos_recovery/world_refit.py`. Targets: demos below
   nested under the champion (min stage across champion reps), EXCLUDING lying
   incumbents 234/318 (upright-only grids can't re-fit them without changing their
   semantics). Grids: `cpu_research.can_grid` verbatim — wide for demos below
   placed, narrow otherwise — anchored at incumbent xy + grasp-closure xy;
   incumbent is scored IN-PASS (same process) and is the bar; upgrades require ×2
   reproduction, final stage = min of 3 observations (flakes rejected). Output:
   `world_search/placements_v3.json` + `refit_results.json`.
   **trial_placements.json is never written.**
4. **Claim discipline:** phase 2 is a disclosed RECOVERY pass for the new world with
   an explicit budget (logged rollout count), NOT a controlled comparison — θ0's
   placements embody months of heterogeneous search, so "matched budget" is
   unattainable; the controlled physics claim remains §2's fixed-placement holdout
   verdict. The joint funnel (champion + re-fit vs θ0 + original) is reported as
   the ceiling estimate for "what we have to work with."
5. **Phase 3 (proposed, needs user + this doc's rules re-instantiated):** second
   physics pass at the NEW placements as a fresh frozen frame — new stratified
   split, champion as the new θ0. This delivers the user's "re-fit then continue"
   loop as clean coordinate ascent, one honest baseline per step.

## §4 Results (2026-09-02 19:29 — filled in AFTER the sealed holdout; nothing above edited)

**Verdict: CONFIRM** under the §2 decision rule, with the world-identity caveat in §4.3.

### 4.1 Search
- Baseline θ0 on all 74, ×2 at 6-way load: picked 73 / placed 57 / contact 33 /
  nested 28 (score 219), **0 flips**. Fit-set θ0 = 147 under baseline sharding,
  **143 under search sharding in every generation** (same-shard determinism; the
  4-point offset is within-process call-history dependence, not noise — every
  candidate is compared at the 143 reference).
- 10 generations (100 evals, ~7 min each, 6-way). Best-ever per gen:
  150 / 151 / 151 / 156 / 156 / **161** / 161 / 161 / 161 / 161 → STALL-STOP at gen 9
  (Amendment 1 rule). Population mean 121 → ~150.
- Validation (top-3, ×2 re-evals at search load): 161→[161,161], 159→[159,159],
  158→[158,158]. **Champion = gen-5 candidate:** finger_kp 44.8, finger_force 69.9,
  can_rho 1039, can_friction 0.168, table_friction 0.346, goal_friction 1.068.
  Fit-set counts 49 / 40 / 26 / 23 vs θ0 49 / 36 / 20 / 19.

### 4.2 Sealed holdout (24 demos, SOLO runs, idle box)
| | picked | placed | contact | nested | contact+nested |
|---|---|---|---|---|---|
| θ0 ×1 | 24 | 17 | 9 | 7 | 16 |
| champion rep 1 | 23 | 18 | 11 | 11 | 22 |
| champion rep 2 | 23 | 18 | 11 | 11 | 22 |

Reps identical (0 flips). Decision rule: contact+nested +6 ≥ +3 ✓; picked 23 ≥ 24−1 ✓;
placed 18 ≥ 17−1 ✓ → **CONFIRM**. Per-demo: 246 placed→nested, 248 contact→nested,
257 picked→nested, 316 contact→nested; **234 picked→none** (one of the two lying-can
z=0.085 entries — the only regression, and the class Amendment 1 already excludes
from re-fit).

### 4.3 What this does and does not say (read before citing)
1. **Wrong world for the paper.** The search ran on the `trial_placements.json` world
   block (finger_kp 40 / finger_force 50, no gravity comp, base arm gains, no riser /
   shelf6, contact timeconst 0.02) because that is the basin every placement was
   validated in. The paper's corrected world is `gc_kp4_riser3_shelf6` (arm kp×4,
   gravity comp, riser 3 cm, shelf +6 cm) — and, confirmed by the user from the
   cluster session on 09-02, **without** `grasp_timeconst 0.005`. This CONFIRM is a
   pilot on the legacy world; it is NOT a world-v3 recommendation.
2. **One dimension was inert by construction.** Finger–can friction is
   `max(URDF 1.0, can_friction)` (gripper_lab §1.3), so can_friction < 1.0 cannot
   affect the grasp; the search's drift 0.2 → 0.17 is noise-fitting. Reported as-is.
3. **Direction of the gains is consistent with compensating the soft-contact grasp:**
   finger_force 50 → 70 (population mean drifted to ~82), goal_friction 2.0 → 1.07,
   table_friction 0.5 → 0.35; all gains downstream of pick (the pick funnel was
   already 73/74). The gripper lab's `grasp_timeconst 0.005` fixes the defect these
   knobs are compensating for (penetration 9 → 0.9 mm; tips 38 → 23; nested 17 → 20 on
   the corrected world). **Next physics work belongs on `gc_kp4_riser3_shelf6_ts5`,
   not here.**
4. **Two basins, fixed placements.** Replay-basin numbers at placements fit under θ0
   (§1.11, §1.13). No env-basin spot-check has been run. No training claim.
5. Phase 2 (`world_refit.py`) was built but **NOT launched**: re-fitting placements to
   a legacy-world champion has no paper use once (1) is known. Kept for phase 3 on the
   corrected+ts5 world if the user wants the loop there.

## Amendment 2 (2026-09-02 20:20 — the corrected+ts5 world; user-directed, written BEFORE launch)

User (evening): "you would just do the world change and the subsequent improvements, not
the retraining." Scope of this local session = world of record → `gc_kp4_riser3_shelf6_ts5`
(paper/TS5_DROPPED_FIX_2026-09-02.md: the frozen w3 sets carry the 0.02 s contact defect),
then the §2/Amendment-1 loop re-instantiated on that world. Dataset builds and all training
stay with the cluster/primary session. Nothing below edits §1–§4.

### A2.1 What changes vs §2

- **World:** θ realised through the paper recipe — `sim_variants.install(V)` before
  `build_world(θ, FIXED)`, `post_build(w, V)` after — with **V = `gc_kp4_riser3_shelf6_ts5`**
  (arm kp×4/kv×2, gravity comp 1.0, riser 0.03, shelf +0.06, finger+can contact timeconst
  0.005 s). θ0 = the same 6 values as §2 (= the world block every paper set was built on).
  Everything else FIXED as §2. Outputs: `can_pos_recovery/world_search_gc_kp4_riser3_shelf6_ts5/`.
- **Stage rule:** the gripper-lab / `fulltask_fidelity_lab.run_shard` predicates on the WHOLE
  tape (no stop-at-first-contact), shelf-aware: picked = can z > pick_z with grip closed;
  contact = picked & bottle–goal contact & eef x < can x (sampled every 30 cmds); placed =
  max z > pick_z & final z > shelf top (0.17); nested = picked & xy-dist ≤ 0.081 & both
  upright after the 100-step settle at tape end. Same values 0/1/2/3/5. (§2's
  `rollout()` placed-band is hard-wired to the un-raised shelf and its contact stop makes
  the trailing retreat invisible; the lab's rule is what the 08-25 ts5 evidence and tonight's
  same-box replication were measured with. Smoke: 232 → nested, 239 → placed, identical to
  the lab shard log.) Same-shard determinism assumed as §3 (baseline ×2 flips reported).
- **Split:** the SEALED `world_search/split.json` copied verbatim (fit 50 / holdout 24; the
  holdout uids were never evaluated by either search; strata were the legacy-world θ0
  stages — a random split w.r.t. this world, disclosed).
- **Gen-0 seeds (transfer hypothesis):** population 0 = θ0 + the legacy champion θ_L
  (ff 69.9, goal_fric 1.07, table_fric 0.35, can_fric 0.17, kp 44.8, rho 1039) + 8 samples.
  One extra evaluation; tests whether the legacy gains were world-specific compensation.
- **Budget:** gens until 05:30 (a gen that would overrun is not started) or the stall rule
  (Amendment 1.1a); then validation top-3 ×2 (fit), holdout SOLO θ0 ×1 + champion ×2 on an
  otherwise idle box; decision rule = §2 verbatim.

### A2.2 Pre-registered predictions (falsifiable, written before the baseline ran)

- **P1 (transfer):** θ_L scores ≥ θ0 + 3 (contact+nested) on the fit set at gen 0. If θ_L
  < θ0, the legacy gains were compensation for the soft contact, not physics.
- **P2 (direction):** the ts5 champion's finger_force ≤ 50 (θ0) — the opposite of the
  legacy champion's 50 → 70. Rationale: with stiff contact the fingers stall at the surface
  holding full PD error (147 → 245 N in the lab); more force should not help, less might.
- **P3 (headroom):** search best on fit ≥ θ0 + 5 points; holdout verdict CONFIRM with
  probability ~0.5 (the legacy pass found +6 on 24 holdout demos; the ts5 world already
  removed the largest defect, so less headroom is expected).
- **P4 (the reference numbers):** θ0 on this world, all 74, lands within ±2 of the lab's
  g_stiff5 counts restricted to the same uids (same-box replication running as
  `gripper_lab.py full --cfg g_stiff5`, §3 of TS5_DROPPED_FIX).

### A2.3 Decision rules for what follows the verdict

- **CONFIRM →** phase 2 (`world_refit.py --variant …`, Amendment 1.3 rules verbatim, lab
  predicates) under the champion; the champion becomes a PROPOSED variant
  `gc_kp4_riser3_shelf6_ts5_w4` in `sim_variants.py` (install/post_build extended to carry
  finger_kp/can_rho/frictions) for the cluster session to adopt or not — no built set
  changes here.
- **EDGE / NULL →** the world of record stays `gc_kp4_riser3_shelf6_ts5` at θ0; phase 2
  runs under θ0-ts5 (placements that the ts5 world lost — the lab's Appendix A names
  255/303/316 picks, 247/267/297/320/330 nests — are re-fit there).
- Either way: **`trial_placements.json` is never written**; phase-2 winners go to
  `placements_v3.json` in the run dir with the same ×2-reproduction rule; a human-tape
  re-collection on the final world (recorder path, env basin) is the last local step and
  is reported as a keep-rate + stage census, not as a training set.

### A2.4 Results log (appended as they land; predictions above untouched)

- **20:23** search launched (`drive --variant gc_kp4_riser3_shelf6_ts5 --deadline 05:30
  --workers 6 --pop 10 --split-from world_search/split.json --seed-theta θ_L`); worker logs
  show `[sim-variant] grasp_timeconst 0.005 s on 5 geoms`.
- **20:44 baseline θ0, 74 demos:** rep1 = rep2 = **69 / 57 / 27 / 24** (picked/placed/
  contact/nested, hierarchical stages), 0 flips, 614 s/rep. For reference the legacy world's
  θ0 was 24/17/9/7 on the 24-demo holdout; full-74 legacy baseline was in
  `world_search/baseline.json`.
- **P4 scored: PARTIAL.** Same-box lab `g_stiff5` on the same 74 uids (303 is lab-only):
  picked 69 = 69, placed 57 = 57, contact-or-nested 27 = 27 — exact; **nested 24 vs 19
  (+5, outside ±2)**. Per-uid: 12/74 disagree, both directions (search higher: 245, 254,
  259→nested, 298→nested, 302→nested, 317→nested, 320→nested, 330→nested; lab
  higher: 235, 246, 267, 311). Predicates are the same code path (whole tape, 100 settle at
  end, picked & xy ≤ 0.081 & tilts < 20; goal z 0.263 in both); the pipelines differ in
  process structure (lab: fresh process per episode + per-step contact/penetration reads;
  search: 6 shards, many tapes per scene). So the same world + same predicates on the same
  box reproduce the aggregates exactly at picked/placed/contact and to ±5 at nested,
  with a 16% per-uid disagreement rate. **Consequence for the verdict rule:** a CONFIRM
  (+3 within the search pipeline, solo holdout, ×2) is a within-pipeline signal; the
  cross-pipeline noise floor at nested is ~5 on 74, so any champion must reproduce its gain
  in the recorder-path census (A2.3) before it is called a world improvement.
- **20:58 gen 0, fit set (50):** cand0 = θ0 → **146** (47/41/20/19); cand1 = θ_L (legacy
  champion) → **137** (47/41/19/15). (θ0 re-scored alone = 146 vs 142 inside the 74-demo
  baseline sharding: the same-shard effect, +4 on 50 demos, within-pipeline.)
- **P1 scored: FALSIFIED.** θ_L is −9 score / −5 contact+nested vs θ0 on the ts5 world at
  the same pick/placed. Per the pre-registered reading: the legacy champion's gains (finger
  force 50 → 70, goal/table friction ↓) were compensation for the soft-contact grasp, not
  transferable physics. The legacy search result (§4) is now labelled as such; it must not
  be cited as evidence for any physics parameter on the paper world.
- **21:54 gen 0 done** (10 cands, 7 min each at 6-way): best cand3 = **154** (47/44/23/20,
  +8 score / +4 c+n over θ0 in-generation) with finger_kp 72, finger_force 77, can_rho
  900, table 0.38, goal 2.04; runner-up cand6 = 150 (kp 60). The two worst (114, 125) are
  the two lowest finger_kp (14, 25); cand9 tripped the picked guard (table 0.96, goal 4.0).
  Early read: on the stiff-contact world the lever is finger stiffness, not force —
  but P2 is scored on the validated champion, not gen 0.
- **~22:00 cluster-session answers relayed** (`TS5_DROPPED_FIX` §4): ts5 dropped by
  omission; no reported arm on ts5; §8 pick-recreation re-run was a wash (57 vs 58/66);
  **E5 caveat: ts5 also stiffens can–goal/can–shelf contacts 1.6× (sol_params averaged)**,
  so contact/nested gains on this world are not purely grasp effects; cluster
  recommendation B (disclose) unless a champion shows a LARGE CLOSED-LOOP gain; rebuild
  target if any = θ0-ts5. A2.3 CONFIRM branch amended: the `_ts5_w4` variant proposal
  must (i) be a new name only, (ii) make `post_build` assert the geom count it touched,
  (iii) ship with a selftest — and the replay verdict is explicitly NOT the bar for
  option A; a closed-loop eval of an existing w3 checkpoint on w3 vs ts5 is.
- **01:25 gens 1–3:** gen 1/2 no new best (stall 2); **gen 3 cand4 = 159** (47/43/25/22,
  +13 score / +8 c+n over θ0 in-generation): kp 64.3, force 59.3, **can_rho 853**,
  can_friction 0.068, table 0.44, goal 2.12. Across 40 candidates the only clear lever is
  finger_kp (≥ 55 → 148–159; ≤ 41 → 132–146); force/frictions inert within noise. Flag for
  the morning: rho 853 = a 0.295 kg can vs the 0.35 kg (rho ≈ 1010) the prereg computed —
  a lighter-than-real can is a fit-to-demos smell, inside the registered bounds but worth
  a real measurement (weigh the can) before it becomes a world of record.
- **06:24 RESULT: CONFIRM.** Stall-stop gen 6 (70 candidates); validation top-3 ×2 exact
  (159/159, 157/157, 155/155); champion = gen-3 cand4 {kp 64.34, force 59.27, rho 852.5,
  can_fric 0.068, table 0.443, goal 2.118}. **Holdout SOLO (24): θ0 22/17/9/6 → champion
  23/20/13/10 ×2 identical** (c+n 15 → 23 = +8 ≥ +3; picked +1, placed +3; UP 7: 244→placed,
  246→nested, 257→nested, 263→nested, 297→contact, 316 none→picked, 333→nested; DOWN 1:
  311 contact→placed). Fit (50, min of 2 val reps vs baseline): UP 8 / DOWN 2 (298, 320
  nested→contact). All-74: 69/57/27/24 → **70/63/38/32**. P2 FALSIFIED (force 59 > 50, though
  force was inert across the population); P3 CONFIRMED (fit +13 ≥ +5; holdout CONFIRM).

### A2.5 Post-hoc ablations, pre-registered before running (07:20, 09-03)

Two champion dims are physically suspect and, by the population tables, likely inert:
`can_rho 853` (a 0.295 kg can; the prereg's computed value is ≈1010 for the 0.35 kg can) and
`can_friction 0.068` (under the pair rule friction = max(a, b) it is dominated by table 0.44
and goal 2.12 everywhere, gripper_lab §1.3). Real2sim rule: where the fit is indifferent,
use measured reality. Two single evals, fit set, 6-way, same sharding as validation
(champion reference 159 = 47/43/25/22):

- **R (rho):** champion with can_rho = 1010. Rule: if score ≥ 157 and picked ≥ 47 → the
  proposed variant uses rho 1010; else champion rho verbatim, flagged for a weigh-in.
- **F (friction):** champion with can_friction = 0.2 (θ0). Rule: expected BIT-IDENTICAL
  stages (max rule); if identical → variant uses 0.2 and the doc states can_friction is
  inert on this world; if not identical → champion value verbatim and §1.3's rule is
  re-examined.
- Then the chosen θ_w4 is scored on all 74 in one 6-way pass (baseline-shaped run) to give
  the per-uid stages phase 2 refits from, and on the holdout SOLO ×1 as the number of record
  for the variant (must stay ≥ champion −2 c+n, else revert to the champion verbatim).

#### A2.5 results (appended as they land)

- **07:23 R scored: rho 1010 → 147** (47/42/22/18) vs champion 159 (47/43/25/22): −12 score,
  −7 c+n, picked unchanged. Per-uid: 7 DOWN (295, 330 nested→contact; 328, 267 nested→placed;
  298, 235 contact→placed; 325 placed→picked), 1 UP (274 placed→contact). **Rule → champion
  rho 852.5 verbatim, flagged for a weigh-in.** So rho is NOT inert: on this world an 18 %
  heavier can loses 4 nested on the fit set. Reading (not a claim): the lighter can may be
  standing in for a real effect the world lacks (e.g. the real can's contents / a lower
  centre of mass, or the real gripper's compliance) — the honest next step is the scale,
  not the fit. (First driver crashed on str-vs-int json keys after the R shards finished —
  `world_ablate.py` fixed, relaunched with `--reuse` on the six R part files; no re-run.)
- Tooling landed meanwhile: `baselines/sim_variants.py` gained the **`gc_kp4_riser3_shelf6_ts5_w4`**
  variant (new name only; a `world` block realized by material substitution at `add_entity`
  + finger kp/force in `post_build`; `post_build` asserts the 4 material swaps and the 5
  grasp geoms it touched; `--selftest` reads every claimed value back from the built solver
  — PASS for both `_ts5_w4` and the untouched `_ts5`); `world_search.build_theta_world`
  refuses a variant that carries a `world` block (double-application guard). The variant's
  values are provisional until F lands (rho fixed at 852.5 by R above).
- **07:33 F scored: can_friction 0.2 → 159, stages BIT-IDENTICAL to the champion (diff {}).**
  Rule → the variant uses 0.2; can_friction is inert on this world (pair rule = max: the can's
  0.07–0.2 never wins against table 0.44 / goal 2.12 / the finger geoms). **θ_w4 =
  {kp 64.34, force 59.27, rho 852.5, can_fric 0.2, table 0.443, goal 2.118}** — differs from
  the validation champion only in the inert dim, so its all-74 (6-way) and holdout SOLO ×1
  passes are running as the numbers of record for the variant (rule: c+n ≥ champion −2).
  `gc_kp4_riser3_shelf6_ts5_w4` now carries exactly θ_w4 (selftest PASS).
- **Recorder-path warning (1 uid, pre-census):** a full-scope `record_demos.py --teacher human`
  dry run on uid 233 (the user-validated nested demo; replay = nested under θ0 AND θ_w4) ends
  **contact** on `gc_kp4_riser3_shelf6` and `_ts5`, but only **picked** on `_ts5_w4` (can released
  at (0.77, −0.12), 14 cm from the goal). One uid is an anecdote; it is exactly what the A2.3
  recorder census is for. Census plan (idle box, sequential, 6 shards): pick scope (the paper's
  recipe, src `episodes_pick_phase_v2r_all`) AND full scope (src `episodes_all_v2r`, deepest env
  grant) × {w3, ts5, ts5_w4} on the 74 search uids — `can_pos_recovery/census_variants.sh`,
  summarized by `census_summary.py`. Pre-stated reading: the pick-scope keep-rate is the number
  the paper's arms depend on; if ts5_w4 loses ≥ 2 picks vs ts5 there, or its full-scope c+n does
  not beat ts5's, the variant is NOT proposed for retraining regardless of the replay verdict.
- **07:44 θ_w4 all-74 (6-way, OLD harness): 70/63/34/27** vs the champion's "all-74" 70/63/38/32
  (= fit 6-way + holdout SOLO). Per-uid: on the FIT uids the identical θ_w4 physics scored
  47/44/21/17 in the 74-uid partition vs 47/43/25/22 in the 50-uid F partition (7 flips:
  242/267/302 nested→placed, 328/330 nested→contact, 286 picked→placed, 298 contact→placed);
  on the holdout 4 flips vs the champion's SOLO. Same θ, same box, idle both times — the only
  difference is which tapes preceded each uid in its worker process. → Amendment 3.

### Amendment 3 — replay-harness history leak (pre-registered 07:55, 09-03, before any re-score)

**Finding.** Every replay-path reset (`replay_harness.rollout`, `world_search.score_variant`,
`fulltask_fidelity_lab`, `gripper_lab`, `render_*`, `remeasure_contact`) did
`set_dofs_position(HARDCODED_START)` + `zero_all_dofs_velocity()` + `scene.step()`. Genesis
0.2.1 keeps the PREVIOUS episode's `control_dofs_position` target across the teleport (fresh
process: CTRL_MODE.FORCE, zero torque), so that reset step — and the arm/finger state the
tape's first command meets — depends on the tape that ran before in the same process.
`GenesisCanEnv.reset` was fixed for exactly this on 2026-08-14 (P2 FIX); the replay path was
not. **Probe (`can_pos_recovery/leak_probe.py`, θ_w4 on ts5, uid 242):** solo → nested prox
0.0682; after 305 → nested 0.0690; after 263 → **placed** 0.0812 (the two partition contexts,
reproduced by the immediate predecessor alone). With the PD target re-issued at reset: nested
0.0756 in all three contexts, bit-identical. Fix landed as `replay_harness.reset_arm()` and is
now used at all seven reset sites (patched files compile; probe re-run on the patched
`score_variant` reproduces 0.0756).

**What it means for the numbers so far.** (i) Every 6-way replay census was a
partition-specific experiment: comparisons made WITHIN one partition (all CEM candidates vs
θ0 on the fit-50; θ0 vs champion holdout SOLO in the same 24-uid order) were fair — every θ
met the same history — but their run-to-run noise was never measured, because "×2 exact"
reproduced the same history. The 7/50 fit flips from a partition change alone (−9 c+n on
θ_w4) are the same size as the search's headline holdout gain (+8 c+n). (ii) The
gripper-lab / fidelity-lab tables and the recovery campaign's placements (cpu_research runs
many candidates per process) carry the same artifact; magnitude unknown — flagged to the
head agent as a CONFOUNDS row, not re-run here. (iii) The recorder path (`record_demos` →
`FullTaskEnv.reset` → `GenesisCanEnv.reset`) has been history-independent since 08-14, so
the recorder census (A2.3) is unaffected and remains the decisive gate.

**Re-score plan (`can_pos_recovery/world_rescore.py`, fixed harness, 6-way, all 74 uids;
with `reset_arm` the partition is irrelevant, so 6-way = solo):** θ0 on `_ts5`, θ_w4 on
`_ts5` (= `_ts5_w4`), θ0 on `gc_kp4_riser3_shelf6` (paper world); plus a NOISE FLOOR: θ0-ts5
and θ_w4-ts5 under 1 mm can-placement jitters (+1,0), (0,+1), (−1,−1) mm (well inside the
placements' own uncertainty), i.e. 4 conditions per world. Old-harness A2/A2.5 numbers stay
in this file as the record of what was run but are superseded for every decision below.

**Decision rules (fixed harness):**
- R1 (holdout, the A2 rule re-applied): θ_w4 vs θ0 on `_ts5`, sealed holdout 24, nominal
  placements: contact+nested gain ≥ +3 AND picked not lower by > 1 → CONFIRM; else REVERT: the
  `world` block is withdrawn from `gc_kp4_riser3_shelf6_ts5_w4` (the proposal becomes plain
  `_ts5`, θ0) and the CEM must be re-run on the fixed harness before any θ is cited.
- R2 (noise floor, all-74): min over the 4 conditions of θ_w4's c+n > max over the 4
  conditions of θ0's c+n → "gain above the jitter floor"; otherwise "within floor" — disclosed
  as such, and the variant is NOT proposed for retraining on replay evidence alone (the
  recorder census rules of A2.5 still apply unchanged and could still carry it).
- R3: θ0-ts5 vs θ0-w3 (nominal) is descriptive only (the ts5 effect on the replay path under
  the fixed harness) — no decision hangs on it.

**Predictions.** P5: the fixed-harness holdout gain is smaller than the old +8 c+n (part of
the old gain is history artifact) but still ≥ +3 (finger_kp was the one consistent lever
across 70 candidates). P6: the 1 mm jitter floor on all-74 c+n is ±3–6 for either world (the
7/50 partition flips set the scale). P7: θ0-ts5 ≥ θ0-w3 on c+n (the gripper-lab direction).

#### Amendment 3 readout (fixed harness `reset_arm`, 09-03 08:08 / 08:21)
- θ0 on `_ts5`, nominal: all-74 **69/57/28/22** (c+n 50) · fit 47/41/20/16 · holdout **22/16/8/6** (c+n 14).
- θ_w4 on `_ts5`, nominal: all-74 **69/63/36/29** (c+n 65) · fit 47/42/23/20 · holdout **22/21/13/9** (c+n 22).
- **R1 (holdout, nominal): c+n +8, picked 22 = 22 → CONFIRM** (rule: ≥ +3 and picked not lower by > 1). Fit +7 c+n; all-74 +15 c+n.
- Per-uid (all-74, * = holdout): UP 13 — 244*:picked→nested, 245*:picked→placed, 246*:picked→nested, 247:picked→nested, 295:contact→nested, 297*:picked→contact, 298:placed→contact, 306:placed→nested, 311*:placed→contact, 317:contact→nested, 320:contact→nested, 328:placed→contact, 333*:picked→nested. DOWN 2 — 235:contact→placed, 330:nested→contact.
- P5 (leak explains the θ_w4 "all-74 regression"): SUPPORTED — with history-independent resets θ_w4 beats θ0 on every split; the old-harness 70/63/34/27 vs 47/43/25/22-on-fit was a partition artifact.
- R2 (jitter floor) and R3 (θ0-ts5 vs θ0-w3): PENDING — 7 more conditions running.
- 08:33 **R3 (descriptive): θ0 on the paper world `gc_kp4_riser3_shelf6` (no ts5), fixed harness: all-74 69/62/31/19 (c+n 50) · fit 46/42/21/12 · holdout 23/20/10/7.** vs θ0-ts5 69/57/28/22 (c+n 50): adding grasp_timeconst ALONE at θ0 is c+n-neutral on the replay funnel (nested +3, contact −3, placed −5). The gain sits in θ_w4 (gains/mass/frictions re-fit ON ts5), not in ts5 by itself → post-hoc descriptive add-on queued: θ_w4 on the paper world (`w4_w3`), to separate "re-fit" from "re-fit + ts5". Not pre-registered; reported as descriptive only.
- 09:35 **Post-hoc descriptive `w4_w3`: θ_w4 on the paper world (no ts5): all-74 66/53/22/15 (c+n 37) · fit 44/36/16/10 · holdout 22/17/6/5.** The 2×2 on the fixed harness (all-74 c+n): θ0-w3 **50** · θ0-ts5 **50** · θ_w4-w3 **37** · θ_w4-ts5 **65**. ts5 alone is neutral, the re-fit alone is HARMFUL (−13), together +15 → the gain is the interaction (gains/mass re-fit *on* the 5 ms grasp time-constant), not a parameter tweak that could be ported to w3. Caveat: θ_w4 was fit on ts5, so θ_w4-w3 is not "the best θ on w3"; it only shows the two changes are not separable. Proposal stays exactly `gc_kp4_riser3_shelf6_ts5_w4`.
- 09:22 Bug: the (−1,−1) mm jitter pair failed at launch (`--jitter -0.001,-0.001` parsed as a flag; fixed to `--jitter=` in `eval_candidate`), re-running; R2 on the three completed conditions: θ_w4 c+n {65, 64, 67} vs θ0 {50, 50, 48} → min θ_w4 − max θ0 = +14, θ0's own jitter spread 2 → **gain above the floor** (final 4-condition numbers below when the re-run lands).
- 10:00 **R2 FINAL (4 conditions, all-74 c+n): θ_w4 {65, 64, 67, 67} (min 64) vs θ0 {50, 50, 48, 49} (max 50) → gain ABOVE the 1 mm jitter floor by 14.** Holdout c+n: θ_w4 {22, 22, 22, 21} vs θ0 {14, 16, 14, 14}; picked 68–70 vs 68–69 in every condition. **Amendment 3 verdict: θ_w4 CONFIRMED on the fixed harness (R1 + R2); the variant `gc_kp4_riser3_shelf6_ts5_w4` stands as proposed.** Replay-path evidence only — the recorder-path census (pre-stated gate: ts5_w4 loses ≥2 picks vs ts5, or no full-scope c+n gain → NOT proposed) is next. All numbers in `world_search_gc_kp4_riser3_shelf6_ts5/rescore.json` (`harness: reset_arm`).

#### A2.3 recorder-path census readout (09-03 11:20 census DONE; scored 11:25–11:45)
Recorder path = `record_demos.py --teacher human --ic-mode demo --arrival either --max-sim-steps 12000`,
74 search uids, idle box, 6 shards, `baselines/demos_v2/census_ts5w4_0903/`. Cumulative ≥stage counts
(the recorder grants picked/contact/nested; no `placed` grant in this build). c+n = contact + nested.

| scope | variant | picked | contact | nested | c+n |
|---|---|---|---|---|---|
| pick | w3 `gc_kp4_riser3_shelf6` | 68 | – | – | – |
| pick | `_ts5` | 66 | – | – | – |
| pick | `_ts5_w4` | 67 | – | – | – |
| full | w3 | 69 | 26 | 5 | 31 |
| full | `_ts5` | 67 | 22 | 5 | 27 |
| full | `_ts5_w4` | 69 | 21 | 7 | 28 |

- **Gate as pre-stated (vs ts5): picks 67 vs 66 (not ≥2 lost), full c+n 28 vs 27 → passes by +1.**
- **Against the paper world w3 (the one the frozen sets came from): c+n 28 vs 31 (nested +2, contact −5);
  picks 69 = 69.** Per-uid ts5_w4 vs w3: UP 9 (261/262/304 picked→nested, 320 contact→nested,
  263/311/315/328 picked→contact, 301 none→picked), DOWN 12 (242/250 nested→picked, 233/236/237/273/286/
  298/321/325/326 contact→picked, 316 contact→none). No recorder-path noise floor was measured (a 1 mm
  can-jitter census would cost ~40 min per scope×variant); the churn (9 up / 12 down) says ±1 is not
  resolvable.
- **The replay-path gain (+15 c+n all-74, jitter-robust, Amendment 3) does NOT transfer to the recorder
  path.** θ_w4 was fit by CEM in the replay harness (open-loop PD targets at tape rate); the recorder's
  closed-loop follower (delta cap 0.025 / arrival test / leash) is a different basin near contacts.
  Lesson: a world fit must be done in the pipeline that builds the datasets.
- **Real-ground-truth fidelity (user request 09-03; `can_pos_recovery/real2sim_fidelity.py`, timed bag
  re-extraction `extract_real_timed.py`, frame-exact with the reader of record on 71/74 uids, one
  window differs on 254/295/300):** tool path vs the bag's `tool_pose(t)`, same instant: median 1.6 /
  1.7 / 1.7 cm (p90 6.7 / 6.4 / 6.9 cm) full scope, 0.3 cm (p90 ~3.2) pick scope; shape after optimal
  time-warp 0.3 cm; mean |time lag| 0.78 / 0.90 / 0.90 s full (0.33–0.43 pick); tape-duration ratio
  sim/real 0.98 / 0.98 / 0.97; closure −0.12 s, lift +0.10 / +0.22 / +0.21 s, release −0.17 / −0.16 /
  −0.15 s; the env's `picked` grant lands +0.06 / +0.10 / +0.12 s after the human's lift (median).
  Mismatched-pair negative control 15–17 cm. **Paired per-uid |error| deltas vs w3 are 0.00 on every
  metric with balanced signs** (e.g. same-instant 31 better / 25 worse / 18 same for ts5_w4). The real
  CAN trajectory is not available (no camera click pass) — contact/nested have no real anchor beyond
  the tape end (sim contact grant lands 5–12 s before the human stops, all worlds).
- **Verdict for the retraining question ("a world that gives better real2sim"): NOT demonstrated.**
  On the ground truth we have (tool position + timing, can-up timing) the three worlds are
  indistinguishable; on recorder-path outcomes ts5_w4 is +1 vs ts5 and −3 vs w3, inside the churn.
  I do not recommend proposing `gc_kp4_riser3_shelf6_ts5_w4` for retraining; the variant stays in
  `sim_variants.py` as a named, documented, selftest-passing option. Phase-2 placement refit under
  it is NOT run. User decides.
- Videos for review (recorder-path re-execution, bit-exact `max_obs_dev=0`): the nested flips
  242/250 (w3 nested → ts5_w4 picked) and 261/262/304/320 (→ nested under ts5_w4), real | w3 |
  ts5_w4 with wrist cams, `can_pos_recovery/videos_census/<uid>_full_gc_kp4_riser3_shelf6_vs_gc_kp4_riser3_shelf6_ts5_w4.mp4`.

##### A2.3 addendum (09-03 12:20) — anatomy of the real2sim time offset (user question "why are we lagging the human?")
Not a systematic lag: signed DTW offset by human phase (w3, full, 74 uids) approach +0.00 s / carry −0.12 s / after release −0.48 s (sim slightly AHEAD late in the demo); fraction of demos where sim is later 0.41–0.46. The |lag| 0.7–0.9 s is the size of a per-demo offset that swings both ways, and 2/3 of it is the replay clock: `HumanFollower` plays 4 frames / 0.12 s (33.3 fps) while the reader-of-record tapes run 29.6–39.2 fps per bag (median 32.8; both robot topics are 40 Hz, the 1/60 s both-topics window drops 2–26 % of samples as a beat artifact). Lag at tape end correlates +0.91 with the clock-only prediction; removing the clock term: median |lag| 0.72 → 0.25 s (full), 0.36 → 0.15 s (pick). Worst cases 287 (+16 s by 126 s, tape 38.3 fps, 0 dwell stalls) and 286 (+17 s, 37.6 fps) are pure rate mismatch. Residual 1–2 decisions = PD/arrival lag of the follower + the 4-frame block (grip target taken from the block's last frame → closure 0.12 s early; lift 0.1 s late; picked grant +0.06 s after the human's lift-off). CONFOUNDS row 46. Fix (not applied): time-scheduled waypoints from `_timed.npz`.
