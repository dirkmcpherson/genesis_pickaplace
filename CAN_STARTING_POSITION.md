# Trial-specific can starting-position — status & handoff

_Working notes from a debugging session. Goal: give each demo episode the **true** can
starting position instead of the current 3-bucket guess._

---

## TL;DR
- The can's true start pose was **never recorded** — each `inthewild_trials/<uid>_episodes.npy`
  holds only `vel_cmd` (arm joint angles) + `gripper_pos`. No object pose.
- `example.py` currently snaps every episode to one of **3 hand-coded positions**
  (`POSITION_0/1/2`) via UID lists in `kinova.py`. Those buckets are **wrong by up to ~5 cm**
  and quantize away the real spread.
- **Working method:** recover each can position by **forward-kinematics on the recorded
  trajectory at the grasp instant** (pure FK, no physics). Deterministic, robust to human-demo
  noise. Tool: `can_pos_recovery/recover_can_position.py`.
- **Open question / blocked:** how to *validate* the recovered positions. Physics **playback is
  the wrong yardstick** (see "Why close-loop doesn't work"). Validate against vision / the
  robot's reported `tool_pose` instead.

---

## The problem in detail
- `trial_reader.py` builds the episodes from ROS bags but keeps only joint angles + gripper. It
  **reads and then discards** the robot's Cartesian `tool_pose` (`trial_reader.py:15`).
- `example.py` → `setup()` looks up the UID in `kinova.py`'s `TRIALS_POSITION_0/1/2` lists and
  places the can at one of three fixed `(x, y)` points (all at `x=0.4381`, only `y` differs).
- Consequence: the replayed arm reaches for where the can *really* was, but the sim put it
  somewhere else → grasp misses/clips. Real positions are continuous; buckets are 3 points.

### Measured bucket error (12-episode sample, 4 per position)
| pos | bucket (x, y)   | recovered mean (HIGH/MED conf) | x error |
|-----|-----------------|--------------------------------|---------|
| P0  | (0.438, +0.100) | **(0.475, +0.096)**            | ~+0.05 m |
| P1  | (0.438, −0.050) | (0.456, −0.032)                | ~+0.02 m |
| P2  | (0.438, −0.200) | (0.441, −0.165)                | ~+0.01 m |

---

## The method that works: FK-at-grasp (pure trajectory, NO playback)
`can_pos_recovery/recover_can_position.py`

1. **Find the grasp instant** robustly against human-demo noise (people exercise the gripper in
   mid-air and miss the can, so "first gripper close" is unreliable):
   - Binarize `gripper_pos` with an **absolute** closed threshold (rejects "never closed").
   - The real pickup = the closed interval that **lifts** and **releases near the known goal**
     (anchor on the fact that the can must end at the goal).
   - Grasp instant = **reach-bottom** (min EEF height) inside that interval.
   - Attach a **confidence** (HIGH/MED/LOW); LOW = a miss/never-closed → fall back, don't trust.
2. **Forward-kinematics** the recorded joint angles at that instant → world position of the
   **fingertip-midpoint** = can `(x, y)`. `z` = known table/ground height (~0.05).
   - This uses Genesis only as an FK calculator: `set_dofs_position(...)` then read link poses.
     **It never calls `scene.step()`** — no gravity, contacts, or dynamics. Deterministic.

### Result on the 12-episode sample
- **10/12 recovered with confidence**; the 2 LOW (`322`, `290`) are genuine misses where the
  gripper never closed — correctly flagged, not silently wrong.
- Recovered positions cluster tightly within each bucket (P0 x spans just 1.4 cm across 4
  independent episodes) — that consistency is the signal it's real.

### Per-episode recovered positions (sample)
```
uid pos  conf   recovered (x,y)      (LOW -> keep bucket)
235  0   HIGH   (0.478, +0.088)
232  0   HIGH   (0.488, +0.086)
257  0   MED    (0.442, +0.098)
242  0   HIGH   (0.493, +0.114)
322  1   LOW    -- miss (gripper never closed); FK leans toward P2 y, consistent w/ its p2_fail label
243  1   MED    (0.452, -0.037)
236  1   HIGH   (0.469, -0.055)
280  1   MED    (0.447, -0.004)
290  2   LOW    -- miss
278  2   HIGH   (0.445, -0.174)
237  2   HIGH   (0.453, -0.182)
234  2   HIGH   (0.424, -0.137)
```

### Why the blocking concern does NOT break recovery
The can can physically block/resist the arm during the demo — but `vel_cmd` is the robot's
**actual measured joint-encoder angles** (`/my_gen3_lite/joint_states`), not commands. They
already embed that interaction. The can being there is *why* the gripper stopped there. FK on
real achieved angles → true gripper pose → can position. Blocking helps, not hurts.
(Caveat: if a *failed* approach nudged the can before the successful grasp, the recovered point
is where the can was *at grasp*, which may differ from its original rest pose. Minor for clean
grasps; the confidence flag catches the messy ones.)

---

## Why "close-loop" (playback) does NOT validate this
We tried placing the can at the recovered position and re-running the replay to see if success
went up. It **did not** (`close_loop.py`: bucket 2/12 vs recovered 1/12). That is **not**
evidence the recovery is wrong — playback is the wrong test, for fundamental reasons:

1. **The recorded trajectory is a closed-loop human artifact.** The person felt the can
   resist/block and adjusted in real time. Replaying those joint-position commands **open-loop**
   against the can means the position controller jams into the can (force spikes, can shoved),
   with no feedback to adapt. Placing the can *correctly* can make playback *worse* by surfacing
   the blocking the human negotiated away.
2. **The success metric conflates three things** — grasp + transport + the place landing on the
   *fixed* `goal_bottle` at (0.6, −0.2, 0.19). Correct pickup doesn't fix the place.
3. **GPU contact non-determinism** flips borderline episodes between identical runs (e.g. ep 237
   passed in one baseline run, failed in another). With ~1–3 successes/12 the metric is noise.
4. Even baseline replay only reproduces ~**3/12 (25%)** of *real-world-successful* trials → the
   open-loop replay is low-fidelity regardless of can position.

**Takeaway:** validate the recovery with a method that doesn't depend on physics rollout (below),
and treat closed-loop control as the real fix for the task, not open-loop replay.

---

## Status
- [x] Confirmed the data lacks object pose; buckets are wrong (~5 cm, measured).
- [x] FK-at-grasp recovery implemented, robust to test-pulses/misses, confidence-flagged.
- [x] Recovered 10/12 sample episodes; flagged the 2 misses.
- [x] Established that playback/close-loop is the wrong validation (and wrong control approach).
- [ ] **IN PROGRESS / NEXT** (see below).

## Next steps (priority order)
1. **Validate recovery without playback.** Two gripper-independent ground truths:
   - **`tool_pose`**: re-process the raw bags keeping the Cartesian EEF pose that
     `trial_reader.py:15` currently throws away. Robot-reported, no FK/URDF mismatch. Compare to
     the FK estimate; should agree.
   - **Vision**: detect the can in frame 0 of the raw camera MP4s
     (`inthewild_trials/raw/user_<id>/cam_dev_video*`) + `config.yaml` (calibration), back-project
     to the table plane. Independent check.
2. **Run recovery over ALL ~77 successful episodes** (not just the 12 sample) → produce a
   per-episode `{uid: (x, y)}` table to replace the bucket lookup in `kinova.py` / `example.py`.
3. **Fix the place/goal too**, not just pickup: recover each episode's *release* point (already
   computed as a by-product) and/or make `goal_bottle` per-episode, so the success reward is
   actually reachable.
4. **Use the confidence flag as a demo-quality filter** — drop/triage the flagged misses.
5. **For task success, move to closed-loop control** (servo on can/EEF pose) instead of open-loop
   joint playback — the open-loop replay can't reproduce a closed-loop human demo.

---

## Tooling (in `can_pos_recovery/`, runs in the project venv)
```bash
.venv/bin/python can_pos_recovery/recover_can_position.py        # recovered positions + confidence
.venv/bin/python can_pos_recovery/validate_grasp.py <force|baseline>   # success across episodes/positions
.venv/bin/python can_pos_recovery/close_loop.py <bucket|recovered>     # the playback test (see caveats above)
```
These are standalone; none of them import or modify `example.py`.

## Environment notes
- venv at `.venv` (Python 3.12), `genesis-world` 1.2.0, torch cu130, runs on GPU.
- Robot meshes **vendored** into `kortex_description/` — no ROS install needed.
- `inthewild_trials/` is a **gitignored symlink** to the external dataset (too big to vendor).

## Related issues found along the way (not the position work, but relevant)
- Don't "fix" gripper clipping by lowering grip force — it **breaks the open-loop grasp** (drops
  the can). The current `example.py` finger-force cut (±10/±5) regressed pickup.
- Gyration/"wild" motion is separately reducible via solver `substeps` (orthogonal to position).
- `kinova.py` label bugs: UID **322** is in both `p1_succ` and `p2_fail`; **331** is duplicated.

---

# FOLLOW-UP SESSION (2026-07-04): root causes found, placements solved

## TL;DR of what changed
The FK recovery was **right all along** — validated against the robot's own `tool_pose`
from the raw bags (agreement to the millimeter at the grasp instant). What was wrong was
the **world model**, in four ways:

1. **The table is missing.** The bags prove it: `tool_pose_z` at grasp is 0.016-0.039 m
   above the ROBOT BASE. The cans sat on the robot's own mounting table (z=0.05 in sim
   coords, the "raise to account for table mount"), but the sim dropped them onto the
   ground plane at z=0 — 5 cm too low. Every grasp therefore closed near/above the sim
   can's rim instead of low on the can body like the humans actually grasped.
2. **The can was the wrong size and weight.** The trial videos show a Campbell's soup can
   (66 x 101 mm, ~0.35 kg). The sim cylinder was 70 x 75 mm at rho=2000 (~0.58 kg).
3. **The finger gain/force cut couldn't grip** (kp 20/5, +-10/+-5 N). Restored to Genesis
   defaults (kp 100) with +-50 N range. Note kp=200+ makes things WORSE (squeezes the can
   out of the pinch) — 100 is the sweet spot.
4. **substeps=1 drops the can mid-carry** (slow slip out of the pinch during transport).
   substeps=4 fixes carry stability (same knob validate_grasp.py's force mode used).

With the corrected world, the FK-recovered positions + a small per-trial search produce
working placements for most trials (see `can_pos_recovery/trial_placements.json`).

## Evidence trail
- `can_pos_recovery/fk_recovered.json` — FK grasp/release for all 96 trials.
  Grasp heights: mean z=0.070, p75=0.077, max=0.140 (sim frame) => impossible for a
  0.075-tall can on the ground, exactly right for a 101 mm can on a 0.05 table.
- Bags (`inthewild_trials/raw/user_23x/trial_data.bag`): `base_feedback.tool_pose` at the
  gripper-close instant matches FK fingertip-midpoint within ~1 cm in xy and ~1 mm in z
  (232: bag z 0.039 vs FK 0.089-0.05; 235: bag 0.016 vs FK 0.066-0.05).
  `joint_states` is a clean 40 Hz (dt p95 25.4 ms) — timing was never the problem.
- Videos (`cam_dev_video4/output.mp4`): Campbell's can, marked start circles on the
  robot's table, acrylic shelf rack, can inserted against the goal can.

## The pipeline that produced the placements (can_pos_recovery/)
- `fk_all_trials.py` — FK-at-grasp + release for every episode -> fk_recovered.json
- `replay_harness.py` — example.py-mirror rollout with checkpoints (pick/place/slide),
  configurable world; CPU backend is 15-20x faster than GPU for this single-env scene.
- `search_placements.py` — sequential per-trial search (pick spiral -> goal move -> verify).
- `batch_harness.py` / `batch_rescue.py` — batched-GPU search: B=32 candidate placements
  of one demo evaluated in a single rollout (open-loop replay => same commands per env).
  In-batch success uses a distance proxy for contact; winners re-confirmed single-env.
- `merge_and_validate.py` — merges search outputs into `trial_placements.json` and
  re-validates every solved trial with example.py's exact contact test.
- `example.py` auto-loads `trial_placements.json` (world corrections + per-trial can and
  goal positions). `--legacy` restores the old behavior, `--cpu` runs the fast backend.

## Open items
- A minority of trials still fail (never pick / drop in transit) even under the corrected
  world — candidates for demo-quality filtering (the two LOW-confidence "misses" 290, 322
  among them) or for closed-loop control rather than open-loop replay.
- The shelf is still a solid box; the real thing is an open acrylic rack whose deck the
  cans sit BETWEEN rails on. If residual failures matter, model the rack.
- Label bugs remain in kinova.py (322 in both p1_succ and p2_fail; 331 duplicated).

## FINAL NUMBERS (0.2.1 baseline, world v2, 2026-07-06)
Denominator: 75 legitimately-successful demos (77 labeled minus stub recordings 290/322).

- **Coverage: 50/75 (67%)** have a placement that completed the full task at least once
  (pick -> place -> slide to contact). CORRECTION: 8 additional "fallen-can" solutions
  were rejected -- the raw camera feeds are stored rotated ~90 deg and an upright can
  reads as lying; the collection protocol always starts cans upright. Those 8 demos
  (234 247 259 262 266 301 321 329) grasp very low on the can (fingers ~2cm above the
  table), which the sim gripper cannot do without clipping the table -- a finger
  collision-geometry fidelity issue, revisit post-upgrade.
- **Per-run rates (CPU backend, x3 reps, exact contact test):**
  contact-success 0.53, nested-success 0.28 (nested = upright + touching + at rest at
  the moment of first contact, the human judge's criterion).
  29 trials succeed >=2/3 runs on contact; 15 on nested.
- **Backend transfer caveat:** 14 winners exist only under the GPU backend; 6/14
  reproduce on GPU single-env (4 nested), 8 were batch flukes. CPU-verified winners
  (status 'ok') transfer deterministically.
- **Negative control:** 17/19 fail-labeled demos correctly fail on contact; 19/19 on
  nested (zero false positives under the strict metric).
- Residual unsolved (17): ~6 fallen-can demos the lying grid couldn't grasp, drag-heavy
  demos needing substeps=8 (6 rescued in a probe -- deferred to post-upgrade world),
  and long-fumble demos that likely need closed-loop control.


## CORRECTED METRICS + DP BASELINE (2026-07-08)
Two metric bugs found by rendering a policy rollout and checking it (not trusting aggregates):
1. **Contact had no ordering precondition** — a can shoved along the table into the goal's
   base counted as success (trial 284: contact with picked=False, can at z=0.10). Fixed:
   contact now requires the can was PICKED first. Inflated DP contact by ~21% (6/29 spurious).
2. (False alarm, ruled out) a rendered rollout *looked* like the arm stayed low, but the
   can genuinely reached shelf z=0.226 — camera-angle misread, `placed` is honest.

**Corrected replay baseline** (50 success-labeled solved, x3, CPU): picked 0.84,
contact 0.57/run, nested 0.35/run; 27 trials reliable(>=2/3) on contact, 18 on nested.

**DP state-only baseline, in-distribution** (35 trained trials x3): picked 0.50,
placed 0.32, contact 0.22, nested 0.06. Funnel P(pick)=0.50 -> P(place|pick)=0.64 ->
P(slide|place)=0.68 -> P(nested|slide)=0.26. Bottleneck is the initial grasp (0.50) and
the final upright nest; the middle (place->slide) is comparatively strong.


## INDEPENDENT PANEL REVIEW (2026-07-08) — findings & status corrections
A 4-agent adversarial panel (sim-fidelity, BC/DP, rigor, search) reviewed the work.
Strong convergence. What they affirmed and what they overturned:

AFFIRMED: FK-at-grasp position recovery is sound and independently validated vs bag
tool_pose (~1cm). Infrastructure careful. Directional "DP < replay on pick/contact" holds.

OVERTURNED / OVERSTATED (verified against the tree):
- **Negative control was rigged** — it disabled goal relocation. Run through the FULL
  pipeline, 11/16 fail-labeled demos get "solved" and 7 pass strict nested (all
  goal_moved=True). The "19/19 zero false positives" claim is FALSE; true nested
  false-positive rate on known failures ~40%.
- **Coverage (50/75) is a fit, not recovery.** Search winner drifts from FK position up to
  17cm (307), 14.6cm (260); 60% of solved trials depend on goal relocation (median 12.8cm,
  max 32cm). "Position that works" != "recovered true position" for the high-drift trials.
- **52/61 solved are ok_batch, never CPU-revalidated** (success_rate: None). Batch proxy
  was missing the picked precondition (FIXED). Selection bias (best-of-32) + verify loop
  pre-counting the finding run inflate rates.
- **nested metric lacked the picked precondition** (only contact had it) — inflated both
  DP nested (0.06) and replay nested (0.35) with shove-ins. FIXED; both need re-measurement.
- **DP eval is in-distribution refit on privileged ground-truth state, uncaveated.**
  Randomized-IC (generalization): picked 0.34 placed 0.14 contact 0.14 nested 0.04.
- Action off-by-one (stores vel[i+1] as label for a transition produced by vel[i]).

REMEDIATION (gated on all-bags rsync, in progress):
1. Re-run negative control THROUGH the full pipeline; report the false-positive rate.
2. Re-report coverage at FROZEN FK placements (no per-trial search) to separate recovery
   from fit; report goal_moved=False subset as the defensible number.
3. Vision ground truth: Hough circle detection (prototyped, works) on top cam (can start)
   + bottom-up cam through translucent shelf (goal/slide landing). No factory calibration
   in config.yaml (handoff doc was wrong); calibrate via FK-homography. Bounds goal
   relocation to reality — the panel's #1 fix.
4. Re-measure all metrics under corrected nested; add held-out split; fix action off-by-one.

## HONEST COVERAGE RE-MEASUREMENT (2026-07-08, post-panel)
Corrected metric (contact & nested both require picked). Success-labeled demos, n=75.

| definition | contact cov | nested cov |
|---|---|---|
| searched can + goal RELOCATED (original headline) | 50/75 | — |
| searched can, STATIC goal (no relocation) | 20/75 | — |
| FROZEN FK position, STATIC goal (zero fit) | 15/75 | 13/75 |

Frozen-FK per-run: picked 0.59, contact 0.16, nested 0.14. So the recovery-based coverage
is ~15-20/75 (20-27%), NOT 50/75; the lift to 50 is goal relocation, which also
false-positives 69% (contact) / 38% (nested) of KNOWN-FAILED demos (negative control run
through the full pipeline — the honest version).

OPEN QUESTION the vision work resolves: is the goal can physically FIXED across trials
(=> 15/75 is the true number, relocation was fitting sim error) or did it VARY per trial
(=> relocation captured something real)? Bottom-up camera through the translucent shelf
settles it. Until then, report 15-20/75 as coverage, not 50/75.

## PICK+PLACE CEILING: it's control-limited, not placement-limited (branch: grasp-from-gripper-current)
Tested the hypothesis "spawn the can right -> high pick+place." Isolated pick+place
(goal parked far), corrected metric, 75 success-labeled demos, 3 reps:

| spawn source | pick/run | place/run | pick cov | place cov |
|---|---|---|---|---|
| FK-recovered            | 0.59 | 0.24 | 49/75 | 18/75 |
| current first-contact   | 0.55 | 0.20 | 44/75 | 15/75 |

Findings:
- Current-based first-contact (gripper-current rest position, video-validated to the frame)
  is MARGINALLY WORSE, not better. For drag demos, spawning at the true rest forces the sim
  to reproduce the human's drag (contact-heavy, unreliable); FK/lift-based pre-stages the can
  where it lifts, skipping the flaky drag. More faithful != easier for open-loop sim.
- Even at the tool_pose-validated position, pick+place caps ~60% pick / ~20-24% place.
  Placement is NOT the limiter (FK agrees with robot tool_pose to ~1cm). The limiter is
  OPEN-LOOP REPLAY FIDELITY: the human closed the loop (felt/adjusted the grasp+carry),
  blind playback cannot. The pick->place drop (65%->24% cov) is the un-corrected carry.
- Implication: high success rates come from the CLOSED-LOOP POLICY, not from better spawns.
  Recovered positions + demos are training data; ~20% open-loop place is the FLOOR the
  learned agent must beat, not a number to push by tuning initial conditions.

## CORRECTION: the pick+place ceiling was a substeps artifact, not an open-loop limit (2026-07-08)
Earlier claim "open-loop caps ~20% place, fundamental" was WRONG. Diagnosed the pick-but-
no-place trials: they LIFT the can then drop it mid-carry (278,305: lift to z=0.25-0.29
then fall). Swept the physical knobs on 8 drop-prone trials:
- can friction 0.2/0.5/1.0: NO effect (grip fails for lack of normal force, not friction)
- finger force 50/100/200: NO effect
- substeps 4->8: place 1/8 -> 3/8 (278,305 flip p->P) <-- THE LEVER
Contact-solver resolution: at substeps=4 the finger-can contact isn't resolved accurately
during the dynamic carry (arm accelerating) so the can slips out; substeps=8 holds it.
Principled fidelity (finer integration), not fitting. Cost ~2x compute; substeps=16 ~4x
(too slow to sweep here). NEXT: re-run full pick+place + coverage at substeps=8.

## substeps=8 full result (FK spawn, 75 demos x3, goal parked far)
             pick/run  place/run  pick cov   place cov
substeps=4:    0.59      0.24      49/75      18/75
substeps=8:    0.74      0.34      57/75      28/75    <-- converged (16 == 8)
substeps=8 improves EVERY metric incl pick (finer contact resolves the grasp too).
Adopt substeps=8 as world default; re-derive coverage numbers (all prior were at ss=4,
so understated). Residual: place still 37% -- the pick->place gap (76% pick cov but 37%
place cov) is NOT more substeps (16==8); it's carry/place trajectory + strict place band,
partly genuinely open-loop-hard. Tradeoff: ss=8 is 2x compute (matters for RL rollout
throughput at scale -- pick ss deliberately: high for faithful eval, lower for bulk train).

## FULL COVERAGE @ substeps=8 (2026-07-09)
Recovery-honest (frozen FK, static goal, corrected metric, x3):
              pick/run  contact cov  nested cov
substeps=4:     0.59      15/75        13/75
substeps=8:     0.73      22/75        15/75     <- real lift from physics alone, no fit

Searched placements per-run: contact 0.57, nested 0.35->0.41; reliable-nested 18->21.

NEGATIVE CONTROL still fails at ss=8: fail-labeled demos false-positive 9/11 contact,
8/11 nested. Substeps does NOT fix this -- it's the goal-relocation artifact, not physics.
So the searched "50/75" stays inflated; the honest recovery-based coverage is
22/75 contact (29%) / 15/75 nested (20%).

Path to higher HONEST coverage: (a) vision goal ground-truth to bound relocation to
reality (then relocated successes become trustworthy), or (b) closed-loop policy.
Substeps=8 adopted as world default.
