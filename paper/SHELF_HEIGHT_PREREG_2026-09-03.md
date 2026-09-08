# PREREG — shelf height + stiff grasp on the recorder path (sim box, 2026-09-03 13:30)

Registered BEFORE any census of the candidate variants is read. Mandate (user, 09-03): "if you
think you can improve any of these phases you should. contact matters more than nested, but it is
better if the can doesn't fall over"; per-phase worlds ("break it into phases at the dataset level
and customize the simulation for each phase") are allowed.

## 1. Findings that motivate this (all pre-readout, from existing data)

**F1 — the recorder's stage proxies under-count the slide (honest re-score, w3, 74 uids, full scope,
`can_pos_recovery/honest_rescore.py`, bit-exact re-execution 74/74):**

| w3 (`gc_kp4_riser3_shelf6`) | picked | placed | set-down | contact | nested |
|---|---|---|---|---|---|
| recorder proxies (what `record_demos` stamps) | 69 | **0** (band inside the shelf) | — | 26 | **5** |
| honest (band moved with the shelf; `_nested()` of record) | 69 | 66 | 46 | 26 | **16** |

`placed` (genesis_can_env.py:267) tests the can centre in [0.12, 0.18] with the un-shifted
BOX_TOP_Z 0.11 → inside the solid shelf (top 0.17) in every shelf6 world: 0/222 census tapes.
The recorder's `nested` proxy (full_env.py:597) needs hard contact AND grip cmd < 0.3; humans
release partially (grip 0.33–0.98) → 13 honest-nested tapes are stamped contact/picked (2 go the
other way: proxy fired, settle drifted to 8.4/9.5 cm). Picked-not-nested (53): **tipped 32**,
upright but short 21 (8.4–23 cm), goal knocked 0.

**F2 — the tip mechanism (scratchpad `tip_anatomy.py`, census tapes):**

| tapes ending tipped | w3 | ts5 | ts5_w4 |
|---|---|---|---|
| total | 32 | 26 | 22 |
| falls from height at release (can bottom > 3 cm above the shelf when the grip opens) | 11 | 10 | **13** |
| pushed over by the open gripper after release | 8 | 4 | 0 |
| tips in hand (grip closed, can in the jaw) | 7 | 4 | 2 |
| off shelf / other | 6 | 8 | 7 |
| contact granted before the tip | 7 | 3 | 1 |

**F3 — the shelf is ~4 cm lower in the sim than in the real rig.** Measured DIFFERENTIALLY from
the bags (`inthewild_trials/<uid>_timed.npz`, base_feedback tool z): tool z at the release minus
tool z at the pick closure, same grasp throughout, so the grasp height and the tool-frame offset
cancel. 43 demos with a > 8 cm lift and a full release (reading < 30): **median 15.8 cm above
the table** (p25 14.3, p75 17.0, sd 2.4; end-of-demo slide height gives 15.4). The sim: shelf6
top = 12.0 cm above the table; shelf10 = 16.0 cm. (The 08-23 lab's absolute estimate — release
tool z − an assumed grasp offset → 0.21 world — agrees; it was set aside for the "box stands on
the table" default pending a measurement of the rig that was never taken.)

**F4 — why w3 hides it and ts5 exposes it:** in-grasp vertical slip during the carry (can centre −
ee z, release minus pick): **w3 −2.5 cm** (p25 −4.2), ts5 −0.9, ts5_w4 −0.8. The w3 penetration
grasp lets the can walk 2.5 cm down the pads, so it is released only 1.6 cm above the shelf6 top;
the stiff ts5 grasp holds it where the human held it and releases it 3.5 cm above → F2's drop
class. With shelf10 (+4 cm) the stiff grasp releases the can ON the shelf (can bottom 0.204–0.207
vs top 0.21); the w3 grasp would press it 2 cm INTO the shelf before the release.

## 2. Candidates (new variant names only; `sim_variants.py --selftest` PASS for both new ones)

| variant | shelf top (world) | grasp | role |
|---|---|---|---|
| `gc_kp4_riser3_shelf6` (w3) | 0.17 | timeconst 0.02 (penetration grasp) | paper world, baseline |
| `gc_kp4_riser3_shelf10_ts5` **(P1)** | 0.21 | 0.005 | the hypothesis |
| `gc_kp4_riser3_shelf10` | 0.21 | 0.02 | shelf-only control (F4 predicts it does NOT help) |
| `gc_kp4_riser3_shelf8_ts5` | 0.19 | 0.005 | bracket (IQR lower half) |
| `gc_kp4_riser3_shelf6` + `--ic-jitter 0.001` | 0.17 | 0.02 | recorder-path noise floor |

## 3. Protocol
Recorder path, identical to `census_variants.sh` (full scope only, 74 uids = fit 50 + holdout 24 of
`world_search_gc_kp4_riser3_shelf6_ts5/split.json`, 6 shards, `--max-sim-steps 12000`, idle box);
every census is then re-scored bit-exactly with `honest_rescore.py`. Metrics of record: picked /
contact (primary) / tipped-at-end (secondary, lower is better) / honest nested (tertiary) /
upright-short. Noise floor = the 1 mm-jitter w3 run vs the w3 census (paired per-uid flips and
the count deltas). Physical adjudication requested from the user: one tape-measure reading of the
shelf surface height above the table (F3 predicts ~16 cm; the sim assumes 12).

## 4. Predictions (74 uids, full scope)

| | picked | contact | tipped | honest nested |
|---|---|---|---|---|
| w3 (measured) | 69 | 26 | 32 | 16 |
| noise floor (w3 + 1 mm) | 69 ± 2 | 26 ± 3 | 32 ± 4 | 16 ± 3 |
| **P1 shelf10_ts5** | 66–69 | **≥ 29** (expect 28–34) | **≤ 18** | ≥ 19 |
| shelf10 (no ts5) | 67–70 | ≤ 28 | 24–34 (in-hand/drag tips replace drop tips) | 14–20 |
| shelf8_ts5 | 66–69 | between ts5 (22) and P1 | between ts5 (26) and P1 | — |

**Adopt P1 as the proposed world** iff, on the 74: contact ≥ 26 + max(3, 2·sd_noise) AND tipped
≤ 24 AND picked ≥ 66, with the same sign on the holdout 24 alone. **Disconfirm branch:** contact
≤ 26 or tipped ≥ 28 → the drop class does not convert into contact on the recorder path; report
it, try the bracket, and if the bracket also fails keep w3 with F1/F3 disclosed (CONFOUNDS).
**Per-phase branch:** if P1 loses > 3 picks but wins the slide, propose pick-phase sets on w3 and
place-phase sets on P1 (the place-entry bank / place-phase datasets carry F1's stale band and
must be rebuilt with the shifted band either way).

**Amendment 1 (13:40, jit census running, no candidate census started):** `sd_noise` for a metric
= sqrt(number of uids whose per-uid value of that metric FLIPS between the w3 census and the
1 mm-jitter w3 census) — the count delta of a replicate is a sum of ±1 flips, so its sd is ≈ sqrt(k)
under independent flips; the raw count delta |Δ| is reported beside it. Fit/holdout split and
per-uid flip lists come from `can_pos_recovery/score_shelf_prereg.py` (written now, before readout).
Tipped = picked ∧ ¬honest-nested ∧ settled can tilt ≥ 20 (the `honest_rescore.py` definition).

**Amendment 2 (14:05, P1 census running, nothing read):** the first "jitter" census was NOT jittered
— `record_demos.py --ic-jitter` sat in the `ic_mode == 'demo'` branch while the human teacher takes
its own IC branch; the run stamped `ic_jitter 0.001` and reset to the unperturbed placements (a
silent-default bug of the class in AUDIT_REQUEST_Fable.md; my smoke test checked the stamp, not the
`[ic]` line). Fixed: jitter applied in every branch through one helper, and the recorder now
REFUSES to run if `--ic-jitter` is requested and any plan entry is un-jittered; `honest_rescore.py`
read `config` where the merged manifest has `configs` (fixed). The un-jittered run is kept as
`census_shelf_0903_jit1_UNJITTERED_rerun`: a same-partition determinism check — 74/74 tapes
bit-identical to the w3 census (same stages, same settled distances). The real noise floor re-runs
AFTER the three candidates (queued behind ALL DONE), so the candidates will be read before
`sd_noise` exists: **the adopt rule stays as written** (its floor, +3 contact, applies regardless;
if 2·sd_noise turns out > 3 the verdict is re-stated against the larger threshold). Also: a
mis-anchored waiter launched the jit run in parallel with P1 for 34 s (14:00:51–14:01:25, 6 extra
processes, mostly scene build); killed. The P1 rollouts in flight during those seconds are
covered by the honest re-score's bit-exact re-execution on the idle box (`exact` flag per uid).

## 5. What this is NOT
Not a refit of θ (finger_kp etc.): the shelf height is a physical measurement from the bags, ts5 is
the gripper-lab's validated contact fix; nothing is fitted to the census outcome. Not a dataset
build and not a retraining decision (cluster/head agent). Side-by-side videos (real | w3 | P1) of
the flipped uids go to the user with the readout.

## 6. Readout (written as each census lands; scorer `can_pos_recovery/score_shelf_prereg.py`)

**P1 `gc_kp4_riser3_shelf10_ts5` (census 13:55–14:10, honest re-score 14:10–14:29, exact 74/74):**

| | picked | contact | tipped | honest nested | short | set-down |
|---|---|---|---|---|---|---|
| w3 | 69 | 26 | 32 | 16 | 21 | 46 |
| P1 all 74 | 68 | **18** | 29 | 17 | 22 | 36 |
| w3 / P1 holdout 24 | 23 / 22 | 7 / **3** | 13 / 9 | 4 / 6 | 6 / 7 | 13 / 11 |

Adopt rule: contact 18 < 29 ✗, tipped 29 > 24 ✗, picked 68 ✓, holdout same sign ✗ → **NOT ADOPTED;
prediction "contact ≥ 29" FALSIFIED (−8, the wrong sign).** Contact lost on 13 uids [232 233 236 248 250
286 294 295 297 309 316 325 326], gained on 5 [235 261 262 306 328]; tipped fixed 16 / new 13 (33 stage
flips — a very churny world; the noise floor for that churn is the queued jitter run).

**Mechanism (tapes, `real2sim_fidelity.load_tapes`; can bottom = z − 0.0505, the world's can_height/2 —
NB `replay_harness.BOTTLE_HEIGHT` 0.075 is stale, the world can is 0.101 × r 0.033 = the real 10.5 oz
can):** the human carries enter the shelf footprint with the can bottom only p10 2.8 / p50 5.5 cm above
the shelf6 top (ts5 tapes; w3 2.7 / 5.2), i.e. 15–17.5 cm above the table for the low decile. Raising
the shelf 4 cm puts the taller box in those paths: at shelf10, 27/67 picked tapes never get the can
upright over the footprint (ts5 at shelf6: 9) — 236/295/297/325/326 are stripped at the shelf front
edge and drop to the table (release "height" p10 −16.5 cm), 232/233 end upright 6.6–7.5 cm from the
goal instead of touching it. The sim tool follows the real joints to ~1 cm, so a real 16 cm shelf
would have clipped the real carries the same way. **Hence F3 is withdrawn as a shelf-height claim:**
the 15.8 cm release-minus-pick tool differential is not the shelf height; it contains real in-hand
slip, push-down at the grasp (the real fingers over-travel past contact, gripper lab §1.6, so the
pick-closure tool z is not a fixed offset from the can), and lifting while releasing. The sim's
release heights are what they are because the human released from there. Best current estimate of the
real shelf: 12–13 cm (shelf6 = 12; shelf8 = 14 is the bracket still to be read). The user's tape
measure remains the adjudication.

**What P1 did show:** with the stiff grasp and no drop, the lever-class tips of w3 (235 257 258 262
298 → nested in P1) do convert to nested — the release itself is fixable — but the shelf move costs
more than it buys. That points at the release contact, not the shelf: frame-by-frame in w3 (235 239
258) the can's tilt rises 4° → 12° → 35° → 45° while its z is unchanged and it is still within
1.5 cm of the ee, as the grip command ramps 0.4 → 0.3 (pad gap 63 → 74 mm around the 66 mm can): the
can pivots in the opening pinch before it falls. Next lab: `release_lab.py` (branch re-executions at the
release; separate prereg).

**Shelf-only control `gc_kp4_riser3_shelf10` (honest 15:08, exact 74/74) and bracket
`gc_kp4_riser3_shelf8_ts5` (honest 15:47, exact 74/74):**

| | picked | contact | tipped | honest nested | short | set-down |
|---|---|---|---|---|---|---|
| w3 | 69 | 26 | 32 | 16 | 21 | 46 |
| shelf10 (no ts5) | 70 | **20** | 28 | 24 | 18 | 42 |
| shelf8_ts5 | 69 | **20** | **17** | 24 | 28 | 48 |
| w3 / shelf10 / shelf8_ts5 holdout 24 | 23 / 23 / 22 | 7 / 5 / 5 | 13 / 10 / 6 | 4 / 7 / 7 | — | — |

shelf10: prediction "nested 14–20" missed (24), contact −6 (down 11 [232 233 242 250 286 294 295 309 316
320 321], up 5 [247 261 262 301 306]), tipped fixed 14 / new 10, 32 stage flips → NOT ADOPTED (contact wrong
sign; adopt rule ✗). shelf8_ts5: tipped 32 → 17 (fixed 19 / new 4) and nested 16 → 24, but contact 26 → 20
(down 11 [233 236 242 250 286 297 316 317 321 325 326], up 5 [261 262 293 315 329]) with the holdout also
down (7 → 5) → NOT ADOPTED (contact ✗, holdout sign ✗). Both shelf raises trade the hard-contact slide for
upright-short set-downs (short 21 → 28): the can is put down on the higher shelf and the human's slide
tape no longer pushes it the last centimetre into the goal. Every shelf variant therefore fails the
primary metric; the disconfirm branch applies: **keep w3 with F1/F3 disclosed** (CONFOUNDS row 48), and
attack the release itself (the pinch-hinge mechanism, separate prereg) rather than the shelf.

**Noise floor `gc_kp4_riser3_shelf6` + 1 mm IC jitter (census 15:47–16:26, honest exact 74/74; scorer
`score_shelf_prereg.py`):** picked 69 / contact 27 / tipped 35 / nested 18 / short 16 / set-down 40 (holdout
23 / 7 / 13 / 5). Per-uid flips vs the record: picked 0, contact 7 (+4/−3, sd = √k 2.65), tipped 7 (2.65),
nested 4 (2.00), short 7, set-down 8; 10 stage flips [237 244 248 256 259 261 269 316 327 328]. So the §4
adopt threshold is contact ≥ 26 + max(3, 2·2.65) = 31.3 (no candidate above came within 8 of it — the three
verdicts stand), and a candidate's stage-flip list must be read against those 10 noise-suspect uids.
Row 48 is WITHDRAWN as a shelf-height claim (CONFOUNDS 17:20); the release mechanism turned out to be
contact-constraint creep, not a pinch hinge — see CREEP_PREREG_2026-09-03.md.

**Shelf-height adjudication, 17:55 (three independent lines; the F3 withdrawal STANDS).** After the withdrawal I
found a second arm-side argument for ~16 cm (real tool-z rise closure→release median 16.0 cm ⇒ "rigid grasp ⇒ can
bottom 16 cm above the table at release") and it is WRONG for a stated reason: the tool-frame origin is not the pad
centre, and the wrist pitch differs between the pick (x ≈ 0.48, gripper pitched into the table) and the release, so
the tool-z differential is not the pad-z differential. The rotation-invariant in-hand measure on the w3 tapes
(|ee − can| at pick+5 vs release−2, 67 picked uids) gives slip median +0.6 cm (p25 0.0 / p75 +2.5), while the
vertical can−ee offset changes −1.85 cm median (p25 −3.3 / p75 −0.7): most of what the release-height budget called
"walk" is wrist pitch, not slip, and the "3.8 cm above shelf6 without the walk" number double-counted it. Corrected:
sim can bottom at release median shelf6 + 0.3 cm with ≤ 1 cm of true slip ⇒ the arm data put the real plate at
~12–14 cm. Camera (`camera_audit/cam4_model_final.npz`, cross-checked, `scratchpad/cam_geom.py`): (i) the fitted
goal-top ray with the slide-derived goal xy (0.672, −0.221) gives the goal-can bottom 10.7 cm above the table; for
a 16 cm plate the goal would have to sit at x = 0.605 (7 cm nearer the robot than the human-validated goal), 12 cm
needs 1.7 cm; (ii) the acrylic stand's front-leg feet back-project to (0.574, 0.091) and (0.562, −0.203) — the sim
box front face is x = 0.55 — and the plate's front edge crosses each leg image at a rod height of ≈ 12 cm (11.5 on
the right leg), which is a lower bound if the edge protrudes past the legs; (iii) 233 cam4 at release−1 s shows the
placed can's top on the same image row as the goal can's top (can at plate level BEFORE the release, no drop).
Verdict: real plate 11–13 cm, shelf6 (12.0) is right to ~1 cm, box front face right to ~1–2 cm; the tape-measure
ask to the user is a confirmation, not a blocker. The real stand is a clear acrylic plate on four rods (not a solid
box) — the sim's solid footprint only matters for carries that clip it, none of which are in the w3 census.
