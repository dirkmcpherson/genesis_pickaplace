# Pre-registration: contact-constraint creep fix (impedance) — 2026-09-03 17:05

Sim-box session (world-change agent). Registered BEFORE any candidate census is run. Successor to
`SHELF_HEIGHT_PREREG_2026-09-03.md` (all three shelf-height rows NOT ADOPTED, §6 there). Mandate of
record (user 09-03 12:40): improve the recorder-path phases; contact > nested; the can should not
fall over; per-phase worlds allowed.

## 1. Findings that motivate this (all from the w3 world of record `gc_kp4_riser3_shelf6`)

**F1 (census of record, honest re-score, 74 uids, bit-exact 74/74):** picked 69 / contact 26 / tipped 32 /
honest nested 16 / upright-short 21 / set-down 46. Tipped = picked ∧ ¬nested ∧ settled tilt ≥ 20°.

**F2 (noise floor, w3 + 1 mm IC jitter, 74/74 exact):** picked 69 / contact 27 / tipped 35 / nested 18.
Per-uid flips vs the record: picked 0, contact 7 (sd = √k = 2.65), tipped 7 (2.65), nested 4 (2.00);
10 stage flips [237, 244, 248, 256, 259, 261, 269, 316, 327, 328]. Any candidate delta must clear this.

**F3 WITHDRAWN.** The "shelf 4 cm too low" reading came from `replay_harness.BOTTLE_HEIGHT` (0.075,
stale pre-panel constant; the world's can is 0.101 tall); the differential bag measurement re-done with
the correct half-height puts the real shelf surface ~12–13 cm above the table = shelf6 (12.0). The
shelf10/shelf8 candidates all lost contact (18/20/20 vs 26) — consistent with the shelf NOT being low.

**F4 (release lab, 20 w3 tipped tapes, `can_pos_recovery/release_lab.py`, branch at the release onset,
pre-branch bit-exact 20/20):** no friction / knuckle / timeconst change rescues the release broadly:
control 0/20 upright; fric05 10/20; nk_fric05 8/20; ts5 8/20; shift10 6/20; ff2 5/20; noknuckle 4/20;
knuckle_fric0 2/20. `open_now` (snap the fingers open at the onset instead of the human's slow
release) = 17/20 upright, honest nested 5/20. So the can tips DURING the slow partial release, while it
is still pinched — not from the friction level and not from the drop.

**F5 (the mechanism, pinch lab `can_pos_recovery/pinch_lab.py`, isolated from the arm: two fixed pads
pinch a horizontal can 3 cm off-centre → 0.10 N·m gravity torque about the pinch line):** a pinched can
under a sustained sub-limit tangential load CREEPS — droop grows linearly in time (3.7 / 19.4 / 34.1° at
0.1 / 0.5 / 1.0 s), is nearly independent of the pinch force (pen 0.5 mm 59° vs 2 mm 43°) and of the
contact count (split-can seg6 45° vs seg1 62° at 0.2 mm — the "free hinge / split can" hypothesis is
the WRONG lever), scales with the contact `timeconst` (tc 0.005: 21.5°) and halves with impedance
0.9 → 0.95. Genesis's friction constraints are regularised by R ∝ (1−imp)/imp
(`constraint_solver_decomp._func_add_contact`; `imp_aref` in `utils/geom.py`): a constant tangential
force below the cone limit produces a constant constraint VELOCITY ∝ timeconst·(1−imp)/imp, i.e. the
grasp is a viscous joint, not a stick. This is the standard MuJoCo "objects creep on slopes" symptom;
the standard fix is solimp d → 0.99. Same lab, tc 0.020 (w3):

| impedance (dmin/dmax) | droop 1 s, pen 0.5 mm | pen 2 mm | centred control slip (cm) | |F| pen 0.5 mm | stiffness × |
|---|---|---|---|---|---|
| 0.90/0.95 (engine default = w3) | 59.1° | 43.1° | 0.30 | 31 N | 1 |
| **0.99/0.99** | **8.5°** | **8.4°** | **0.03** | 175 N | 5.6 |
| 0.999/0.999 | 0.8° | 0.8° | 0.01 | 1717 N | 55 (rejected: force blow-up) |

The same viscous creep explains the in-grasp slip during the carry (w3 2.5 cm per carry vs ts5 0.9 cm;
pinch-lab creep-rate ratio 59/21.5 = 2.7 ≈ the slip ratio 2.8) and the in-hand tips (contact demos
carry the can pitched in-hand).

## 2. Candidates (`baselines/sim_variants.py`, selftest PASS 17:00 on all three + the w3 baseline)

| row | variant | change vs w3 | role |
|---|---|---|---|
| P1 | `gc_kp4_riser3_shelf6_imp99` | sol_params dmin = dmax = 0.99 on ALL 16 geoms; timeconst untouched (0.02) | primary |
| C1 | `gc_kp4_riser3_shelf6_gimp99` | 0.99 on the 4 finger geoms + the can only (5 geoms); shelf/table/goal keep 0.9/0.95 | scope control: is it the grasp or the world? |
| B1 | `gc_kp4_riser3_shelf6_ts5_imp99` | P1 + `grasp_timeconst 0.005` on the grasp geoms (gripper_lab defect-A fix) | bracket: does stiffer + faster stack or over-stiffen? |

The collider averages a pair's sol_params (0.5·(a+b)), so in C1 finger–can pairs get 0.99 and
can–shelf / can–goal pairs get ~0.945/0.97. Cost: contact stiffness at fixed penetration ×5.6 (P1) and
×(5.6 × 4) for B1's grasp pairs — risk of grip-force / chatter artifacts in the pinch (the pinch lab at
tc 0.005 + 0.99 read 2.7 kN at 0.5 mm penetration; the recorder path's real penetration is what matters
and is unknown until the census runs).

## 3. Protocol (identical to the shelf-height prereg)

`CENSUS_ROOT=baselines/demos_v2/census_creep_0903 nohup can_pos_recovery/census_shelf_0903.sh
gc_kp4_riser3_shelf6_imp99 gc_kp4_riser3_shelf6_gimp99 gc_kp4_riser3_shelf6_ts5_imp99` — same 74 uids
(fit 50 + holdout 24 from `world_search_gc_kp4_riser3_shelf6_ts5/split.json`), same recorder path
(`record_demos.py --teacher human --ic-mode demo --arrival either --scope full --max-sim-steps 12000`),
6 shards, sequential, idle box, then `honest_rescore.py run --par 6` (bit-exact `exact` flag must be
74/74 or the row is rerun). Score: `score_shelf_prereg.py --new baselines/demos_v2/census_creep_0903
--variants <rows>`. ~35 min per row + ~15 min honest.

A 2-uid smoke of P1 (uids 232, 235) runs first for crash/timing only; its outcome is NOT used and is
disclosed here.

## 4. Predictions (74 uids) and decision rule

Mechanism says: less creep ⇒ less in-grasp slip and less rotation during the slow release ⇒ fewer
tips at release, more cans set down upright, more contact (the human's slide commands land the can
where the human meant it). Risk: stiffer pinches eject the can (defect A) ⇒ picks lost.

| row | picked | contact | tipped | honest nested |
|---|---|---|---|---|
| w3 record | 69 | 26 | 32 | 16 |
| P1 imp99 | 65–69 | **≥ 32** | **≤ 22** | ≥ 20 |
| C1 gimp99 | 65–69 | ≥ 30 (≥ 80 % of P1's gain if the grasp is the site) | ≤ 24 | ≥ 18 |
| B1 ts5_imp99 | 60–69 (ejection risk) | ≥ 30 | ≤ 22 | ≥ 18 |

**Adopt P1 iff** contact ≥ 26 + max(3, 2·2.65) = **31.3 → ≥ 32** ∧ tipped ≤ 24 ∧ picked ≥ 66 ∧ the
same sign on the holdout 24 (contact 7 → up, tipped 13 → down). Ties between rows that all pass: prefer
the one with the fewest picks lost, then the most contact; C1 over P1 only if C1 passes and P1 loses
≥ 3 picks vs C1 (the world contacts outside the grasp then cost more than they give).

**Disconfirm branches.** (a) contact up but tipped NOT down: the creep fix helps the slide but the
release still tips ⇒ open the per-phase branch (release-phase world only), do not adopt globally.
(b) picked drops ≥ 4 in all rows: stiffness ejects the can ⇒ try imp 0.95/0.97 (one more row,
pre-registered here as the fallback ladder, same rule) before giving up. (c) all rows within the noise
floor (|Δcontact| < 5.3, |Δtipped| < 5.3): the creep is NOT the recorder-path mechanism (the pinch lab
loads differently than the arm) ⇒ keep w3 with F1/F4/F5 disclosed; report to the user that the
recorder path is control-limited under this engine.

**Disclosures.** The 20 release-lab uids and the pinch lab informed the design (no census uid was
tuned against; the impedance value is the MuJoCo convention, not a fit). Everything the scorer prints
(ALL / FIT / HOLDOUT tables, flip lists, noise-suspect uids) is reported whatever the verdict. The
adopted world, if any, changes the physics for EVERY arm's re-execution — the head agent decides
whether to retrain (user bar: "if you make a world that gives us better real2sim we'll retrain
everything"). `tip_anatomy.py` / `slide_anatomy.py` + real | w3 | candidate videos of the flipped
uids follow the verdict.

## 5. Verdicts (filled in after the readout)

**P1 `gc_kp4_riser3_shelf6_imp99` (census 16:55–17:07, honest 17:07–17:25, exact 74/74):**

| | picked | contact | tipped | honest nested | short | set-down |
|---|---|---|---|---|---|---|
| w3 | 69 | 26 | 32 | 16 | 21 | 46 |
| w3 + 1 mm jitter | 69 | 27 | 35 | 18 | 16 | 40 |
| P1 all 74 | 68 | **20** | **43** | 13 | 12 | **23** |
| w3 / P1 holdout 24 | 23 / 22 | 7 / 6 | 13 / 14 | 4 / 5 | 6 / 3 | 13 / 6 |

Adopt rule: contact 20 < 31.3 ✗, tipped 43 > 24 ✗, picked 68 ✓, holdout same sign ✗ → **NOT ADOPTED;
prediction FALSIFIED with the wrong sign on every metric** (Δcontact −6, Δtipped +11, Δset-down −23 — all
far outside the noise floor's 5.3). Contact down on 12 uids [232 233 236 237 250 255 286 297 305 316 325 326],
up on 6 [247 261 262 293 311 315]; tipped fixed 8 / new 19; 33 stage flips (8 noise-suspect).

**Mechanism of the failure (`can_pos_recovery/tip_anatomy.py`, corrected can height):** the tip taxonomy
changes class. w3's 32 tips: 9 dropped from height during the carry, 8 pushed by the open gripper after
release, 7 in hand, 2 at release. P1's 43: **25 dropped during the carry** (grip cmd 0.47–0.53 = the
human's hold level in 21 of them, can 5–12 cm from the ee at z 0.25–0.31, 5–17 s into the carry), 5 in
hand, 5 at release, 5 off shelf, 2 pushed. Set-down 46 → 23 says the same thing: the can never arrives.
The stiff pinch EJECTS the can. The pinch lab could not see this because its pads were position-fixed
(penetration prescribed); on the arm the finger PD (kp 40, 50 N·m cap) sets the penetration from the
contact stiffness, so a 5.6× stiffer contact is a shallower, harder pinch on a cylinder — a squirt.

**Disclosure (should have been in §1):** the August gripper lab already had this row — `g_dmax99`
(impedance 0.95 → 0.99 on pads + can, `paper/gripper_lab_2026-08-25.md` §4): slip 3.79 → **5.71 mm**,
tilt 4.8 → **10.8°**, worse on both, the only stiffening lever in that bench that hurt. I missed it when
writing §1; it predicted this outcome. The creep measurement (F5) stands as an engine property, but it is
NOT the recorder-path tip mechanism, and the impedance lever is dead for the grasp: w3's grasp is a
penetration grasp (gripper lab §6.3 — the pads cage the can in a converging V), and every stiffening of
the finger–can pair that shrinks the cage without a compliant pad trades cage for force.

C1 (gimp99) and B1 (ts5_imp99) run to completion as registered (scope control: is it the grasp pair; the
lab's `g_padcanstiff_dmax99` row suggests the impedance stops hurting once the timeconst is stiff).

**C1 `gimp99` (scope control, honest 17:55, exact 74/74) and B1 `ts5_imp99` (honest 18:26, exact 74/74):**

| | picked | contact | tipped | honest nested | short | set-down |
|---|---|---|---|---|---|---|
| w3 | 69 | 26 | 32 | 16 | 21 | 46 |
| w3 + 1 mm jitter | 69 | 27 | 35 | 18 | 16 | 40 |
| P1 imp99 (all 16 geoms) | 68 | 20 | 43 | 13 | 12 | 23 |
| C1 gimp99 (grasp pair only) | 69 | 23 | 38 | 13 | 18 | 30 |
| B1 ts5_imp99 (P1 + tc 0.005) | 67 | 20 | 29 | 17 | 21 | 36 |
| holdout 24 (w3 / P1 / C1 / B1) | 23/22/22/21 | 7/6/8/5 | 13/14/13/10 | 4/5/4/4 | — | 13/6/6/7 |

Both **NOT ADOPTED** (contact 23 and 20, both < 31.3; tipped 38 and 29, both > 24; holdout sign ✗ for both).

Three things the two controls settle. (1) **The damage is the grasp pair, not the world contacts.** C1 touches only
finger–can (5 geoms) and reproduces ~half of P1's loss on every metric (contact −3 vs −6, tipped +6 vs +11, set-down
−16 vs −23); the collider averages sol_params, so C1's finger–can pair is 0.99 exactly as in P1 while its can–shelf
pair is 0.945 — the residual difference is the softer landing, not a different grasp. (2) **`grasp_timeconst 0.005`
partly rescues stiffening but does not pay for it.** B1 = P1 + ts5 recovers tipped 43 → 29 and set-down 23 → 36 and is
the only row whose nested (17) matches w3 (16), i.e. the lab's `g_padcanstiff_dmax99` reading was right that the
impedance stops hurting once the timeconst is stiff — but contact stays at 20 and picks fall to 67, so it is w3 with
a worse slide. (3) **No impedance row improves contact.** The best contact of the three is C1's 23 vs w3's 26, and the
holdout falls for P1 and B1. Fallback branch (b) of §4 (the imp 0.95/0.97 ladder) is **not taken**: it was registered
for "picks drop ≥ 4 in all rows", and picks did not drop (68/69/67 vs 69); the failure is contact and tips, and the
monotone ordering w3 > C1 > P1 in contact says the ladder's interior points can only interpolate between a row that
already fails and w3.

**Verdict for the prereg as a whole: disconfirm branch (c) — the contact-constraint creep measured in the pinch lab
is NOT the recorder-path mechanism.** F5 stands as an engine property (the numbers are reproducible and the code
path is identified) and is worth one sentence in the paper's sim-fidelity section, but the lever is dead: every
stiffening of the finger–can pair trades the penetration cage for force, and the recorder path loses cans to
ejection during the carry faster than it gains them at the release. w3 `gc_kp4_riser3_shelf6` remains the world of
record. Next lever (release lab first, then a separate prereg if it earns one): the release itself — `open_now`
(snap the fingers open at the release onset) already scores 17/20 upright on the 20 w3 tip tapes vs 0/20 control,
which is a follower-side change, not a physics change.
