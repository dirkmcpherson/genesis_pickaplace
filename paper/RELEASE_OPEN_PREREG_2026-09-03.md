# Pre-registration: fast-open release filter (grip_open_gain) — 2026-09-03 23:55

Sim-box session (world-change agent). Registered BEFORE the candidate census runs. Successor to
`CREEP_PREREG_2026-09-03.md` (all three impedance rows NOT ADOPTED, §5 there — disconfirm branch (c):
the creep is an engine property but not the recorder-path tip mechanism). Mandate of record (user
09-03 12:40): improve the recorder-path phases; contact > nested; the can should not fall over;
per-phase worlds / dataset-level phase customisation explicitly allowed.

## 1. Findings that motivate this

**F1 (release lab, complete 11-condition table, 20 w3 tip tapes, pre-branch bit-exact 20/20):**
`open_now` (snap the fingers fully open at the release onset) = **17/20 upright, 5/20 nested** vs
control 0/20. Every physics condition ≤ 10/20 (fric05 10, ts5 8, nk_fric05 8, shift10 6, ff2 5,
noknuckle 4, knuckle_fric0 2). The follower-side `unload` rule (registered 09-03 afternoon as the
next lever) is **0/20** — dead; `unload_ts5` 8/20 = plain ts5. The tips happen while the pads pass
through the partial-open band, whatever the friction/stiffness — the only thing that works is not
lingering in that band.

**F2 (bag check, 2026-09-03 23:15):** the bags carry NO independent gripper command topic
(`g_frame` ≡ `fb_grip`, max diff 1.3 on a 0–89 scale, best-lag 0.02 s, corr 1.0000; topics in the
bag: cartesian_velocity, joint_states, base_feedback). The human's release is genuinely a ~1.7 s
ramp (80→20% fall, median 1.74 s over 56 tapes with a clean release), the tape carries it
faithfully (sim command fall 1.80 s), and the sim gripper motor tracks its command within ~0.2 s
(an earlier "3–45 s follower lag tail" was a measurement artifact of multi-cycle regrasp tapes —
withdrawn before it reached any doc). **So this candidate is NOT a fidelity fix**: the real can
survives the same slow release that tips the sim can (the real can is presumably resting/steadied;
the sim can rotates in the pinch). It is a per-phase customisation of the recorder, disclosed as
mechanism-unfaithful, outcome-faithful (the real demos it re-executes DID end placed/nested).

**F3 (shelf height, adjudicated 17:55, SHELF_HEIGHT_PREREG addendum):** three independent lines
(corrected in-hand-slip arm measure, camera goal-top ray, stand leg back-projection) agree the real
plate is 11–13 cm above the table = shelf6 to ~1 cm. The release is NOT happening 4 cm in the air;
the tips are not a drop-height artifact.

## 2. Candidates

The filter (in `record_demos.HumanFollower._filter_grip`, keyed off the sim-variant table —
`sim_variants.VARIANTS[name]['grip_open_gain']`; no CLI flag, no default-on path; human teacher
only; the variant NAME carries the identity; worlds bit-identical to w3, selftest PASS 23:45 both):
causal, per decision. `hi` = running hold plateau, `lo` = running min since the plateau was set.
A rise to ≥ hi, or > 0.05 above lo (a regrasp), resets hi=lo=g and passes g through — closing and
holds are NEVER altered (unit test over all 91 v2r grip streams: 0 violations). A drop below hi of
≤ 0.05 (deadband; real hold noise ~0.01) passes through. Beyond it: `g_eff = clip(hi − K·(hi − g))`.

| row | variant | K | role |
|---|---|---|---|
| R1 | `gc_kp4_riser3_shelf6_ognow` | ∞ (snap past the deadband) | the lab condition, made causal |
| R2 | `gc_kp4_riser3_shelf6_og4` | 4 (1.7 s ramp → ~0.4 s) | graded version, less unfaithful |

**Known risks, stated up front:** (a) an amplified mid-carry partial grip adjustment (e.g. hold
0.9 → 0.57) drops the can early → new tips/shorts during the carry; R1 is maximally exposed. (b)
6/91 tapes end with the filter fully open where the tape ends at 0.51–0.65 (the Group-B tip tapes
whose real can was already free) — intended, but it changes the final state of those re-executions.
(c) og4 fraction of decisions altered: median 21.5% of a tape's decisions (mostly the post-release
tail where the tape idles at ~0.05 and the filter sits at 0).

## 3. Protocol

`CENSUS_ROOT=baselines/demos_v2/census_og_0903 nohup can_pos_recovery/census_shelf_0903.sh
gc_kp4_riser3_shelf6_ognow gc_kp4_riser3_shelf6_og4` — same 74 uids (fit 50 + holdout 24), same
recorder path, 6 shards, sequential, idle box, honest re-score bit-exact 74/74 or the row reruns
(the honest re-exec replays the SAVED action tape, which already contains the filtered grip — the
filter identity travels in the tape's `sim_variant` field). Score:
`score_shelf_prereg.py --new baselines/demos_v2/census_og_0903 --variants <rows>`.

## 4. Predictions (74 uids) and decision rule

Noise floor (w3 + 1 mm jitter): contact flips sd 2.65, |Δ| floor 5.3; w3 record 69 / 26 / 32 / 16 /
21 / 46 (picked/contact/tipped/nested/short/set-down).

| row | picked | contact | tipped | nested | set-down |
|---|---|---|---|---|---|
| R1 ognow | ≥ 66 (closing untouched) | 22–30 (slide mostly untouched) | **≤ 20** (lab 17/20 on the 20-uid tip subset, minus new early-drop tips) | ≥ 20 | ≥ 50 |
| R2 og4 | ≥ 66 | 22–30 | ≤ 26 (between w3 and R1) | ≥ 16 | ≥ 46 |

**Adopt a row iff** tipped ≤ 32 − max(3, 2·2.65) = 26.7 → **≤ 26** ∧ contact ≥ 26 − 5.3 → **≥ 21**
(no loss beyond the noise floor) ∧ picked ≥ 66 ∧ nested ≥ 16 ∧ holdout tipped down (13 → ≤ 12) with
holdout contact down by at most 2. Both pass → prefer R2 (less unfaithful) unless R1 beats it by
≥ 5 tipped AND ≥ 3 nested. This candidate targets TIPS, not contact — a contact GAIN is not
required, only no loss; that is within the mandate ("it is better if the can doesn't fall over").

**Disconfirm branches.** (a) tips fixed but contact drops > 5.3: the snap disturbs the set-down
that feeds the slide ⇒ do not adopt globally; report as a release-phase-only world for the
per-phase dataset split (user's dataset-level customisation), decision to the head agent/user.
(b) everything within noise: the lab result does not transfer to the recorder path (as with the
impedance lab) ⇒ keep w3, close the release lever, report the release as control-limited in this
engine. (c) picked drops ≥ 4: the deadband is too tight for some hold streams ⇒ one registered
fallback: deadband 0.10, same rule, one row (R2 only).

**Disclosures.** The 20 release-lab uids that motivated F1 are census members — 13 in the fit set,
**7 in the holdout [243 245 248 257 308 331 333]**: the holdout tipped-direction check is therefore
partially contaminated by design-informing uids; the per-uid flip lists will mark them, and the
adopt rule's holdout clause weighs the 17 non-lab holdout uids qualitatively in the writeup. The
filter constants (deadband 0.05, reset 0.05, K ∈ {4, ∞}) were chosen from the lab + stream noise
BEFORE this census; no census readout has informed them. Everything the scorer prints is reported
whatever the verdict. If adopted, this changes the recorder for every arm's re-execution — the head
agent decides the retrain (user bar unchanged).

## 5. Verdicts (filled after the readout)

**Readout (censuses 23:11–00:10, honest exact 74/74 both rows; scorer `score_shelf_prereg.py --new
baselines/demos_v2/census_og_0903`):**

| | picked | contact | tipped | honest nested | short | set-down |
|---|---|---|---|---|---|---|
| w3 (record) | 69 | 26 | 32 | 16 | 21 | 46 |
| w3 + 1 mm jitter | 69 | 27 | 35 | 18 | 16 | 40 |
| R1 ognow | 69 | 28 | **19** | **25** | 25 | 51 |
| R2 og4 | 69 | 28 | **20** | **23** | 26 | 50 |
| holdout 24 (w3 / R1 / R2) | 23/23/23 | 7/9/9 | 13/**5**/**5** | 4/**11**/10 | — | 13/17/— |

Both rows pass every §4 clause: tipped 19 and 20 ≤ 26 (−13/−12, noise floor 5.3); contact 28 ≥ 21
(both actually +2); picked 69; nested 25 and 23 ≥ 16 (+9/+7, noise 2·2.0 = 4); holdout tipped
13 → 5 ≤ 12 with holdout contact UP (7 → 9), not down. All five §4 predictions for R1 hit (69 ✓,
28 ∈ 22–30 ✓, 19 ≤ 20 ✓, 25 ≥ 20 ✓, 51 ≥ 50 ✓); og4's too. The early-drop risk (§2a) produced only
2–3 new tips (R1: 236 276; R2: 236 276 317 — 317 is not in the 1 mm-jitter flip set, the other two
are not either; disclosed). Contamination split (the §4 disclosure): the CLEAN holdout (17 uids never
seen by the release lab) confirms independently — R1 tipped 6 → 3, nested 4 → 9; R2 tipped 6 → 4,
nested 4 → 8, contact 5 → 7. The design-informing 7 lab uids do not carry the verdict.

**VERDICT: ADOPT R2 `gc_kp4_riser3_shelf6_og4`** by the registered preference (R1 leads by 1 tipped
and 2 nested, below the ≥ 5 ∧ ≥ 3 margin; og4 keeps a finite, disclosed 4× ramp instead of a snap).
Funnel of record on the candidate: **picked 69 / placed 64 / set-down 50 / contact 28 / honest
nested 23 / tipped 20** vs w3 69 / 66 / 46 / 26 / 16 / 32. This is the first candidate in three
prereg rounds (3 shelf rows, 3 impedance rows, 11 release-lab physics conditions) to move ANY
primary metric outside the noise floor in the right direction — and it is a recorder-side, per-phase
customisation, not a physics change: the WORLD is bit-identical to w3. Adoption scope: the recorder
path (demo re-execution / dataset builds). The head agent / user decide whether the retrain uses it
(the tapes it writes carry `sim_variant = gc_kp4_riser3_shelf6_og4`; policies TRAINED on such tapes
act in the unmodified w3 MDP — the filter only shapes the demonstrations' grip streams, which is
exactly the dataset-level phase customisation the user sanctioned; the DISCLOSURE is that machine
arms harvesting from teachers do NOT get this shaping, an asymmetry the comparison design must
either accept and disclose or neutralise by applying the same filter to teacher grip outputs).
