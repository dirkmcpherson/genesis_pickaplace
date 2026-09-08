# Slide anatomy: why the recorder path stops short of contact, at stroke level (2026-09-07)

Sim-box session (owner of the slide work per the 09-07 handoff). Disk-only analysis of the
`gc_kp4_riser3_shelf6_og4` census of record (`baselines/demos_v2/census_og_0903`, honest exact
74/74; funnel picked 69 / set-down 50 / contact 28 / honest nested 23 / tipped 20). Every number
below is from the census tapes + the real bag streams (`real2sim_fidelity.real_series`); scripts
inline in the session log, class lists reproducible from `slide_anatomy.py` (CENSUS=census_og_0903).

## 1. The residual failure class

After og4 removed the release tips, the slide-failure population is: **19 tapes upright but
stopped short of the goal** (end dist 8–15 cm, median 9.9 cm vs the honest nested band ≤ 8.1 cm),
11 at the goal but not nested, 17 still tipped. The short class is the contact frontier: a ~2–4 cm
shortfall per tape.

## 2. Three hypotheses, three measurements, two overturned

**(a) "The stroke misses the sim can" — NO.** The tool's closest post-set-down approach to the sim
can is median 1.0 cm with median 32 engaged decisions: the hand comes right to the can.

**(b) "The sim can is set down at the wrong spot" — NO.** Measured tool-to-tool (sim `eef_pos` at
set-down vs real bag tool xy when `fb_grip` falls through 0.5, both in the Kortex base frame): median
offset **0.2 cm** over 45 tapes (short class median 0.4, p75 3.4; the p90 23.8 is two late-regrasp
tapes, 279/245, where the release-event match picks a different grip cycle). The arm puts the hand
where the human put it, to millimetres.

*Correction to my own first pass:* I initially compared the sim CAN centre at set-down against the
real TOOL at release and reported "1.7 cm". That comparison silently absorbs the grasp lever arm —
the can centre sits median 1.5 cm from the tool point at set-down — so it was measuring the grasp
geometry, not placement fidelity, and its error bar was the same size as its answer. The tool-to-tool
number above is the one that means what the sentence says.

The real release itself happens at median **12.8 cm from the goal** — the HUMAN also places short,
then slides the can home with closed-loop nudges.

**(c) "The open-loop nudges under-transfer" — YES, and it is small and lateral.** From the first
post-set-down re-approach (tool within 4.5 cm of the can): over the rest of the tape the tool's
NET goalward motion is negative (−112 cm pooled over 17 tapes — the hand repositions between
nudges and withdraws at the end), the can gains +66 cm pooled (~4 cm/tape) and loses 41 cm pooled
(~2.4 cm/tape) LATERALLY (squirt: a fingertip pushing a cylinder off-axis at z ≈ can-centre
height). Strokes that move the tool ≥ 2 cm goalward while engaged exist in only 4/18 short tapes;
their pooled transfer is 0.65. The real slide covered the same ~6 cm because the human corrected
each nudge against the live can position; the tape replays the corrections against a can that is
no longer where the real one was after the first ~1 cm of under-travel.

**(d) "The can is set down in a worse pose relative to the hand on the tapes that fail" — NO
(tested because (b) invited it).** Can centre relative to the tool point at set-down, decomposed on
the can→goal axis: arrived (nested, n=20) lever 1.5 cm / along +0.3 / lateral 1.4; did not arrive
(n=32) lever 1.5 / along +0.3 / lateral 1.4 — identical, exact permutation on the along-axis median
p = 0.82. The only set-down variable that separates the two is plain DISTANCE to the goal (10.7 cm
arrived vs 12.3 cm not). So the failures are not mis-set-down and not a bad push geometry: the
nudges transfer so poorly that only the tapes already close enough arrive.

## 3. Verdict

**The slide is control-limited at the stroke level, not physics-limited.** The re-execution
engages the right spot (0.2 cm tool-to-tool placement fidelity), and the shortfall is the accumulated ~2–4 cm
difference between closed-loop micro-nudges and their open-loop replay, half of it lateral squirt.
No world parameter can make an unpushed (or off-axis-pushed) can arrive: the levers were already
tested and are inert or harmful (shelf-height rows: contact 26 → 18–20; impedance rows: 26 → 20–23;
world-search frictions inert across the population; og4 itself moved contact only 26 → 28, inside
the 5.3 noise floor). This is the evidence behind the standing "slide remains control-limited"
line: closing the last ~2 cm needs a closed-loop slide (a policy, or a per-phase slide controller
seeded from the tape), which is a learner/dataset question, not a recorder-world question.

Caveat: `contact` here is the predicate of record (picked ∧ pick-can↔goal solver contact; the
ee clause is vacuous per ADVERSARIAL_REVIEW_eval_env_2026-09-07 S2-5). The stricter `contact_push`
(tool-point far side ∧ no gripper–goal contact, paper/CONTACT_PUSH_2026-09-07.md) can only lower
these counts; it does not change the mechanism above, which is about the can not arriving at all.
