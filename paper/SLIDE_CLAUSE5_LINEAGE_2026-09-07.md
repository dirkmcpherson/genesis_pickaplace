# Clause-5 calibration and the phase-bank lineage (2026-09-07, sim box)

Answers the two questions put to this session after PHASE_PLAN amendment (p) withdrew (l)'s
concurrent `grip_cmd < 0.3` clause: (1) calibrate clause 5 ("the can is supported by the shelf
rather than clamped at contact") from the demonstration traces rather than by guess, against a
pre-registered acceptance test; (2) say which recording lineage is authoritative for the phase bank
and give the widened uid list on it. Read-only on all tapes, CPU only. Tools:
`can_pos_recovery/slide_score.py` (re-executes a tape and calls the landed
`GenesisCanEnv.end_of_episode()`; the registered predicate is never reimplemented) and the offline
calibration reads recorded below.

## 1. Why (l) failed, in one line

`slide_success` as registered passes **2 of 74** human tapes (255, 305), with 44 `grip_closed`,
18 `no_contact`, 10 `not_picked`. The humans set the can down, open, then **re-close and push**.
Median last-commanded grip across the 74 tapes is 0.39; only 27/74 ever command below 0.3. Nine of
the eleven tapes in the current slide set fail on `grip_closed` and **seven of those nine are
honest-nested** — the predicate rejected demonstrations that provably completed the task. User,
unprompted, on watching the videos: *"sometimes its easier to push with the gripper closed."*

## 2. Clause 5: two candidate signals, measured

**Candidate A — can height (supported vs lifted).** Resting can-centre z = shelf_top + h/2 =
0.2205. Over all 74 tapes, post-pick frames with the can upright in the footprint:

| population | n | \|z − rest\| median | p95 | p99 | max |
|---|---|---|---|---|---|
| gripper commanded open (< 0.15) — unambiguously released | 897 | 0.00 cm | 0.09 | 0.65 | 0.77 |
| gripper commanded closed (> 0.55) | 5232 | 2.13 cm | — | — | — |

The released population is tight: **99 % within 0.65 cm, max 0.77 cm** of the resting height. But
height does **not** separate held from free: 41 % of commanded-closed frames also sit within 1 cm of
resting height, because the human routinely holds the can *down on the shelf* while still gripping.
Height therefore cannot be the clamp test — it can only be the *supported* test, which is what
clause 5 actually asks for.

**Candidate B — finger position (clamped vs not), and a trap.** `state[6]` is the actual finger
position (0 open, 1 closed). While the can is unambiguously in the hand (airborne > 3 cm above
resting, n = 3545): min 0.355, p1 **0.423**, median 0.577, p99 0.709, max 0.847. Unambiguously free
(commanded open and can resting, n = 753): median 0.029, p95 0.112, **max 0.269**. A 66 mm can
between the pads stalls the fingers, so a held can pins the reading into a band.

**The trap:** a reading *above* that band means the fingers shut on empty air — a **closed fist with
the can outside it**, which is exactly the push the user described. At the first post-release contact
frame the current slide set reads: 255 → 0.000, 305 → 0.113, 237 → 0.330, 236 → 0.389, 233 → 0.389,
232 → 0.405, **317 → 0.755, 259 → 0.866, 325 → 0.945, 321 → 0.997, 273 → 1.000**. Five of eleven are
fist-pushes. **Any clause of the form "the gripper must be open" — at any threshold — excludes those
just as wrongly as the withdrawn `grip < 0.3` did.** A two-sided band test (reject only
0.42 ≤ motor ≤ 0.71) would be defensible in principle, but it separates 232 (0.405) from the clamp
floor (0.423) by 1.8 % of range, which is far too fine to register.

**Recommendation: clause 5 = supported, measured by height, no gripper term.**

> can-centre z ≤ resting_z + **0.010 m** at the contact frames, where resting_z = shelf_top + h/2.

1.0 cm is ~1.5× the p99 (0.65 cm) and ~1.3× the max (0.77 cm) of the released-and-resting
population, so it admits every genuinely-supported frame with margin, and it excludes a can carried
in at any real carrying height (median lift 2.13 cm). It contains no gripper term, so it cannot
re-introduce the (l) failure. If an implementation prefers a threshold-free form, the direct
equivalent is a **pick-can ↔ shelf solver contact** at the contact frame; that is strictly better
physics and needs no constant, and I recommend it if the env can expose it cheaply.

**Acceptance test (pre-registered by the requester, reported as run):**
- *Must pass uid 232* — **PASSES** on both lineages.
- *Must fail a can carried in still grasped* — no human tape does this (the prior-release clause
  already removes it), so the clause is untested against a positive instance here and only bites on
  machine policies. **Reported, not fitted.** The 3 tapes it does exclude (`lifted_or_off_shelf`)
  are all tipped.
- *Should land near ~18* — **it lands at 14 (census) / 12 (dHfull).** I did not tune to 18; §4
  explains where the gap is and why I think the eyeball count is measuring something different.

## 3. Lineage: the census is authoritative

`dHfull_w3` (what the phase banks were cut from) and the local w3 census are **different
recordings of the same 74 ICs**: their `actions_delta` differ on **24/74** uids and their recorded
contact flags disagree on 11 [248 250 259 286 294 295 300 316 326 328 333]. That, not any predicate
difference, is why "26 contact" (census) and "21 contact" (dHfull) both looked right.

Decisive evidence, same scorer, same box:

| lineage | re-execution bit-exact | notes |
|---|---|---|
| census w3 | **15/15** so far (run continuing); honest_rescore separately 74/74 | reproduces here |
| dHfull_w3 | **2/74** | replaying its actions locally loses the pick entirely on 286, 293, 294, 295, 300 — all five are picked in the census, two of them nested |

**Recommendation: build and score the phase bank on the census lineage**, because it is the only one
that reproduces on the machine doing the scoring, and because every reported funnel number already
comes from it. **Conditional:** if the phase training and evaluation run on the cluster, the binding
rule is that the bank, the scorer and the runs must share one lineage *and one machine* — this is the
same reproduction question as the open end-to-end determinism bug, and it should be
settled before anything is re-cut. **Refinement received 2026-09-07 late (orchestration session):
the divergence tracks PHYSICAL CORE COUNT, not the instruction set** — a 40-core Broadwell and a
64-core Sapphire Rapids agree bit-for-bit across the AVX2/AVX-512 boundary, while a 36-core and a
40-core Broadwell disagree on the same instruction set, and all 53 same-core-count comparisons are
identical. So the binding constraint is not "same machine" but "same physical core count", which is
a stricter and more testable rule and is consistent with what I measured here (this box is a 6-core
i5-10600K; `dHfull_w3`, recorded elsewhere, re-executes bit-exact 2/74 locally). I am not asserting the census is correct and the cluster wrong;
I am asserting they differ and that mixing them is what produced four irreconcilable counts.

## 4. The widened list, and the honest gap to 18

Amendment-(p) shape with clause 5 at 1.0 cm, scored offline on each lineage's own recorded flags
(one tape per uid, `find_tape` precedence). This is a **calibration read, not the authoritative
score** — clause 5 is not yet in the env; re-score in-env once it lands.

| lineage | n | PASS | uids |
|---|---|---|---|
| census w3 | 74 | **14** | 232 233 236 237 255 273 294 297 298 299 305 317 325 330 |
| dHfull_w3 | 74 | 12 | 232 233 236 237 255 259 273 298 299 305 317 330 |

Agreement 11; only-census [294 297 325], only-dHfull [259]. Of the census 14, **10 are
honest-nested**, 2 contact-not-nested, 2 tipped.

Census failure structure (n = 74): `no_release` 26 (22 of them tipped — the can never rests upright,
correctly excluded), `no_contact_after_release` 26, `lifted_or_off_shelf` 3 (all tipped),
`not_picked` 5 (two of which, **234 and 318, are the bogus lying-can ICs of CONFOUNDS row 51** and
cannot pass anything).

**Where the missing ~4 are.** Of the 26 `no_contact_after_release`, **16 are the short/upright class**
[244 252 254 256 259 261 265 266 269 276 277 279 281 327 328 335]: the human set the can down and
pushed, and the **sim can stopped short of the goal**. Per `SLIDE_ANATOMY_2026-09-07.md` that
shortfall is control-limited and systematic (~2–4 cm; the sim tool tracks the real tool to 0.2 cm,
the can simply under-transfers), so those are sim slide *failures*, not bad demonstrations. If the
user judged "does the slide well" from the real footage — which is the left panel of every review
video — the population he was counting is PASS + short/upright = **30**, and ~18 sits between our 14
and that 30. So neither number is wrong; **they measure different things**: 14 = "the slide succeeded
in simulation", ~18 = "the human performed a good slide".

**Consequence for the phase design, and it is the important one:** selecting the human set by whether
the *simulation* completed the slide selects for the tapes this world happens to reproduce, and
biases the phase's human demonstrations toward easy geometry. If the slide phase is meant to teach
the slide, the defensible selection is on demonstrated intent (released, then pushed toward the goal)
with the sim outcome recorded but not used as the filter — that yields ~30 on the census lineage.
Which of the two selections to use is a design decision for the phase owner and the user, not mine;
I have given both sets and the reason they differ.

## 5. Corrections to my own work in this task

- My first census scoring globbed `full_gc_kp4_riser3_shelf6*`, which also matched the **og4 and
  ognow** worlds — that column was a three-world mixture. Caught and redone with the three w3
  directories and one tape per uid; the pass sets happened to be unchanged, but the first run was
  not valid and is recorded here rather than quietly replaced.
- My first offline pass required 3 consecutive in-episode decisions and so dropped tapes that end at
  contact; the registered predicate has a settle route for exactly that case. Added (2 of the 14
  qualify by that route).
- `slide_score.py`'s `exact` flag compares against the *tape's own* `sim_states`. For `dHfull_w3`
  that reference is a different-machine recording, so a large deviation there indicts the reference,
  not the replay — which is how the lineage split was found.

## 6. Adjudication: the entry gate vs the sim shortfall (asked 2026-09-07, after the place agent's account)

A competing account arrived: both (m) and (o) gate phase ENTRY on `placed_v2`, and 7 human tapes reach
`contact` with no `placed_v2` at all, so 11 + 7 = 18 exactly matches the user's eyeball count and the
defect is in the entry gate rather than the scorer. Verdict: **the mechanism is real, the arithmetic
is a coincidence, and the two accounts are almost disjoint.**

**The 7 (dHfull lineage — I reproduce the set exactly) fail for three different reasons:**

| uid | binding clause | evidence | census outcome |
|---|---|---|---|
| 308 | grip < 0.45 never satisfied | 0/270 post-pick frames pass grip; 27 pass the other three | tipped (62°) |
| 328 | grip < 0.45 never satisfied | 0/194 pass grip; 20 pass the other three | short/upright |
| 333 | grip < 0.45 never satisfied | 0/422 pass grip; 81 pass the other three | tipped (90°) |
| 242 | sustain | all four clauses hold 3 consecutive frames, needs 10 (grip binds: 3 of 18) | contact-not-nested |
| 297 | z-band | with grip+foot+tilt satisfied, can z 0.2442–0.2482 = 2.4–2.8 cm ABOVE resting | tipped (90°) |
| 309 | tilt | 82 frames pass grip, none pass foot+z+tilt; tilt<20 on 116/368 | tipped (90°) |
| 320 | tilt | same shape; tilt<20 on 60/159 | tipped (90°) |

Three (308, 328, 333) are the *same* grip-threshold disease as the withdrawn (l) clause — the entry
gate does have it, and that is a genuine defect worth fixing on its own merits. Three (297, 309, 320)
end at 90° and are correctly excluded. One (242) is a sustain near-miss. **Only 328 and 242 end with
the can still standing.**

**Overlap with the §4 sets:** 7 ∩ my 14 = [297]; 7 ∩ my 16 short/upright = [328]. The two accounts
describe different tapes.

**Why 11 + 7 = 18 is not the explanation:** the identical statistics on the census lineage are 13 and
9, giving 22, and only 4 of the 7 [242 308 309 320] survive the lineage change. A sum that moves by 4
when the recording changes cannot explain a fixed count. My own account does not pin 18 either, and I
am not claiming it does: ~18 lies inside [14, 30] and **no automated count reproduces it, because the
user was judging the human's performance while every automated count scores the simulation's outcome.**

**Full ledger, each number with its pipeline:** 21 = tape-recorded contact, dHfull · 26 = same
statistic, census (recorded and re-executed agree 74/74; the gap to 21 is lineage, not predicate) ·
16 = honest nested after settle, census · 11 = contact-after-`placed_v2`, dHfull (census 13) ·
7 = contact with no `placed_v2`, dHfull (census 9) · 14 = calibrated (p) predicate, census (dHfull 12)
· 30 = census tapes with demonstrated slide intent (14 + 16 shortfalls).

**Proposed corrected entry (measured, not registered here):**

> entry = picked earlier ∧ can in shelf footprint ∧ |can_z − resting_z| ≤ 0.010 m ∧ tilt < 20°,
> sustained 10 frames. **No gripper term.**

Same calibration as clause 5. A carried can is 2.13 cm up at the median so it cannot qualify; the
"hold the can down on the shelf, then push" case (41 % of commanded-closed frames) is admitted, which
is correct for an entry definition. Measured effect: entry 39 → 43 (dHfull) and 43 → 48 (census); full
predicate 12 → 13 and 14 → **15** (census set 232 233 236 237 255 273 294 297 299 300 305 308 317 325
330, 11 of 15 honest-nested). It recovers 308 of the three fist-push uids; 328 and 333 still fail on
no-contact-after-entry and tilt. **The corrected entry buys 1 uid, not 7** — recovering all 7 would
require admitting tapes that settle at 90°.

## 7. The morning decision: 15 vs 18 vs 30 (census lineage, corrected entry gate)

Per-uid table: **`paper/slide_per_uid_2026-09-07.txt`** (74 rows: outcome, entry frame, best goalward
push after entry, whether the slide completes in sim, settled distance, and a plain-English note).

**The ladder.** Entry (can set down upright on the shelf, sustained 10 frames, no gripper term) fires
on 48 of 74. Of those, "the human demonstrably pushed the can toward the goal after setting it down",
by minimum goalward displacement:

| min push | n | NESTED | contact | short | tipped |
|---|---|---|---|---|---|
| any (> 0) | 46 | | | | |
| ≥ 1 cm | 36 | 15 | 2 | 11 | 8 |
| ≥ 2 cm | 33 | 14 | 2 | 9 | 8 |
| **≥ 3 cm** | **30** | 12 | 2 | 8 | 8 |
| ≥ 4 cm | 25 | 8 | 2 | 8 | 7 |
| ≥ 6 cm | 15 | 6 | 1 | 3 | 5 |

The registered "~30" is exactly the ≥ 3 cm push set. The registered "15" is the sim-completes set
[232 233 236 237 255 273 294 297 299 300 305 308 317 325 330], 11 of 15 honest-nested.

**Nothing in the ladder lands on 18** — and that is the finding, not a failure. Two *unrelated*
constructions both total 18:

- the place agent's 11 + 7 = [232 233 236 237 242 255 259 273 297 305 308 309 317 320 321 325 328 333]
- "human pushed ≥ 3 cm but the sim did not complete" = [243 247 248 256 259 265 266 269 274 275 276
  277 279 302 304 321 326 333]

**They share 3 uids and disagree about 15 of the 18.** Two different sets of the same size is
conclusive that hitting the number is numerology. The sets are what matter, and neither is "the 18".

**What to tell the user.** ~18 is a human judgement of the *real* footage; every automated count
scores the *simulation*. The two bracket it for a reason we already measured: 15 tapes complete the
slide in sim, and a further 18 show the human pushing ≥ 3 cm with the sim can stopping short — the
systematic 2–4 cm under-transfer of `SLIDE_ANATOMY_2026-09-07.md`, where the sim tool tracks the real
tool to 0.2 cm and the can simply does not follow. So 15 = "the world reproduces the slide",
30 = "the human performed one", and the ~15 in between are the world's shortfall, not the human's.

**Recommendation for the phase set, unchanged and now quantified:** cutting at 15 trains on the tapes
this world happens to reproduce and biases the phase toward easy geometry; cutting at 30 (≥ 3 cm
push) keeps every demonstrated slide and lets the learner see the hard ones. If a middle is wanted,
≥ 4 cm gives 25 and drops the weakest pushes. The choice is the user's; all three sets are in the
per-uid table.

## 8. Human analogue of the corrected-contact measurement (added after the 09-07 late readout)

The orchestration session re-measured `contact` with the tool point and a far-side test and found
that in the policy cells only **14–16 %** of episodes earning `contact` do so by gripper contact,
while **84–86 %** earn it by carrying the can in and parking it against the goal without releasing;
a true release-scored slide occurs in **0–2 of ~1200 episodes per cell**. The same decomposition on
the human tapes, census w3 lineage, using the corrected entry gate (can set down upright on the
shelf, sustained 10 frames):

| | n | share |
|---|---|---|
| human tapes earning `contact` | 26 | — |
| ...contact **after** the can was set down | **14** | **54 %** |
| ...contact with no prior set-down (carried in) | 12 | 46 % |

uids after a set-down: 232 233 236 237 248 255 273 294 299 305 317 321 325 326.
uids carried in: 242 250 286 295 297 298 300 308 309 316 320 330.

So the humans earn contact by a genuine set-down-then-push in **54 %** of their contact episodes
against the policies' 14–16 %, and demonstrate the full release-then-push slide in **15/74 = 20 %**
of tapes against the policies' ~0.1 %. Two consequences worth carrying into the phase decision:

1. **The policies never learned this slide**, so the human set is not being chosen to match a
   behaviour the learner already has — it is the only source of the behaviour. That argues against
   trimming the human set to the 15 the current world happens to reproduce.
2. **Nearly half of the human `contact` grants are also carry-ins — but the two populations are not
   equally contaminated.** Bare `contact` conflates the two routes on both sides, at **46 %** carry-in
   for the human demonstrations against **84–86 %** for the policy episodes. Any cell scored on bare
   `contact` is therefore comparing populations that are substantially not doing the scored
   behaviour, and doing so to different degrees. (My first draft of this paragraph said the two were
   "each roughly half carry-in"; that understates the asymmetry and would leave a reader unprepared
   for the corrected policy cells halving. Corrected on the orchestration session's challenge.)
   This is a reason the corrected predicate matters for the human-vs-machine comparison itself, not
   only for the phase.
