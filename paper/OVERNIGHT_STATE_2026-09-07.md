# Overnight state, 2026-09-07 ~23:00 → morning of 09-08

*Read `WHAT_EACH_PHASE_TRAINS_ON_2026-09-07.md` first (what Pick/Place/Slide actually optimise), then this. Corrections of record live in `REVIEW_GUIDE_2026-09-07.md` §8 + addendum.*

## Decisions waiting for you

1. **Slide reward.** The phase pays and terminates on bare `contact` (which a policy collects while still gripping) but is scored on release-based `slide_success`. Its 32 training jobs are **held**, and the place agent is registering amendment (o) to make the reward match the score — this completes your "train for contact with the can placed on the shelf and not held" instruction rather than being a new decision. Release when you have seen it and the count reconciliation below.
2. **How many human tapes actually slide.** You counted ~18; our labels say 26 contact / 21 any-route / 16 nested / **11 contact-after-release** of 74, and the 11 is what made the phase sub-floor and capped the machine arm at 11. The slide session is scoring all 74 under the registered predicate and will list the passing uids plus the failing clause for each. **If it comes back ~18, both Slide arms get rebuilt before training.**
3. Whether to widen end-to-end to 16 v 16 (current minimum detectable effect ≈ 0.21).

## Open problem (nothing is being published against it)

The evaluator does not reproduce one end-to-end cell of record: `dHfull_all_s3` rnd30 episode 0, identical IC/checkpoint/mode/seed/horizon, record = nested in 32 steps, rerun = timeout at 300 with no pick. Two facts now in hand:
- **Our phase evaluations are bit-exact across nodes** — 3488 episodes, 23 cells, zero differences in steps/outcome/contact, aggregate 0.7314 vs 0.7314 (`$W/repro_check.py`). This **contradicts** the earlier "up to 62 % of episodes differ" claim, which the contact_push agent has been asked to trace to its cells and withdraw if it came from its superseded wrist-based run. One of the two must be struck, not softened.
- **Full-scope episodes look order-dependent**: the same start reproduces exactly when run first and diverges when run second — phase scopes cannot do this because each episode teleports a banked entry.

Running now: isolation 3355832 (current×2, (j)+(l)-reverted, history arm, rnd30 ep0) and **sequence check 3355868**, a 30-episode rerun in the record's own order diffed per-episode *and* on aggregate. **That job decides whether `PHASE_RESULTS` §5.1's 8v8 end-to-end numbers stand as published.** The 320-cell re-score stays held until it reads out.

## Results that landed today

- **Robomimic RLPD:** mixed-human 0.455 vs SAC-generated 0.147, p 0.008 — the registered falsifier fired. **No-demo control ran tonight and passed at 0.000**, so the defensible sentence is "MG helps less than MH", not "MG is worthless". Action-statistics arms are queued; note the SAC demos cannot be smoothed at all (0/200 survive re-execution), so that control is a bounded dose test, not a clean separation.
- **Robomimic DP (partial):** mixed-human 41–46/50, SAC-generated 1–8/50. Contrast withheld until three retrains finish.
- **dv3 second world model:** G2 passed — 0.700/0.700 human, 0.633/0.633 machine on random starts, hold 15/15. The clamp fix works on both ports. G3 four-seed comparison lands ~17:00 today. **`RESULTS` §7 item 9 and CONFOUNDS row 44 say dv3 has no working configuration and must be updated.**
- **Slide predicate, measured:** carrycontact policies never release — contact 124/148, geometric push test 27/148, `slide_success` 0/148; slide-phase smokes fail with reason `grip_closed` on every checked episode. Your suspicion about what "contact" was rewarding is confirmed at the mechanism level.

## Running overnight

Place 32 (DP+RLPD) · end-to-end 32 (DP+RLPD, amendment (n)) · robomimic recovery 72 across four batches · dv3 G3 4 · contact/slide re-score 192 cells · in-distribution `spots60` evals (12 held to free CPU quota, resubmit in the morning) · machine-first teachers (the fork's, after their disk-full loss).

**Held deliberately:** 32 Slide runs (reward), 320-cell re-score (reproduction), 12 spots60 evals (quota).

## Infrastructure lessons now enforced

Filesystem hit 100 % and killed every job; 434 GB freed, all finals/selected checkpoints/evals/datasets kept. Launchers now keep only final checkpoints, refuse to start under 100 GB free, and refuse to score a checkpoint whose step ≠ the budget. Registrations are committed before submission. Short CPU jobs must use `--qos=preempt -p preempt` (the standard quota caps at 250 cores).

## Late additions (23:40–00:10) — read these with the sections above

**The Slide predicate I registered was wrong, and it is withdrawn (amendment (p)).** `slide_success` as written in (l) required the gripper commanded open (`< 0.3`) at contact. Scored against the demonstrations it passes **2 of 74**, with **44 failures on the grip clause alone**: the human sets the can down, opens fully, then re-closes to ≈ 0.4 and pushes the can home half-closed (user's own words, "sometimes its easier to push with the gripper closed"; worked example uid 232, in our 11, fails on grip alone). Median last-commanded grip across the 74 is 0.39; only 27 of 74 ever go below 0.3. Amendment (o), which would have made the training reward match that predicate, was **stopped before landing** — it would have trained the arm away from the demonstrated solution. Replacement registered: prior release onto the shelf, plus contact, footprint and tilt, plus a not-clamped clause whose threshold must be **calibrated from the demonstration traces** with a pre-stated acceptance test (must pass 232, must reject carry-in, should land near the ~18 counted by eye — and if it does not, report that rather than tune to it). 32 Slide jobs stay held.

**Two anomalies, now properly separated** (I conflated them earlier; corrected here):
- *Recording-lineage divergence*: `dHfull_w3` and the census recording of the same 74 ICs differ on 24 of 74 action streams and 11 contact flags. This explains why our four counts (26 / 21 / 16 / 11) never reconciled — it is a lineage gap, not a predicate gap. **Whichever lineage the phase bank is built from must be the one that is scored**; nothing is re-cut until the slide session says which.
- *Evaluator reproduction failure*: a replay of ONE checkpoint, record on cluster node pax109, rerun on pax154, same lineage. Not the same mechanism. Today's patches are exonerated (reverted and current are identical at every step; 3488 episodes bit-exact post-freeze). Under test now: the same episode on pax109 and two other nodes. If pax109 reproduces and the others do not, the axis is the node; if pax109 also fails, the record is not reproducible on its own hardware and the cause is unnamed.
- The peer's earlier same-node divergence evidence came from **pre-freeze** cells (trees being edited, disk filling), so node and run-condition are confounded there; only the running comparison separates them.

**Load-bearing job to look for first in the morning: sequence check 3355868** — a 30-episode rerun in the record's own order, diffed per episode and on aggregate. It decides whether `PHASE_RESULTS` §5.1's 8v8 end-to-end numbers stand as published, independently of which explanation wins.
