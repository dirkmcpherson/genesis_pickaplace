# Place-phase results — human vs machine demonstrations, r2dreamer (draft skeleton written 2026-09-04 23:40; numbers regenerate)
*Design and registrations: `paper/PHASE_PLAN_2026-09-04.md` (§1–6 and the dated amendments). Data provenance and every job id: `paper/WM_FIX_LOG_2026-09-03.md` (entries from 09-04 09:00 on) and `$LAB/wm_fix_2026-09-03/COMMANDS.log`.*

## 1. What was run
Phase 2 = **Place**: start from a banked entry state at the pick grant (can lifted, held), succeed on `placed_v2` = grip
command < 0.45 ∧ can inside the shelf footprint ∧ can-centre z in the WORLD's shelf band 0.18–0.24 m ∧ tilt < 20°, sustained
10 frames; tips terminate (no penalty); horizon 600 sim steps (150 decisions). Reward: exactly one +1 at the grant, on the
demo rows and online. Learner: r2dreamer, recipe `bnormclamp1ent5` (state obs, stock actor, return clamp 1.0, entropy 3e-5),
1M sim steps, 8 seeds per arm, corrected world `gc_kp4_riser3_shelf6`.
- **Human arm** `dH_place`: 39 segments (39 ICs) cut from the 74 full-task human recordings (`record_demos.py --teacher
  human --scope full`); training entry bank = the 64 human pick-grant states.
- **Machine arm** `dDP_place`: 63 segments (63 ICs, first tape per IC) cut from the 195-tape harvest of the DP full-task
  teacher (`dHfull_pruned_DP_s0`: pick hold 14/15, rnd 14/30); training entry bank = 179 machine pick-grant states
  (all attempts). Uncapped 104-segment set kept, not run.
- **Shared evaluation banks**: `holdE` = the 13 hold-uid human entries (in-distribution); `polE` = 148 pick-grant states
  produced by the eight phase-1 human-arm checkpoints on the 30 random placements (out of the demo support — the
  random-IC analogue). Fresh process, LAST checkpoint, sampled AND deterministic actions.
- **Contact phase**: human contact-after-release demos = 11 < registered floor 20 → human runs GATE-SKIPPED; the
  machine arm (25 demos) ran one-armed as a learnability read, not a comparison.
- Yields (corrected release band): human 74 tapes → pick 64, placed_v2 39, contact 21 (11 after a release); machine 195
  tapes / 72 ICs → pick 179, placed_v2 (one per IC) 63, contact 46 (25 after a release).

## 2. Result
<!-- regenerate: ssh pax 'cd $LAB/wm_fix_2026-09-03 && python3 phase_table.py runs place' ; paste per-run cells, per-arm totals, permutation tests -->
(pending — place runs 3263097–104 (dH) and 3270587/92/93/94 + 3272139–42 (dDP) finish 2026-09-04 late evening, cluster clock)

## 3. Negative controls and gates
- Registered floor: ≥ 20 phase demos per arm (place: 39 / 63 ✓; contact: 11 / 25 → human skipped).
- Learnability floor for reporting a comparison: ≥ 0.5 on holdE in ≥ 3/8 seeds per arm (PLAN §4) — scored below.
- Pipeline smoke (5k steps, human arm): holdE 0/13, polE 7/148 places, 74% tips — the harness does not manufacture success.
- Random-policy control from the human place bank: 0 places in 150 decisions (single episode, 09-04 09:00 smoke).

## 4. Asymmetries (disclosed)
1. Demo counts 39 vs 63 (one tape per IC on both sides; the machine teacher solved more ICs than the humans placed).
2. Training entry banks 64 (human) vs 179 (machine, all attempts) — the reset distribution over starts, not the demo count.
3. Prefill counts toward the step budget (human 39 short segments ≈ 3.9k rows; machine 63 ≈ 6.3k) — negligible here.
4. The machine tapes come from a DP teacher trained on PRUNED human full-task demos (pre-pick idle removed), harvested
   with 3 attempts per IC and no open-loop verify (pick-scope only); the human tapes are the recorder's follower on the
   raw joystick streams. Both recorded with the plain recorder (no fast-open release filter — CONFOUNDS row 50).
5. polE entries come from HUMAN-arm phase-1 policies (the only ones with a matching state layout at the time); the
   entry distribution is therefore human-policy-shaped for both arms — symmetric across arms, disclosed.
6. Four machine seeds were restarted after landing on a P100 node (torch.compile failure); the restarts are clean runs.

## 5. Not claimed
Not an H4 verdict (pre-registration by the main session pending); not a contact-phase comparison (one-armed); not a
statement about the release-filter recorder variant (og4), which neither arm used.
