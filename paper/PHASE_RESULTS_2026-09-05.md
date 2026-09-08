# Place-phase results — human vs machine demonstrations, r2dreamer (draft skeleton written 2026-09-04 23:40; numbers regenerate)

> **Terminology — "mode" vs "deterministic" (corrected 2026-09-07).** `--mode mode` selects the *mode* of the action distribution rather than sampling it. For RLPD this is genuinely deterministic: the policy returns `tanh(mean)` and repeats exactly. For the world models (r2dreamer, dv3) it is **not** run-to-run deterministic, because the agent samples its stochastic latent inside `act` and there is no per-episode reseed; a cell reproduces exactly only when the whole episode sequence is replayed with the same RNG stream (verified: 0/30 differences on the §5.1 sequence check). No comparison is biased by this — both arms are evaluated identically — but "deterministic" overstates it for world-model cells, and the word is used below in that looser sense.

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
Generated 2026-09-05 09:30 (cluster clock) by `phase_table.py`, all 16 place runs and 8 machine contact runs at 1M steps, LAST checkpoint,
fresh process; the polE-dDP columns (machine-policy-generated bank, 149 entries) were computing at generation time (32 jobs 3290111–42).

| run (place) | holdE S | holdE M | polE S | polE M | polE-dDP S | polE-dDP M |
|---|---|---|---|---|---|---|
| dDP bnormclamp1ent5 s0 | 13/13 | 13/13 | 95/148 | 92/148 | — | — |
| dDP bnormclamp1ent5 s1 | 13/13 | 13/13 | 114/148 | 117/148 | — | — |
| dDP bnormclamp1ent5 s2 | 13/13 | 12/13 | 101/148 | 101/148 | — | — |
| dDP bnormclamp1ent5 s3 | 12/13 | 13/13 | 109/148 | 113/148 | — | — |
| dDP bnormclamp1ent5 s4 | 13/13 | 13/13 | 101/148 | 103/148 | — | — |
| dDP bnormclamp1ent5 s5 | 13/13 | 13/13 | 91/148 | 99/148 | — | — |
| dDP bnormclamp1ent5 s6 | 13/13 | 13/13 | 110/148 | 112/148 | — | — |
| dDP bnormclamp1ent5 s7 | 13/13 | 12/13 | 109/148 | 103/148 | — | — |
| dH bnormclamp1ent5 s0 | 13/13 | 13/13 | 113/148 | 113/148 | — | — |
| dH bnormclamp1ent5 s1 | 13/13 | 13/13 | 82/148 | 86/148 | — | — |
| dH bnormclamp1ent5 s2 | 13/13 | 13/13 | 110/148 | 112/148 | — | — |
| dH bnormclamp1ent5 s3 | 13/13 | 13/13 | 109/148 | 112/148 | — | — |
| dH bnormclamp1ent5 s4 | 13/13 | 13/13 | 77/148 | 82/148 | — | — |
| dH bnormclamp1ent5 s5 | 13/13 | 13/13 | 103/148 | 99/148 | — | — |
| dH bnormclamp1ent5 s6 | 13/13 | 13/13 | 111/148 | 110/148 | — | — |
| dH bnormclamp1ent5 s7 | 13/13 | 13/13 | 109/148 | 118/148 | — | — |

| arm | n | holdE S | holdE M | polE S | polE M | polE-dDP S | polE-dDP M |
|---|---|---|---|---|---|---|---|
| dH | 8 | 104/104 (1.000, n=8) | 104/104 (1.000, n=8) | 814/1184 (0.688, n=8) | 832/1184 (0.703, n=8) | — | — |
| dDP | 8 | 103/104 (0.990, n=8) | 102/104 (0.981, n=8) | 830/1184 (0.701, n=8) | 840/1184 (0.709, n=8) | — | — |
- polE MODE (statistic of record; human-policy entries): dH [113, 86, 112, 112, 82, 99, 110, 118] vs dDP [92, 117, 101, 113, 103, 99, 112, 103] -> Δ per-seed count -1.00, exact two-sided perm p = 0.878 (n=8 vs 8)
- polE SAMPLE: dH [113, 82, 110, 109, 77, 103, 111, 109] vs dDP [95, 114, 101, 109, 101, 91, 110, 109] -> Δ per-seed count -2.00, exact two-sided perm p = 0.748 (n=8 vs 8)
- holdE MODE: dH [13, 13, 13, 13, 13, 13, 13, 13] vs dDP [13, 13, 12, 13, 13, 13, 13, 12] -> Δ per-seed count +0.25, exact two-sided perm p = 0.467 (n=8 vs 8)

| run (contact) | holdE S | holdE M | polE S | polE M | polE-dDP S | polE-dDP M |
|---|---|---|---|---|---|---|
| dDP bnormclamp1ent5 s0 | 10/11 | 10/11 | — | — | — | — |
| dDP bnormclamp1ent5 s1 | 10/11 | 10/11 | — | — | — | — |
| dDP bnormclamp1ent5 s2 | 11/11 | 10/11 | — | — | — | — |
| dDP bnormclamp1ent5 s3 | 11/11 | 11/11 | — | — | — | — |
| dDP bnormclamp1ent5 s4 | 11/11 | 11/11 | — | — | — | — |
| dDP bnormclamp1ent5 s5 | 11/11 | 11/11 | — | — | — | — |
| dDP bnormclamp1ent5 s6 | 11/11 | 11/11 | — | — | — | — |
| dDP bnormclamp1ent5 s7 | 11/11 | 11/11 | — | — | — | — |

| arm | n | holdE S | holdE M | polE S | polE M | polE-dDP S | polE-dDP M |
|---|---|---|---|---|---|---|---|
| dH | 0 | — | — | — | — | — | — |
| dDP | 8 | 86/88 (0.977, n=8) | 85/88 (0.966, n=8) | — | — | — | — |

**Place-phase verdict (registered statistic: polE MODE, n = 8 v 8): human 832/1184 = 0.703 vs machine 840/1184 = 0.709, Δ −0.006,
exact permutation p = 0.878** (SAMPLE 0.688 vs 0.701, p 0.748). holdE (13 hold-uid human starts): 104/104 vs 102/104 — saturated; the
learnability floor (≥ 0.5 holdE in ≥ 3/8 seeds) is met by every seed of both arms. The registered prediction |Δ| < 0.10 is MET.
Per-seed spread on polE MODE: human 82–118, machine 92–117 (two human seeds, s1 and s4, sit at 0.55–0.58; the arms' medians are 111 and 103).
**Contact phase, one-armed (machine, 25 demos):** holdE 86/88 sampled / 85/88 deterministic across 8 seeds; the human arm (11 demos) was
gate-skipped, so this is a learnability read: contact-from-release is readily learned with 25 machine segments.


### 2.y MATCHED-COUNT machine arm (39 demos) — the place number of record (amendment (e)); read out 2026-09-06

> **REPUBLISHED 2026-09-08 — the figures below are superseded; the finding is not.** They were scored on the superseded raw-grip entry bank and with an evaluator that silently substituted a different start when an entry failed to restore. Re-scored on the rebuilt bank with entry pinning and hardware pinned, 8 v 8, polE MODE:
>
> | arm | published | re-scored | movement |
> |---|---|---|---|
> | human (39) | 0.703 | **0.715** | +0.013 |
> | machine (39) | 0.647 | **0.652** | +0.005 |
>
> **Δ +0.056 (p 0.227) → +0.063 (p 0.112).** The registered |Δ| < 0.10 is met before and after. Machine-63 (descriptive) 0.736 → 0.726.
>
> **polE SAMPLE — the statistic the three-learner table uses — moves much more, and the two arms move in OPPOSITE directions:**
>
> | arm | published | re-scored | movement |
> |---|---|---|---|
> | human (39) | 0.688 | **0.708** | **+0.020** |
> | machine (39) | 0.674 | **0.652** | **−0.022** |
>
> **Δ +0.014 (p 0.743) → +0.056 (p 0.206).** The registered margin is met both times, so the conclusion holds — **but the gap quadrupled.** This is the sharp form of the caveat above: the corrections are *symmetric by construction* (exactly 40 restore-failure episodes per arm) yet *asymmetric in effect*. "Each arm moved only about 0.02" and "the gap moved 0.042" are both true, and only the second one bears on the comparison. Never judge the size of a correction by how far each arm moved.
>
> **The hold-out cells isolate one cause cleanly, which is why they were re-run even though their bank file is byte-identical.** That bank was never rebuilt, so no bank effect and no restore failures are in play, leaving only the entry-pinning fix: MODE human 1.000 → 0.990, machine 0.990 → 0.962; SAMPLE unchanged at 1.000 / 0.962. **Drawing entries with replacement rather than enumerating each once was worth up to 0.029 on its own** on a 13-entry bank. That decomposes the policy-bank movement into its two causes instead of leaving them entangled.
>
> **Do not read the small movement as evidence that the corrections were negligible — two of them push in opposite directions and happen to nearly cancel.** Restore failures rise from 0 to 5 per cell (exactly symmetric, 40 episodes per arm), so those episodes now count as failures instead of being silently replaced by a different start, which pushes rates *down*; meanwhile the rebuilt bank's corrected grip makes the restored entries slightly more tractable, which pushes them *up*. Netting to +0.013 and +0.005 is a coincidence of the two, not a measure of either.

| arm | demos | holdE MODE | polE SAMPLE | polE MODE |
|---|---|---|---|---|
| human | 39 | 104/104 (1.000) | 814/1184 (0.688) | 832/1184 (0.703) |
| machine, matched | 39 | 103/104 (0.990) | 798/1184 (0.674) | 766/1184 (0.647) |
| machine, uncapped (secondary) | 63 | 102/104 (0.981) | 830/1184 (0.701) | 840/1184 (0.709) |

Per-seed MODE: human [113, 86, 112, 112, 82, 99, 110, 118] vs machine-39 [109, 79, 111, 105, 90, 79, 95, 98] → Δ +8.25 per seed (**+0.056**), exact p 0.227; SAMPLE Δ +2.0 (+0.014), p 0.743. Registered prediction (|Δ| < 0.10) MET. Reading: at equal demonstration counts the human-demo arm is 0.056 ahead on the policy-generated starts, not significant at n = 8; the earlier 0.709 for the machine arm was carried by its extra 24 demonstrations (+0.062 from 39 to 63 demos, itself a demo-count effect worth noting). Runs 3291443–50.

### 2.x Symmetry control: the MACHINE-policy entry bank (polE-dDP), read out 2026-09-05 10:10

The registered statistic (§2) starts every place episode from entries generated by the eight HUMAN-arm pick policies (polE). To rule out an entry-distribution asymmetry, the same 16 checkpoints were also evaluated on a bank generated the same way by the eight MACHINE-arm pick policies (`polE_place_dDP.json`, 149 entries, dumps 3280431–38, evals 3290111–42, `polE_dDP_sweep.sh`).

| arm | n | polE-dDP SAMPLE | polE-dDP MODE | (registered polE MODE, for reference) |
|---|---|---|---|---|
| human (dH) | 8 | 802/1192 (0.673) | 812/1192 (0.681) | 832/1184 (0.703) |
| machine (dDP) | 8 | 848/1192 (0.711) | 858/1192 (0.720) | 840/1184 (0.709) |

Per-seed counts, MODE: dH [114, 84, 107, 104, 74, 104, 109, 116] vs dDP [106, 113, 102, 105, 106, 105, 110, 111] → Δ per-seed −5.75 (rate −0.039), exact two-sided permutation p = 0.374 (n = 8 v 8). SAMPLE: Δ −5.75, p = 0.434.

Reading: on the machine-policy bank the human arm is 0.04 lower, on the human-policy bank 0.006 lower; neither is significant and both are inside the registered |Δ| < 0.10 margin. The human arm's seed spread is wider on both banks (two seeds, s1 and s4, at 0.50–0.58 on every bank; the machine arm's worst seed is 0.62). The bank of origin does not change the conclusion; the source of the entry states is not a confound of the §2 null. Both banks are 148/149 entries from 8 pick policies × the rnd30 set, deduplicated per IC; the two banks share no entries.

## 3. Negative controls and gates
- Registered floor: ≥ 20 phase demos per arm (place: 39 / 63 ✓; contact: 11 / 25 → human skipped).
- Learnability floor for reporting a comparison: ≥ 0.5 on holdE in ≥ 3/8 seeds per arm (PLAN §4) — scored below.
- Pipeline smoke (5k steps, human arm): holdE 0/13, polE 7/148 places, 74% tips — the harness does not manufacture success.
- Random-policy control from the human place bank: 0 places in 150 decisions (single episode, 09-04 09:00 smoke).

## 3. Contact after release (registered statistic: polE_contact MODE, amendment (f)); read out 2026-09-05 21:20 — matched pair PENDING

> **Predicate caveat added 2026-09-07 — read before citing this section.** These cells are scored on bare `contact`, which does not distinguish a set-down-then-push slide from carrying the can in and parking it against the goal. Measured with the corrected predicate (tool point rather than wrist; tool required on the far side of the pick-can from the goal), **84–86 % of policy grants are carry-ins**, and the human demonstrations are contaminated too, at 12 of 26 (46 %). Corrected-predicate cells, 8 v 8, **now complete at full n (192/192 cells)**: contact-after-release **0.346 human vs 0.366 machine** (Δ −0.020, p 0.31); carrycontact **0.285 vs 0.250** (Δ +0.035, p 0.55); release-scored `slide_success` ≈ 0 in both phases. The registered wrong-side disconfirm branch has fired: gripper–goal contact explains only 11–33 % of failing episodes, the rest being the can still in the grasp. **The demonstration-source null is unaffected** — |Δ| < 0.10 on every comparison. **Read the three predicates as three different things:** `slide_success` is the *task outcome* and the finding there is that **no arm learns a true slide**; `contact_push` is the *discriminating statistic* that carries the human-versus-machine comparison; bare `contact` is retained as the *legacy predicate* and **overstates capability by 1.5–3×**, so its failing fraction must travel with it. Do not put a p-value on `slide_success` — at 0–6 % it has no discriminating power and a null there would be an artefact of the floor.

Entry = the placed_v2 state. Two banks: holdE (11 human placed states; the can already touches-distance from the goal) and **polE_contact** (160 placed states generated by the 16 place checkpoints, 10 per checkpoint; typically far from the goal).

| arm | demos | holdE MODE | polE_contact SAMPLE | polE_contact MODE |
|---|---|---|---|---|
| human (sub-floor) | 11 | 88/88 (1.000) | 747/1280 (0.584) | 759/1280 (0.593) |
| machine, own yield (unmatched) | 25 | 85/88 (0.966) | 784/1280 (0.613) | 780/1280 (0.609) |
| machine, matched | 11 | 83/88 (0.943) | 769/1280 (0.601) | 770/1280 (0.602) |

Per-seed MODE counts: human [95, 91, 79, 117, 101, 88, 76, 112] vs machine-25 [97, 92, 97, 121, 97, 89, 106, 81] → Δ −2.6 per seed (−0.016), exact p 0.711; SAMPLE Δ −4.6 (−0.029), p 0.413. Descriptive only (25 vs 11 demos); the matched machine-11 arm is the comparison of record and is evaluated on both banks in-job. Reading so far: with a bank that is not at ceiling, the human arm trained on 11 demonstrations is within 0.03 of the machine arm trained on 25; restore failures 0 on this bank. Zero of the human seeds, and zero of the machine seeds, fall below 0.47.

**Matched pair, read out 2026-09-06 (of record):** polE_contact MODE human-11 [95, 91, 79, 117, 101, 88, 76, 112] vs machine-11 [103, 91, 94, 92, 100, 115, 85, 90] → 759/1280 (0.593) vs 770/1280 (0.602), Δ −1.4 per seed (**−0.009**), exact p 0.841; SAMPLE 0.584 vs 0.601, Δ −0.021, p 0.644. Registered prediction (|Δ| < 0.10) MET. holdE MODE 88/88 vs 83/88 (Δ +0.6 per seed, p 0.026): the human arm is perfect on the human-entry starts and the machine arm drops 5 episodes of 88 — significant but at ceiling and on 11 entries; it does not carry to the discriminating bank. Both arms are below the registered 20-demo floor (11 each) and are reported as a sub-floor pair. Runs: human 3290207–14, machine 3291451–58.

## 4. Carrycontact (contact by any route from the pick-grant state; matched 21 vs 21); read out 2026-09-06

> **Predicate caveat added 2026-09-07 — read before citing this section.** These cells are scored on bare `contact`, which does not distinguish a set-down-then-push slide from carrying the can in and parking it against the goal. Measured with the corrected predicate (tool point rather than wrist; tool required on the far side of the pick-can from the goal), **84–86 % of policy grants are carry-ins**, and the human demonstrations are contaminated too, at 12 of 26 (46 %). Corrected-predicate cells, 8 v 8, **now complete at full n (192/192 cells)**: contact-after-release **0.346 human vs 0.366 machine** (Δ −0.020, p 0.31); carrycontact **0.285 vs 0.250** (Δ +0.035, p 0.55); release-scored `slide_success` ≈ 0 in both phases. The registered wrong-side disconfirm branch has fired: gripper–goal contact explains only 11–33 % of failing episodes, the rest being the can still in the grasp. **The demonstration-source null is unaffected** — |Δ| < 0.10 on every comparison. **Read the three predicates as three different things:** `slide_success` is the *task outcome* and the finding there is that **no arm learns a true slide**; `contact_push` is the *discriminating statistic* that carries the human-versus-machine comparison; bare `contact` is retained as the *legacy predicate* and **overstates capability by 1.5–3×**, so its failing fraction must travel with it. Do not put a p-value on `slide_success` — at 0–6 % it has no discriminating power and a null there would be an artefact of the floor.

The two-armed contact comparison of record when contact-after-release yields too few human tapes: entry = the pick-grant state (the place banks), +1 on goal contact whether the can is held or released, 600 decisions.

| arm | demos | holdE MODE | polE SAMPLE | polE MODE | polE MODE over the 143 restorable entries |
|---|---|---|---|---|---|
| human | 21 | 104/104 (1.000) | 951/1184 (0.803) | 955/1184 (0.807) | 955/1144 (0.835) |
| machine, matched | 21 | 103/104 (0.990) | 944/1184 (0.797) | 942/1184 (0.796) | 942/1144 (0.823) |

Per-seed MODE: human [116, 124, 120, 120, 115, 121, 123, 116] vs machine [117, 119, 114, 119, 123, 115, 116, 119] → Δ +1.6 per seed (**+0.011**), exact p 0.348; SAMPLE Δ +0.9, p 0.660. Registered prediction (|Δ| < 0.10) MET. Five polE entries fail to restore in this scope for every arm (rf 40 = 5 × 8 seeds); they are counted as failures symmetrically and the 143-entry column drops them. Human runs 3290191–98; machine 3291435–42 (the unmatched 36-demo machine runs were cancelled before they started).

## 5. End-to-end full task (amendment (d)); wave 1 read out 2026-09-06, **wave 2 read out 2026-09-07 08:40 — 8 v 8 of record below**

From the pick-scope starts to nested, staged sparse reward (pick 1 / place 1 / contact 2 / nested 4), 2e6 steps, return clamp 8, human = all 74 tapes incl. 10 no-picks vs machine = best tape per start (72). Success-by-stage = the stage was granted at any time in the episode. `placed_v2` is a phase-scope key and is never granted in scope=full; the release-based `placed` key is the full-scope place stage.

| cell (4 seeds each) | picked | contact | nested |
|---|---|---|---|
| rnd30 MODE, human | 64/120 (0.533) | 48/120 (0.400) | 25/120 (0.208) |
| rnd30 MODE, machine | 60/120 (0.500) | 34/120 (0.283) | 16/120 (0.133) |
| rnd30 SAMPLE, human | 63/120 (0.525) | 54/120 (0.450) | 27/120 (0.225) |
| rnd30 SAMPLE, machine | 55/120 (0.458) | 33/120 (0.275) | 14/120 (0.117) |
| hold15 MODE, human | 56/60 (0.933) | 46/60 (0.767) | 18/60 (0.300) |
| hold15 MODE, machine | 46/60 (0.767) | 36/60 (0.600) | 15/60 (0.250) |

Per-seed rnd30 MODE: picked human [8, 18, 19, 19] vs machine [14, 10, 22, 14] (Δ +0.033, p 0.83); contact [4, 16, 15, 13] vs [5, 6, 18, 5] (Δ **+0.117**, p 0.49); nested [2, 4, 9, 10] vs [4, 1, 7, 4] (Δ +0.075, p 0.40). Against the registrations: P1 (both arms learn the pick, ≥ 0.5 in ≥ 3/4 seeds) — human 3/4 yes, machine 1/4 (mean exactly 0.500): met for the human arm only; P2 (|Δ| < 0.10 at every stage reached ≥ 0.2) — met at pick and nested, **not at contact** (+0.117, n.s.); P3 (nested < 0.2 both) — human 0.208, machine 0.133: marginally not met for the human arm. Every difference points the same way (human ≥ machine at every stage and on both IC sets), none is significant at n = 4. The registered wave-2 trigger (both arms ≥ 0.5 picked on rnd30 MODE) is met by the letter (0.533 / 0.500); wave 2 = seeds 4–7 per arm submitted 2026-09-06 22:40 (3337021–28, fixed launcher, in-job evals) → the 8 v 8 readout decides. Descriptive: these are the first policies in this project to reach nested from the pick-scope starts (0.21–0.30 on the human arm). **Route:** the release-based `placed` stage is almost never granted (human 0–1 of 120, machine 0–3 of 120 per cell) while contact and nested are — the policies carry the held can straight into the goal can and settle there without ever releasing it (the held-contact route the carrycontact phase rewards; `nested` requires picked + proximity + both upright + settled, not a release). The human demonstrations mostly release before sliding (39 of 64 picked tapes); the learned policies do not copy that. Wave-1 runs 3290391–98, evaluated post hoc (checkpoints intact, protocol identical).


### 5.1 Eight seeds per arm (of record; wave 1 seeds 0–3 evaluated post hoc, wave 2 seeds 4–7 in-job; identical checkpoint protocol)

| cell (8 seeds each) | picked | contact | nested |
|---|---|---|---|
| rnd30 MODE, human (all 74 tapes) | 120/240 (0.500) | 91/240 (0.379) | 39/240 (0.163) |
| rnd30 MODE, machine (best 72) | 129/240 (0.537) | 81/240 (0.338) | 46/240 (0.192) |
| rnd30 SAMPLE, human | 124/240 (0.517) | 107/240 (0.446) | 52/240 (0.217) |
| rnd30 SAMPLE, machine | 127/240 (0.529) | 86/240 (0.358) | 47/240 (0.196) |
| hold15 MODE, human | 106/120 (0.883) | 90/120 (0.750) | 38/120 (0.317) |
| hold15 MODE, machine | 104/120 (0.867) | 79/120 (0.658) | 42/120 (0.350) |

Per-seed rnd30 MODE, human vs machine: picked [8, 18, 19, 19, 18, 9, 11, 18] vs [14, 10, 22, 14, 19, 19, 15, 16] (Δ −0.037, p 0.643); contact [4, 16, 15, 13, 13, 7, 7, 16] vs [5, 6, 18, 5, 12, 8, 14, 13] (Δ +0.042, p 0.639); nested [2, 4, 9, 10, 2, 1, 6, 5] vs [4, 1, 7, 4, 9, 6, 9, 6] (Δ −0.029, p 0.621). SAMPLE: contact Δ +0.088 (p 0.232), nested Δ +0.021 (p 0.747). hold15 MODE: contact Δ +0.092 (p 0.328), nested Δ −0.033 (p 0.778).

**Verdict against the registrations.** P1 (both arms learn the pick): 5/8 seeds ≥ 0.5 on rnd30 MODE in EACH arm — met. P2 (|Δ| < 0.10 at every stage either arm reaches ≥ 0.2): met at every cell (largest |Δ| 0.092, hold15 contact). P3 (nested < 0.2 for both): met (0.163 / 0.192). The wave-1 direction (human ahead at every stage, contact +0.117) did not survive four more seeds per arm: at 8 v 8 the signs alternate by stage and by cell, and no difference approaches significance. Conclusion of record: **no demonstration-source effect on the end-to-end task at 2e6 steps**, consistent with pick, place, contact and carrycontact. Per-seed spread is large (picked 8–19 of 30 within an arm), which is the seed-variance story of the whole project; the `placed` route note above stands (0–4 of 240 granted).

## 6. Asymmetries (disclosed)
1. Demo counts 39 vs 63 (one tape per IC on both sides; the machine teacher solved more ICs than the humans placed).
2. Training entry banks 64 (human) vs 179 (machine, all attempts) — the reset distribution over starts, not the demo count.
3. Prefill counts toward the step budget (human 39 short segments ≈ 3.9k rows; machine 63 ≈ 6.3k) — negligible here.
4. The machine tapes come from a DP teacher trained on PRUNED human full-task demos (pre-pick idle removed), harvested
   with 3 attempts per IC and no open-loop verify (pick-scope only); the human tapes are the recorder's follower on the
   raw joystick streams. Both recorded with the plain recorder (no fast-open release filter — CONFOUNDS row 50).
5. polE entries come from HUMAN-arm phase-1 policies (the only ones with a matching state layout at the time); the
   entry distribution is therefore human-policy-shaped for both arms — symmetric across arms, disclosed.
6. Four machine seeds were restarted after landing on a P100 node (torch.compile failure); the restarts are clean runs.

## 7. Not claimed
Not an H4 verdict (pre-registration by the main session pending); not a contact-phase comparison (one-armed); not a
statement about the release-filter recorder variant (og4), which neither arm used.

## 6. Clock control: the repeat-1 pilot (registered as PHASE_PLAN amendment (c′); read out 2026-09-05 14:55)

Question (user): could action_repeat 4 be hiding a human-vs-machine effect? The 30 Hz diagnostic (`clock_diag.py`) showed machine per-step targets reverse direction inside the 4-step window far more than human ones (d ≈ −1.9), content the repeat-4 learner never sees. Pilot: the pick stage re-trained with one decision per simulator step, the same 66/58 matched tapes re-encoded at one row per sim step (cap 0.025 per step, as at repeat 4, which integrates a×cap every sim step), discount and gradient updates per sim step held equal, 4 seeds per arm, 1e6 sim steps.

| cell | human (4 seeds) | machine (4 seeds) | Δ | p |
|---|---|---|---|---|
| rnd30 MODE (registered) | 77/120 (0.642) | 78/120 (0.650) | −0.008 | 1.000 |
| rnd30 SAMPLE | 76/120 (0.633) | 74/120 (0.617) | +0.017 | 0.829 |
| hold15 MODE / SAMPLE | 60/60 / 59/60 | 60/60 / 60/60 | 0 / −0.017 | 1.000 / 1.000 |

Every seed of both arms learned (0.60–0.73 on rnd30 MODE), so the recipe transfers to the finer clock, and the null is unchanged: the hold was not masking a source effect. Uncompensated by construction and disclosed: 4× more decisions per unit experience and a 4× shorter model context in sim time (batch_length 64 decisions). Cross-clock rates are descriptive only.
