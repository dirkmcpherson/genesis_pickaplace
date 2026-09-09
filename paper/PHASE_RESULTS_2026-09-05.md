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

> **RE-SCORED 2026-09-08 on the rebuilt bank with pinned entries, 8 v 8 — conclusions unchanged, and the control is now STRONGER.**
>
> | cell | published | re-scored | Δ human − machine |
> |---|---|---|---|
> | polEdDP MODE v machine-39 | 0.681 v 0.643 | **0.695 v 0.623** | +0.039 (p 0.398) → **+0.072 (p 0.129)** |
> | polEdDP SAMPLE v machine-39 | 0.673 v 0.639 | 0.681 v 0.647 | +0.034 (p 0.535) → +0.034 (p 0.431) |
> | polE MODE v machine-63 | 0.703 v 0.709 | 0.715 v 0.714 | −0.007 (p 0.878) → **+0.002 (p 0.980)** |
> | polEdDP MODE v machine-63 | 0.681 v 0.720 | 0.695 v 0.702 | −0.039 (p 0.374) → −0.007 (p 0.892) |
>
> **Both registered clauses of amendment (i) hold, and the second holds more tightly than before.** Clause 1 (|Δ| < 0.10 on the machine-policy bank) is met at 0.072 mode and 0.034 sampled. Clause 2 — that the bank of origin does not carry the null, |Δ_polE − Δ_polEdDP| < 0.05 — is met at **0.009**, against 0.017 on the superseded cells. **Correcting the bank therefore strengthens the symmetry argument rather than weakening it.** This is worth stating explicitly, because "the symmetry control moved too" invites the opposite reading.
>
> The opposite-directions pattern recurs: on polEdDP mode the human arm moved +0.014 and machine-39 moved −0.019, so each shifted under 0.02 while the gap nearly doubled.
>
> **An independent consistency check passed on the way.** Restore failures went 0 → 24 per arm on this bank, which is exactly 3 non-surviving entries of 149 across 8 seeds — matching the standalone survival measurement of §2.2, on a *different* bank from the one that measurement was calibrated against.
>
> Machine-63 is now 8 seeds and essentially tied with the human arm on the corrected bank (0.715 v 0.714). It remains the disclosed secondary under amendment (e): its margin over machine-39 is a demonstration-count effect, not a source one.

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

> **RE-SCORED 2026-09-08 on the rebuilt bank with pinned entries, 8 v 8: 0.593 v 0.602 (p 0.841) → **0.580 v 0.598 (p 0.690)**. The null survives the corrections.**
>
> **A caveat on the `slide_success` column, which affects how this phase has been described all day.** That column implements amendment **(l)**'s predicate, requiring the gripper to be open at contact — and **(p) withdrew that clause**, because it passes only **2 of 74 human demonstrations**: people release fully, re-close to about 0.4, and push the can home with the fingers partly shut. (p)'s replacement, requiring a prior release plus a clause that is still uncalibrated, is not implemented. **So the column scores a predicate the demonstrations themselves fail**, and the two (l) predictions it "meets" are met for the withdrawn version.
>
> **`slide_success` is therefore demoted from statistic of record to diagnostic.** No other column is affected. The discriminating statistic for this phase remains `contact_push` — the geometric test using the tool point, with the tool required on the far side of the can from the goal — and bare `contact` remains the legacy predicate, which overstates capability by 1.5–3× and must travel with its failing fraction.
>
> This also means the Slide phase's success definition is **still open**: it depends on calibrating (p)'s clause 5, which is now on the critical path for the held training jobs, for any sparse end-to-end reward, and for the statistic this section reports.

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

> **RE-SCORED 2026-09-08 on pinned hardware, 8 v 8. Every registered prediction is met — but this section must NOT be republished as an equivalence claim.**
>
> | stage | human (record → re-scored) | machine (record → re-scored) | Δ | p | MDE |
> |---|---|---|---|---|---|
> | picked | 0.500 → **0.492** | 0.537 → 0.537 | −0.046 | 0.554 | **0.210** |
> | placed_v2 *(new)* | — → 0.163 | — → 0.200 | −0.038 | 0.520 | 0.154 |
> | contact | 0.379 → **0.388** | 0.338 → 0.338 | +0.050 | 0.569 | **0.243** |
> | contact_push *(new)* | — → 0.204 | — → 0.212 | −0.008 | 0.942 | 0.169 |
> | nested (proxy — the published figure) | 0.163 → **0.138** | 0.192 → 0.192 | −0.054 | 0.300 | 0.141 |
> | **nested_honest** *(new)* | — → **0.046** | — → **0.104** | −0.058 | **0.087** | 0.088 |
> | slide_success *(l)* | — → 0.042 | — → 0.017 | +0.025 | 0.277 | 0.053 |
>
> **A REGISTERED PREDICTION FAILS, in the machine arm's favour.** On the in-distribution set, `placed_v2` reads human **0.083** against machine **0.242** — Δ **−0.158**, outside the ±0.10 margin, and (d) P2 applies because the machine arm exceeds 0.2. **P2 is not met there.** It is underpowered (MDE 0.254, p 0.106) and in-distribution rather than on the random support, so it is not evidence of an effect — but it is a failure and it is recorded as one rather than folded into "all predictions met", which is what I wrote earlier and which was true only of the out-of-distribution cells.
>
> **Every hint in this section runs the same way, toward the machine arm.** `nested_honest` is negative in all three cells (−0.058, −0.017, −0.067) and `placed_v2` in all three (−0.038, −0.062, −0.158). None is significant. The honest completion rate is 0.046 human against 0.104 machine, a 2.3× ratio where the published proxy showed 1.4×.
>
> **The caveat that bears on every one of those hints:** the machine demonstration set is **best-of-3 per initial condition** while the human set keeps every attempt including failures (Σ reward 206 against 118). So a consistent machine-favouring lean is equally consistent with **how the set was built** as with anything about demonstration source, and cannot be read as the latter.
>
> **Why this is still not an equivalence.** On the random support no stage falls outside ±0.10 and those registered predictions are met — (d) P2 at picked, placed_v2, contact and contact_push; (d) P3 on the honest predicate; (l)'s margin; and (l)'s ordering, `slide_success ≤ nested_honest`, in both arms. **But five of the seven stages have a minimum detectable effect larger than the margin they are tested against** — 0.210 and 0.243 at picked and contact against a 0.10 margin. Those nulls cannot exclude an effect the full width of the region we would need to rule out. The honest report is "no effect detected, at a power that could not have detected one", not "the arms are equivalent".
>
> **The stage to watch is `nested_honest`, and it points the other way.** The machine arm completes the task honestly **2.3× as often** — 0.104 against 0.046, p 0.087 — where the published proxy showed only 1.4×. It is also one of only two stages actually powered below its margin. So replacing the training proxy with the honest predicate both lowers the numbers and **sharpens the one difference that may be real, in the machine arm's favour.**
>
> **A reproduction asymmetry that supports the hardware account.** Between record and re-score, the **machine arm reproduces exactly at every stage** (picked 129→129, contact 81→81, nested 46→46) while the **human arm moves** (120→118, 91→93, 39→33) — and the human arm is exactly where the minority-hardware-class originals sit. The two independent re-scores agree with each other to 0.0000 at every stage in both arms; it is the *records* that differ from them, on one arm only.

| cell (8 seeds each) | picked | contact | nested |
|---|---|---|---|
| rnd30 MODE, human (all 74 tapes) | 120/240 (0.500) | 91/240 (0.379) | 39/240 (0.163) |
| rnd30 MODE, machine (best 72) | 129/240 (0.537) | 81/240 (0.338) | 46/240 (0.192) |
| rnd30 SAMPLE, human | 124/240 (0.517) | 107/240 (0.446) | 52/240 (0.217) |
| rnd30 SAMPLE, machine | 127/240 (0.529) | 86/240 (0.358) | 47/240 (0.196) |
| hold15 MODE, human | 106/120 (0.883) | 90/120 (0.750) | 38/120 (0.317) |
| hold15 MODE, machine | 104/120 (0.867) | 79/120 (0.658) | 42/120 (0.350) |

Per-seed rnd30 MODE, human vs machine: picked [8, 18, 19, 19, 18, 9, 11, 18] vs [14, 10, 22, 14, 19, 19, 15, 16] (Δ −0.037, p 0.643); contact [4, 16, 15, 13, 13, 7, 7, 16] vs [5, 6, 18, 5, 12, 8, 14, 13] (Δ +0.042, p 0.639); nested [2, 4, 9, 10, 2, 1, 6, 5] vs [4, 1, 7, 4, 9, 6, 9, 6] (Δ −0.029, p 0.621). SAMPLE: contact Δ +0.088 (p 0.232), nested Δ +0.021 (p 0.747). hold15 MODE: contact Δ +0.092 (p 0.328), nested Δ −0.033 (p 0.778).

**Verdict against the registrations.** P1 (both arms learn the pick): 5/8 seeds ≥ 0.5 on rnd30 MODE in EACH arm — met. **[CORRECTED 2026-09-08 — this heading is FALSE as written.** The ±0.10 margin **FAILS** on in-distribution `placed_v2`: human 0.083 v machine 0.242, Δ −0.158, with the machine arm above the 0.2 trigger that makes P2 apply. The predictions listed are met on the *random-uniform* cells only. Two further cautions: the `slide_success` entries are 'met' for a predicate the project **withdrew** under (p) and must never be counted among satisfied predictions; and the registered **equivalence procedure (TOST) was never run on any contrast**, so no sentence here rests on the test registered for it.**]** P2 (|Δ| < 0.10 at every stage either arm reaches ≥ 0.2): met at every cell (largest |Δ| 0.092, hold15 contact). P3 (nested < 0.2 for both): met (0.163 / 0.192). The wave-1 direction (human ahead at every stage, contact +0.117) did not survive four more seeds per arm: at 8 v 8 the signs alternate by stage and by cell, and no difference approaches significance. Conclusion of record: **no demonstration-source effect on the end-to-end task at 2e6 steps**, consistent with pick, place, contact and carrycontact. Per-seed spread is large (picked 8–19 of 30 within an arm), which is the seed-variance story of the whole project; the `placed` route note above stands (0–4 of 240 granted).

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

## 9. Re-scored cells (2026-09-08) — evaluation fixes, rebuilt banks, and two reproducibility findings

Registered as `PHASE_PLAN_2026-09-04.md` amendment (j) (+ (l) for `slide_success`). Full detail, every number and every
withdrawal: **`paper/EVAL_FIXES_2026-09-07.md`**. The sections above are NOT rewritten; this records what changed and where
the figures of record now live.

**Why anything was re-scored** (from `ADVERSARIAL_REVIEW_eval_env_2026-09-07.md`): place-scope evaluation drew bank entries
with replacement and silently substituted any entry that failed to restore (S1-3); `--dump-entries` wrote the policy's
grip in `[-1, 1]` while the restore reads physical `[0, 1]`, so 30 of 148 `polE_place` entries restored with the fingers
commanded more open than measured (S2-4); the full scope reported a training proxy as `nested` and a stale, unearnable
`placed` (S1-1, S1-2). Cells of record were never overwritten: re-scores live in `fresh_eval_<tag>_<mode>_v2/`.

**§2.y place, matched 39 (statistic of record, polE MODE, 8 v 8):** human **0.703 → 0.715**, machine **0.647 → 0.652**;
Δ +0.056 (p 0.227) → **+0.063 (p 0.112)**. Registered |Δ| < 0.10 **MET before and after — the conclusion stands, the
figures move by ~0.01.**
**polE SAMPLE** (the statistic the three-learner table uses): human 0.688 → 0.708, machine 0.674 → 0.652, so
Δ **+0.014 (p 0.743) → +0.056 (p 0.206)** — the gap quadruples because the arms move in opposite directions.
**holdE** (bank never rebuilt, so it isolates the pinning fix): MODE human 1.000 → 0.990, machine 0.990 → 0.962.
*Reading:* the small net movement in §2.y is two corrections nearly cancelling — restore failures now count as failures
(40 episodes per arm, symmetric) while the corrected-grip entries restore better — not evidence that either was harmless.

**§2.x, §3 and §4 (polEdDP symmetry, contact-after-release, carrycontact): re-scores RUNNING**, same treatment; results
will be appended to `EVAL_FIXES` §8 in the same old-versus-new form.

**§5.1 end-to-end: the published aggregates reproduce exactly.** The whole 30-episode cell re-run in order on the record's
own CPU model gives **0/30 differing episodes** and picked 19/30, contact 13/30, nested 10/30 — Δ +0.000 on all three. Two
constraints were found while establishing that, and both are properties of the evaluation, not of the environment:
1. **Long-horizon cells are CPU-class sensitive.** 36-core Xeon E5-2695 v4 (AVX2) nodes reproduce; 40/48/64/96-core
   AVX-512 nodes do not, at any thread pinning. Perfect separation: of the re-scored cells, 16/16 with a 36-core original
   moved and 0/176 with any other original did (`EVAL_FIXES` §7.2, §7.7). A cross-class re-run moved `contact` by
   **+0.100** — the width of the registered equivalence margin.
   *Consequence for §5.1:* the human arm had 2 of 8 seeds evaluated on that minority class and the machine arm none, so
   **hardware class is partially confounded with arm in the published 8 v 8**. All 64 end-to-end cells are being re-scored
   pinned to one class; those figures supersede the mixed-hardware ones.
2. **World-model cells are "mode" cells, not deterministic ones.** `Dreamer.act` samples the RSSM posterior latent even at
   `eval=True`, and no evaluator re-seeds between episodes, so a cell reproduces only when re-run as a whole sequence from
   process start. Pulling one episode out of a recorded sequence and comparing it is invalid.

**New columns now reported for the full scope** (measured on the reproduced §5.1 cell, `dHfull_all` s3 rnd30 MODE):
`nested_honest` **4/30 = 0.133** against the published `nested` (= the training proxy) **10/30 = 0.333** — a 2.5×
over-count; `placed_v2` **8/30 = 0.267** where the stale `placed` is 0/30, which **refutes by measurement** the
"policies carry the can in without releasing" reading in §5.1/`REVIEW_GUIDE` §2.7; `contact_push` 10/30 against
`contact` 13/30; and `slide_success` (amendment (l)) **2/30 = 0.067**, both grants earned in the held continuation.


## 5.2 End-to-end, Diffusion Policy — first large significant source effect, and it favours MACHINE (2026-09-09)

**Preview cells** (in-job evaluations, not the pinned pass), random-uniform starts, sampled, 8 v 8:

| stage | human | machine | Δ | p | MDE |
|---|---|---|---|---|---|
| picked | **0.237** | **0.496** | **−0.258** | **0.000** | 0.150 |
| contact | 0.029 | 0.100 | −0.071 | **0.004** | 0.046 |

This is the first end-to-end comparison in the project to clear its own detectable-effect threshold, and it is **large, significant, and in the machine arm's favour** — the opposite direction to the project's original hypothesis.

**It must not be read as a demonstration-source effect, for a specific reason.** The end-to-end machine set is **best-of-three per start** (Σ demonstrated reward 206 against 118; 16 demonstrated completions against 3) while the human set keeps **every** attempt including failures. Diffusion Policy is pure imitation — the learner most directly sensitive to the quality of what it is shown — so a machine-favouring result here is **exactly the signature that selection would produce**, independent of who generated the data. The same pattern appeared on the independent benchmark, where imitators lost most and the effect dissolved once the sets were matched.

**The de-confounded arm is the test**, and it is already running: the same machine tapes rebuilt as *first attempt per start*, the human protocol exactly. Until it reads out, the supported sentence is "the machine **set** trains a better end-to-end Diffusion Policy", never "machine **demonstrations** are better".

Status: RLPD's cells exist but its stage columns are unpopulated and need investigation; the world-model rows reproduce §5.1 exactly (0.500 v 0.537 picked, mode).

## 5.3 End-to-end RLPD, including the DE-CONFOUNDED arm (2026-09-09)

The RLPD cells were not missing — they live under `baselines/rl/checkpoints/e2e/`, not the outputs tree the table reads, which is why its stage columns were blank. Random-uniform starts, mode, `picked`:

| arm | n | picked |
|---|---|---|
| human (every attempt, incl. 10 no-picks) | 7 | **0.648** |
| machine, best-of-3 per start | 8 | **0.600** |
| machine, **first attempt per start** (de-confounded) | 4 | **0.641** |

| contrast | Δ | p |
|---|---|---|
| human v machine-best | +0.048 | 0.359 |
| **human v machine-FIRST — the de-confounded comparison** | **+0.006** | **0.864** |
| machine-best v machine-FIRST — selection alone | −0.041 | 0.624 |

**The de-confounded RLPD comparison is a null at 0.006.** Removing our best-of-three selection leaves the two sources indistinguishable, and selection itself did not help RLPD — if anything the curated set trained slightly *worse*.

**Set against Diffusion Policy on the same task, sets and starts, this is the sharpest contrast in the project:**

| learner | human | machine (best-of-3) | Δ | p |
|---|---|---|---|---|
| Diffusion Policy (imitation) | 0.237 | 0.496 | **−0.258** | **0.000** |
| RLPD (online RL) | 0.648 | 0.600 | +0.048 | 0.359 |

**The same demonstration sets produce a large machine advantage for the imitator and nothing for the online learner.** The most likely mechanism is composition rather than provenance: the human end-to-end set contains **10 tapes that never pick**, which an imitator copies and an online learner can simply out-explore. That reading is supported from two directions — selection is worth nothing to RLPD here, and on the independent benchmark the ordering was the same, with imitators losing most and the effect dissolving once sets were matched.

**Supported sentence:** *the composition of a demonstration set matters in proportion to how much a learner imitates it.* **Not supported:** any statement that machine demonstrations are better, or that source affects end-to-end performance.

Caveats: the de-confounded arm has 4 seeds (its remaining 4 are training), the human arm 7, and the Diffusion Policy rows are preview cells rather than the pinned pass.
