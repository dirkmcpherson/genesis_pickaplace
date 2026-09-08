# Morning table — human vs machine demonstrations, like-to-like across learners (2026-09-04, draft written 23:55 on 09-03; WM rows regenerate)

> **Terminology — "mode" vs "deterministic" (corrected 2026-09-07).** `--mode mode` selects the *mode* of the action distribution rather than sampling it. For RLPD this is genuinely deterministic: the policy returns `tanh(mean)` and repeats exactly. For the world models (r2dreamer, dv3) it is **not** run-to-run deterministic, because the agent samples its stochastic latent inside `act` and there is no per-episode reseed; a cell reproduces exactly only when the whole episode sequence is replayed with the same RNG stream (verified: 0/30 differences on the §5.1 sequence check). No comparison is biased by this — both arms are evaluated identically — but "deterministic" overstates it for world-model cells, and the word is used below in that looser sense.


**Design of record (James, 09-03 evening):** human arm = **authentic raw human** `dHv2raw` (N=66 tapes, 14,323–14,484 rows)
for RLPD / r2dreamer / (dv3); **pruned human** `dH` (N=58, 6,927–6,985 rows) for DP (DP degrades on raw — RESULTS §1);
machine arm = **the good machine set** `dDP` (frozen, pruned-matched, N=58, 7,476 rows) for every learner. A machine
arm trained from the raw base (`dDPv2`) is not competitive and is NOT the comparator. Corrected world
`gc_kp4_riser3_shelf6` everywhere. Statistic of record: **LAST checkpoint, rnd-30 random-IC picks** — the rnd-30 file is
byte-identical across `eval_ics.json` and `eval_ics_v2_w3.json` (sha c87c17074b5f) so it is the one cell every learner
shares. Per-seed counts, exact two-sided permutation test, n per arm as stated. Details, per-seed counts and file:line
citations for the DP and RLPD rows: `paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` (audit, 09-03 23:30).

## 1. The comparison (rnd-30, LAST checkpoint, corrected world)

| learner | human arm | machine arm (dDP) | Δ (human − machine) | test | eval actions | n |
|---|---|---|---|---|---|---|
| DP (diffusion policy, lerobot, state) | dH pruned **0.520** (LAST; selected 0.547) | **0.467** (LAST; selected 0.487) | **+0.053** (LAST) / +0.06 (selected, perm p 0.041, Holm 0.082) | LAST exact perm **p = 0.123** (computed 09-03 23:58 from the audit's per-seed counts) | sampled | 10 vs 10 |
| RLPD (SAC + demos, state) | dHv2raw **0.600** (LAST; selected 0.671) | **0.517** (LAST; selected 0.521) | **+0.083** (LAST) | LAST exact perm **p = 0.485** (computed 23:58; one dead seed in each arm: dHv2raw s65 2/30, dDP s45 0/30) | deterministic | 8 vs 8 |
| r2dreamer (world model, state, fixed recipe `bnormclamp1ent5`) | dHv2raw **0.617** MODE (148/240) / 0.613 SAMPLE (147/240) | **0.608** MODE (146/240) / 0.629 SAMPLE (151/240) | **+0.008** MODE / −0.017 SAMPLE | exact perm **p = 0.875** (MODE) / 0.641 (SAMPLE); registered prediction \|Δ\| < 0.10 MET | MODE (compare with RLPD) and SAMPLE (compare with DP) | 8 vs 8 (all 16 runs to 1M, fresh evals 04:00 on 09-04) |
| dv3 (world model) | not run on pick — no working configuration | not run on pick | — | — | — | reach proxy only (fresh eval, 15 hold ICs): baseline {1/15,0/15}, fp32 {15/15,0/15}, EEF {11/15,3/15}, EEF+fp32 {4/15,15/15}; every lever bimodal; excluded by the registered rule (PLAN §7 1e) |

r2dreamer n=2 pilot on the PRUNED pair (not the design of record, disclosed): dH 0.600|0.617 vs dDP 0.617|0.633
(sample|mode), hold-15 28/30 vs 29/30, alldemo-74 0.95 vs 0.95–0.97 — indistinguishable at n=2.

Context rows (same statistic): RLPD frozen pair dH-pruned 0.496 vs dDP 0.517, Δ −0.021, p 0.983 (the pre-registered A20 null — reproduces the audit's value, validating the test code);
RLPD raw pair dHv2raw 0.600 vs dDPv2 0.567, Δ +0.033, p 0.646 (not the design; dDPv2 is the raw-matched machine set).
Every RLPD contrast is carried by one dead seed per arm; with those excluded the arms sit at 0.68–0.70 vs 0.68–0.74.

## 2. World-model rows (regenerate: `ssh pax 'cd $LAB/wm_fix_2026-09-03 && python3 morning_table.py runs'`)

Generated 2026-09-04 04:45 by `morning_table.py` (cluster), all cells complete. Rows other than `bnormclamp1ent5` are the failing
recipe settings from the trainer-fix ladder (kept for the record; the `resume400k` row is a disclosed warm restart, not a run of record).

| run | setting | seed | hold15 S | hold15 M | rnd30 S | rnd30 M | holdv2(66) S | holdv2 M | alldemo(74) S | alldemo M |
|---|---|---|---|---|---|---|---|---|---|---|
| dDP | bnormclamp1ent5 | s0 | 15/15 | 15/15 | 20/30 | 20/30 | 66/66 | 66/66 | 71/74 | 72/74 |
| dDP | bnormclamp1ent5 | s1 | 14/15 | 14/15 | 17/30 | 18/30 | 66/66 | 66/66 | 70/74 | 72/74 |
| dDP | bnormclamp1ent5 | s2 | 15/15 | 15/15 | 19/30 | 18/30 | 65/66 | 66/66 | 71/74 | 72/74 |
| dDP | bnormclamp1ent5 | s3 | 15/15 | 15/15 | 19/30 | 18/30 | 66/66 | 65/66 | 72/74 | 72/74 |
| dDP | bnormclamp1ent5 | s4 | 15/15 | 15/15 | 20/30 | 16/30 | 65/66 | 64/66 | 69/74 | 71/74 |
| dDP | bnormclamp1ent5 | s5 | 15/15 | 15/15 | 20/30 | 19/30 | 65/66 | 65/66 | 68/74 | 71/74 |
| dDP | bnormclamp1ent5 | s6 | 15/15 | 15/15 | 16/30 | 19/30 | 66/66 | 66/66 | 71/74 | 72/74 |
| dDP | bnormclamp1ent5 | s7 | 15/15 | 15/15 | 20/30 | 18/30 | 62/66 | 66/66 | 70/74 | 71/74 |
| dDP | clamp1ent5 | s0 | 0/15 | 0/15 | 0/30 | 0/30 | — | — | 0/74 | 0/74 |
| dDP | clamp1ent5 | s1 | 0/15 | 0/15 | 0/30 | 0/30 | — | — | 0/74 | 0/74 |
| dH | bnormclamp1ent5 (resume400k, disclosed) | s1 | 14/15 | 15/15 | 20/30 | 19/30 | 65/66 | 66/66 | 72/74 | 71/74 |
| dH | bnormclamp1ent5 | s0 | 15/15 | 14/15 | 18/30 | 18/30 | 62/66 | 64/66 | 71/74 | 70/74 |
| dH | bnormclamp1ent5 | s2 | 13/15 | 14/15 | 18/30 | 19/30 | 61/66 | 63/66 | 70/74 | 70/74 |
| dH | clamp1 | s0 | 0/15 | — | 0/30 | — | — | — | 0/74 | 0/74 |
| dH | clamp1 | s1 | 0/15 | — | 0/30 | — | — | — | 0/74 | 0/74 |
| dH | clamp1ent5 | s0 | 0/15 | — | 1/30 | — | — | — | 0/74 | 0/74 |
| dH | clamp1ent5 | s1 | 14/15 | 13/15 | 17/30 | 17/30 | — | — | 62/74 | 68/74 |
| dHv2raw | bnormclamp1ent5 | s0 | 12/15 | 12/15 | 16/30 | 15/30 | 60/66 | 64/66 | 61/74 | 66/74 |
| dHv2raw | bnormclamp1ent5 | s1 | 15/15 | 15/15 | 20/30 | 19/30 | 66/66 | 66/66 | 71/74 | 72/74 |
| dHv2raw | bnormclamp1ent5 | s2 | 15/15 | 15/15 | 20/30 | 19/30 | 64/66 | 65/66 | 72/74 | 72/74 |
| dHv2raw | bnormclamp1ent5 | s3 | 13/15 | 14/15 | 18/30 | 19/30 | 65/66 | 62/66 | 64/74 | 71/74 |
| dHv2raw | bnormclamp1ent5 | s4 | 15/15 | 15/15 | 19/30 | 21/30 | 66/66 | 66/66 | 72/74 | 72/74 |
| dHv2raw | bnormclamp1ent5 | s5 | 14/15 | 15/15 | 16/30 | 17/30 | 63/66 | 64/66 | 68/74 | 72/74 |
| dHv2raw | bnormclamp1ent5 | s6 | 15/15 | 15/15 | 18/30 | 20/30 | 66/66 | 65/66 | 68/74 | 68/74 |
| dHv2raw | bnormclamp1ent5 | s7 | 15/15 | 15/15 | 20/30 | 18/30 | 63/66 | 63/66 | 69/74 | 70/74 |

Per-arm totals, recipe bnormclamp1ent5, LAST checkpoint (S = sampled actions, M = deterministic mode):
| arm | n | hold15 S | hold15 M | rnd30 S | rnd30 M | holdv2 S | holdv2 M | alldemo S | alldemo M |
|---|---|---|---|---|---|---|---|---|---|
| dH | 2 | 28/30 (0.933, n=2) | 28/30 (0.933, n=2) | 36/60 (0.600, n=2) | 37/60 (0.617, n=2) | 123/132 (0.932, n=2) | 127/132 (0.962, n=2) | 141/148 (0.953, n=2) | 140/148 (0.946, n=2) |
| dHv2raw | 8 | 114/120 (0.950, n=8) | 116/120 (0.967, n=8) | 147/240 (0.613, n=8) | 148/240 (0.617, n=8) | 513/528 (0.972, n=8) | 515/528 (0.975, n=8) | 545/592 (0.921, n=8) | 563/592 (0.951, n=8) |
| dDP | 8 | 119/120 (0.992, n=8) | 119/120 (0.992, n=8) | 151/240 (0.629, n=8) | 146/240 (0.608, n=8) | 521/528 (0.987, n=8) | 524/528 (0.992, n=8) | 562/592 (0.949, n=8) | 573/592 (0.968, n=8) |

- rnd30 MODE (statistic of record): dHv2raw [15, 19, 19, 19, 21, 17, 20, 18] vs dDP [20, 18, 18, 18, 16, 19, 19, 18] → Δ per-seed count +0.25, exact two-sided perm p = 0.875 (n=8 vs 8)
- rnd30 MODE (statistic of record): dH [18, 19] vs dDP [20, 18, 18, 18, 16, 19, 19, 18] → Δ per-seed count +0.25, exact two-sided perm p = 1.000 (n=2 vs 8)
- rnd30 SAMPLE: dHv2raw [16, 20, 20, 18, 19, 16, 18, 20] vs dDP [20, 17, 19, 19, 20, 20, 16, 20] → Δ per-seed count -0.50, exact two-sided perm p = 0.641 (n=8 vs 8)
- rnd30 SAMPLE: dH [18, 18] vs dDP [20, 17, 19, 19, 20, 20, 16, 20] → Δ per-seed count -0.88, exact two-sided perm p = 0.622 (n=2 vs 8)
- hold15 MODE: dHv2raw [12, 15, 15, 14, 15, 15, 15, 15] vs dDP [15, 14, 15, 15, 15, 15, 15, 15] → Δ per-seed count -0.38, exact two-sided perm p = 0.733 (n=8 vs 8)
- hold15 MODE: dH [14, 14] vs dDP [15, 14, 15, 15, 15, 15, 15, 15] → Δ per-seed count -0.88, exact two-sided perm p = 0.067 (n=2 vs 8)
- holdv2(66) MODE: dHv2raw [64, 66, 65, 62, 66, 64, 65, 63] vs dDP [66, 66, 66, 65, 64, 65, 66, 66] → Δ per-seed count -1.12, exact two-sided perm p = 0.106 (n=8 vs 8)
- holdv2(66) MODE: dH [64, 63] vs dDP [66, 66, 66, 65, 64, 65, 66, 66] → Δ per-seed count -2.00, exact two-sided perm p = 0.044 (n=2 vs 8)
- alldemo(74) MODE: dHv2raw [66, 72, 72, 71, 72, 72, 68, 70] vs dDP [72, 72, 72, 72, 71, 71, 72, 71] → Δ per-seed count -1.25, exact two-sided perm p = 0.238 (n=8 vs 8)
- alldemo(74) MODE: dH [70, 70] vs dDP [72, 72, 72, 72, 71, 71, 72, 71] → Δ per-seed count -1.62, exact two-sided perm p = 0.022 (n=2 vs 8)

Secondary cells, n=8 vs 8 (MODE): hold-15 116/120 vs 119/120 (p 0.733); holdv2-66 (training ICs of the raw set) 515/528 vs 524/528
(p 0.106); alldemo-74 563/592 vs 573/592 (p 0.238). The human arm's in-distribution cells trail the machine arm's by
1–2 percentage points; none is significant at n=8. Spread: worst human seed hold 12/15, rnd 15/30 (s0); worst machine seed rnd 16/30
(s4, MODE). No dead seeds in either arm (contrast RLPD: one dead seed per arm).

dv3 reach-proxy full-demo-set cells (deterministic, 74 uids; the cartesian env exposes 86 solved uids so its `all` set is 86): baseline
{0.10, 0.01}, fp32 {0.95, 0.00}, EEF {0.69, 0.24}, EEF+fp32 {0.21, 0.97} — the same bimodality as the hold-15 cells.

## 3. Asymmetries that remain after matching (disclosed, not absorbed)

1. **Budget units are incommensurable**: DP 100k gradient steps (offline); RLPD 100k decisions = 400k sim steps at UTD 10;
   r2dreamer 1M sim steps (= 250k decisions at action_repeat 4) of online interaction plus the demo prefill. No
   common currency exists without changing the learners; each is at its own recipe-of-record budget.
2. **Prefill counts toward the WM step budget**: dHv2raw prefill 57,936 vs dDP 29,904 ⇒ the raw-human WM runs get ≈3%
   fewer online steps (≈942k vs ≈970k) — against the human arm, if anything.
3. **Demo-set size differs by design**: 66 raw human tapes / ≈14.4k rows vs 58 machine tapes / 7.5k rows (1.9×). Same
   asymmetry inside RLPD's raw-vs-frozen-dDP row. DP's row is the pruned pair (58 vs 58), so DP is size-matched and
   the other two are not.
4. **Action parametrization at training**: DP learns absolute window-end joint targets, converted to the delta_joint MDP
   only at eval; RLPD and r2dreamer learn delta_joint natively (cap 0.025, leash 5 caps, repeat 4 — matched).
5. **Eval action selection**: RLPD deterministic; DP sampled (no deterministic re-score exists); r2dreamer reports both —
   compare r2dreamer MODE with RLPD and r2dreamer SAMPLE with DP.
6. **In-distribution cell differs**: DP-v2/RLPD-raw rows use `eval_ics_v2_w3.json` hold = the 66 training ICs; the WM
   rows add that exact cell (`holdv2`) plus hold-15 and the full 74-uid demo set. For the frozen dDP arm the 66 ICs are
   not all training ICs (dDP was matched to the pruned base) — same caveat applies to RLPD's dDP row.
7. **Checkpoint rule**: LAST for all rows above (DP's registered headline was selected-of-5; both are tabulated in the audit).
8. **Eval process**: DP/RLPD one fresh process per episode; WM one process per IC set with in-process resets. Same
   predicate (`picked`), horizon (1200 sim steps), 17-dim state content, world.
9. **Seeds**: DP 10, RLPD 8, r2dreamer 8 (pending) — RLPD's dH s45 selected-rnd cell is 18/29 (one episode missing; LAST unaffected).
10. **dv3**: no pick run under any working configuration; every dv3 number is the reach proxy with a 1-in-2 seed
    failure pattern. Excluded from the pick comparison as "no working configuration", not silently.

## 4. What is NOT claimed
The r2dreamer human-vs-machine contrast is reported, not interpreted as H4, until PREREG A37 (main session) registers
the fixed recipe; the WM-fix ladder's own registered predictions (PLAN §7) are scored in the log.

## 5. Additions 2026-09-04 evening (James's work order, PHASE_PLAN §6)
**Broader random-IC retest (300 placements, seed 1, same support box), LAST checkpoint, MODE:** dHv2raw 1444/2400 = **0.602**,
dDP 1484/2400 = **0.618** (8 seeds; one cell counts a deterministic simulator stall as a failure), dH-pruned pilot 367/600 = 0.612 — each within
0.02 of its rnd30 estimate (registered prediction: within 0.05, MET). SAMPLE: 0.597 / 0.619 / 0.593. Human-vs-machine on 300 placements:
Δ −0.017 (MODE), exact perm p 0.546, n = 8 v 8 — the null stands.

**ALL-DATA human arm `dHv2all`** (106 tapes: 66 successes + 24 non-picking raw-run recordings + 16 real-fail demos, 73 rewarded),
r2dreamer, 8 seeds, LAST, all cells complete: rnd30 **0.588 MODE / 0.571 SAMPLE** vs dHv2raw 0.617 / 0.613 (Δ −0.03 / −0.04, p 0.58 / 0.32);
rnd300 **0.593 / 0.574** vs 0.602 / 0.597 (p 0.79 / 0.48); hold15 0.917 vs 0.967 (p 0.32); holdv2-66 0.939 vs 0.975 (p 0.18); alldemo-74 0.921
vs 0.951 (p 0.22). Against the machine arm the in-distribution deficit is significant (holdv2 p 0.035, alldemo p 0.044).
**Registered prediction (WM gains ≥ 0.05 from failures) NOT met** — no gain on random starts, a small in-distribution cost.
RLPD dHv2all (8/8): LAST rnd 18, 21, 1, 18, 19, 18, 19, 19 = **0.554** vs dHv2raw 0.600 (Δ −0.046, p 0.567; one dead seed per arm; 0.629 vs 0.676 without them).
