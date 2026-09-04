# Morning table — human vs machine demonstrations, like-to-like across learners (2026-09-04, draft written 23:55 on 09-03; WM rows regenerate)

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
| DP (diffusion policy, lerobot, state) | dH pruned **0.520** (LAST; selected 0.547) | **0.467** (LAST; selected 0.487) | +0.05 (LAST) / +0.06 (selected, perm p 0.041, Holm 0.082) | LAST-ckpt test NOT in any doc — compute from per-seed counts in the audit §2 | sampled | 10 vs 10 |
| RLPD (SAC + demos, state) | dHv2raw **0.600** (LAST; selected 0.671) | **0.517** (LAST; selected 0.521) | +0.08 (LAST) | not yet tested — per-seed counts in audit §2 | deterministic | 8 vs 8 |
| r2dreamer (world model, state, fixed recipe) | dHv2raw — *regenerate* | dDP — *regenerate* | — | exact perm, `morning_table.py` | MODE (like RLPD) and SAMPLE (like DP) | 8 vs 8 when the 23:05 expansion lands (≈02:45) |
| dv3 (world model) | not run on pick | not run on pick | — | — | — | reach proxy only: baseline {1/15,0/15}, fp32 {15/15,0/15}, EEF {11/15,3/15}, EEF+fp32 pending (≈01:15) |

r2dreamer n=2 pilot on the PRUNED pair (not the design of record, disclosed): dH 0.600|0.617 vs dDP 0.617|0.633
(sample|mode), hold-15 28/30 vs 29/30, alldemo-74 0.95 vs 0.95–0.97 — indistinguishable at n=2.

## 2. World-model rows (regenerate: `ssh pax 'cd $LAB/wm_fix_2026-09-03 && python3 morning_table.py runs'`)

<!-- paste morning_table.py output here in the morning: per-run cells hold15 / rnd30 / holdv2(66) / alldemo(74) × sample|mode, per-arm totals, permutation tests -->

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
