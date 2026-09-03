# RESULTS — for the plane writing session (living; post-blackout update 2026-08-31)

**Read with:** `CONFOUNDS.md` (living confound ledger — every claim must map to a row), `METHODS_draft_2026-08-28.md` (UPDATE 08-29 blocks), `PREREG_final_round_robin_2026-08-23.md` A9–A19,
`ADVERSARIAL_AUDIT_2026-08-29.md` (what a reviewer will say), `WM_CANDIDATES_2026-08-29.md`.
Statistics: `analysis/stats.py` (seed is the unit; Welch + exact permutation; min attainable p quoted).
Readouts: **sel** = 15 selection ICs (ceiling 14/15, training ICs) — selection only, never a headline; **hold** = 15 ICs
(⊂ training ICs, A8); **rnd** = 30 random ICs. Headline = selected checkpoint on hold+rnd (PREREG §5/A7).
Worlds: **old** = matched_v2 (RLPD, r2dreamer, dv3 to date); **corrected** = matched_w3 `gc_kp4_riser3_shelf6` (DP n=10;
r2dreamer gate s80–83). One world per table (A19). Version stamp at the bottom.

---
## 1. DP (lerobot Diffusion Policy, state-only) — HEALTHY, the one settled learner

| world | arm | n | hold | rnd |
|---|---|---|---|---|
| corrected (seeds 20–29) | dH | 10 | 0.887 | **0.547** |
| corrected | dDP | 10 | 0.873 | **0.487** |

- (1a) at matched N=56: hold flat (+0.013); rnd human edge +0.06, Welch CI [+0.006, +0.114], exact-perm p 0.041 —
  ONE unadjusted comparison, top-up not pre-registered as this test (AUDIT_results §4). Old world n=5 replicate: no spread.
- (1b) not applicable to DP by design (no IL on failures, PAPER_PLAN §3). (2) not applicable (no reward).
- 35 further DP jobs are user-held (replicates); not needed for the claim above.
- **Unpruned-human control (user 08-30):** base dH is leading-idle-pruned (29.6 % of frames); density controls removing ALL idle
  decisions (dHallpruned_1e3/1e2) had no effect; the fully UNPRUNED human set (`dHunpruned`, N=52 — 9 dH ICs lack an unpruned
  success — rows 8005 vs 7002) trains in the old world. **COMPLETE n=3 (08-31): NO detectable penalty.** Selected rnd 13, 17, 16/30
  (s32/33/34; hold 13, 12, 13/15) vs pruned dH s10-14 rnd 15-19/30: diff -0.056, CI [-0.194, +0.083], Welch p 0.31 /
  perm 0.36. The first seed (13/30) had suggested a drop; seeds 33/34 land inside the pruned band. Honest statement for
  the paper: raw (unpruned) human tapes cost DP nothing detectable at this n — the pruning step is a training-efficiency
  choice (29.6 % fewer frames), not a load-bearing data-cleaning step, and "DP reacts the same to both sources" survives
  the control. (s30/s31 died on a bad GPU and were replaced by fresh seeds s33/s34 per A9.)

- **v2 full-pool DP (A25/A29; FINAL n=8v8 per world, 09-02; names of record: dH = raw dHv2raw, dHpruned = dHv2):**
  raw human tapes DEGRADE DP — rnd-30: old world 0.350 (raw) vs 0.512 (pruned), pruned−raw **+0.16 [+0.07,+0.26],
  perm p 0.006**; corrected world 0.237 vs 0.508, **+0.27 [+0.19,+0.35], perm p <0.001**; in-dist hold +0.21 / +0.15
  (both p<0.001). **A29's registered prediction (pruned ≥ raw + 0.15) MET in both worlds** (fig11). Dataset difference
  (fig12): raw pools carry 2.0–2.1× the rows; ~70 % of the extra rows come from the 8 raw-only trials (16 set slots
  across worlds; 237–1244 rows each, all ≥ picked) that were formerly over-horizon; raw idle-decision fraction 0.25–0.27
  vs pruned 0.19–0.20 (arm columns); EEF coverage on the 60 common ICs is IDENTICAL (578 ≡ 577 voxels) — pruning
  removes time, not space, and the raw pool's extra coverage is entirely those 8 long trials. The horizon-capped
  dHunpruned control (n=3, no penalty) could not see this. DP's human arm of record for v2 is dHpruned (A31); the raw
  leg stays as this disclosed within-source result. RLPD on raw dH: 16 runs trained, evaluation being re-run from
  archived checkpoints (the in-job eval was killed by a cache purge — my error, logged 09-02).

**v2 pool, corrected world (09-03 morning, wave v2fullPw3, fixed 100k, selected ckpt): dHpruned (dHv2, N=60) rnd 17,13,16,19,13,15,13,16 = 0.508 (n=8) vs
dDPpruned (dDPv2p, N=60) rnd 15,14,16,16,14,13 = 0.489 (n=6; s56/57 pending) → Δ +0.02. Hold: 62–65/66 vs 58–66/66. DP indifferent to
source on the full pool as on the frozen block (+0.06). Self-distillation framing applies (CONFOUNDS row 26).**

## 2. RLPD (SB3 SAC subclass, E=10 Z=2 UTD 10, 50/50 demo batches, LayerNorm critics)

**Recipe restart (A17):** every γ = 0.998 run is diagnostic-only — 69/74 diverged (critic loss 10²–10⁵). At the published
γ = 0.99 the learner is healthy on human demos. All numbers below are γ 0.99, old world, sparse unless stated, seeds ≥ 30.

| arm | seed | max critic loss | watchdog trips | selected: hold / rnd | final: hold / rnd |
|---|---|---|---|---|---|
| dH | 30 | 0.018 | 0 | 14/15 / 20/30 | (12 h limit hit during final eval) |
| dH | 31 | 0.016 | 0 | 14/15 / 21/30 | 14/15 / 21/30 |
| dH | 32 | 0.027 | 0 | 14/15 / 19/30 | 11/15 / 19/30 |
| dDP | 30 | 22.1 | 4 | 10/15 / 14/30 | 10/15 / 14/30 |
| dDP | 31 | 83.6 | 5 | 13/15 / 12/30 | 13/15 / 12/30 |
| dDP | 32 | 0.043 | 0 | 14/15 / 20/30 | 10/15 / — |
| dH (UTD 5) | 30–32 | ≤0.045 | 0 | 14/15 / 20, 26, 21 | final hold 14, 8, 14 |
| dDP (UTD 5) | 30–32 | 530 / 65 / 0.037 | 8 / 6 / 0 | 8/15 14/30; 8/15 7/30; 13/15 14/30 | |
| γ 0.998 UTD 5 (control) | dH 30,31; dDP 30 | 292–1.5e5 | 6–9 | hold 0–2/15 | |

- Reading so far (n=3, NOT a claim): human demos → stable critic 3/3; DP demos → 2/3 diverge with partial performance.
  Demo sets are indistinguishable on every tape statistic (`tape_census_2026-08-29.txt`), so this is not a format artefact.
- **Pre-registered statistic (A16):** divergence rate (Fisher) + LAST-checkpoint rnd-30 (Welch/perm). Predictions: dH 0–1/8
  vs dDP ≥ 3/8; LAST rnd dH > dDP by ≥ 0.10. Matrix in flight: dH/dDP sparse s33–37, dHHfails/dDPfails s30–37,
  dH/dDP dense s30–35, dHsucc_dup/dDPsucc_dup s30–33 (row-matched controls). First s33–37 headlines: see §2.1 (updated
  at each check). Early: dH s33 2 mild trips (CL 0.16), s36 CL 2.9 (crosses the A16 threshold) — dH is not immune.
- Watchdog trip count is NOT valid on dense arms (shaping lifts Q); use max critic loss + final≈selected there.

### 2.1 g99 matrix readouts (appended as they land; sparse, old world)
| arm | seed | max CL | trips | selected hold / rnd | final hold / rnd |
|---|---|---|---|---|---|
| dH | 30 | 0.018 | 0 | 14/15 / 20/30 | (12 h limit hit during final eval) |
| dH | 31 | 0.016 | 0 | 14/15 / 21/30 | 14/15 / 21/30 |
| dH | 32 | 0.027 | 0 | 14/15 / 19/30 | 11/15 / 19/30 |
| dH | 33 | 0.157 | 2 | 11/15 / 19/30 | 14/15 / 21/30 |
| dH | 34 | 0.138 | 2 | 15/15 / 17/30 | 15/15 / 17/30 |
| dH | 35 | 0.017 | 0 | 14/15 / 20/30 | 14/15 / 20/30 |
| dH | 36 | **4.42** | 4 | 14/15 / 25/30 | 14/15 / 25/30 |
| dH | 37 | 0.019 | 1 | 14/15 / 24/30 | 14/15 / 24/30 |
| dDP | 30 | 22.1 | 4 | 10/15 / 14/30 | 10/15 / 14/30 |
| dDP | 31 | 83.6 | 5 | 13/15 / 12/30 | 13/15 / 12/30 |
| dDP | 32 | 0.043 | 0 | 14/15 / 20/30 | 10/15 / — |
| dDP | 33 | 39.2 | 6 | 12/15 / 17/30 | 12/15 / 17/30 |
| dDP | 34 | 0.03 | 0 | 12/15 / 16/30 | 12/15 / 16/30 |
| dDP | 35 | 5.28 | 4 | 12/15 / 13/30 | 12/15 / 13/30 |
| dDP | 36 | 18.5 | 7 | 11/15 / 17/30 | 11/15 / 17/30 |
| dDP | 37 | 4.56 | 4 | 13/15 / 18/30 | 13/15 / 15/30 |
**HEADLINE (1a) for RLPD — pre-registered A16, old world, sparse, γ 0.99, seeds 30–37 (COMPLETE n=8 v 8; dH s30 has no final-ckpt readout, dDP s32 none → LAST n=7 v 7):**
| statistic | dH | dDP | diff | 95 % CI | Welch p | exact perm p |
|---|---|---|---|---|---|---|
| selected ckpt, rnd-30 | 0.688 (n=8) | 0.529 (n=8) | +0.158 | [+0.062, +0.254] | 0.003 | 0.004 |
| **LAST ckpt, rnd-30 (A16 statistic)** | 0.700 (n=7) | 0.495 (n=7) | **+0.205** | [+0.111, +0.299] | 0.001 | 0.002 |
| selected ckpt, hold-15 | 0.917 (n=8) | 0.808 (n=8) | +0.108 | [+0.022, +0.195] | 0.018 | 0.032 |
| divergence (max critic loss ≥ 1) | 1/8 | 6/8 | — | — | Fisher **0.041** | |
A16 predictions ("dH 0–1/8 vs dDP ≥ 3/8"; "LAST rnd dH > dDP by ≥ 0.10") both met. Relative loss on random ICs 25–29 %.
Tape census rules out format artefacts (§2 of METHODS UPDATE); the remaining mechanism candidate is demo-state coverage
(DP-teacher trajectories are narrower in state space than human ones — the same argument as the exposure/coverage story).
Per-seed: dH selected hold 14,14,14,11,15,14,14,14 / rnd 20,21,19,19,17,20,25,24; dDP hold 10,13,14,12,12,12,11,13 / rnd
14,12,20,17,16,13,17,18. LAST rnd where a final readout exists: dH 21,19,21,17,20,25,24; dDP 14,12,17,16,13,17,15.
One dH seed (s36) crossed the critic-loss threshold yet scored 14/15, 25/30 — a divergence event is not a performance
failure; the performance statistic is the one to lead with, the divergence rate is the mechanism.
Row-matched control dHsucc_dup (56 successes + 8 longest own successes duplicated), sparse, first 3 seeds (01:50 08-30):
s30 hold 13 rnd 17, s31 15/23, s32 14/23 — all clean (max CL ≤ 0.08); i.e. adding ~1.7k rows of duplicated human successes
leaves dH at its level (rnd 0.70 vs 0.69). Fails arms (1b), dup controls, dense arms → §2.2 (landed during the blackout).

### 2.2 Fails arms (1b), dup controls, dense (landed 08-30/31; γ 0.99, old world)
**(1b) fails arms — COMPLETE n=8 v 8** (dHHfails = 56 human successes + 8 human fail tapes; dDPfails = 56 DP successes +
8 DP fail tapes; fail rows 2145 ≈ 23–25 % of rows — METHODS §4.6; an earlier draft of this line said 16, corrected 08-31):
| statistic | dHHfails | dDPfails | diff | 95 % CI | Welch p | exact perm p |
|---|---|---|---|---|---|---|
| **LAST ckpt, rnd-30 (A16 statistic)** | **0.746** (n=8) | 0.537 (n=8) | **+0.208** | [+0.048, +0.368] | 0.017 | <0.001 |
| divergence (max CL ≥ 1) | 1/8 (s35, 50.5) | 1/8 (s32, 83.3) | | | | |
Per-seed LAST rnd: dHHfails 23,23,20,21,21,25,22,24; dDPfails 19,15,3,18,15,20,20,19.
- **The human>DP gap survives adding failure data** — same size as the success-only gap (+0.21 both).
- **Adding own-failure tapes largely CURES dDP's divergence** (6/8 diverged on successes alone → 1/8 with fails;
  max CL drops to ≤ 0.1 on 7/8 seeds). Mechanism-consistent: fails widen state coverage where the critic extrapolated.
- **Fails effect vs row-matched dup controls (A17 reading rule — same direction on both sources: MET, both positive):**
  dH: +fails 0.746 vs +succ_dup 0.675 (n 8 v 4, +0.07, perm p 0.20 — ns); dDP: +fails 0.537 vs +succ_dup 0.383
  (n 8 v 4, +0.15, perm p 0.26 — ns). Directionally consistent, individually not significant — write as "failure tapes
  help RLPD if anything, and specifically de-diverge the DP-demo arm; the effect on final score is within noise of a
  row-count control."
- Dup controls TOP-UP COMPLETE n=8 v 8 (s34-37 added 08-31 pm; LAST rnd): dHsucc_dup 17,23,23,18,19,25,16,24
  (**0.688**); dDPsucc_dup 19,13,5,9,15,17,15,21 (**0.475**). Source gap inside the dup controls:
  **+0.213, CI [+0.051, +0.374], Welch p 0.014 / perm 0.013** — the old-world gap now replicates at the SAME
  size (+0.21) in three independent preparations (sparse, +fails, +dup), all n=8v8, all significant; it is not
  a row-count artefact. Divergence in the dup controls at n=8: dH 2/8 (s34 3.4, s35 9.0) vs dDP 5/8 — the sparse
  asymmetry (1/8 v 6/8) echoes, softer. Fails-vs-dup at full n: dH +0.058 (p 0.26), dDP +0.062 (p 0.53) — direction
  consistent (A17 rule still met), magnitude within noise; the fails claim remains the DE-DIVERGENCE, not a score lift
  (fails 1/8+1/8 diverged vs dup 2/8+5/8 — failure tapes stabilise BOTH sources beyond what duplication does).
- **(2) dense arms — COMPLETE n=6 v 6** (4 of the original 12 died on one node's bad GPU; refills = fresh seeds per A9).
  dH-dense selected rnd 8,6,17,14,17,6 (mean 0.378); dDP-dense 9,7,12,4,11,11 (mean 0.300). Read: potential-shaped
  dense reward HURTS both sources relative to sparse (dH 0.38 vs 0.69; dDP 0.30 vs 0.50) and does not close the source
  gap. dDP-dense still shows critic instability (orig s33 max CL 428; refill s36 final 0/15, s38 selected ckpt_020 with
  final 3/30 — final≠selected, the dense-arm health read). Diagnostic answer to question (2): reward density does not
  rescue machine demos; if anything sparse is the better recipe for both. Per A16 the trip-count watchdog is invalid on
  dense — max CL and final≈selected are the health reads there.

### 2.3 A20 corrected-world replication (scored 08-31 ~17:00; γ 0.99 sparse, matched_w3, seeds 40–47) — **REGISTERED PREDICTIONS FAIL; the RLPD source effect is WORLD-DEPENDENT**
| statistic | dH | dDP | diff | 95 % CI | Welch p | exact perm p |
|---|---|---|---|---|---|---|
| **LAST ckpt, rnd-30 (A20 statistic)** | 0.496 (n=8) | 0.517 (n=8) | −0.021 | [−0.301, +0.259] | 0.875 | 0.983 |
| selected ckpt, rnd-30 | 0.565 (n=8) | 0.521 (n=8) | +0.044 | [−0.157, +0.245] | 0.640 | 0.665 |
| divergence (max CL ≥ 1) | 3/8 (117, 20, 50) | 3/8 (15, 1.04, 29) | — | — | Fisher 1.0 | |
Per-seed LAST rnd: dH 19,1,19,19,1,21,20,19 (s41/s44 selected EARLY ckpts then collapsed to 0–1 by ckpt_100);
dDP 18,19,17,19,20,0,20,11 (s45 dead throughout). dH s42 diverged by CL (20) yet scored 19/30 — the
divergence↔performance decoupling seen in the old world (s36) repeats. A20's registered predictions
("dH > dDP by ≥ 0.10"; "divergence dH ≤ 2/8 vs dDP ≥ 3/8") BOTH FAIL: the corrected world shows **no source
effect on any statistic** (dH ≈ dDP ≈ 0.50–0.52, symmetric 3/8 divergence). The old-world (1a) result stands
as pre-registered and met; this replication stands as pre-registered and failed; the claim that survives both
is: **the RLPD demo-source effect is world-dependent.** The mechanism is not mysterious — it was measured on
the tapes (fig6/fig7, committed after the 9/16 interim but before this full-n scoring): the corrected world's
stiff, gravity-compensated arm produces machine demos that are nearly human-broad (EEF coverage deficit −21 %
old → −5 % corrected; the w3 teacher's tapes sit 2× closer to the human tapes), and the corrected world
retains ~20 % more of the real command signal — where machine-demo coverage matches human, the value learner
stops caring which source it came from. DP calibrates the worlds (unchanged across them, §1), so this is an
RLPD × world × source interaction, not a harder world. Cross-check in flight: r2dreamer n=8 in the SAME
corrected world (~09-01) — its n=4 direction (human >> DP) is the opposite pattern in the same world, so the
WM readout decides whether world-dependence is learner-general or RLPD-specific.

### 2.4 A33 readout (09-02 night): RLPD with DENSE (potential-shaped) reward COLLAPSES in the corrected world, both arms
Wave g99w3dense, frozen matched_w3 dH (pruned, N=58) vs dDP (N=58), s60-67, --demo-shaping on (demo half relabeled with
the same potential, φ(terminal)=0), 100k decisions, selection on sel-15 among 5 ckpts, confirm on hold-15 + rnd-30; all
16 headlines present (rlpd_3163620-36.out, SWEEP-HEADLINE). Selected-ckpt rnd (/30): **dH 4,4,4,1,1,8,5,2 = 29/240 (0.12)**;
**dDP 0,1,0,11,7,1,1,0 = 21/240 (0.09)**. LAST (ckpt_100) rnd: dH 4,2,4,1,1,0,0,1 (0.05); dDP 0,1,0,2,1,0,1,0 (0.02).
Selected hold (/15): dH 5,2,4,0,0,9,1,2; dDP 0,0,0,7,5,3,5,0. Compare SPARSE RLPD in the same world: A20 dH/dDP ≈ 0.6
(selected rnd) and the raw-human v2 arm 0.68 (n=7, §2.5). **Verdict: dense reward is not a "more learnable" variant here —
the shaping DESTROYS RLPD (≈ 0.6 → ≈ 0.1 rnd) in both arms; source contrast under dense = null-by-death (Δ +0.03).**
Implications: (i) CONFOUNDS row 3 resolves in the unexpected direction: reward density alone flips a learner from working
to dead, so no cross-learner row that differs in density (WM dense vs RLPD sparse) is interpretable; (ii) the WM's dense
shaping is now suspect on its own, independently of the clamp (rows 13/16/17) — the SPARSE-RS1 A32 pair is the only WM
configuration with a working-learner precedent; (iii) the 08-19 old-world dense RLPD legs (RESULTS §2.2) were already
weak; this is the corrected-world confirmation. Not yet checked: critic-loss divergence rows for these 16 (row 24 statistic).

### 2.5 v2 raw-human RLPD arm, corrected world (re-evaluated from archived ckpts, rlpd_select_confirm.sh; n=7 of 8, s65 rerunning)
g99v2fullw3 dHv2raw (N=66, uncapped raw tapes), s60-67, selected=ckpt_100=final for all 7: hold 66,66,66,65,66,66,66 (/66);
rnd 23,20,18,20,18,24,19 (/30); s65 (rerun, ckpt_080 selected) hold 66/66 rnd 19/30 but LAST collapsed (1/66, 2/30) → **n=8: selected rnd 161/240 = 0.671, 8/8 alive at selection, 7/8 at LAST**. Machine arm (dDPv2 N=66, A31, s50-57)
training since 09-02 14:20. Note the row-count/horizon asymmetries inside this pair (CONFOUNDS rows 8, 19).


## 3. r2dreamer (in-house DreamerV3; replay critic loss = official DV3 v2 component; return clamp = deviation)

- **Old world, dense, 3M (historic block, seeds 50–53, 100–103, 80–87 dR2D):** selected checkpoints reach 13–15/15 hold on
  dH and dR2D; **sparse never learns** (0/7,700 episodes). LAST-checkpoint hold re-scores: dH s50–53 = 0, 15, 0, 7;
  dH s101/103 = 0, 0; dDP s100/102/103 = 1, 10, 14; dR2D s80–83/86/87 = 15, 0, 13, 0, 13, 11; dR2DDPfails = 5, 5, 0, 1, 15, 4
  → final policies are **bistable** (0 or ≥ 10 of 15 in 16/21); BEST-of-K on sel does the work. Disclose.
- **Corrected world gate (dH/dDP s80–83, dense, 3M, packed, 08-29): IGNITES.** In-job sel max per seed: dH 0.40, 0.93, 0.93,
  0.93 (nonzero ckpts 3/17/13/14 of 28); dDP 0.00, 0.87, 0.13, 0.93 (0/17/4/8). Same adjacent-checkpoint bistability.
  Protocol re-scores (BEST hold/rnd, LAST hold; `n12_rescore/*_W3_*`) → §3.1.
- **Standard arm (clamp off, repval kept) NOCLAMP dH/dDP s122/123 — VALUES RUN AWAY; competence is transient.** Values
  oscillate far past the 100 maximum across snapshots (dH s122: 806 → 190; dDP s123: 417 → 75 — vs clamped runs pinned
  96–98). Finals (in-job sel eval of the BEST-of-K ckpt): dH s122 picked 0.87 (best ≈ 0.52M), s123 0.47; dDP s122 0.60,
  s123 0.13 (best ckpt very early). Honest statement: without the clamp the world model still reaches transient
  competence that BEST-of-K can catch, but the value function is unbounded and the endpoint unreliable. **Re-scores
  (08-31, protocol ICs) confirm: LAST ckpt is DEAD on all four runs** (hold 0,0,1,0 of 15; rnd 0,0,1,0 of 30) **and even
  BEST is weak** (hold 7,0,7,1 of 15; rnd 9,3,10,2 of 30 — vs clamped W3 dH BEST hold 10–15). So "runs away" cashes out
  as: transient competence, unbounded values, endpoint zero. The clamp remains the load-bearing deviation; neither torch
  port is stable under the published recipe on this task. Write-up: the WM arm = DreamerV3-with-return-clamp, disclosed;
  the standard-recipe negatives (dv3-std ×3, r2d-NOCLAMP ×4) reported as such.
- N15 (dR2D + DP-fails, mixed source): direction-only (n 4 v 3, CI spans 0, min p 0.057); mechanism cell, not a matrix cell.

### 3.1 W3 protocol re-scores (corrected world, dense, 3M; BEST = highest in-job sel among archived ckpts)
| arm | seed | sel max | BEST hold | BEST rnd | LAST hold |
|---|---|---|---|---|---|
| dH | 80 | 0.40 | 10/15 | 20/30 | 4/15 |
| dH | 81 | 0.93 | 15/15 | 25/30 | 13/15 |
| dH | 82 | 0.93 | 11/15 | 22/30 | 15/15 |
| dH | 83 | 0.93 | 10/15 | 16/30 | 0/15 |
| dDP | 80 | 0.00 | 1/15 | 1/30 | 1/15 |
| dDP | 81 | 0.87 | 2/15 | 2/30 | 5/15 |
| dDP | 82 | 0.13 | 2/15 | 3/30 | 1/15 |
| dDP | 83 | 0.93 | 15/15 | 28/30 | 13/15 |
Seed-level (analysis/stats.py, n=4 v 4): BEST hold dH 0.767 vs dDP 0.333, diff +0.43, CI [−0.24, +1.11], perm p 0.143
(min attainable 0.029); BEST rnd 0.692 vs 0.283, diff +0.41, CI [−0.26, +1.07], p 0.143; LAST hold 0.533 vs 0.333, p 0.63.
Reading: in the corrected world r2dreamer learns from human demos on 4/4 seeds (hold ≥ 10/15) and from DP demos on 1/4;
dDP s81's sel 0.87 collapsed to 2/15 on hold (sel-selected checkpoint was a fluke — another reason sel is never a headline).
Direction: human > DP for the world model; NOT significant at n=4 (min p 0.029). Seeds 84–87 per source: the dH pack
(3025535) hit OUT_OF_MEMORY at ~12 h / ~1M steps (4 packed runs × ~33 GB > 120 GB) and was resubmitted 08-31 as two
2-seed packs at 90 GB (3085547/48; logdirs resume from latest.pt, DUPLICATE_OK reason logged); the dDP pack (3025536)
is past 13 h and healthy. Done ~09-01, then the same BEST pinning + re-score protocol takes the table to n=8 v 8
(min p 0.0002, ~80 % power at the observed gap).

**✅ 09-03 CORRECTED NUMBERS OF RECORD (WM_BEST5_RESCORE §5; corrected world, fresh process, BEST-of-5 fraction ckpts by in-job sel, sampled, 192/192 cells present):**
BEST-of-5 rnd per seed (/30): **dH 1,22,17,20,0,11,16,7 = 0.392** vs **dDP 2,19,2,8,8,10,7,7 = 0.262** — diff **+0.13, exact perm p 0.30**;
BEST-of-5 hold: dH 0.550 vs dDP 0.250; ignition (BEST-of-5 hold ≥ 8/15): **dH 4/8 vs dDP 1/8, Fisher p 0.28**. LAST (F100) rnd: dH 0.275 vs dDP 0.146
(alive at LAST: dH s81/s83/s86, dDP s81). For the record, the same checkpoints re-scored BEST-of-K in the corrected world give
dH 0.558 vs dDP 0.392 rnd (+0.17) — the K-inflation and the world error partly cancelled in the old table. **Verdict unchanged
in direction, weaker in size, and still not significant: directional WM preference for human demos under selection; no
endpoint claim.** The paragraph below is the superseded base-world/BEST-of-K table, kept for the audit trail only.

**⚠ superseded 09-02 night — the table below was INVALID AS AN ESTIMATE (WM_BEST5_RESCORE §0): every r2d re-score of record was run WITHOUT `R2D_SIM_VARIANT`, i.e. policies trained in the corrected world were scored in the BASE world (0 of 2289 n12_rescore logs carry the [sim-variant] line); AND the seeds' budgets are unmatched (3M / ≈3.9M / 6M after OOM warm restarts, s84-87). A fresh-process BEST-of-5 re-score in the correct world is running (cluster/r2d_best5_submit.sh). Do not quote any number in this paragraph until it lands.**

**n=8v8 FINAL (09-01, all 32 re-scores in): BEST rnd dH 20,25,22,16,1,21,11,17 (0.554) vs dDP 1,2,3,28,22,1,12,5
(0.308) — diff +0.246, CI [−0.084, +0.576], Welch p 0.132 / perm 0.133; Bayes (hierarchical Beta-Binomial,
analysis/bayes_triple_2026-09-01.py) Δ +0.20, CrI [−0.05, +0.42], P(Δ>0) 0.946. Ignition (BEST hold ≥ 8/15):
dH **6/8** vs dDP 3/8, Fisher p 0.315, P(ign_H > ign_M) 0.926, Δignition +0.30 CrI [−0.12, +0.67]. BEST hold per-seed: dH 10,15,11,10,1,11,3,13;
dDP 1,2,2,15,13,0,9,6. **CORRECTION 09-02 (AUDIT_approach / PREREG A35):** this passage previously said 7/8 v 3/8 (Fisher 0.119, P 0.974); that count is what BEST rnd ≥ 8/30 gives (only s84 fails), not the registered BEST-hold criterion, under which dH s84 (hold 1) AND s86 (hold 3) fail. The registered criterion is the one of record; 7/8 v 3/8 is withdrawn everywhere. Honest verdict: consistently DIRECTIONAL on every statistic, significant on none at
n=8 — seed bimodality in both arms eats the power. A27 (n=12v12, ignition pre-registered co-primary) is the
confirmatory readout (~09-03). An interim n=7v8 reading (before s84's dead-seed score landed) had CrI
[+0.07, +0.49] — superseded; do not quote it.**

### 3.2 CORRECTION (09-02): the return clamp is mis-set; the NOCLAMP "runaway" verdict is withdrawn
`return_clamp: 100` was set to reward_scale×pick and ignores the training-only potential shaping. Replayed episode
returns in the corrected-world runs: median ≈ 560–1030, p90 ≈ 1290–2460, max ≈ 4900 (s81/s83). The clamped
critic's value_replay_max sits at 100.2 (median) — the target is clamped on essentially every state, the critic
learns "≈100 everywhere", advantages become noise, actor entropy collapses (dDP −6.1 by 1M; dH −2.5→−2.9; fig13),
and checkpoints go bistable in BOTH arms. NOCLAMP's values (median 150–380, p90 350–900, max ≈ 1000–1250) are of the
same order as the true shaped returns — §3's "values 115–825 against a 100 maximum = runaway" was judged against
the wrong ceiling and is WITHDRAWN as a claim; NOCLAMP still had dead endpoints (LAST hold ≤ 1/15), so neither
variant is healthy, and a correctly scaled critic target has not yet been run. The port already has DV3's
percentile return normalization (ReturnEMA), so this is a scale error, not missing machinery. The dv3-dense
"value past attainable max" readings in §4 were re-checked against dv3's OWN replayed returns (09-02): dv3's shaping
scale is different — its train_return is ≈20–35 (max 60) while value_mean sits at 300–750 — so the dv3 runaway
verdict STANDS (10–20× over its attainable return); only the r2dreamer NOCLAMP verdict is withdrawn. Two ports,
two different pathologies: dv3 = genuine critic runaway; r2dreamer = mis-set clamp saturation. Pilots registered as A32 (C2000, RS1, SPARSE-RS1; dH first, corrected world).
What survives: the ignition asymmetry (human 6/8 vs machine 3/8 under identical mis-scaling; corrected from 7/8, see §3.1) is a
within-block contrast, directional only (Fisher 0.315), and AUDIT_approach f11 argues ignition under a saturated critic measures demo-dynamics coverage rather than value learning; the endpoint (LAST) claims are suspended pending A32.

### 3.3 A32 readout (09-03 morning): critic-target-scale fixes do NOT keep WM endpoints alive; sparse never ignites
All 10 dH pilots complete (3M, corrected world, in-job sel = 29 snapshots each, LAST = final snapshot):
- **C2000** (return_clamp 2000) s130-133: max in-job sel 0.80/0.93/0.93/0.93 → **LAST sel 0.00 ×4**.
- **RS1** (reward_scale 1, clamp 0) s134-137: max sel 0.93/0.87/0.33/0.93 → **LAST sel 0.00 ×4**.
- **SPARSE-RS1** s138-139: max sel **0.00 ×2** (never ignited, as registered).
Registered A32 predictions (≥ 3/4 endpoints alive under C2000/RS1) **FAIL**. The clamp mis-set (row 13) is real but is NOT the
cause of the endpoint collapse: with a correctly scaled target every run still ignites mid-training and dies by 3M. Remaining
candidate causes: unshaped demo rows vs shaped online rows (row 17), the 400-step train horizon (row 18), entropy coefficient
3e-5 (row 16), FIFO demo eviction at 450k buffer (all demo frames gone by 0.45M online steps; re-injection every 150k).
A33 (§2.4) shows the same shaping kills RLPD too. **WM status: no configuration tried (clamp100, C2000, RS1, sparse) yields
a learner that keeps a working policy; the WM arm reports a directional selection-time preference for human demos (§3.1
corrected numbers) and a disclosed endpoint failure.** A36 machine packs (7, queued) would add n=4v4 source contrasts under
C2000/RS1/sparse at selection time only — recommendation: cancel unless the selection-time contrast is wanted at that n.

## 4. dv3 (dreamerv3-torch, NM512 port) — mechanism found; standard arm under test

- **Failure mechanism:** on every unclamped ×100-terminal run `value_mean` runs past the attainable return (263–390 vs 100 by
  300k, 3/3 5M baselines, zero success); touchgoal-from-scratch s0 learned to 0.82–0.96 (230–440k) then collapsed as value
  crossed 100 (132, 140). Critic runaway via bootstrapped reward-head leak; the port lacks the official replay critic loss /
  EMA regulariser (`WM_CANDIDATES` §1). Old runs' "transients" (N4) are this phenomenon.
- **Standard arm `genesis_dv3std`** (reward_scale 1 symlog/two-hot, no clamp, train_ratio 512, TL 400, demos terminal 1):
  s20 first picks 0.22 @284k (0.06, then 0 × 8 samples to 550k), value bounded ~33; s21 0.04 @191k, s22 0.02 @177k,
  values 30–36. clamp s20: 0.25 @~290k then 0, value pinned 98; fix s20: 0.18 max, sporadic 0.02, value 97.
  → transient picks under every recipe, none sustained yet (r2dreamer's phenotype before its 1–2M ignition).
  1M verdict → §4.1; 3M std pack (s23–25) with resume chain runs through the blackout.
- Disclosures: dv3 jobs before 08-28 21:30 ran without overlays (launcher bug); 5M baselines unaffected.

### 4.1 dv3 std @1M (appended)
**08-31 (post-blackout, all three 1M runs finished): NEGATIVE ×3.** s20 ended 1.02M (max 0.22, zero since 303k);
s21 ended 1.03M (max 0.38 @~730k, last-4 samples 0.12/0.02/0.06/0.00 — the most active seed, still no sustained pick);
s22 1.04M (max 0.02). 3M pack s23–25 at ~1.07M: max 0.14/0.32/0.04, last-4 ≈ 0 — same phenotype at 1M; resume chain
(3017793) carries them to 3M (~09-01). Protocol evals for s20/s21 (eaten by the stale-dir launcher bug) re-run by hand
08-31 (`sbatch_dv3_handeval.sh`, LAST ckpt, hold+rnd, all denominators complete): **s20 hold 0/15, rnd 2/30;
s21 hold 0/15, rnd 1/30** — the dv3-std negative is now protocol-grade, not just train-time (s22's in-launcher
protocol eval was already clean). Touchgoal probes:
tg_none s0 hit 0.96 @~500k then collapsed with value 143 > 100 (the mechanism, again); tg_none s1 flat; tg_clamp
s2 flat at 529k (n=1 — s0/s1 died pre-fix on the clamp TypeError).
18:30: std s20 @890k — no success since 303k, value now DECREASING (30 → 19); s21 @505k 9 nonzero samples (max 0.22), value 8;
s22 @510k 1 nonzero, value 8. Verdict at 1M for s20: NEGATIVE (transient picks, no learning; value 12 and falling at 950k). The launcher's post-training
eval failed on s20 (stale timestamp dir from a cancelled job; fixed in the launcher 22:00, but s21 inherits the bug — its
protocol eval must be run by hand after the blackout; s22 is clean). 3M pack (s23–25) started 18:00 and
runs through the blackout with its resume chain. DEMO3/MoDem cluster smoke attempt launched 18:40 (A18 trigger).
15:50: std s20 @780k (4 nonzero, none since 303k, value 30); s21 @410k 8 nonzero incl. 0.22 @~350k and 0.12 @366k then 0,
value 21; s22 @418k 1 nonzero; clamp s20 @847k 7 nonzero, value 97; fix s20 @572k 10 nonzero, value 99. Still no sustained
learning; s21 is the most active. Earlier (13:20): std s20 @670k — 4 nonzero success samples total (all ≤ 0.22, none after 303k), value 28 bounded; s21 @310k 5 nonzero
(≤ 0.04); s22 @310k 1; clamp s20 @710k 6 nonzero (≤ 0.25), value 97; fix s20 @476k 10 nonzero (≤ 0.18), value 98.
No sustained learning under any dv3 recipe yet.

## 5. Fallback WM (A18)
DEMO3 codebase (TD-MPC2 backbone; MoDem/TD-MPC2 by flag; MIT) prepared locally: converter round-trip verified on dH,
env wrapper import-checked; `policy_pretraining=false` flag set is actor-BC-free. Cluster smoke 3034689 **FAILED in 26 s**
(hydra: "Primary config module '..config' not found" — a config-path packaging bug in the prep, not a GPU/env issue;
torch 2.7.0+cu126 verified on the node first). **Deprioritized 08-31:** A18's trigger did fire (dv3-std ×3 negative at 1M,
NOCLAMP values unbounded), but the clamped r2dreamer — the disclosed WM arm since 08-29 — is healthy and igniting in the
corrected world, so "everything else fails" (the user's bar for MoDem) has not been met. The fix is known and cheap if
the r2d n=8 gate disappoints; decision rests with the user.

## 6. What can be written now (claims with evidence) — see ADVERSARIAL_AUDIT §7
1. Methods/provenance contribution (one recorder, tape contract, registry, six silent-default bugs caught, run-identity rule).
2. DP: DP-teacher demos ≈ human at matched N; small human edge on random ICs (p 0.041, unadjusted).
3. **RLPD (1a), REVISED after A20 (08-31 ~17:00): the source effect is WORLD-DEPENDENT. Old world: human beats DP-teacher
   demos by +0.21 on random ICs (LAST ckpt, p 0.001, n 7v7, A16 predictions met) and diverges less (1/8 vs 6/8, Fisher
   0.041). Corrected world: NO effect on any statistic (−0.02, perm p 0.98, n 8v8; divergence 3/8 v 3/8; A20 predictions
   failed). Both blocks pre-registered; both reported. The mechanism carries the pair: the source effect appears exactly
   where machine-demo coverage is narrow (old-world dDP −21 % EEF coverage vs human; corrected world −5 %, fig6) — the
   claim is coverage-of-the-generating-process, with demo SOURCE as its old-world proxy.** Recipe (γ) sensitivity still
   dominates everything at γ 0.998.
4. WM: r2dreamer (return-clamped DreamerV3) learns pick with dense reward + demos in BOTH worlds, checkpoint-bistable,
   human > DP in the corrected world (0.77 vs 0.33 hold, n=4, p 0.14; n=8 pending); sparse never ignites. Under the published
   recipe (no clamp) BOTH torch ports run away (dv3-std ×3 no learning; r2d-NOCLAMP values 115–825 vs max 100) — a
   characterised negative, not a missing experiment.
5. **RLPD (1b), controlled (NEW 08-31): the human>DP gap survives failure data at the same size (+0.21, p 0.017/<0.001,
   n 8 v 8), and adding own-failure tapes cures the DP arm's divergence (6/8 → 1/8). The fails-vs-dup effect is
   directionally positive on both sources (A17 rule met) but within noise of the row-count control — claim the
   de-divergence, not a score lift.** The source gap also replicates inside the dup controls (+0.29, n 4 v 4).
6. DP unpruned-human control COMPLETE n=3: no detectable penalty (−0.056, CI [−0.19, +0.08], perm p 0.36) — the
   "DP reacts the same to both sources" claim survives; pruning was efficiency, not load-bearing cleaning.
7. RLPD (2) dense COMPLETE n=6v6: dense shaping hurts both sources (dH 0.38 / dDP 0.30 vs sparse 0.69 / 0.50) and
   does not close the gap — reward density does not rescue machine demos.
8. r2d NOCLAMP fully quantified: endpoint dead on protocol ICs (LAST hold ≤1/15 ×4); BEST weak. The clamp is
   load-bearing, with numbers.
9. **A20 scored (08-31 ~17:00): corrected-world RLPD n=8v8 shows no source effect (§2.3) — registered predictions
   failed, reported as such.** With §1 (DP unchanged across worlds) this is an RLPD × world × source interaction;
   fig6/fig7 carry the coverage mechanism that predicts it.
10. **Demo recovery + camera audit (other-machine session, 08-31 evening; a478d95): solved successes 61 → 74/75**
   (only 303 remains, no video; funnel picked 0.67→0.96, nested 0.21→0.33, single-collection), all six real
   nested demos re-nest; camera instrument calibrated to ~1.5–3.5 cm with the close-time protocol correction, and
   the camera-free consistency stat (|close-xy − placement| median 2.9 cm, 14/15 ≤ 5 cm) already answers the
   07-08 panel's "search fits" criticism with a measurement. Frozen matched sets UNTOUCHED — v2 tape pool
   (36 tapes) and any audit scoring are PREREG-gated. Docs: DEMO_RECOVERY_RESULTS / CAMERA_GATE /
   CRITIQUE_demo_recovery (all 2026-08-31). **Training-IC drift audit (09-01, state[0] of the frozen dH
   tapes vs the newly validated positions):** 19/56 old-world (21/58 w3) training ICs were later re-validated;
   median shift 2.0 cm, 5 exceed 3 cm, max 13.2 cm (uid 319, the drag demo, arm-base edge cluster). Within the
   camera instrument's own error for all but a handful; NO reruns required (both arms share every IC by
   construction, so any offset is common-mode). Disclose as: training ICs for re-validated trials sit a
   median 2 cm from ground truth.
Still open: r2d corrected-world n=8 (~09-01; dH pack resumed after OOM) — now DECISIVE for whether world-dependence
is learner-general or RLPD-specific; dv3 3M (~09-01); dup n=8 top-up (running).

---
## 7. State of the queue (08-31 00:30, post-blackout)
Running: r2d w3 s84–87 packs (~1M of 3M each); dv3-3M pack (resume chain 3017793 next); DP dHunpruned s33/s34; RLPD
dense refills (dH s36, dDP s36–38); dv3 hand-eval (CPU); RLPD dup top-up s34–37 ×2 sources (n=8 for §2.2's dup rows);
r2d touchgoal pack (took TWO fixes: registry null-demo 138a3b5 + a silent `ls|wc` rc=2 under pipefail, a43bf39 — 8th
silent-default sighting); DEMO3 smoke resubmitted after the hydra config_path fix (RESULTS §5 stays deprioritized);
16 NOCLAMP re-scores (CPU). Still user-held: 29 r2d-train singles (superseded by packs), 35 DP replicates
(BeginTime fillers). Blackout casualties (all transient CUDA on one node, all refilled with fresh seeds per A9):
DP dHunpruned s30/31, RLPD dense ×4. Two scheduled harvests (3017796/97) still pending — will self-append.

**08:30 update:** A20 corrected-world RLPD block (g99w3 s40-47 ×2 arms, 16 jobs) running — submitted by the staged
VPN-wait loop, all past the world/demo gates; dup n=8 top-up running; touchgoal r2d pack training (~790 metric lines);
DEMO3 smoke now dies at env creation depth (gymnasium API fix in, resubmitted 3085524); dH w3 pack OOM → resumed as
2×2-seed packs (3085547/48).

_version: 2026-08-31 08:40 — morning update. Complete: DP (n=10) + unpruned control (n=3, no penalty), RLPD (1a) sparse
n=8v8, (1b) fails n=8v8 + dup n=4v4, (2) dense n=6v6, dv3-std 1M ×3 protocol-grade negative, r2d NOCLAMP BEST+LAST
re-scores. Pending: r2d W3 n=8 (~09-01), RLPD g99w3 (A20) + dup n=8 (today), dv3 3M, touchgoal, DEMO3 smoke._
