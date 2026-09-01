# RESULTS — for the plane writing session (living; post-blackout update 2026-08-31)

**Read with:** `METHODS_draft_2026-08-28.md` (UPDATE 08-29 blocks), `PREREG_final_round_robin_2026-08-23.md` A9–A19,
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
   CRITIQUE_demo_recovery (all 2026-08-31).
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
