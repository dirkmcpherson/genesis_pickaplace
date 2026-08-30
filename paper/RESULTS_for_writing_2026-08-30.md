# RESULTS — for the plane writing session (living; final version stamped before the 08-30 blackout)

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
Still to come (during the blackout): dense arms (question 2), fails arms + row-matched dup controls (1b).

## 3. r2dreamer (in-house DreamerV3; replay critic loss = official DV3 v2 component; return clamp = deviation)

- **Old world, dense, 3M (historic block, seeds 50–53, 100–103, 80–87 dR2D):** selected checkpoints reach 13–15/15 hold on
  dH and dR2D; **sparse never learns** (0/7,700 episodes). LAST-checkpoint hold re-scores: dH s50–53 = 0, 15, 0, 7;
  dH s101/103 = 0, 0; dDP s100/102/103 = 1, 10, 14; dR2D s80–83/86/87 = 15, 0, 13, 0, 13, 11; dR2DDPfails = 5, 5, 0, 1, 15, 4
  → final policies are **bistable** (0 or ≥ 10 of 15 in 16/21); BEST-of-K on sel does the work. Disclose.
- **Corrected world gate (dH/dDP s80–83, dense, 3M, packed, 08-29): IGNITES.** In-job sel max per seed: dH 0.40, 0.93, 0.93,
  0.93 (nonzero ckpts 3/17/13/14 of 28); dDP 0.00, 0.87, 0.13, 0.93 (0/17/4/8). Same adjacent-checkpoint bistability.
  Protocol re-scores (BEST hold/rnd, LAST hold; `n12_rescore/*_W3_*`) → §3.1.
- **Standard arm (clamp off, repval kept) NOCLAMP dH/dDP s122/123 — RUNS AWAY.** At ~0.7M sim steps `train/val` = 825, 343
  (dH), 226, 115 (dDP) against a 100 maximum — the same critic runaway as dv3, now in r2dreamer with the official replay
  critic loss present. So the clamp is what makes r2dreamer learn; neither torch port is stable under the published recipe on
  this task. Write-up: the WM arm = DreamerV3-with-return-clamp, disclosed; the standard-recipe negatives (dv3-std ×3,
  r2d-NOCLAMP ×4) are reported as such. Final numbers land in the +44/+62 h harvests.
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
Direction: human > DP for the world model; NOT significant at n=4 (min p 0.029). Seeds 84–87 per source submitted 08-29
13:30 (finish during the blackout) → n=8 gives min p 0.0002 and ~80 % power at the observed gap.

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
env wrapper import-checked; `policy_pretraining=false` flag set is actor-BC-free. ~4–6 h to a cluster smoke. Triggered 08-29 (both
standard arms negative): venv installed on the cluster (torch 2.7.0+cu126 kept; torchrl 0.8.1, tensordict 0.8.3), smoke
job 3034689 pending → result in `paper/DEMO3_SMOKE_2026-08-29.md` / the harvest files.

## 6. What can be written now (claims with evidence) — see ADVERSARIAL_AUDIT §7
1. Methods/provenance contribution (one recorder, tape contract, registry, six silent-default bugs caught, run-identity rule).
2. DP: DP-teacher demos ≈ human at matched N; small human edge on random ICs (p 0.041, unadjusted).
3. **RLPD (1a): under the published recipe, human demos beat DP-teacher demos on random ICs by +0.21 (LAST ckpt, p 0.001,
   n 7 v 6; pre-registered A16) and diverge less (1/8 vs 6/8, Fisher p 0.041); recipe (γ) sensitivity dominates everything
   at γ 0.998.**
4. WM: r2dreamer (return-clamped DreamerV3) learns pick with dense reward + demos in BOTH worlds, checkpoint-bistable,
   human > DP in the corrected world (0.77 vs 0.33 hold, n=4, p 0.14; n=8 pending); sparse never ignites. Under the published
   recipe (no clamp) BOTH torch ports run away (dv3-std ×3 no learning; r2d-NOCLAMP values 115–825 vs max 100) — a
   characterised negative, not a missing experiment.
5. Fails-tape effect: confounded (share × 2.7, length × 12) until the row-matched controls read out.
Nothing yet on (1a)/(1b) for RLPD or WMs beyond the above; nothing on (2) beyond "sparse WM never ignites".

---
## 7. State of the queue at the blackout (08-29 23:25 cluster time)
Running: dv3 std s21/s22 (1M) + std-3M pack (s23–25, resume chain 3017793) + tg_clamp s2; r2d NOCLAMP dH/dDP s122/123;
RLPD dHsucc_dup s30–32. Pending in order: 33 RLPD (dup, fails ×16, dense ×12), r2d w3 s84–87 ×2 packs, DEMO3 smoke, then
35 DP replicates as filler from 08-31 00:23. Harvest files: `paper/harvest_2026-08-3{0,1}_*.md` on the cluster (+20/+44/+62 h).
Post-blackout list: `paper/NEXT_2026-08-28.md` (bottom).

_version: 2026-08-29 23:25 (cluster clock) — FINAL pre-blackout stamp. Numbers above are complete for: DP (n=10), RLPD (1a)
sparse (n=8 v 8), r2dreamer corrected-world (n=4 v 4). Pending: RLPD dense/fails/dup, r2d n=8 and NOCLAMP finals, dv3 3M, DEMO3._
