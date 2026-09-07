# Matched-start review reels — 2026-09-07

Arm H = human-demo policy, arm M = machine-demo policy; cells are the deterministic (mode) evals; runs under `/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03/runs/`. Built by `baselines/review_matched_reels.py` from `metrics.json` + the episode videos only (no simulation). Numbers below come from `metrics.json` only.

**Selection rule.** Per arm and cell the *median* seed by that cell's success count (lower median for even n, ties → lowest seed) so the reel shows a typical policy. Shared starts = episode index k (uid / can_pos checked equal across arms); starts with `restore_failed` on either arm are skipped. Up to 3 starts per joint-outcome class, sampled uniformly with `random.Random(0)` (one generator per comparison×cell, classes drawn in the order both / human-only / machine-only / both-fail). Reels: 4 fps (half speed), cap 150 frames, ended clips dimmed.

**How to read the joint-outcome table.** `both` / `H-only` / `M-only` / `neither` count the shared starts by (H outcome, M outcome). If the two arms were interchangeable policies, H-only ≈ M-only. The all-seed-pairs row sums seed i vs seed i over all seeds; its units are start×seed-pair (starts repeat across seeds, so the exact binomial p on the discordant counts is descriptive, not an independent-sample test).


## pick

pick scope; rnd30 = 30 shared random starts (OOD), hold15 = 15 held-out human starts (ID). H = `s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s{s}`, M = `s2_r2d_pick_state_dDP_bnormclamp1ent5_s{s}`, seeds [0, 1, 2, 3, 4, 5, 6, 7], success = `picked`.

### fresh_eval_rnd30_mode (OOD)

Per-seed successes — H: s0=15/30, s1=19/30, s2=19/30, s3=19/30, s4=21/30, s5=17/30, s6=20/30, s7=18/30; M: s0=20/30, s1=18/30, s2=18/30, s3=18/30, s4=16/30, s5=19/30, s6=19/30, s7=18/30.  
**Chosen (median) seeds: H s1 (19/30), M s1 (18/30).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s1) | 18 | 1 | 0 | 11 | 30 | 1.000 |
| all seed pairs (i vs i, 8 pairs) | 130 | 18 | 16 | 76 | 240 | 0.864 |

Per pair both/H-only/M-only/neither: s0: 14/1/6/9; s1: 18/1/0/11; s2: 17/2/1/10; s3: 16/3/2/9; s4: 15/6/1/8; s5: 14/3/5/8; s6: 19/1/0/10; s7: 17/1/1/11.

Sampled starts: both-succeed [21, 22, 3]; human-only [7]; machine-only []; both-fail [24, 19, 16].

Files: `review_matched/pick_fresh_eval_rnd30_mode_matched.mp4`, `review_matched/pick_contact_sheet.png`

### fresh_eval_hold15_mode (ID)

Per-seed successes — H: s0=12/15, s1=15/15, s2=15/15, s3=14/15, s4=15/15, s5=15/15, s6=15/15, s7=15/15; M: s0=15/15, s1=14/15, s2=15/15, s3=15/15, s4=15/15, s5=15/15, s6=15/15, s7=15/15.  
**Chosen (median) seeds: H s1 (15/15), M s0 (15/15).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s0) | 15 | 0 | 0 | 0 | 15 | nan |
| all seed pairs (i vs i, 8 pairs) | 115 | 1 | 4 | 0 | 120 | 0.375 |

Per pair both/H-only/M-only/neither: s0: 12/0/3/0; s1: 14/1/0/0; s2: 15/0/0/0; s3: 14/0/1/0; s4: 15/0/0/0; s5: 15/0/0/0; s6: 15/0/0/0; s7: 15/0/0/0.

Sampled starts: both-succeed [13, 6, 12]; human-only []; machine-only []; both-fail [].

Files: `review_matched/pick_fresh_eval_hold15_mode_matched.mp4`


## pick_r1

repeat-1 pick (4 seeds per arm); same cells as pick. H = `s2_r2d_pick_state_dHv2raw_r1_bnormclamp1ent5_s{s}`, M = `s2_r2d_pick_state_dDP_r1_bnormclamp1ent5_s{s}`, seeds [0, 1, 2, 3], success = `picked`.

### fresh_eval_rnd30_mode (OOD)

Per-seed successes — H: s0=22/30, s1=18/30, s2=18/30, s3=19/30; M: s0=18/30, s1=20/30, s2=19/30, s3=21/30.  
**Chosen (median) seeds: H s1 (18/30), M s2 (19/30).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s2) | 18 | 0 | 1 | 11 | 30 | 1.000 |
| all seed pairs (i vs i, 4 pairs) | 70 | 7 | 8 | 35 | 120 | 1.000 |

Per pair both/H-only/M-only/neither: s0: 17/5/1/7; s1: 18/0/2/10; s2: 17/1/2/10; s3: 18/1/3/8.

Sampled starts: both-succeed [21, 22, 3]; human-only []; machine-only [13]; both-fail [24, 19, 16].

Files: `review_matched/pick_r1_fresh_eval_rnd30_mode_matched.mp4`, `review_matched/pick_r1_contact_sheet.png`

### fresh_eval_hold15_mode (ID)

Per-seed successes — H: s0=15/15, s1=15/15, s2=15/15, s3=15/15; M: s0=15/15, s1=15/15, s2=15/15, s3=15/15.  
**Chosen (median) seeds: H s0 (15/15), M s0 (15/15).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s0 vs M s0) | 15 | 0 | 0 | 0 | 15 | nan |
| all seed pairs (i vs i, 4 pairs) | 60 | 0 | 0 | 0 | 60 | nan |

Per pair both/H-only/M-only/neither: s0: 15/0/0/0; s1: 15/0/0/0; s2: 15/0/0/0; s3: 15/0/0/0.

Sampled starts: both-succeed [13, 6, 12]; human-only []; machine-only []; both-fail [].

Files: `review_matched/pick_r1_fresh_eval_hold15_mode_matched.mp4`


## place

place scope, matched 39; polE = 148 shared policy-generated entries (OOD), holdE = 13 shared human entries (ID). H = `s2_r2d_place_state_dH_bnormclamp1ent5_s{s}`, M = `s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s{s}`, seeds [0, 1, 2, 3, 4, 5, 6, 7], success = `placed_v2`.

### fresh_eval_polE_mode (OOD)

Per-seed successes — H: s0=113/148, s1=86/148, s2=112/148, s3=112/148, s4=82/148, s5=99/148, s6=110/148, s7=118/148; M: s0=109/148, s1=79/148, s2=111/148, s3=105/148, s4=90/148, s5=79/148, s6=95/148, s7=98/148.  
**Chosen (median) seeds: H s6 (110/148), M s6 (95/148).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s6 vs M s6) | 80 | 30 | 15 | 23 | 148 | 0.036 |
| all seed pairs (i vs i, 8 pairs) | 640 | 192 | 126 | 226 | 1184 | 0.000 |

Per pair both/H-only/M-only/neither: s0: 96/17/13/22; s1: 57/29/22/40; s2: 98/14/13/23; s3: 93/19/12/24; s4: 65/17/25/41; s5: 64/35/15/34; s6: 80/30/15/23; s7: 87/31/11/19.

Sampled starts: both-succeed [95, 102, 9]; human-only [46, 75, 72]; machine-only [63, 107, 49]; both-fail [86, 68, 109].

Files: `review_matched/place_fresh_eval_polE_mode_matched.mp4`, `review_matched/place_contact_sheet.png`

Context (from the same `metrics.json` files): this reel uses the matched-39 machine arm (`s2_r2d_place_state_dDP_bnormclamp1ent5_n39_s{s}`), polE mean 0.647 vs H 0.703. The UNCAPPED machine arm `s2_r2d_place_state_dDP_bnormclamp1ent5_s{0..7}` (104 demos, not in this reel) scores 0.709 on the same cell (per seed s0=0.622, s1=0.791, s2=0.682, s3=0.764, s4=0.696, s5=0.669, s6=0.757, s7=0.696) — that is the 0.709 the 09-05 status line quotes; the matched-39 arm is the one the brief names.

### fresh_eval_holdE_mode (ID)

Per-seed successes — H: s0=13/13, s1=13/13, s2=13/13, s3=13/13, s4=13/13, s5=13/13, s6=13/13, s7=13/13; M: s0=13/13, s1=13/13, s2=13/13, s3=13/13, s4=13/13, s5=13/13, s6=13/13, s7=12/13.  
**Chosen (median) seeds: H s0 (13/13), M s0 (13/13).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s0 vs M s0) | 13 | 0 | 0 | 0 | 13 | nan |
| all seed pairs (i vs i, 8 pairs) | 103 | 1 | 0 | 0 | 104 | 1.000 |

Per pair both/H-only/M-only/neither: s0: 13/0/0/0; s1: 13/0/0/0; s2: 13/0/0/0; s3: 13/0/0/0; s4: 13/0/0/0; s5: 13/0/0/0; s6: 13/0/0/0; s7: 12/1/0/0.

Sampled starts: both-succeed [6, 12, 0]; human-only []; machine-only []; both-fail [].

Files: `review_matched/place_fresh_eval_holdE_mode_matched.mp4`


## contact

contact after release, matched 11; polE = 160 shared entries (OOD), holdE = 11 (ID). H = `s2_r2d_contact_state_dH_bnormclamp1ent5_subfloor_s{s}`, M = `s2_r2d_contact_state_dDP_bnormclamp1ent5_n11_s{s}`, seeds [0, 1, 2, 3, 4, 5, 6, 7], success = `contact`.

### fresh_eval_polE_mode (OOD)

Per-seed successes — H: s0=95/160, s1=91/160, s2=79/160, s3=117/160, s4=101/160, s5=88/160, s6=76/160, s7=112/160; M: s0=103/160, s1=91/160, s2=94/160, s3=92/160, s4=100/160, s5=115/160, s6=85/160, s7=90/160.  
**Chosen (median) seeds: H s1 (91/160), M s3 (92/160).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s3) | 70 | 21 | 22 | 47 | 160 | 1.000 |
| all seed pairs (i vs i, 8 pairs) | 589 | 170 | 181 | 340 | 1280 | 0.594 |

Per pair both/H-only/M-only/neither: s0: 81/14/22/43; s1: 72/19/19/50; s2: 56/23/38/43; s3: 83/34/9/34; s4: 82/19/18/41; s5: 77/11/38/34; s6: 61/15/24/60; s7: 77/35/13/35.

Sampled starts: both-succeed [122, 131, 16]; human-only [61, 127, 124]; machine-only [81, 68, 94]; both-fail [64, 120, 40].

Files: `review_matched/contact_fresh_eval_polE_mode_matched.mp4`, `review_matched/contact_contact_sheet.png`

### fresh_eval_holdE_mode (ID)

Per-seed successes — H: s0=11/11, s1=11/11, s2=11/11, s3=11/11, s4=11/11, s5=11/11, s6=11/11, s7=11/11; M: s0=11/11, s1=11/11, s2=10/11, s3=10/11, s4=10/11, s5=10/11, s6=11/11, s7=10/11.  
**Chosen (median) seeds: H s0 (11/11), M s2 (10/11).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s0 vs M s2) | 10 | 1 | 0 | 0 | 11 | 1.000 |
| all seed pairs (i vs i, 8 pairs) | 83 | 5 | 0 | 0 | 88 | 0.062 |

Per pair both/H-only/M-only/neither: s0: 11/0/0/0; s1: 11/0/0/0; s2: 10/1/0/0; s3: 10/1/0/0; s4: 10/1/0/0; s5: 10/1/0/0; s6: 11/0/0/0; s7: 10/1/0/0.

Sampled starts: both-succeed [7, 10, 0]; human-only [5]; machine-only []; both-fail [].

Files: `review_matched/contact_fresh_eval_holdE_mode_matched.mp4`


## carrycontact

carrycontact, matched 21; polE = 148 shared entries (OOD; entries that fail restore are skipped), holdE = 13 (ID). H = `s2_r2d_carrycontact_state_dH_bnormclamp1ent5_s{s}`, M = `s2_r2d_carrycontact_state_dDP_bnormclamp1ent5_n21_s{s}`, seeds [0, 1, 2, 3, 4, 5, 6, 7], success = `contact`.

### fresh_eval_polE_mode (OOD)

Per-seed successes — H: s0=116/148, s1=124/148, s2=120/148, s3=120/148, s4=115/148, s5=121/148, s6=123/148, s7=116/148; M: s0=117/148, s1=119/148, s2=114/148, s3=119/148, s4=123/148, s5=115/148, s6=116/148, s7=119/148.  
**Chosen (median) seeds: H s2 (120/148), M s0 (117/148).** Skipped starts (restore_failed / no video) for this pair: [2, 7, 55, 58, 126].

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s2 vs M s0) | 114 | 6 | 3 | 20 | 143 | 0.508 |
| all seed pairs (i vs i, 8 pairs) | 904 | 51 | 38 | 151 | 1144 | 0.203 |

Per pair both/H-only/M-only/neither: s0: 110/6/7/20; s1: 116/8/3/16; s2: 110/10/4/19; s3: 116/4/3/20; s4: 113/2/10/18; s5: 112/9/3/19; s6: 116/7/0/20; s7: 111/5/8/19 (skipped over all pairs: 40).

Sampled starts: both-succeed [142, 72, 131]; human-only [75, 15, 71]; machine-only [42, 39, 18]; both-fail [59, 97, 62].

Files: `review_matched/carrycontact_fresh_eval_polE_mode_matched.mp4`, `review_matched/carrycontact_contact_sheet.png`

### fresh_eval_holdE_mode (ID)

Per-seed successes — H: s0=13/13, s1=13/13, s2=13/13, s3=13/13, s4=13/13, s5=13/13, s6=13/13, s7=13/13; M: s0=13/13, s1=13/13, s2=13/13, s3=13/13, s4=13/13, s5=12/13, s6=13/13, s7=13/13.  
**Chosen (median) seeds: H s0 (13/13), M s0 (13/13).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s0 vs M s0) | 13 | 0 | 0 | 0 | 13 | nan |
| all seed pairs (i vs i, 8 pairs) | 103 | 1 | 0 | 0 | 104 | 1.000 |

Per pair both/H-only/M-only/neither: s0: 13/0/0/0; s1: 13/0/0/0; s2: 13/0/0/0; s3: 13/0/0/0; s4: 13/0/0/0; s5: 12/1/0/0; s6: 13/0/0/0; s7: 13/0/0/0.

Sampled starts: both-succeed [6, 12, 0]; human-only []; machine-only []; both-fail [].

Files: `review_matched/carrycontact_fresh_eval_holdE_mode_matched.mp4`


## full

end-to-end; success = nested; tiles carry the stages flags [p=picked c=contact n=nested]. H = `full_r2d_state_dHfull_all_bnormclampS8ent5_s{s}`, M = `full_r2d_state_dDPfull_bnormclampS8ent5_s{s}`, seeds [0, 1, 2, 3, 4, 5, 6, 7], success = `nested`.

### fresh_eval_rnd30_mode (OOD)

Per-seed successes — H: s0=2/30, s1=4/30, s2=9/30, s3=10/30, s4=2/30, s5=1/30, s6=6/30, s7=5/30; M: s0=4/30, s1=1/30, s2=7/30, s3=4/30, s4=9/30, s5=6/30, s6=9/30, s7=6/30.  
**Chosen (median) seeds: H s1 (4/30), M s5 (6/30).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s5) | 2 | 2 | 4 | 22 | 30 | 0.688 |
| all seed pairs (i vs i, 8 pairs) | 10 | 29 | 36 | 165 | 240 | 0.457 |

Per pair both/H-only/M-only/neither: s0: 2/0/2/26; s1: 0/4/1/25; s2: 2/7/5/16; s3: 1/9/3/17; s4: 2/0/7/21; s5: 0/1/6/23; s6: 1/5/8/16; s7: 2/3/4/21.

Sampled starts: both-succeed [21, 11]; human-only [0, 19]; machine-only [23, 9, 12]; both-fail [22, 16, 26].

Files: `review_matched/full_fresh_eval_rnd30_mode_matched.mp4`, `review_matched/full_contact_sheet.png`

### fresh_eval_hold15_mode (ID)

Per-seed successes — H: s0=2/15, s1=5/15, s2=5/15, s3=6/15, s4=3/15, s5=5/15, s6=6/15, s7=6/15; M: s0=3/15, s1=0/15, s2=7/15, s3=5/15, s4=7/15, s5=4/15, s6=11/15, s7=5/15.  
**Chosen (median) seeds: H s1 (5/15), M s3 (5/15).**

| starts | both | H-only | M-only | neither | n | exact binomial p (H-only vs M-only) |
|---|---|---|---|---|---|---|
| chosen seeds (H s1 vs M s3) | 1 | 4 | 4 | 6 | 15 | 1.000 |
| all seed pairs (i vs i, 8 pairs) | 12 | 26 | 30 | 52 | 120 | 0.689 |

Per pair both/H-only/M-only/neither: s0: 0/2/3/10; s1: 0/5/0/10; s2: 3/2/4/6; s3: 0/6/5/4; s4: 2/1/5/7; s5: 2/3/2/8; s6: 4/2/7/2; s7: 1/5/4/5.

Sampled starts: both-succeed [8]; human-only [13, 2, 6]; machine-only [14, 9, 11]; both-fail [5, 4, 3].

Files: `review_matched/full_fresh_eval_hold15_mode_matched.mp4`

