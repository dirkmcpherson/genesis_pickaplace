# RUN LEDGER — every run family's state, one table (2026-08-14)

Purpose: no run whose provenance is not understood. Each family lists the code
state it ran under, across the three change boundaries of 08-13/14:
- **E1** = #26 inner-horizon fix (3a7a713, 08-14 ~00:20): phantom _nested past
  1200 inner steps. Affects replays/evals past 1200; training <=900 unaffected
  (EXCEPT cartesian 1800-step training — exposed).
- **E2** = P2 reset fix (e9f6e24, 08-14 00:49:57): controller targets re-issued
  at reset. Changes eval baselines; solver-cache residue remains (fresh-process
  protocol is the real resolution).
- **T1** = entropy-backup fix (f906ee6, 08-14 ~morning): backup_entropy off for
  sparse scope. Affects ALL SB3-based training (RLPD and stock-SAC SACfD) before
  it — their critics chased a 500*alpha*H fixed point, 400:1 against terminating.

| family | trained under | evaluated under | verdict / standing |
|---|---|---|---|
| DP rows (BC central table, dH/dDP, n=8) | no RL trainer | pre-E1/E2, sequence protocol | **STANDS** — robustness check: mid/high pick rates survived E2 + protocol change episode-for-episode. Methods paragraph, no re-run (joint rec) |
| SACfD historical (16 zeros + ~30 runs) | pre-T1 (stock SB3 = backup ON), pre-E1/E2 | mixed | zeros CONFOUNDED by T1 — reinterpret, do not re-run; RLPD-nb becomes the RLfD arm (pending user framing call) |
| RLPD stride-1 s0..s6 (+s0_fixa) | pre-T1, pre-E1/E2 | post-hoc sweep post-E1/E2 fresh-process @100k | magnitudes SUPERSEDED (document T1 bug); s0@150k 0.40 stands as that checkpoint's measured capability (fresh-process replicated, horizon-invariant 400/1200) |
| RLPD ar4 s0..s5 | pre-T1, pre-E1/E2 | post-hoc @100k fresh-process (primary); sensitivity column DEAD | PROVISIONAL — flat-across-N is what a T1-blinded critic produces |
| RLPD ar8 s0..s5 | pre-T1; env pre-E2 for most of training | 180 fresh processes post-E1/E2 | "not worth pursuing" RE-DECLARED PROVISIONAL (same reason) |
| **RLPD nb s0..s2 (RUNNING)** | **post-E1/E2/T1** (f906ee6+), authored sidecars incl backup_entropy | pre-registered: fresh-process @100k, >=3/15 bar | first clean-trainer wave; sweep by newbox_supp on completion |
| r2dreamer champion (0.91/1.00) | own codebase (no SB3/T1); env pre-E1/E2 | n=45 sampled + mode 15/15 | **STANDS** (mid/high robust basin; training-in-residue doctrine) |
| r2dreamer short wave s10-19 (dH 3/10, dDP 0/10) | pre-E1/E2 env | in-train + best-ckpt evals | stands as measured; DO NOT POOL with post-fix waves |
| r2dreamer firming wave s20-29 (queued) | post-E1/E2 (cluster pull) | TBD | within-wave dH-vs-dDP clean; cross-wave pooling carries the E2 baseline caveat |
| dv3 all historical | own codebase; env pre-fixes | various | NULL — stands; superseded by msrecipe port as the go-forward |
| dv3 msrecipe (queued for cluster) | post-E1/E2 (gates re-verified post-fix 4/5) | ManiSkill entropy fingerprint gate | READY — commit 69b11e4 (dreamerv3-torch) + demo set rsync |
| phase sweep pre-fix (34/31/5/4) | — | pre-E1, sequence | superseded |
| phase sweep post-#26 (42/40/5/1) | — | post-E1 pre-E2, sequence | the E1-effect measurement (matched schedule); absolutes carry P2 caveat |
| measured-ref tape experiments (0/5, 3/5) | — | post-E1/E2 | closed: tape-in-measured-space refuted; superseded by re-record |
| re-record censuses r1/r4/r8 (61/55/28/20 etc.) | — | post-E1/E2, SEQUENCE shards | directionally strong; **fresh-process re-census before paper use**; npz stamped (delta_ref/scale/repeat) |
| demo labels (65/62/25/16 on 72) | — | collector (velocity, self-healing, E1-fixed since 07-20) | reference measurement; P2-exposed in principle, most robust in practice |
| eval sweeps: 19-eval post-hoc, ar8 180, horizon A/B, s0 replications | — | all post-E1/E2, fresh-process, explicit flags | the clean measurement layer; cite these |

## Artifact provenance rules (in force)
1. Sidecars: action_mode / action_repeat / backup_entropy authored per checkpoint
   (backfilled ones carry backfilled=true).
2. Re-record npz: delta_ref / delta_scale / action_repeat stamped inside the file.
3. **Git hash stamping (added 08-14): train_rlpd sidecars and re-record manifests
   record `git` = short HEAD at launch.** Anything older: attribute by launch
   time vs the E1/E2/T1 commit timestamps above.
4. Eval logs record mode/repeat/horizon/checkpoint; invoking script kept.
5. Fresh process per episode for paper numbers; matched horizons or stated.
| RLPD expansion wave (exp, 08-17) | post E3-demo-rng fix (2fbed2a): per-seed demo streams | new seeds only (dH s3-7, clean s3-6) | do NOT pool blindly with pre-fix seeds; seed-0 era-equivalent |
