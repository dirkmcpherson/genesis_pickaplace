# Session state 2026-08-13 (~morning) — budget-exhaustion handoff

## Headlines since last handoff (all wandb-verifiable)
1. **CLUSTER ANOMALY RESOLVED = LOTTERY.** Render probe: cluster pixels IDENTICAL
   to local (MAE 0.24/0.44/1.13 to the digit). Canary report
   (paper/canary_local_vs_cluster_2026-08-12.md): cluster dH runs inside local
   non-discovery envelope; found (a) warm-restart resets shredded old wave into
   ~13 partial attempts, (b) dDP prefill dilution (reward_frames/batch 2.6-3.4
   vs healthy 4.5-5.4), (c) dDP_R2D_s4 dual-writer (sbatch guard added 2026-08-12).
   Short wave (20x 1M seeds, dH + dDP w/ DUPLICATE=7): **dH 3/10 ignited —
   s10 final-1M eval picked=0.93 (cluster champion), s13, s15; dDP 0/10.**
   Local baseline 2/4. Lottery confirmed. NOTE: I wrongly declared "0/20 lottery
   dead" mid-wave — my poller filter matched '_R2D_' which misses 'R2Dshort'
   names (fixed in scratchpad wave_poll.py; lesson: verify filters against live
   run names). dH-vs-dDP ignition contrast 3/10 vs 0/10 (p~0.10) = candidate
   paper finding (demo source affects world-model RLfD ignition); one more
   matched wave would firm it.
2. **RLPD WORKS — first model-free nonzero under paper config** (~30 SACfD zeros
   before it). Impl (Opus agent, commit 587990f local-only, NOT yet pushed):
   baselines/rl/rlpd_sac.py (E=10 LayerNorm ensemble, min over random 2, 128/128
   demo/online, UTD10) + train_rlpd.py + sacfd_delta_gate.py (tensor-equality +
   open-loop gate passed). 7 seeds x 200k local: **6/7 ignited**, first nonzero
   75-175k, snapshot peaks 0.2-0.4 (n=10 → SD~0.145, treat single snapshots as
   noisy), s0@150k = 0.40 indist/0.30 random, s4 posted placed=0.10.
   Fixed-alpha (ent_coef 0.005) arm = dud (0 throughout) → auto-alpha is right.
   Checkpoints: baselines/rl/checkpoints/rlpd_dH_s{0..6}/rlpd_{50,100,150,200}000_steps.zip.
   **IN FLIGHT: s0 150k x3 confirmation evals** (setsid, log scratchpad
   rlpd_s0_confirm.log, wandb group dH_RLPD_confirm) — report numbers + SEND
   VIDEOS to user (standing directive), then run best-per-seed confirmations.
3. **Literature check (paper/rlpd_literature_comparison_2026-08-13.md): our RLPD
   behavior TYPICAL.** Two structural deltas explain modest/unstable peaks:
   (a) reward density 0.08% (66 rewarded frames/83k demo transitions; Adroit
   "sparse" rewards EVERY solved step) + demos truncated ~2 frames past pick;
   (b) 900-step episode > effective gamma-horizon 500 @ 0.998, 30Hz no repeat.
   Levers (NOT launched, user hasn't chosen): action-repeat-4 for SAC arm
   (cleanest), demo extension past pick, n=30 eval snapshots.

## Immediate TODO for next context
- Read rlpd_s0_confirm.log → if ~0.3 holds, matrix row dH_RLPD ~0.3 random-IC,
  P(ignite)=6/7. Send eval videos (checkpoints dir wandb_eval/videos/).
- Update PAPER_PLAN decision log + results matrix: cluster-anomaly resolution
  (+my 0/20 retraction), RLPD row, dH>dDP ignition contrast. Push 587990f.
- Re-arm monitors if compacted: wave_poll.py diff loop (30min), RLPD confirm watch.
- Suggest to user: matched second wave (dDP seeds 20-29 + dH 20-29) to firm the
  ignition contrast; RLPD action-repeat pilot.
- dv3: dDP_DV3_s1 3M eval = 0 (row stays null). SACfD row complete (all zero).

## Standing rules (unchanged)
No actor-BC in world-model arm. Honesty protocol (x3 seeds, neg controls,
selection≠confirmation). Videos to user for every policy. Datasets rsync only.
Cluster paths: /cluster/tufts/shortlab/jstale02/ (never ~). No conda installs
into genesis env. Primary evidence before urgent claims (see poller-filter bug
AND 08-11 syncer false alarm — pattern: my monitoring layer, not the runs).
wandb: scan_history per-key; run state ≠ liveness. pgrep self-match trap: kill
by setsid session. Local box free after RLPD confirms; cluster quota gated by
QOSMaxGRESPerUser.
