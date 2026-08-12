# Session state 2026-08-12 (~11:30) — pre-compaction handoff

## Live decision points (user)
1. **Render probe on cluster** — the discovery-suppression hypothesis: demo
   images are DEV-BOX pixels; cluster env renders its own; gates only checked
   physics. User about to run cluster/render_probe.py (command in
   R2DREAMER_CLUSTER-adjacent chat; env vars GENESIS_PICKAPLACE_ROOT +
   DELTA25 + OUT, r2d venv). LOCAL CONTROL: MAE 0.24/0.44/1.13, frac>8 =
   1.3/2.1/3.3% (uids 232/242/243). Cluster in that band -> hypothesis dead ->
   fall back to leading-indicator canaries (tips/short-eps/entropy at 100-300k).
   Cluster MAE >> control -> domain gap CONFIRMED -> fix = re-render demo
   images by replaying gated tapes through the cluster env; then test-2 =
   champion checkpoint eval on cluster (145MB, VPN rsync).
2. **RLPD implementation go/no-go** — plan in baselines/rl/RLPD_PLAN.md
   (this commit). Motivated: gamma pilot FAILED (0.00 @200k, 08-12), dj wave
   closed ~zero. User said plan-only so far.
3. dH_R2D_s4 resubmit status unknown (NFS-flake casualty; never in wandb).

## Ledger (all pushed)
- results_matrix_2026-08-12.md + figs (fresh-pulled); METHODOLOGY.md (incl
  delta-EEF rejection 7.5); PAPER_PLAN decision log current through gamma-pilot.
- LOCAL r2dreamer: 4 completed 3M runs, 2 discoveries: s0 champion
  CHAMPION_1576820.pt (0.91 sampled n45 / 1.00 mode), clamp champion
  ckpt_1875040.pt (0.89 conf n45 / 0.93 mode; discovery 1.00). Clamp verdict:
  spikes unchanged, peaks unhurt, checkpoint durability 16/20 vs 11/27
  (Fisher p=.008). s1 (killed 1.9M) + s2 (3M) = non-discovery.
- CLUSTER wave: ~5 complete at 3M all ZERO + partials; one blip dH_s3 0.067
  @1.07M. ALL inputs certified (checksums, gates 3/3 incl dDP w/ --raw-dir,
  config diff, prefill 43104 exact, fp16 identical). 0/9-vs-2/4 = the open
  anomaly the render probe targets.
- SACfD: absolute 0x15; dj 0x14 + one 0.067 (s6/s7 crashed at rew .01);
  gamma-0.998 delta pilot 0.00 (dH_SACfD-djg_s0_pilot). gamma + eval-callback
  fixes pushed (train_sacfd 0.98 was silent killer: 0.98^662 ~ 1.6e-6).
- dv3: null through ~5M (s7).

## Infra notes for the next context
- Monitors all die at compaction; re-arm: genesis_paper poller, wave-v2 poller
  (progress-buckets, state-agnostic — syncer marks runs finished every cycle,
  NEVER use run state as liveness), dv3 poller. No local trainers running.
- pgrep patterns self-match wrapper cmdlines (bit 5x): use ps+sid checks;
  kill by SESSION (pkill -s <sid of the setsid python>, NOT the echoed $!).
- wandb history API: history() downsamples; scan_history(keys=[...]) returns
  only rows containing ALL keys; per-key scans for episode metrics.
- Two same-day false alarms were retracted 08-11 (syncer-state 'crash',
  clamp mechanism): primary evidence (.out, metrics.jsonl) before any
  urgent-action recommendation.
- Champion eval protocol: select on eval-seed-0, confirm x3 fresh seeds +
  mode; eval_genesis --append-metrics RUNDIR puts eval/* curves INTO the
  training wandb run via the live-syncer.
- User path habit: '~' on cluster is WRONG for them — everything lives under
  /cluster/tufts/shortlab/jstale02/ (they substitute manually; my one 75MB
  orphan sits in their real ~/dreamerv3-torch, deletable).
