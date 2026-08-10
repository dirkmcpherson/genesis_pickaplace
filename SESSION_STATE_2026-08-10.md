# Live state at compaction (2026-08-10 evening)

READ FIRST: PAPER_PLAN.md (claims log incl. 08-10 retraction + predicate cert),
STATUS_2026-08-10.md, dreamerv3-torch/MANISKILL_VS_GENESIS.md,
r2dreamer/GENESIS_PORT_STATUS.md + R2D_PICK_DIAGNOSIS.md.

## Running NOW
- LOCAL GPU: r2dreamer `runs/pick_msparity_s0` (absolute joint + 4 ManiSkill
  pillars: tl100/+100-hardened/clean-pruned-demos/train_ratio512). GATE: if
  actor entropy has not begun collapsing by ~12h (ManiSkill fingerprint: -2.1
  nats by 17k updates), KILL and launch the delta variant:
  `train.py env=genesis_pick_v4_delta ... demo_dir=.../genesis_pick_pruned_delta
  logdir=runs/pick_delta_s0` (full cmd in GENESIS_PORT_STATUS.md). Delta demos
  replay-gate PASSED 3/3.
- CLUSTER: SACfD retrain wave (16 seeds, hardened predicate, fresh dirs under
  baselines/outputs/paper/); re-eval of OLD checkpoints from
  paper_oldpred_sacfd/ (sbatch locally sed-ed to that path — expect
  *-hardened-eval runs in wandb genesis_paper); dSACfD wave-2 column queued
  behind a harvest keyed to the NEW dH_SACfD_s0 zip (timing race: if harvest
  starts before retrain writes it, it dies teacher-missing → rerun launcher);
  dH_DV3_extra still training.

## Expected next verdicts
1. Old-SACfD honest re-evals (tonight): likely big drops — RLfD row levels TBD.
2. Fresh hardened SACfD evals (~tomorrow): the REAL RLfD rows.
3. msparity entropy fingerprint (tonight) → maybe delta swap.
4. Cluster r2dreamer port ready (cluster/R2DREAMER_CLUSTER.md) — user can run
   msparity+delta side-by-side full-spec after rsyncs.

## Standing facts (do not re-derive)
- BC rows CERTIFIED (DP control 0.67 unchanged under hardened predicate):
  dH_DP n=8 mean 0.62; dDP_DP n=8 mean 0.80; P(model>human)=0.994 in-dist,
  0.45 random. SACfD numbers PRE-hardening are fling-inflated (all learners
  that optimized reward gamed the old predicate; harvest guards were right).
- World-model arms: zero genuine policies ever (v2's 0.20 retracted via
  control); every "success" was an artifact caught by paired controls.
- r2dreamer wandb: jambotime/r2dreamer_genesis (22 runs backfilled, evals
  auto-log with --wandb). Monitors in this session died at compaction — re-arm
  wandb pollers (genesis_paper + r2dreamer_genesis) and a local-run watcher.
- uid331 entry = IC artifact; genesis_vec_env.py:265 predicate copy STILL
  unguarded (fix before any batched/dv3 retrain).
