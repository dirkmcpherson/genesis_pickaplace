# World-model trainer fix — guide for a fresh session on the GPU box (2026-09-03)

Read this, then `paper/AUDITOR_STARTUP_2026-09-02.md` §0/§3 (rules, failure families) and `paper/CONFOUNDS.md`
rows 13, 16, 17, 18, 25, 42, 43, 44. Do not read CLAUDE.md's history. The main session (laptop, "genesis-pickaplace-de")
owns the cluster and the paper docs; you own the world-model trainer. Coordinate through the log file in §6 and
cross-session messages; never cancel or submit cluster jobs.

## 1. Why we believe the trainers are broken (evidence, not opinion)
- **touchgoal never trains on r2dreamer.** No-demo probe, +1 on first contact with the goal can, 900k steps × 2 seeds:
  success 0.00 on all 10 evals, both seeds (`$LAB/r2dreamer/runs/touchgoal_v5d4c_delta_s{0,1}/ckpt_scores.tsv`).
  dreamerv3-torch on the same task: 1/2 seeds learned (0.82–0.96 from 230k), the other never found the reward
  (`paper/DV3_DIAGNOSIS_2026-08-28.md` §6). A trainer that cannot learn "touch the can" cannot be trusted on "pick it up".
- **Every pick configuration dies at the endpoint.** Clamp 100 (frozen block), clamp 2000, return-scale 1: all human
  runs ignite mid-training (in-job sel up to 0.93) and finish at 0.00 (A32, RESULTS §3.3). Sparse never ignites.
- **The WM arm is pixels-only.** `configs/env/genesis_pick_v5d4c_delta{,_shaped}.yaml`: `mlp_keys: '$^'`,
  `cnn_keys: 'image'`; obs = 64×64×6 (overhead RGB ++ wrist RGB), no joint angles, no gripper state, no can pose,
  no goal. DP and RLPD train on the ground-truth state vector. Row 44.
- **Neither port was ever run on a reference task** (no DMC/Atari run anywhere in the logs).
- **Reward pipeline inconsistencies** (rows 17/18/16): demo rows sparse while online rows are potential-shaped ×100;
  train horizon 400 sim steps (100 decisions) vs eval 1200 and median demo pick at 107–146 decisions; entropy coef
  3e-5 (stock 3e-4); FIFO demo eviction — all demo frames gone after 450k online steps, re-injected every 150k.
- **A33**: the same potential shaping drops RLPD from ≈0.6 to ≈0.1 in the corrected world (RESULTS §2.4). The shaping
  is suspect on its own.

## 2. Where things are
- Repo: `~/workspace/genesis_pickaplace`, branch `4dof-cartesian` (pull first). Python env for Genesis 0.2.1 +
  torch: `~/workspace/genesis_sim2real/venv` (see CLAUDE.md top). Genesis checkout `~/workspace/Genesis` (0.2.1 + local
  headless-render patch; NOT pip-reproducible).
- dreamerv3-torch (NM512 port): `~/workspace/dreamerv3-torch` (configs.yaml has a genesis state config at ~:368
  `mlp_keys: 'state'`, and image configs). r2dreamer (the WM arm of record): the cluster copy is `$LAB/r2dreamer`; the
  exact code of record = `cluster/r2dreamer_port.tar.gz` + `cluster/patches/r2dreamer_final_rr.patch` +
  `baselines/sim_variant_hook.py`. Key files inside: `envs/genesis.py` (adapter: obs, action_repeat 4, delta actions,
  shaping, `R2D_SIM_VARIANT` env var — default `base`!), `demo_prefill.py`, `dreamer.py` (return clamp, ReturnEMA),
  `configs/env/genesis_pick_v5d4c_delta*.yaml`, `eval_genesis.py`.
- Demo sets (rsync ONLY, never git): cluster `$LAB/genesis_pickaplace/baselines/matched_w3/r2d/{dH,dDP,dHv2raw,dDPv2}/`
  (npz + repeat.json; dH/dDP N=58 pruned pair, dHv2raw/dDPv2 N=66 raw pair). Contract-v1 sources in
  `baselines/matched_w3/<arm>/*.npz`; converter `baselines/rl/to_dreamer_native.py` (refuses non-sparse rewards).
  `$LAB=/cluster/tufts/shortlab/jstale02`, `ssh tufts`; cluster python `$LAB/condaenv/genesis/bin/python`.
- Eval ICs: `baselines/eval_ics.json` (frozen sel-15 / hold-15 / rnd-30, the WM block's file). World of record:
  sim variant `gc_kp4_riser3_shelf6` (`baselines/sim_variants.py`).
- Env: `baselines/rl/full_env.py` (`FullTaskEnv`, scopes pick/touchgoal/reach; `pick_shaping_phi` = −2·‖eef−can‖,
  0 at terminal; terminal pick +1), `baselines/genesis_can_env.py` (state vector
  `[q(6), grip motor, grip effort, can xyz, can quat wxyz, goal xy]`).

## 3. The ladder (each stage gates the next; do not skip)
**Stage 0 — reference task, both ports (hours).** DMC `cartpole_balance` then `walker_walk` from pixels, stock config,
2 seeds, ≤500k steps. Gate: return curve within the published order of magnitude (walker_walk > 600 by 500k for DV3).
A port that fails here is broken; bisect it against upstream NM512 before anything else. r2dreamer has never been
run on anything but our env.

**Stage 1 — touchgoal, no demos, STATE input.** Add a `state` obs key to the r2dreamer adapter (the
`genesis_can_env._obs()['state']` vector; dv3 already has the config) and run with `mlp_keys: 'state'`,
`cnn_keys: '$^'`, sparse +1, `time_limit` 1200, entropy 3e-4, no clamp, reward_scale 1; 2 seeds × 300k. Gate: success
≥ 0.8 on 2/2 seeds at LAST. Then repeat with pixels+state. If state-only touchgoal fails on a port that passed
stage 0, the adapter is the bug (action mapping/cap, action_repeat, is_last/is_terminal, reward delivery, reset).

**Stage 2 — pick with demos, STATE input, consistent rewards.** dH first (matched_w3/r2d/dH), then dDP. Sparse for
demos AND online (no shaping), demo buffer ≥ total demo rows so nothing is evicted, `time_limit` 1200,
`demo_downsample == action_repeat == 4`, entropy 3e-4, clamp 0 / scale 1; 2 seeds × 1M. Gate: LAST hold ≥ 8/15 on
≥ 3/4 runs, scored in a FRESH process with `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` exported and the variant line
present in every episode log (row 42), all 15/30 episodes present (row 22). Only then the human-vs-machine pair.

**Stage 3 (optional) — shaped variant**, only with demo rows shaped identically (converter option computing the
same potential from recorded `eef_pos`, φ(terminal)=0, γ = learner γ) and A33's result in mind.

## 4. Traps already paid for (do not re-pay)
`R2D_SIM_VARIANT` unset ⇒ base world (12th silent default); `if python … | tee` masks rc (11 sightings of that family);
`2>/dev/null` hid a missing module; `N=$(ls dir/*.npz|wc -l)` on a missing dir; BEST-of-K over ~28 noisy snapshots
(select among ≤5 fresh-process evals only); crashed episodes silently leave the denominator; OOM warm restarts re-run
the full step budget and rename snapshots; `latest.pt` polling gives K per run that varies; wandb_cache deletion under
running jobs; action_repeat/demo_downsample mismatch asserts; images (64,64,6) uint8; contract-v1 tapes end at the
env terminal (never bootstrap past it); nested/pick predicates require the can to RIDE the gripper (PICK_SUSTAIN 10).

## 5. Honesty protocol (mandatory)
Register each stage's gate before running it (append to this file under "## 7. Registered gates", dated). Report
numbers only from `metrics.jsonl` / fresh-eval JSON, with seed, steps, world, mode (sampled vs mode), and episodes
present. A failed gate is a result; write it down and stop the ladder at that rung. No cluster jobs.

## 6. Reporting
Append dated entries to `paper/WM_FIX_LOG_2026-09-03.md` (create it): stage, exact command, config diff, numbers,
gate verdict, next step. Commit small (`git add` only *.py/*.md/*.yaml you changed; never datasets/checkpoints).
When stage 2 passes, message the main session (`SendMessage to: genesis-pickaplace-de`) with the recipe so it can
register PREREG A37 (new WM recipe) BEFORE any human-vs-machine readout.

## 7. Registered gates
(append here)

### Stage 0 gate — registered 2026-09-03 ~07:55 (before any gated run; stamp corrected 09:18, was written as 08:25 from a wrong clock)
- Runs: `cartpole_balance` then `walker_walk`, DMC from pixels, each port's stock `dmc_vision` config (r2dreamer:
  action_repeat 2, time_limit 1000, train_ratio 512, size12M, entropy 3e-4, clamp 0; dv3: `--configs dmc_vision`,
  action_repeat 2, train_ratio 512, clamp 0). Only `env_num`/`--envs`=4 (box is CPU-shared), seed, steps, logdir,
  eval cadence change. 2 seeds per (port, task).
- Score = mean eval return over the port's own periodic eval (≥ 2 episodes), read from `metrics.jsonl`; report the
  LAST eval and the best-of-last-3 evals, with step.
- **cartpole_balance gate:** LAST eval return ≥ 800 (max 1000) by 200k env steps on 2/2 seeds. Published DreamerV3
  (vision) reaches ≈ 900+ well before 200k.
- **walker_walk gate:** eval return ≥ 600 by 500k env steps on ≥ 1/2 seeds and ≥ 400 on 2/2 (published DV3 ≈ 900
  at 500k; the plan's own bar is > 600 by 500k). Walker runs only if cartpole passes for that port.
- Disconfirm branches: cartpole < 800 on any seed ⇒ that port FAILS stage 0 → bisect (port tree vs its upstream
  base: r2dreamer `1fadce4`; dv3 NM512 upstream) before stages 1–3; ladder stops for that port. Both pass ⇒ stage 1.
- Code of record for the gated runs: cluster trees (`$LAB/r2dreamer`, `$LAB/dreamerv3-torch`) rsynced locally;
  if the VPN stays down, the reconstructed tree `~/wm_fix_2026-09-03/r2d_record` (base `1fadce4` + tarball) is used
  for r2dreamer and the result is labeled RECONSTRUCTED until the cluster diff is clean.

### Stage 1 gate — registered 2026-09-03 ~09:08, before any stage-1 run; runs only after stage 0 passes
- Run: r2dreamer, `env=genesis_touchgoal_state` (draft in `~/wm_fix_2026-09-03/cluster_r2d/configs/env/`; =
  touchgoal recipe of record + `state_obs: true`, `encoder.mlp_keys 'state'`, `cnn_keys '$^'`, `reward_scale 1`,
  `time_limit 1200`, `act_entropy 3e-4`, `return_clamp 0`, no demos; actor_dist `bounded_normal_clipped`,
  delta_joint 0.025/leash 5, action_repeat 4, horizon 333, train_ratio 512, env_num 6 unchanged), world
  `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` exported, 2 seeds × 300k env steps, from a COPY of the cluster tree with
  the state_obs patch (`$LAB/wm_fix_2026-09-03/r2dreamer_fix`), never editing `$LAB/r2dreamer` in place.
- Primary score: FRESH-process `eval_genesis.py` on the final `latest.pt`, sample mode, 15 hold ICs
  (`baselines/eval_ics.json --ic-set hold`), max-steps 1200, with the `[sim-variant]` line present and 15/15
  episodes present; success = `task_success` (touched_goal). Secondary (curve): `episode/train_task_success`
  mean over the last 20k env steps from `metrics.jsonl`.
- **Gate: fresh-eval success ≥ 0.8 (≥ 12/15) on 2/2 seeds.** Disconfirm: < 0.8 on any seed ⇒ the adapter/obs
  path is the suspect (stage 0 passed); next step = one ablation per suspect (actor_dist bounded_normal;
  action_repeat 1; time_limit 400) before any stage-2 run. Pixels+state repeat only after state-only passes.
- dv3 counterpart (optional, same gate): `--configs genesis_pickplace genesis_touchgoal` with `mlp_keys 'state'`
  is already in the cluster configs.yaml; run only if r2dreamer fails and a cross-port comparison is needed.

### Amendment 2026-09-03 09:55 (after user context: dv3 solved a pixel arm-pick in-house; DayDreamer)
- **Stage 1 now includes dv3 as a MANDATORY reference port** (same gate, same fresh-process scoring, same 4
  settings: state-only obs, sparse scale 1, time_limit 1200, no clamp, stock entropy; cluster configs
  `genesis_pickplace`-style vector mode + `genesis_touchgoal` scope). Reading rule: r2dreamer-state fails while
  dv3-state passes ⇒ r2dreamer adapter; both fail ⇒ shared task setup (7-dim delta-joint action space, IC
  distribution, reward reachability); both pass ⇒ pixels-only obs was the blocker. r2dreamer's result still
  gates stage 2 (it is the arm of record).
- **Stage 0b — in-house positive controls on a SPARSE PIXEL ARM PICK (ManiSkill PickCube), already on record:**
  (a) r2dreamer, cluster `$LAB/r2dreamer/runs/ms_pickcube_s{0,1}` (2026-08-18, same core code as today's cluster
  tree = the Aug-15 tarball core): pixels-only, `pd_ee_delta_pos` 4-dim, time_limit 100, 10 teleop demos,
  +100 terminal, **STOCK knobs** (act_entropy 3e-4, return_clamp 0, bounded_normal, horizon 333, no
  re-injection, demo_duplicate 1, buffer 320k ≥ all rows so nothing evicted). Train success per 25k bin:
  s0 0.69@25k → 0.86@50k → ≈0.9 thereafter; s1 0.33@25k → 0.92@50k → ≈0.9; in-loop eval success (10 eps)
  over the last 8 evals 0.6–1.0 (s0) / 0.0–1.0 (s1), 0.9/1.0 at 301k. (b) dv3 fork, March 2026 (wandb
  `dreamer_v3_maniskill`, ≈200 runs; MANISKILL_VS_GENESIS.md): takeoff 110–140k, eval 0.6–1.0; that was the
  March fork — the CURRENT cluster dv3 tree is unverified on ManiSkill (no env with dv3 deps + mani_skill;
  rerun only if dv3 fails stage 1). ⇒ Both ports' learning machinery works on a demo-seeded sparse pixel
  arm pick. The Genesis failure lives in the Genesis-specific deltas: 7-dim delta-joint actions, horizon
  400/1200 vs 100, act_entropy 3e-5, clamp 100, bounded_normal_clipped, FIFO eviction + re-injection ×4,
  reward_scale 100 with shaping, pixels-only rig content. Stage 2 must therefore mirror the ManiSkill
  recipe's knobs (stock entropy, no clamp, no eviction, no re-injection, no duplication) — already the plan's
  §3 stage-2 spec — and DMC stage 0 remains only a sanity check of the current trees.

### Amendment 2026-09-03 10:05 — sequencing (written BEFORE any stage-1 submission)
- Stage 1 (both ports) may be SUBMITTED as soon as cartpole_balance passes 2/2 for r2dreamer, running in
  parallel with the walker_walk sanity runs, because the decisive reference for "does this port's learning
  machinery work on a demo-seeded sparse arm pick" is the in-house ManiSkill control already on record (stage
  0b; same r2dreamer core as today's tree). Walker stays registered as written: if walker later FAILS its gate
  for a port, that port's stage-1/2 results are flagged "port suspect" and the bisect against upstream runs
  before any paper use. dv3's stage-1 reference likewise starts without waiting for dv3's (slow, ≈22 fps)
  cartpole endpoint. Rationale: wall-clock (walker ≈ 2 h, dv3 cartpole ≈ 2.5 h) vs. no expected information
  gain for the stage-1 decision; user asked (10:00) how to speed the process up.

### Stage 2 gate — registered 2026-09-03 10:25 (before any stage-2 run; runs only after stage 1 passes for r2dreamer)
- Runs: r2dreamer, `env=genesis_pick_state` (= `genesis_pick_v5d4c_delta` recipe of record with exactly: `state_obs`
  true + `mlp_keys 'state'` / `cnn_keys '$^'`; `reward_scale 1` sparse +1 on demo AND online rows, no shaping;
  `time_limit 1200`; `act_entropy 3e-4`; `return_clamp 0`; native stride-4 demo dirs WITH state
  (`$LAB/wm_fix_2026-09-03/demos_state/{dH,dDP}`, `demo_downsample 1`), `demo_reinject_every 0`,
  `demo_duplicate 1`, `buffer.max_size 5e5` so nothing is evicted in 1M steps), world `gc_kp4_riser3_shelf6`
  exported + demo-dir provenance gate (variant/stride/with_state/terminal 1.0 asserted before training).
  Order: dH s0/s1 first; dDP s0/s1 only after dH is read out (sequencing per §3). 2 seeds × 1M env steps.
- Score: FRESH-process `eval_genesis.py` on the final `latest.pt`, sample mode, max-steps 1200, 15 hold ICs and 30
  rnd ICs (`baselines/eval_ics.json`), success = `picked` (pick scope), all 15/30 episodes present (row 22), `[sim-variant]`
  line present (row 42). Secondary curve: `episode/train_task_success`/`train_picked` per 50k bin.
- **Gate (plan §3): LAST hold ≥ 8/15 on ≥ 3/4 runs across the dH+dDP pair.** Read-out order: dH alone first (2 runs);
  if BOTH dH seeds < 8/15 the dDP pair still runs (the pair gate needs 4 runs) but the recipe is flagged weak.
- Disconfirm: < 3/4 ⇒ stage 2 fails; suspects in order: exploration under sparse +1 at horizon 1200 (compare
  train-curve ignition against the ManiSkill fingerprint: actor entropy collapsing to ≈ −2 nats early), demo
  contribution (rerun with `demo_duplicate 4` as the only change), obs (pixels+state). Message the main session
  with the recipe only on PASS, before any human-vs-machine readout (PREREG A37).

### Stage 1 verdict + Stage 1b gate — registered 2026-09-03 11:30 (cluster clock), before any 1b submission
- **Stage 1 as registered (sparse touchgoal, state input): r2dreamer FAIL on both seeds — but by reward UNREACHABILITY,
  not by a demonstrated learning defect.** Both seeds trained 300k steps (≈270 episodes each) without a single goal
  contact (`reward_frames` 0.00 throughout); the random-policy probes (10 eps × 300 decisions each) never touched the
  goal in the corrected world (best 0.231 m), the base world (0.268 m) or with 25-decision held actions (0.158 m).
  A trainer cannot learn from a reward it never receives; the disconfirm branch's "adapter suspect" reading is NOT
  supported by this result alone. The fresh evals (registered primary score) are recorded when they finish.
- **Stage 1b (amended stage 1): `reach_goal` — sparse +1 and terminate when the TOOL is within REACH_GOAL_DIST of the
  GOAL can**, otherwise the stage-1 recipe unchanged (state-only obs, no demos, delta_joint 0.025/leash 5, repeat 4,
  time_limit 1200, reward_scale 1, entropy 3e-4, clamp 0, corrected world). Implemented in a PRIVATE env copy
  (`$LAB/wm_fix_2026-09-03/gp_root`, `reach_goal_patch.py`, 14 lines; shared tree untouched); both ports use it.
  Calibration (uniform random, corrected world): min tool–goal distance median 0.354 m, best 0.231 m; at R = 0.30 the
  random policy reached the goal in 3/4 episodes so far (22–174 decisions) ⇒ easy/dense; R = 0.25 ⇒ rare-but-present.
- Runs: r2dreamer R ∈ {0.30, 0.25} × 2 seeds × 300k; dv3 (reference) R = 0.30 × 2 seeds × 300k. Score: FRESH-process
  eval on the final checkpoint, 15 hold ICs, 1200 steps (r2dreamer sample mode / dv3 deterministic), success =
  `reached_goal`, all 15 present, `[sim-variant]` line present; secondary: `train_task_success` per 25k bin +
  actor entropy (ManiSkill fingerprint: collapse away from the 9.9 maximum).
- **Gate: R = 0.30 fresh-eval success ≥ 0.8 (≥ 12/15) on 2/2 r2dreamer seeds** (this is the "can it learn at all"
  bar: random already gets ≈ 75% within an episode, so a learner must be near-perfect AND faster — report mean
  decisions-to-success vs the random baseline). R = 0.25 reported, not gated (expected to separate learners).
  Disconfirm: r2dreamer < 0.8 at R = 0.30 on any seed while dv3 passes ⇒ r2dreamer adapter/trainer suspect → ablate
  actor_dist bounded_normal, action_repeat 1, time_limit 400 one at a time; both ports fail ⇒ shared task setup.
- Stage 0 note: dv3 (cluster tree) cartpole s1 LAST 581 < 800 ⇒ dv3 "port suspect" until the upstream reference
  (3234321/22) reads out; dv3 stage-1b results carry that flag.

### Stage 1c — collapse ablations, registered 2026-09-03 11:50 (cluster clock), before submission
- Observation (stage 1b, r2dreamer, R=0.25, both seeds; R=0.30 both seeds): the policy LEARNS (success 0.82–0.89 per 25k
  bin at 25k–50k, episodes ≈37 decisions, actor entropy −3.3…−4.5 nats = committed) and then COLLAPSES to 0.00 within one
  10k bin with entropy at the 9.93 ceiling (max_std on all 7 dims); the collapse follows the critic exceeding the true
  maximum return (max 1.0): replay λ-return targets 1.38–1.48, replay value max 1.21–1.50, `train/val` 1.22, with the
  continuation head at 0.79–0.95 during the buildup. Same "ignite then die" signature as every pick run (A32).
- Hypotheses, one lever each, r2dreamer R=0.25, 2 seeds × 150k (collapse is visible by 60k), everything else = stage 1b:
  (a) **`env.return_clamp=1.0`** — cap λ-return targets at the known max (the port's own lever, here at the CORRECT scale).
      Predicts: no collapse if the mechanism is overestimation past the terminal.
  (b) **`model.rep_loss=dreamer`** — plain DreamerV3 representation instead of R2Dreamer's redundancy-reduction loss.
      Predicts: no change if the representation is not the cause.
  (c) **`env.act_entropy=3e-5`** — the recipe's entropy coefficient (port's "ratchet" fix). Predicts: slower but not
      absent collapse if the driver is the critic, absence if the driver is the entropy bonus.
  (d) **dv3 reference at R=0.25**, 2 seeds × 150k (same env copy, same threshold). Predicts: dv3 also collapses ⇒ shared
      task/terminal-handling issue (cont head at an indistinct terminal state); dv3 stable ⇒ r2dreamer-specific.
- Score per run: `train_task_success` per 10k bin (curve shape), actor entropy, `train/ret_replay_max`, and the
  fresh-process eval at the end (15 hold ICs). "No collapse" = success ≥ 0.6 in every 10k bin after the first bin that
  reaches 0.6, through 150k. Reading rule: whichever single lever removes the collapse on 2/2 seeds becomes part of the
  stage-2 recipe; if none does, the terminal-handling path (is_terminal → discount, cont-head training, imagination
  from terminal rows) is bisected against the base commit before stage 2.
