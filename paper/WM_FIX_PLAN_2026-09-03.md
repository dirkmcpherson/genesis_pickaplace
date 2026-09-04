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
(09-03 additions) two jobs compiling on ONE node with a shared torch-inductor cache die at the first update (`InductorError SubprocException`) — export a per-job `TORCHINDUCTOR_CACHE_DIR`; `pkill -f <pattern>` over ssh kills the shell whose own command line contains the pattern; `dm_control` has no `__version__` (use importlib.metadata); the r2dreamer evaluator's IC-reset helpers bypass `adapter.reset` and must add every obs key the training used; dv3 counts `steps` before action_repeat (runs past the nominal budget) — read the last eval AT OR BEFORE the registered budget. Evaluators must run in the TRAINING env copy with the same env vars (a scope the env does not know fails SILENTLY: no terminal, all timeouts — six evals invalidated 09-03 12:55); a checkpoint's fresh eval can disagree with its own training episodes on one seed (clamp1 s0: train 0.95, eval 3/15) — always run the warm-restart check before trusting either number. In this trainer `train_ratio` LARGER means MORE updates (`Every(batch_steps/train_ratio·action_repeat)`). `episode/train_*` success is an UPPER BOUND on frozen performance: the actor is updated every 2 decisions on a buffer holding the current episode, so it adapts within the episode (clamp1ent5 s1: 0.99 in the loop, 0.62 frozen, 0.6 in the evaluator) — only fresh-process evals of frozen checkpoints count; report sample AND mode. A Genesis spawn worker can HANG mid-run (one worker spinning, parent in `unix_stream_data_wait`, no traceback, job still `R`): watch for a frozen `metrics.jsonl` mtime; the run is lost from that point — replace the seed and disclose any warm restart (`RESUME_FROM=` in wmfix_s2.sbatch).

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
- (e) **distinct terminal — `state_extra: tool_goal`** (registered 12:10 cluster clock, before submission): the 17-dim state
  carries no tool position, so "reached" is a smooth boundary for the cont/reward heads; append [tool xyz, tool–goal
  distance] (21-dim, adapter-side, opt-in; eval passes it through). r2dreamer R=0.25, 2 seeds × 150k, tag `tg`.
  Predicts: if the collapse is head smoothness at an indistinct terminal, `tg` learns and STAYS ≥ 0.6; if it still
  collapses, the leak is in the imagination/bootstrap path itself (clamp1 is then the only remaining lever).
  Paper relevance: the pick terminal (sustained 10-step hold above pick_z) is not a function of the state either — if
  (e) holds, stage 2 must give the WM a state-function terminal (or a hold-counter feature) before it can be fair.
- Interim 1c readings (12:07): ent3e5 s1 exploded to 9.92 at the 25k bin after all (0.75 → falling); repdreamer s1
  0.86 at 25k–50k, entropy −5.0 (learning; collapse pending); dv3 R=0.30 s0 first successes (25k bin 4/4).

### Stage 2 amendment — registered 2026-09-03 12:20 (cluster clock), before any stage-2 submission
- Stage 1c reading (12:17): `env.return_clamp=1.0` (= the known max return under sparse +1, reward_scale 1) is the single
  lever that removes the learn-then-collapse on 2/2 seeds (0.94–0.98 across consecutive bins, entropy ≈ −3.7); base,
  ent3e5 and repdreamer all collapse on 2/2. Submission of stage 2 waits for the clamp1 runs' 150k endpoints + fresh
  evals (registered "no collapse through 150k" criterion), expected ≈ 12:40.
- Stage 2 runs (dH first, as registered): **clamp1 = `env=genesis_pick_state env.return_clamp=1.0`** (PRIMARY, tag
  `clamp1`, 2 seeds × 1M) AND **clamp0 = the originally registered stock setting** (CONTROL, 2 seeds × 1M), same demo
  set `demos_state/dH`, same everything else. Max return in pick scope = 1.0 (single +1 at the hardened pick terminal,
  `to_dreamer_native` normalises the double-grant rows to 1). dDP then runs with whichever setting passes the dH read-out
  (both if both pass; the human-vs-machine pair is scored per setting).
- Gate unchanged (LAST hold ≥ 8/15 on ≥ 3/4 across the dH+dDP pair, fresh process, corrected world, 15/30 episodes).
  Additional pre-stated reading: if clamp1 passes and clamp0 fails on dH, the paper's WM arm of record (clamp 100 under
  shaped ×100 returns, i.e. mis-set) is confounded by the clamp scale and the corrected recipe is
  `sparse + reward_scale 1 + return_clamp 1 + state obs + no eviction/re-injection`; PREREG A37 message to the main
  session carries exactly that.
- The `tg` (distinct-terminal) diagnostic continues in parallel; its result informs whether the pick terminal
  definition (sustained 10-step hold) needs a state-function form, but does not gate stage 2.

### Stage 0 bisect for dv3 — registered 2026-09-03 12:35 (cluster clock), before submission
- Finding: the cluster dv3 fork's `defaults` differ from upstream 6ef8646 in learning-relevant ways: `batch_size 32`
  (upstream 16), `batch_length 96` (64) ⇒ at `train_ratio 512` the fork does ONE update per 6 env steps vs upstream's
  one per 2 (3× fewer updates per step); `precision 16` (32); `eval_action_mode 'sample'` (upstream evaluates the
  mode); plus `dyn_gru_blocks 8` and other fork-only keys. Upstream reached 797/925 at 65k where the fork sat near
  270–380; the fork's 709 at 205k is ≈ the 3×-slower curve.
- Run: fork tree, `--configs dmc_vision nowandb --batch_size 16 --batch_length 64 --precision 32 --eval_action_mode mode`,
  cartpole, 2 seeds × 200k, tag `updefaults`. Gate: same as stage 0 (LAST ≥ 800 by 200k on 2/2).
  Reading: PASS ⇒ the fork's CODE is fine and its DEFAULT RECIPE is the deficit (every dv3 genesis run used the fork's
  recipe knobs — msrecipe restored 16×64 but not precision/eval mode); FAIL ⇒ a code difference remains (bisect the
  633-line dreamer.py diff next).

### Stage 1c (f) + Stage 2 third arm — registered 2026-09-03 13:15 (cluster clock), before submission
- Finding (clamp1 end-of-run traces, 5k rows): BOTH seeds show transient entropy blow-ups even with the clamp
  (s0: 9.93 @111.6k, 7.15 @136.6k; s1: 9.92 @136.6k, 9.78 @141.6k), each recovering within 5–10k steps to ≈ −4 with
  `ret_replay_max` pinned at 1.0. The 25k-bin success (0.97–1.00) averages over flickers; s0's end-of-run save
  (150k) fell inside an unlogged window after its last row (146.6k) — consistent with its fresh evals (3/15, 0/15,
  1/15) while s1's save fell in a good phase (15/15 ×3). ⇒ the clamp turns permanent collapse into recoverable
  flicker; with imagined returns saturated at the cap, the actor's return gradient vanishes near the goal and the
  entropy bonus (3e-4) briefly dominates.
- (f) **clamp1 + ent3e5** (`env.return_clamp=1.0 env.act_entropy=3e-5`), reach_goal R=0.25, 2 seeds × 150k, tag
  `clamp1ent5`. Predicts: no flickers (entropy stays ≤ 0 after ignition) and fresh evals ≥ 12/15 on 2/2 seeds.
  ent3e5 alone failed (critic runaway); clamp alone flickers; the combination is the untested cell.
- Stage 2 third arm: dH `clamp1ent5` (2 seeds × 1M), same everything else. Read-out rule unchanged (fresh evals);
  additionally each stage-2 run reports its entropy-flicker count (rows with entropy > 5 after ignition).
- Reporting rule for all clamp runs: fresh-eval numbers come from the END-OF-RUN checkpoint as registered, AND the
  checkpoint's phase is disclosed (entropy at the last logged row); no cherry-picking among snapshots.
- (g) **`env.actor_dist=bounded_normal`** (stock r2dreamer actor instead of the port's `bounded_normal_clipped`), reach_goal
  R=0.25, no clamp, 2 seeds × 150k, tag `bnorm` — registered 13:20 (cluster clock), before submission. Motivation:
  the dv3 reference (unclamped, same env copy) is at 0.75–1.00 success with entropy 0.3–2.0 and no collapse on 3/4 runs
  at 75–80k, while every unclamped r2dreamer run collapses ⇒ r2dreamer-specific; the clipped actor is the registered
  suspect not yet ablated (repdreamer ruled out the representation loss). Predicts: if `bnorm` learns and holds,
  the projected-sample actor is the collapse driver (its log-prob of projected tail samples enters the actor loss);
  if it collapses, the driver is elsewhere in the port's actor–critic path (ReturnEMA/advantage handling next).
- Note 13:25 (cluster clock): resume-check confirms clamp1 s0's end-of-run checkpoint is a flicker-phase policy
  (evaluator honest). Stage-2 gate unchanged (end-of-run fresh eval); disclosed alongside: entropy at the last
  logged row and the flicker count. Any FUTURE run that keeps periodic snapshots may add a phase-aware BEST-of-≤5
  fresh-eval number (CONFOUNDS row 25 discipline) — none of the currently running stage-2 jobs keeps snapshots.
- (g) result 13:43: `bnorm` (stock actor, NO clamp) learns and HOLDS with wobble — s0 0.98/0.86/0.96/0.98/1.00 per 25k
  bin to 130k (entropy −1.5 at the end), s1 0.94/0.93/0.82/0.92/0.90 (entropy excursions to 9.15 at 90k, recovers) —
  while its critic overshoots exactly like the clipped runs (val 1.05–1.29, replay targets max 2.9, value max 2.1) and
  its raw samples reach ±4.8 (env clips). ⇒ the port's `bounded_normal_clipped` converts the critic-leak episode into a
  PERMANENT collapse; the stock actor survives it. Two contributors, one trigger (critic past max) and one
  amplifier (projected-sample actor).
- (h) **`bnorm + clamp1`** (`env.actor_dist=bounded_normal env.return_clamp=1.0`) and (i) **`bnorm + clamp1 + ent3e5`**,
  reach_goal R=0.25, 2 seeds × 150k each, tags `bnormclamp1`, `bnormclamp1ent5` — registered 13:48 (cluster clock),
  before submission. Predicts: (h) ≥ (f) in stability (no wobble); (i) tightest. Stage-2 arms for the best of
  {(f), (h), (i)} by fresh eval + flicker count follow after this read-out (≈ 40 min).
- Stage-2 amendment 13:52 (cluster clock): stage-2 dH at 160–340k — clamp1 (2/2) and clamp0 (2/2) dead at entropy 9.93 with
  0 online picks; clamp1ent5 s1 IGNITING (picked 0.13/0.19/0.24 per 50k bin, entropy −5.6, val 0.74) while clamp1ent5
  s0 died (entropy 9.93 @186k). With the clipped actor even (f) is 1/2 in the pick task. Given (g) at the reach rung
  (stock actor survives the overshoot), the stage-2 arm **(i) `bnormclamp1ent5`** (`env.actor_dist=bounded_normal
  env.return_clamp=1.0 env.act_entropy=3e-5`, 2 seeds × 1M) is submitted NOW rather than after the reach read-out;
  (h) `bnormclamp1` is added as a stage-2 arm only if it beats (i) at the reach rung. Gate/scoring unchanged.
- Stage-2 amendment 14:40 (cluster clock): dDP arms submitted NOW (before the dH read-out at ≈17:00) for the two candidate
  settings — `clamp1ent5` and `bnormclamp1ent5`, 2 seeds × 1M each, same demo protocol (`demos_state/dDP`, 58 tapes,
  corrected world, provenance gate). Rationale: wall-clock (user 10:00), and the two settings are already the only
  ones alive on dH at 250–460k (0.70–0.84 online picks) while clamp1/clamp0 are dead. Read-out rule unchanged: the pair
  is scored per setting by the registered fresh evals (sample; mode disclosed); a setting whose dH runs fail the gate
  is not scored on dDP.

### Stage 1b/1c VERDICT — 2026-09-03 14:48 (cluster clock)
Reach-rung (`reach_goal` R=0.25, state input, no demos, corrected world, 150k; fresh-process evals of the end-of-run
checkpoint on 15 hold ICs, 1200 steps; random baseline 4/20 sampled):

| lever | sample-mode fresh eval (s0, s1) | mode fresh eval | verdict |
|---|---|---|---|
| base (clipped actor, no clamp) | 0/15, 0/15 (R=0.30, 300k) and 0/15, 0/15 | — | learn-then-collapse |
| ent3e5 | 0/15, 0/15 | — | collapse |
| repdreamer | 0/15, 0/15 | — | collapse |
| tg (21-dim state) | 0/15, 0/15 | — | collapse |
| clamp1 | 3/15, 15/15 | 0/15 (s0), 15/15 (s1) | holds with flickers; s0 end checkpoint bad |
| clamp1ent5 | 15/15, 9/15 | 15/15 (s1; +30/30 demo ICs) | holds, no flicker; sampled std on real states ≈0.5 |
| bnorm (stock actor, no clamp) | 15/15, 15/15 | — | survives the critic overshoot with wobble |
| (h) bnormclamp1 | 15/15, 15/15 | — | holds (train 1.00/1.00 from 75k; entropy 2–4) |
| (i) bnormclamp1ent5 | 15/15, 15/15 | — | holds (train 1.00/1.00 from 75k; entropy ≈ −5.3) |

Mechanism of record: (1) TRIGGER — λ-return targets exceed the maximum attainable return (imagination through an
indistinct terminal keeps bootstrapping/paying), critic overshoots, the actor's return gradient flattens;
(2) AMPLIFIER — the port's `bounded_normal_clipped` actor turns the resulting entropy excursion into a permanent
collapse (stock `bounded_normal` wobbles and recovers). Fixes: `return_clamp = max return` (removes the trigger),
stock actor (removes the amplifier), `act_entropy 3e-5` (suppresses the residual flicker under the clamp).
Reading of `episode/train_*`: an UPPER BOUND on frozen performance (in-episode adaptation, 0.99 vs 0.62 for
clamp1ent5 s1); only fresh-process evals count, reported in sample AND mode.
Stage-2 recipe candidates (running on dH and dDP): `clamp1ent5` (clipped actor) and `bnormclamp1ent5` (stock actor).
dv3 reference (unclamped, fork+world hook): learns on 3/4 runs with wobble (0.5–1.0 per bin), 1/4 never ignites —
fresh evals pending; the collapse-to-zero is r2dreamer-specific.
- Stage-2 note 14:52 (cluster clock) [SUPERSEDED 15:01: s1 recovered to 0.87 by 450k — a flicker, not a collapse; the dH gate for this setting is undecided until its fresh eval]: dH `clamp1ent5` collapsed on BOTH seeds (s0 ≈100k, s1 ≈400k: entropy −6.15 → 9.93
  and val 0.85 → 0.18 within 25k steps, with the critic never above the clamp) ⇒ under the registered rule that
  setting will fail the dH gate and its dDP runs (3240972/73) are reported but NOT scored for the pair. The stock-actor
  setting `bnormclamp1ent5` is the live candidate (dH 0.83 / 0.97 online at 300k, both seeds committed).
- Stage-0 dv3 verdict 15:45 (cluster clock): upstream NM512 tree PASSES cartpole (998/997 by 195k, 2/2); the cluster fork
  with its own defaults FAILS (513/581); the fork with upstream's four defaults restored is on the upstream curve
  (989/980 @173k). Reading: code OK, default recipe deficient (batch 32×96 = 3× fewer updates, fp16, sampled eval).
  The dv3 "port suspect" flag is downgraded to "recipe suspect"; dv3 genesis diagnostics inherit it + the base-world
  confound.
- Stage-2 note 17:05 (cluster clock): the dH `bnormclamp1ent5` s1 run (3238967) HUNG at 480k (Genesis worker spin) and is
  replaced by a fresh seed 2 (3241649) as the second dH run of record for the pair gate ({dH s0, dH s2, dDP s0, dDP s1});
  the warm restart of s1 from its 428k save (3241650) is DISCLOSED only and does not enter the gate. First number of
  record: dH s0 hold 15/15 (sample), 14/15 (mode), rnd 18/30 (sample).

### Stage 1d — dv3 diagnosis at the reach rung, registered 2026-09-03 18:35 (cluster clock), before submission
Observation: dv3 (fork tree + world hook, unclamped, state input) learns `reach_goal` but never consolidates — per-bin
success wobbles 0.3–1.0 on 3/4 runs, 1/4 never ignites, and the two finished checkpoints score **1/15 and 0/15** frozen
(fresh process, deterministic actor). Two candidate causes, each a single lever, 2 seeds × 150k, everything else = the
dv3 stage-1b config (state-only obs, sparse +1, R=0.25, time_limit 1200, corrected world):
- (a) **EEF action space** (`genesis_cartesian`, `genesis_cartesian_control=delta`): 5-dim ee-position deltas
  [dx,dy,dz,dpitch,grip] instead of 7 joint deltas — the analog of ManiSkill's `pd_ee_delta_pos`, which is the one
  large untested delta from MANISKILL_VS_GENESIS.md between the working March run and every Genesis run. Requires
  `reach_goal` in `CartesianFullTaskEnv` (added to the private env copy `gp_root`, 14 lines, same semantics) and the
  sim-variant hook on the adapter's cartesian branch (it was missing — the branch built the BASE world silently;
  fixed in `dv3_fix`). Tag `eef`.
- (b) **precision 32** (`--precision 32`): the fork's `precision: 16` default is one of the three deltas proven to cost
  it the DMC gate (513/581 at fp16 vs 991/985 with upstream defaults restored); every dv3 genesis run inherited it.
  Tag `fp32`.
Gate for both: fresh-process eval, 15 hold ICs, 1200 steps, `reached_goal`, deterministic actor (dv3's evaluator),
success ≥ 0.8 on 2/2 seeds; secondary = per-25k success curve and whether the wobble disappears. Reading: (a) passes
⇒ the 7-dim joint action space is the dv3 blocker (and the paper's dv3 diagnostics are confounded by it);
(b) passes ⇒ the fork's fp16 default is; both fail ⇒ neither, and the dv3 arm stays "weak at this budget" with
r2dreamer as the WM arm of record. r2dreamer EEF is NOT run here (its adapter has no cartesian path and it already
passes stage 2 with joint deltas); noted as follow-up if the EEF lever is decisive for dv3.

### Stage 2b — RAW demo pair (dHv2raw vs dDPv2) under the fixed recipe, registered 2026-09-03 20:20, before submission
- Motivation (user, 20:15): stage 2 used the PRUNED pair `matched_w3/{dH,dDP}` (N=58). The paper's raw arms are
  `matched_w3/{dHv2raw,dDPv2}` (N=66) — dDPv2 is the machine set matched to the RAW human base, so the pair to run is
  dHv2raw vs dDPv2 (mixing dHv2raw with the pruned-matched dDP would break the matching that makes the contrast fair).
- Recipe: identical to the stage-2 winner `bnormclamp1ent5` — `env=genesis_pick_state`, `env.actor_dist=bounded_normal`,
  `env.return_clamp=1.0`, `env.act_entropy=3e-5`, state obs, sparse +1 both sides, reward_scale 1, time_limit 1200,
  no re-injection/duplication, buffer 5e5, corrected world; 2 seeds × 1M each, evals hold15+rnd30 in sample AND mode.
- Inputs to build first (CPU, minutes): `to_dreamer_native.py --with-state --repeat 4 --terminal-reward 1` from
  `matched_w3/dHv2raw` and `matched_w3/dDPv2` into `demos_state/{dHv2raw,dDPv2}`; the launcher's demo-gate asserts
  world/stride/with_state/terminal-1.0 and the tape count before training (its hardcoded `== 58` check must be
  relaxed to the set's own `n_written`).
- Gate (same as stage 2): LAST hold ≥ 8/15 on ≥ 3/4 of the four runs, fresh process, corrected world, all episodes
  present. Reporting: sample AND mode, hold15 + rnd30, plus each checkpoint's entropy at the save. The human-vs-machine
  READ-OUT still waits on PREREG A37 (main session) — this registers the RUNS, not the comparison's interpretation.
- Blocked at registration time: the VPN dropped at 20:05, so nothing is submitted; commands are staged in
  `~/wm_fix_2026-09-03/stage2b_submit.sh` and fire as soon as the tunnel is back.
- **Stage 2b CORRECTED 2026-09-03 20:25 (user):** the comparison of interest is **dDP (the good machine arm, the
  pruned-matched N=58 set already run in stage 2) vs dHv2raw (authentic, unpruned human, N=66)** — NOT dHv2raw vs
  dDPv2, because a machine arm trained from the raw base does not perform well enough to make the contrast fair.
  Consequence: only the **dHv2raw arm is new** (2 seeds × 1M, same fixed recipe); the machine side is the completed
  stage-2 dDP pair (hold 15/15 and 14/15; rnd 20/30 and 17/30, sample = mode within one episode). dDPv2 is NOT run.
  DISCLOSED asymmetry, by design: the human set is unpruned (N=66 tapes) and the machine set is the pruned-matched
  N=58 — tape and transition counts are reported next to the result (dH pruned = 6,985 rows; dHv2raw and dDP row
  counts recorded at conversion), and the contrast is "best available machine demos vs authentic human demos",
  not a size-matched pair.
- **Full demo-IC-set evaluation — registered 2026-09-03 20:30 (user):** every checkpoint of record is additionally
  scored on the FULL demo IC set (all 61 success-labeled uids, each exactly once, ascending — the training reset
  distribution), in BOTH action modes, alongside the existing rnd30 random-IC numbers. `--ic-set all` added to both
  evaluators (r2dreamer `eval_genesis.py`, dv3 `genesis_eval.py`); the stage-2 launcher now runs hold15 + rnd30 +
  alldemo61 × {sample, mode} at the end of every run. Sweep for already-finished checkpoints:
  `~/wm_fix_2026-09-03/alldemo_sweep.sh` (11 pick checkpoints + 10 reach checkpoints × 2 modes + 8 dv3 runs).
  Reporting: the three IC sets are distinct populations — hold15 ⊂ demo ICs (held out from selection), alldemo61 =
  the full training distribution, rnd30 = placements outside the demo support — and are never pooled.

### Stage 1d VERDICT + 1e registered — 2026-09-03 22:15 (cluster clock)
dv3, reach_goal R=0.25, state input, 150k, corrected world; fresh eval (deterministic actor, 15 hold ICs):

| lever | s0 | s1 | mean | note |
|---|---|---|---|---|
| baseline (fp16, 7-dim joint deltas) | 1/15 | 0/15 | 0.03 | the configuration behind every dv3 genesis number |
| (b) fp32 | **15/15** | 0/15 | 0.50 | best ceiling, worst floor — one seed produces nothing |
| (a) EEF (`genesis_cartesian delta`, 5-dim ee deltas) | **11/15** | 3/15 | 0.47 | both seeds non-zero: best floor |

Gate (≥ 0.8 on 2/2) **FAILS for both levers**; both improve massively on the baseline (0.03 → ≈0.5). They fix
different halves: fp32 raises the good seed's ceiling (1/15 → 15/15), EEF raises the bad seed's floor (0/15 → 3/15).
- **(1e) registered, before submission: EEF + fp32 together** (`genesis_cartesian` + `--genesis_cartesian_control delta`
  + `--precision 32`), 2 seeds × 150k, same gate. Predicts: if the two are complementary, ≥ 0.8 on 2/2; if not, the
  dv3 arm's problem at this budget is seed variance that no single knob removes, and the honest statement for the
  paper is that dv3 needs either more seeds or a different budget — r2dreamer stays the WM arm of record either way.
- Also registered: **dv3 has never been run on the PICK task under a fixed recipe** — every dv3 number in this ladder
  is the reach proxy. Before any claim that dv3 is (or is not) comparable to DP/RLPD, it needs the pick task with
  demos under whichever lever survives (2 seeds × 1M, `demos_state/dH`, same evals). Not submitted yet.

### Stage 3 (cross-learner comparison) — registered 2026-09-03 23:05, before submission (James's overnight directive via the meta session)
Goal: DP, r2dreamer, RLPD (and dv3 if it earns a pick run) compared like-to-like on human vs machine demonstrations.
Arms of record per learner (James): human = **dHv2raw** for RLPD / r2dreamer / dv3, **dH (pruned)** for DP (DP dies on raw
— RESULTS §1); machine = **dDP** (the frozen pruned-matched N=58 machine set) for everyone.
- **r2dreamer seed expansion to n=8 per arm**: dHv2raw s2–s7 and dDP s2–s7 (12 runs × 1M, recipe `bnormclamp1ent5`,
  same launcher/evals), joining dHv2raw s0/s1 (running) and dDP s0/s1 (done). Statistic of record for the contrast:
  LAST checkpoint, **rnd30 in MODE** (deterministic — like-for-like with DP's and RLPD's deterministic evals), n=8 vs 8,
  exact permutation test on the per-seed rnd counts (the paper's A16 statistic), plus hold15 and alldemo74 as
  secondary; SAMPLE numbers reported alongside (disclosed). Prediction registered: from n=2, dHv2raw ≈ dDP (|Δ| < 0.10);
  if RLPD's coverage mechanism generalises to WMs, dHv2raw > dDP by ≥ 0.10.
- **Confound audit (tonight, documentation only)**: for each learner record world, demo set (path, N, tape/row counts),
  obs, action space + caps, budget, eval IC file/set, predicate, horizon, action mode, n seeds, checkpoint rule; every
  mismatch goes into the morning table as a disclosed asymmetry, never silently absorbed. Known already: budgets are
  paradigm-specific (DP 100k grad steps offline; RLPD 100k env steps; r2dreamer 1M env steps) and cannot be
  equalised without changing the learners — disclosed, not matched.
- dv3 on the pick task only if stage 1e clears its gate (≥ 0.8 on 2/2 seeds); otherwise dv3 is reported on the reach
  proxy with its seed-variance verdict and excluded from the pick comparison (not silently — as a row marked
  "not run: no working configuration").
