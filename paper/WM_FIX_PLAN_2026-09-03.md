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
