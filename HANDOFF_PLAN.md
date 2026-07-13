# HANDOFF PLAN — genesis_pickaplace (written 2026-07-12, for the next agent session)

Read `CLAUDE.md` (Agent Status) and `CAN_STARTING_POSITION.md` (full findings log) first.
This file is the actionable queue. The user has granted STANDING AUTONOMY: proceed
without asking permission, EXCEPT destructive/irreversible actions. Two standing rules:
1. **Every trained policy gets eval videos** (`baselines/render_policy_rollout.py`,
   send samples to the user).
2. All numbers follow the honesty protocol: frozen/declared ICs, corrected metric
   (contact AND nested require picked), x3 reps, negative control where applicable,
   pre-registered predictions for any new lever.

## 0. Environment cheat-sheet
- Sim venv (genesis 0.2.1, CPU-fast): `~/workspace/genesis_sim2real/venv/bin/python`
- Combined eval venv (genesis + lerobot + torch2.8 + SB3): `.venv-g12` is genesis 1.2.1
  (FAILED gate, don't use for physics), **`.venv-eval`** = genesis 0.2.1 + lerobot (use this).
- lerobot venv (training): `~/workspace/lerobot/.venv/bin/lerobot-train`
- Engine verdict: **genesis 0.2.1 only** (1.2.1 fails the pinch-grasp gate — ~3.4x lower
  contact force at equal penetration; 12 configs eliminated; see CAN_STARTING_POSITION.md).
- World: substeps=8, table@0.05, can 101x33mm rho=1000, finger kp=100/±50 (world config
  is stamped in `can_pos_recovery/trial_placements.json` -> 'world').
- KNOWN FOOTGUNS (all bitten at least once):
  - `pkill -f X` where X appears in your own command line self-kills the shell.
  - Never embed state mutations (sed on configs) inside long background chains — if the
    chain dies early the mutation silently never happens (v3 once trained on v1 data).
  - Genesis solver can HANG a process forever: EVERY batch/collection loop runs per-trial
    under `timeout` (see `baselines/collect_v4_wrapper.sh`, `can_pos_recovery/run_sweep.sh`).
  - One `gs.init` per process. Background bash tasks die with the session — everything
    important checkpoints to disk incrementally and is resumable.
  - SB3/lerobot getter masks need sorted plain-int dof indices.

## 1. IN-FLIGHT at handoff (check + resume in this order)
a. **DP-v3 randomized-IC eval** — `baselines/eval_v3_random.log` (was ~9/50).
   If dead: rerun `.venv-eval/bin/python baselines/eval_policy.py
   baselines/outputs/dp_v3/checkpoints/last/pretrained_model --random 50 --seed 0`.
   Then COMPLETE THE v3 TABLE (in-dist final: picked 0.49 placed 0.37 contact 0.33
   nested 0.05; funnel P(place|pick)=0.76, P(slide|place)=0.90 — pick is the bottleneck).
b. **DP-v3 videos** (standing rule, not yet rendered): 4-6 uids incl a success + failure:
   `.venv-eval/bin/python baselines/render_policy_rollout.py
   baselines/outputs/dp_v3/checkpoints/last/pretrained_model 232 243 254 273 305 318`
   → send samples to user.
c. **v4 chain** — collect (grip-effort obs, 17-dim) → convert → train:
   log `baselines/collect_v4.log`, then `train_v4.log`, output `baselines/outputs/dp_v4`.
   If the chain died: `baselines/collect_v4_wrapper.sh <remaining solved success uids>`
   (it skips existing npz), then
   `~/workspace/lerobot/.venv/bin/python baselines/convert_to_lerobot.py
    baselines/episodes_raw_v4 baselines/lerobot_dataset_v4/genesis_pickaplace 8`
   then lerobot-train (same args as train_v3, output_dir dp_v4).
   When trained: eval in-dist x3 + random 50 + videos. KEY QUESTION: does grip-effort
   move pick above ~0.49? (v3 proved downstream is nearly solved; pick is stuck at data
   -to-policy 2x gap.)
d. **SACfD build agent** — `baselines/rl/{pick_env,demo_buffer,train_sacfd}.py` exist;
   smoke verification was in flight. Verify yourself:
   `.venv-eval/bin/python baselines/rl/train_sacfd.py --smoke`
   Then launch full training when GPU is free (after dp_v4 train):
   `.venv-eval/bin/python baselines/rl/train_sacfd.py --full` (200k steps, checkpoints
   in baselines/rl/checkpoints/). Sparse pick reward; demos = ALL episodes reward-
   relabeled for pick (SACfD has no BC target — user decision). Eval: pick success rate
   over solved-trial ICs + randomized ICs; VIDEOS (write a small renderer wrapping
   PickOnlyEnv with the camera env if needed).

## 2. PRIORITIZED QUEUE after in-flight completes
1. **v4 verdict + three-way table** (v1 / v3 / v4 / replay floor+teacher, both eval
   protocols, funnel per stage). Decision rule: if v4 pick > 0.6 → grip-effort worked,
   iterate obs richness (add velocities). If pick still ~0.5 → data quantity/coverage is
   binding → priority to item 3.
2. **SACfD pick result**: does off-policy RL beat BC on the pick stage (target: >0.8
   pick from solved-trial ICs)? If yes, consider SACfD-style place/slide stages or
   RL fine-tune of DP (IL panel route 5).
3. **Real-demo BC (dataset v5, the IL panel's route 2)** — the biggest expected win:
   train on ALL 75 real demo trajectories from the bags (40Hz, human's closed-loop
   grasp corrections) instead of sim-verified replays (35).
   - Extract per-trial joint states + gripper (+current) from bags:
     `can_pos_recovery/recover_grasp_current.py extract` pattern (rosbags lib needs its
     own venv/PYTHONPATH: `pip install --target <scratch>/pylibs rosbags`, numpy 2.x OK
     there, keep OUT of genesis env).
   - Relabel privileged can pose kinematically: recovered IC until FK grasp instant
     (`fk_recovered.json` grasp_idx), rigid-attach to fingertip-mid during carry,
     release point after. Gate messy demos with the FK confidence flags.
   - Mix with sim-verified v4 set; train; eval as always.
4. **Vision goal ground-truth (task #12, still open, HIGH VALUE)** — bottom-up camera
   (video0) sees the goal can through the translucent shelf; top camera (video4) the
   start. Hough prototype WORKS (see scratchpad history; cv2.HoughCircles on frame 0).
   No factory calibration exists — fit pixel->table homography from the 75 FK-recovered
   can positions. Deliverables: (a) is the goal truly static? (b) its true position →
   re-run static-goal coverage; possibly legitimizes some of the 30 relocated-goal
   trials → honest coverage may rise from 22/75 toward ~35-40/75.
   All 224 bags are local: `inthewild_trials/raw/user_*/`.
5. **DP size ablation** (cheap, informative): 248.7M params for 35k frames is ~7000
   params/frame. Try down_dims halved via lerobot policy config; expect better
   randomized-IC generalization.
6. **Unlabeled trials 200-231** (~130 recordings, same rig): auto-label via the carry
   detector (closure that lifts + releases near goal = success) + vision; spot-check
   videos on a sample; could double the demo pool for v5.
7. **task #13 leftovers**: CPU-revalidate the 52 `ok_batch` winners (≥10 reps, write
   pass/fail back — merge_and_validate.validate currently only prints); demote failures
   in trial_placements.json.
8. **Cluster prep** (user's endgame: BC + RL at scale): `can_pos_recovery/cluster_smoke.py`
   is the node acceptance test (PASS/FAIL for EGL+CUDA+batched physics). Genesis
   pinned 0.2.1; vendor the editable checkout (`~/workspace/Genesis`) or pin a wheel.
   Batched GPU envs proven at B=32 (`batch_harness.py`) — the RL rollout engine.

## 2b. KEY FINDING (2026-07-13): DP-v3 OVERFITS — randomized-IC generalization regressed
DP-v3 in-dist contact 0.33 (up from v1's 0.22) BUT randomized-IC contact 0.08 (DOWN from
v1's 0.14); picked 0.49 in-dist / 0.20 random; nested 0.05 / 0.00. Cleaner+tighter data
+ 248.7M params = sharp on demo ICs, brittle off them. CONSEQUENCES for the queue:
- DP size ablation is now URGENT (was item 5): quarter-size U-Net via lerobot policy
  config (down_dims), expect better randomized generalization. Run right after v4 evals.
- v4's REAL test is the RANDOMIZED number, not in-dist. If grip-effort only helps in-dist,
  the binding lever is net-size/data-quantity, not obs richness -> go to real-demo BC (v5).
- COMPAT GOTCHA: grip-effort added to genesis_can_env._obs makes state 17-dim. The v3/v1
  checkpoints are 16-dim -> INCOMPATIBLE with the current env (eval would misalign the
  proprio/env split). v1 & v3 numbers are FROZEN/FINAL; do not re-eval them against the
  current env. v4 onward is 17-dim and matches. To re-eval v3, git-revert the _obs change
  in a separate checkout.

## 3. Key numbers (for regression checking; full context in CAN_STARTING_POSITION.md)
- Replay (open-loop, frozen-FK, static goal, corrected metric, ss=8):
  pick 0.73/run; contact cov 22/75 (per-run 0.24); nested cov 15/75; neg-control FP 2/19
  contact, ≤1/19 nested. Searched-placement replay: contact 0.57/run, nested 0.41/run
  BUT 60% of solved rely on goal relocation (40-69% FP on fail demos) — quarantined in
  example.py (goal_moved excluded).
- DP-v1 (defective data): in-dist 0.50/0.32/0.22/0.06; random 0.34/0.14/0.14/0.04.
- DP-v3 (clean labels): in-dist 0.49/0.37/0.33/0.05; random TBD (was running).
- Datasets: v1 35eps/57k frames (3 defects), v2 31/33k (release amputated), v3 35/35.5k
  (clean), v4 collecting (17-dim obs with grip effort).
- 93 trials = 16 fail-labeled + 2 stubs + 25 replay-unsolved + 15 collection-flaky + 35.

## 4. Session-hygiene expectations
- Update CLAUDE.md Agent Status before ending any session (global user rule).
- Commit early and often with honest messages; corrections are appended to
  CAN_STARTING_POSITION.md, never silently rewritten.
- The user values: independent adversarial review at milestones (two 4-agent panels so
  far — their protocol is the law of the land), pre-registered predictions, negative
  controls, and being told plainly when something was overclaimed and corrected.
