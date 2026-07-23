# Migration kit → 5950X / 3080 Ti box

Target: Ryzen 9 5950X (16C/32T), 3080 Ti 12GB, more RAM than the old box's 31GB.
Wins: ~2.5x CPU sim throughput (8–10 parallel collectors/evals instead of 3–6),
uncontended GPU for DP training, dreamer4 gets the old box back.

## 1. Hard-drive manifest (the big stuff)

Copy these directories onto the drive (~30GB total):

| path | size | why |
|---|---|---|
| `~/workspace/genesis_pickaplace/inthewild_trials/` | 26G | bags + raw videos + extracted episodes (the data) |
| `~/workspace/Genesis/` | ~1G | genesis-world 0.2.1 source (editable install target) |
| `~/workspace/lerobot/` | ~2G | lerobot 0.4.5 source (editable install target; its .venv is NOT needed — skip it) |
| `~/workspace/genesis_pickaplace/baselines/outputs/dp_pick_v2/checkpoints/last/` | ~1G | current DP checkpoint |
| `~/workspace/genesis_pickaplace/can_pos_recovery/videos_realsim/` + `videos_gallery/` | 320M | current video set (regenerable, but cheap to carry) |

Do **NOT** copy: `baselines/outputs/` wholesale (70GB of stale runs), old venvs (rebuilt fresh),
`episodes_*_goal662` archives, `videos_ss8`.

The repo itself (code + small datasets like `episodes_all` 13M, `rl/checkpoints` 165M):
rsync over the network or put on the drive too —
`rsync -avP --exclude baselines/outputs --exclude .venv-eval --exclude inthewild_trials ~/workspace/genesis_pickaplace/ <dest>/genesis_pickaplace/`

On the new box, restore to the SAME paths (`~/workspace/...`) — scripts hardcode
`/home/james/workspace/genesis_pickaplace` and the editable installs reference
`~/workspace/Genesis` / `~/workspace/lerobot`.

## 2. ONE virtual environment (verified consolidation)

`.venv-eval` already does EVERYTHING on the old box — genesis sim, SB3 SAC, lerobot
training (`lerobot-train` binary, torch 2.7.0+cu126, CUDA-verified), conversion, eval,
rendering. The old 3-venv split was historical. The other two venvs are retired:
`~/workspace/lerobot/.venv` (skip entirely) and `~/workspace/genesis_sim2real/venv`
(only for `example.py -v` viewer parity; rebuild later only if needed).

```bash
cd ~/workspace/genesis_pickaplace
python3.10 -m venv .venv-eval                      # Python 3.10 (old box: 3.10.12)
.venv-eval/bin/pip install -U pip
.venv-eval/bin/pip install -e ~/workspace/Genesis   # genesis-world 0.2.1 (editable)
.venv-eval/bin/pip install -e ~/workspace/lerobot   # lerobot 0.4.5 (editable, brings lerobot-train)
# pin the rest to the old box's exact versions:
grep -vE "^(-e|genesis|lerobot)" migration_requirements.txt | .venv-eval/bin/pip install -r /dev/stdin
.venv-eval/bin/python -c "import torch; print('cuda:', torch.cuda.is_available())"   # must be True
```

`migration_requirements.txt` (186 pins, in the repo) is the freeze of the working env —
key pins: torch 2.7.0+cu126, taichi 1.7.4, numpy 2.2.6, stable_baselines3 2.8.0,
gymnasium 1.2.3, rosbags 0.11.3. cu126 wheels drive a 3080 Ti (sm_86) fine.

Where old scripts reference `/home/james/workspace/lerobot/.venv/bin/...` (run_v6_train.sh,
run_dp_pick.sh, night2.sh), substitute `.venv-eval/bin/...` — same binaries now. New scripts
should use `.venv-eval` only.

## 3. RE-BASELINE VALIDATION (mandatory before trusting any cross-machine number)

This stack flips borderline grasps on mm-scale numeric differences (precedents: GPU→CPU
transfer cliff; the 11-demo replay-vs-env path gap). A different CPU/BLAS can shift
results. Run all three gates; if any fails tolerance, keep before/after comparisons
same-machine.

```bash
cd ~/workspace/genesis_pickaplace
# GATE 1 — human-validated nested anchors (must be EXACTLY True, both):
.venv-eval/bin/python can_pos_recovery/goal_nested_fit.py --uid 233 --goal 0.672 -0.221
.venv-eval/bin/python can_pos_recovery/goal_nested_fit.py --uid 242 --goal 0.672 -0.221
#   EXPECT: NESTED=True (233 center-dist ~0.066, 242 ~0.078)

# GATE 2 — 25-demo remeasure subset @ south goal (~20min on the 5950X):
.venv-eval/bin/python can_pos_recovery/remeasure_contact.py --label success --every 3
#   EXPECT (old-box): picked 20/25  placed 15/25  contact 5/25  nested 5/25
#   TOLERANCE: +/-2 per stage. More => physics shifted on this hardware.

# GATE 3 — policy-eval reproducibility (same ckpt, same seed => same ICs):
.venv-eval/bin/python baselines/rl/eval_sac.py \
    baselines/rl/checkpoints/sacfd_all_v2/sacfd_final.zip --random 15 --seed 0
#   EXPECT (old-box): picked 0.40 placed 0.27 contact 0.00 nested 0.00
#   TOLERANCE: +/-0.13 on picked (2 rollouts of 15).
```

## 4. New-box operating notes

- Parallel sim procs: start at 8–10 (each Genesis CPU proc ~1.5–2GB RAM). Collection of
  91 demos: ~6h -> ~2.5h expected.
- gs.init once per process; never rebuild a world in-process.
- Detached jobs: `setsid nohup ... </dev/null &`. Never `pkill -f` a pattern that matches
  your own command line.
- lerobot-train batch_size=64 fits in 12GB easily; expect > the old box's ~6.1 it/s.
- First jobs for this box: (1) env-vs-replay stepping fix + verification re-collect
  (worth ~11 pick demos); (2) re-search pass over residual no-picks under the current
  world (drag/fallen seed classes); (3) parallelized comparison matrix (3 algos x 3 seeds).

## 5. State snapshot at migration time (2026-07-20)

- Goal (FINAL, human-validated): (0.672, -0.221). Nested = proximity (<=0.081m center
  dist) + picked + both upright + 100-step settle. Anchor demos: 233, 242.
- Replay @ south, 75 success: picked 61 / placed 47 / contact 15 / nested 16 (0.21).
  Fail neg-control: nested 2/16 (12.5% FP — thin separation, watch it).
- Champions (all random-IC, 3 seeds): pick-SACfD `sacfd_all_resume/sacfd_325000_steps.zip`
  (picked 0.67, old-goal era); `sacfd_all_v2/sacfd_final.zip` (south goal: picked 0.55,
  seeds 0.40/0.33/0.93); `outputs/dp_pick_v2` (south goal: picked 0.33, contact 0.11,
  **nested 0.07 — FIRST policy to complete the full task on random ICs**; seeds 0.13/0/0.07
  nested). Pattern: RL leads pick, DP leads downstream stages.
- Open threads: env-vs-replay stepping gap (11 pick demos, one-sided). TRACE VERDICT
  (uid 300): commands + worlds identical, yet the ARM diverges 17mm in FREE SPACE
  (eef >5mm @cmd 1672, can untouched until 2708; env path never picks). Prime suspect:
  env._obs() per-step reads (get_contacts / get_dofs_control_force) perturbing genesis
  0.2.1 solver state. NEXT TEST (new-box job #1): rerun trace_env with obs reads
  disabled — if it matches replay, batch/defer the reads and re-collect (+~11 demos).
  Trace scripts: scratchpad trace_replay.py / trace_env.py (copy to repo if needed).
- Full history: CLAUDE.md Agent Status + CAN_STARTING_POSITION.md + baselines/NIGHT2_REPORT.txt.
