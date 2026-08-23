# Cluster session log — 2026-08-23 (assistant via ssh; user allocation pax020 = job 2824386)

Exact commands run on the cluster, in order, with outcomes. Reconstructed from the session
transcript for entries before 08:40 EDT; appended live afterwards. Conventions: LOGIN = login
node (`ssh tufts`), NODE = pax020 (`ssh pax020`, user's interactive GPU allocation), REPO =
/cluster/tufts/shortlab/jstale02/genesis_pickaplace, ENV = `conda activate
/cluster/tufts/shortlab/jstale02/condaenv/genesis`, R2DPY = /cluster/tufts/shortlab/jstale02/r2d_venv/bin/python.
All recording/eval used `CUDA_VISIBLE_DEVICES=""` unless stated (sim on CPU). Times EDT.

| # | when | where | command (abridged where long; flags exact) | outcome |
|---|---|---|---|---|
| 1 | ~05:55 | LOGIN | `hostname; ls $LAB; cd REPO && git log -1 && git status --short` | repo at 8c97fa0, local mods RUN_REGISTRY.jsonl + sbatch_reeval_sacfd.sh |
| 2 | ~06:00 | NODE | `nproc; nvidia-smi; ls baselines/episodes_pick_phase_{dppruned,all} \| wc -l; ls ouroboros/*/gen0/dp/checkpoints/last/pretrained_model; find $LAB/r2dreamer -name "CHAMPION*"` | 16 cores, L40S; 66 / 91 demos; gen0 DP teachers present; champion ABSENT |
| 3 | 06:05 | LOGIN | `git fetch; git pull --ff-only origin 4dof-cartesian` | → 39093c3 |
| 4 | 06:07 | NODE | ENV; `python -c "import genesis, torch"`; `python baselines/make_eval_ics.py --check`; `python baselines/tests/test_record_demos_contract.py`; `python baselines/tests/test_terminal_guard.py` | 0.2.1 / 2.7.0+cu126; MATCHES; ALL OK ×2 |
| 5 | 06:11 | NODE | `python baselines/record_demos.py --teacher random --ic-mode demo --n 3 --seed 0 --outdir baselines/demos_v1/_smoke_negctl` | kept 0/3 (2 truncated, 1 tipped@7) — recorder works |
| 6 | 06:14 | NODE | `python baselines/record_demos.py --teacher human --src baselines/episodes_pick_phase_dppruned --uids 232 242 251 --outdir baselines/demos_v1/_smoke_dH` | BUG: --uids ignored (ran all), contract violation on adapter exhaustion (eef n+2) |
| 7 | 06:18 | local→LOGIN | commit 91cd7b1 (fixes); `git pull` on LOGIN | |
| 8 | 06:19 | NODE | 8× background: `python baselines/record_demos.py --teacher human --src baselines/episodes_pick_phase_dppruned --shard-idx K --shard-n 8 --seed 0 --outdir baselines/demos_v1/dH` (K=0..7; logs baselines/demos_v1/logs/dH_shard*.log) | 41 kept / 13 fail of 54 resettable |
| 9 | 06:28 | local→LOGIN | commit bde726d (`--ic-from-tape`); pull | |
| 10 | 06:33 | NODE | 4× background: `... --teacher human --src ... --uids 233 259 262 266 267 275 278 301 319 321 329 333 --ic-from-tape --shard-idx K --shard-n 4 --seed 0 --rollout-base $((100200+K*10)) --outdir baselines/demos_v1/dH` | 10 kept / 2 fail → dH = 51 files |
| 11 | 06:37 | NODE | `python baselines/record_demos.py --teacher human --merge --outdir baselines/demos_v1/dH`; census `python analysis/characterize_demo_sets.py dH=baselines/demos_v1/dH dHfails=baselines/demos_v1/dH_fails --no-cross --out baselines/demos_v1/census_dH.md` | merge under-counted (shard manifests overwritten) → fix e2b3bd2; re-merge + records reconstructed from logs (66 rollouts, 51 kept, dil p50 1.037) |
| 12 | 06:41 | NODE | 2× background retry of the 15 failed uids (`--uids 234 245 246 250 256 257 267 286 293 295 298 299 300 318 333 --ic-from-tape --shard-n 2 --rollout-base $((100300+K*20)) --outdir baselines/demos_v1/dH_retry`) | identical outcomes/dilations → failures deterministic |
| 13 | 06:25 | NODE | pilot dataset: `mkdir baselines/matched_v1_pilot/dH; ln -s demos_v1/dH/*.npz; cp manifest.json; python baselines/convert_to_lerobot.py baselines/matched_v1_pilot/dH baselines/matched_v1_pilot/dH/lerobot 8 4 none image` | 51 episodes, fps 7.5 accepted |
| 14 | 06:27 | NODE | 2× background: `ARM=dH SEED=S WAVE=pilot DEMO_ROOT=baselines/matched_v1_pilot GENESIS_PICKAPLACE_ROOT=$PWD bash cluster/sbatch_dp.sh` (S=0,1; logs baselines/outputs/logs/dp_pilot_dH_s*.log; registered in RUN_REGISTRY as job=2824386) | training on pax020 GPU; ~4 h |
| 15 | 06:34 | NODE | `R2DPY baselines/record_demos.py --teacher r2d --checkpoint $LAB/r2dreamer/runs/pick_v5d4c_delta_shaped_dH_s50/BEST_selected.pt --ic-mode demo --n 3 --mode mode --seed 0 --outdir baselines/demos_v1/_smoke_r2d` | first: ModuleNotFoundError tools → fix 3167d12; then 2/3 picked (234 lying-can) |
| 16 | 06:45 | NODE | 4× background: `R2DPY ... --teacher r2d --checkpoint .../pick_v5d4c_delta_shaped_dH_s51/BEST_selected.pt --ic-mode demo --target-kept 20 --attempts 3 --mode sample --verify --seed 0 --shard-idx K --shard-n 4 --outdir baselines/demos_v1/dR2Dprov` | 52 kept / 60 rollouts (first-attempt 0.93) |
| 17 | 06:49 | local→LOGIN | commit f6d937f (`--ic-from-tape` for model teachers); pull | |
| 18 | 06:50 | NODE | 2× background: same r2d command with `--uids <12 FK uids> --ic-from-tape --shard-n 2 --rollout-base $((100400+K*20))` | 12/12 kept (1 verify-rejected, re-rolled) → dR2Dprov = 64 |
| 19 | 06:52 | NODE | `R2DPY baselines/record_demos.py --teacher r2d --merge --outdir baselines/demos_v1/dR2Dprov`; census `dH= dR2Dprov= --nn-samples 3000 --out baselines/demos_v1/census_dH_dR2Dprov.md` | 64 files; len p50 113 vs 17 decisions |
| 20 | 06:54 | NODE | `timeout 1800 python baselines/record_demos.py --teacher dp --checkpoint ouroboros/ouro_dp_joint/gen0/dp/checkpoints/last/pretrained_model --ic-mode demo --n 5 --mode sample --seed 0 --outdir baselines/demos_v1/_smoke_dp` | first: lerobot device_processor pinned to cuda → fix 54338d0; second: >30 min no tape (CPU DDPM 6.3 s/query) → killed by timeout |
| 21 | 07:15 | LOGIN | `ARM=dH SEED=0 STEPS=3000 WAVE=smoke DEMO_ROOT=baselines/matched_v1_pilot sbatch cluster/sbatch_rlpd.sh` → job 2825395 | trained; sweeps ran max_jobs=1 and aggregation never ran (SELECT -1) → fixes 3e66398; `scancel 2825395` |
| 22 | 07:30 | NODE | timing probe: `python - <<EOF load_dp_runner(...cpu); 12 calls EOF` | 6.3 s per DDPM query on CPU |
| 23 | 07:47 | local→LOGIN | commits 5ef036e (eval_sweep GPU policy), 3634f14 (recorder dp GPU); pull | |
| 24 | 07:48 | LOGIN | `sbatch -p gpu --gres=gpu:1 -c 4 --mem 16g --time 1:00:00 -J dpsmoke --wrap "... record_demos.py --teacher dp --checkpoint ouroboros/.../gen0/... --ic-mode demo --n 5 --mode sample --seed 0 --outdir baselines/demos_v1/_smoke_dp"` → job 2825750 | 1/5 kept (gen-0 stride-1 DP degraded at hold-4; 60 s/episode) |
| 25 | 08:05 | LOGIN | `rm -rf baselines/rl/checkpoints/rlpd_smoke_dH_s0; ARM=dH SEED=0 STEPS=3000 WAVE=smoke DEMO_ROOT=baselines/matched_v1_pilot DUPLICATE_OK=smoke-rerun-after-eval_sweep-fix sbatch cluster/sbatch_rlpd.sh` → job 2825918 | full pipeline OK: 5 sweeps (max_jobs 4), confirmation, SWEEP-HEADLINE 0/15 (expected at 3k) |
| 26 | 08:35 | NODE | 6× background: `python baselines/wandb_eval.py --kind dp --checkpoint baselines/outputs/dp_pilot/dH_DP_s0/checkpoints/060000/pretrained_model --action-repeat 4 --ic-file baselines/eval_ics.json --ic-set sel --ic-index k --max-steps 1200 --seed k --json /tmp/dp60/sel_k.json --no-wandb` (k=0..5) | 4/6 picked (miss: 234 lying-can, 239) |
| 27 | 08:45 | local→LOGIN | commits 58bbd5e (sbatch_record GRES), pull; `DRYRUN=1 ... bash cluster/sbatch_record.sh` | dry only |

(continued below, appended live)
