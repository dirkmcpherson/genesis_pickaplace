# Auditor start-up brief (2026-09-02)

For a FRESH agent (or human) doing an adversarial audit of the human-vs-machine-demonstrations paper.
Everything you need to orient is here or one hop away. Read this file, then only what §4 points you at.
Do not read CLAUDE.md's full history or the 60+ paper/*.md files; most are superseded. `paper/AUDIT_INDEX.md`
is the older (08-28) entry point and is partly stale; this file wins where they disagree.

## 0. Rules for the auditor

- **Read-only on analysis and pipeline code.** Do not edit any *.py / *.sh under baselines/, cluster/, analysis/,
  paper/figures/. Do not touch running cluster jobs (no scancel, no resubmit, no file deletion). If you need a
  computation, write a NEW script under `paper/audit_scratch/` and run it there.
- **Output:** one report `paper/AUDIT_<topic>_2026-09-02.md` (verdict up top: SOUND / SOUND WITH CAVEATS /
  FLAWED per claim; numbered findings ranked by severity; each with file:line evidence, impact on the claim,
  minimal fix). Commit only that file. Under 250 lines.
- **Be concrete and skeptical.** Past audits that helped were the ones that found the exact column, threshold,
  Hz, or shell construct that was wrong. Past audits that did not help restated the design.
- **Numbers of record** come only from artifacts: `analysis/results_table.py` output, a job's `.out`,
  `sweep/HEADLINE.txt`, `RESCORE-RESULT` / `SWEEP-HEADLINE` / `SET-RESULT` lines. Prose numbers without one of
  those behind them are unverified; say so.
- This box is a SPARSE checkout (only *.py, *.md, committed CSV/npz under paper/figures). Datasets, checkpoints
  and job logs live on the cluster (§5). If a question needs them, name the exact cluster path that would
  settle it rather than guessing.

## 1. The paper in six lines

- Question: do imitation/RL learners care whether demonstrations come from a human teleoperator or from a
  machine (a diffusion policy trained on the human demos, then rolled out)?
- Task: Kinova gen3-lite pick-and-place of a can in the Genesis simulator (0.2.1), replaying real
  in-the-wild teleop trajectories. Metric of record = PICK success on 30 fixed random initial conditions
  ("rnd"), secondary = success on the training ICs ("hold"). Seed = unit of analysis, n = 8 per arm.
- Learners: DP (lerobot diffusion policy, BC), RLPD (off-policy RL with demos in the buffer, sparse reward),
  WM = r2dreamer (return-clamped DreamerV3 torch port, shaped reward). dv3 (another DreamerV3 port) is a
  disclosed failure (critic runaway), not a headline arm.
- Arms of record: dH = raw human demos; dHpruned = human demos with idle pre-grasp time pruned; dDP = machine
  demos matched per-IC and per-N to dH; dDPpruned = machine matched to dHpruned. DP uses the pruned pair,
  RLPD and WM use the raw pair.
- World of record: the corrected simulator `gc_kp4_riser3_shelf6` ("w3"). The earlier world ("old") is
  DROPPED as of 2026-09-02 (PREREG A34); old-world numbers survive only as a one-sentence sensitivity note.
- Current headline (corrected world): RLPD human-vs-machine null (−0.02 rnd, pre-registered); DP indifferent
  with pruned data (+0.06); WM directional human benefit (+0.25, n = 8 v 8, perm p 0.13; n = 12 v 12 running).

## 2. Blocks currently producing numbers (all corrected world)

| block | what | status 09-02 |
|---|---|---|
| v2 human arms (A25/A26) | dH RLPD (g99v2fullw3, s60-67), dHpruned DP (v2fullPw3, s50-57) | RLPD re-evaluated from archived ckpts (rlpd_select_confirm.sh); DP done |
| A31 machine arms | dDP RLPD + dDPpruned DP, s50-57 | submitted 14:20 (jobs 3170397..3170426, w3 only) |
| A27 WM confirmatory | dHv2raw vs dDPv2 r2dreamer, s90-101 | queued (3170427-38) |
| A28 burstiness ablation | dDPretimed / dHsmoothed / dDPnoised WM, s110-121 | queued (3120763-68) |
| A32 WM clamp pilots | C2000 / RS1 / SPARSE-RS1 on dH, s130-139 | running (3162457-62) |
| A33 RLPD dense reward | dH vs dDP, s60-67, wave g99w3dense | running (3163620-36), ~done tonight |

## 3. Known failure families (what to look for)

The project has been bitten TEN times by "silent defaults": a wrong column, a fallback that fires without
saying so, a pipe that masks an exit code. Concrete instances, all fixed but instructive:
1. Gripper column used where an arm column was meant (idle_frac; three files in July).
2. Control mode defaulting (cartesian vs joint; delta vs absolute) at three call boundaries.
3. `N=$(ls dir/*.npz | wc -l)` on a missing dir silently exiting under pipefail; `2>/dev/null` hiding a
   missing module; `if python ... | tee` returning tee's rc (09-02, a31 build.sh).
4. Frame-rate confusion: real tapes 60 Hz bins, sim 30 Hz physics, decisions at 7.5 Hz (action_repeat 4).
5. env.step default `max_steps=1200` corrupting tapes longer than 1200 frames (July).
6. Eval metrics sent to the wrong wandb project; run-id reuse on relaunch (run-identity rule A9).
7. Purge script deleting `wandb_cache` under running jobs (31 runs died at eval; re-evaluated from ckpts).
8. Return clamp = 100 vs shaped returns 500–1000 in the WM → critic saturation → bistable collapse.
The live ledger of confounds is `paper/CONFOUNDS.md` (15 rows, each with status). Check any claim you audit
against it; if you find a confound not on it, that is a finding.

## 4. Where things are (read only what your topic needs)

**Claims and numbers**
- `paper/RESULTS_for_writing_2026-08-30.md` — living results doc (§1 DP, §2 RLPD, §3 WM incl. §3.2 clamp).
- `paper/CONFOUNDS.md` — confound ledger. `paper/PREREG_final_round_robin_2026-08-23.md` — frozen spec +
  amendments A1–A34 (A20–A34 are the 09-01/09-02 ones). `paper/figures/ADVISOR_BRIEF_2026-09-01.md` — the
  short story with figure pointers. `paper/METHODS_draft_2026-08-28.md`.
- Stats: `analysis/stats.py` (Welch + exact permutation), `analysis/bayes_triple_2026-09-01.py`
  (hierarchical Beta-Binomial), `analysis/results_table.py` (canonical table; `world_of()` decides world).

**Demo pipeline (human and machine)**
- Human sim tapes: `baselines/record_demos.py` (HumanFollower replays recorded joint commands through the
  env; DPTeacher rolls out the diffusion policy). Machine harvest: `baselines/harvest_ai_demos.py`.
- Matching: `baselines/make_matched_sets.py` (frozen blocks), `baselines/make_v2_matched.py` (v2/A31 sets,
  per-IC per-N, `--allow-short` fills from leftovers). Pruning: `baselines/make_dp_pruned.py`.
  Ablation sets: `baselines/make_ablation_sets.py`. Format converters: `baselines/convert_to_lerobot.py`
  (DP), `baselines/rl/to_dreamer_native.py` (WM). Manifests carry `content_sha256`, `N`, `sim_variant`.
- Env: `baselines/genesis_can_env.py`, `baselines/cartesian_env.py`. ICs: `baselines/ic_sampling.py`,
  `baselines/make_eval_ics_v2.py` (writes `baselines/eval_ics_v2_w3.json`, the 30 rnd ICs).

**Training / eval**
- Launchers: `cluster/sbatch_dp.sh`, `cluster/sbatch_rlpd.sh`, `cluster/sbatch_r2dreamer.sh` (+ `_pack.sh`
  for 2-per-GPU packs). Each has a DRYRUN=1 gate that prints DEMO-SHA / PROVENANCE-OK lines.
- Checkpoint selection: each launcher sweeps saved ckpts on 15 selection ICs ("sel"), picks the best, then
  confirms it on hold + rnd (`cluster/eval_sweep.sh`, `baselines/wandb_eval.py`, `baselines/eval_core.py`).
  `cluster/rlpd_select_confirm.sh` is the standalone re-eval of that stage. The checkpoint rule differs by
  learner (RLPD best-of-sweep, DP fixed 100k, WM best hold) — see ADVISOR_BRIEF §2b for the sensitivity table.
- Registry: `cluster/run_registry.py` + `cluster/RUN_REGISTRY.jsonl` (prevents silent re-execution).
- A31 chain scripts: `cluster/a31_chain/*.sh` (select → shard harvest → merge → build → submit).

**Figures / tape metrics**
- `paper/figures/make_figs_2026-08-31.py` (figs 1–5, data embedded), `make_fig6*`, `make_fig7*`, `make_fig10*`
  (stops-vs-creep), `make_fig12*` (raw vs pruned), `extract_tape_stats.py`, `extract_real_tape_stats.py`,
  `extract_speed_series.py`; `baselines/diagnostics/tape_dynamics_metrics.py`. Notes: `paper/FIGURES_2026-08-31.md`,
  `paper/WM_METRIC_2026-09-01.md`.

**Real data**
- `trial_reader.py` (ROS bag → npz; NOTE `vel_cmd` is measured /joint_states binned at 60 Hz, not a command —
  AUDIT_zero_action finding). Camera audit: `can_pos_recovery/camera_audit/README.md`.
  Demo recovery: `paper/DEMO_RECOVERY_RESULTS_2026-08-31.md`.

## 5. Cluster (read-only for you)

`ssh tufts`; `$LAB=/cluster/tufts/shortlab/jstale02`; repo `$LAB/genesis_pickaplace` (same branch);
python `$LAB/condaenv/genesis/bin/python` (NEVER conda-install into it). Non-login shells do not expand
`$LAB`; use the full path. Datasets: `baselines/matched_w3/{dH,dDP,dHv2,dHv2raw,dDPv2,dDPv2p,...}` (npz +
manifest.json; `r2d/` subdirs for WM format; `lerobot/` for DP). Checkpoints: `baselines/rl/checkpoints/rlpd_*`,
`baselines/outputs/dp_*`, r2dreamer logdirs under `$LAB/r2dreamer/logdir*`. Job logs: `rlpd_<job>.out`,
`dp_*<job>.out`, `r2d_pack_<job>.out` in the repo root; chain log `$LAB/ddpv2_chain.log`. Every cluster
command the main agent ran is in `paper/SESSION_LOG_2026-08-23_cluster.md` with git sha and exit code.

## 6. Audits already done (do not repeat; do extend)

| file | scope | headline |
|---|---|---|
| `AUDIT_zero_action_2026-09-02.md` | idle / stops-vs-creep / pruning / real-vs-sim metrics | idle gap is provenance (exact zeros impossible for DP teacher); strict-stop gap lives < 0.3 mm; fig7 "commanded" mislabel |
| `AUDIT_results_2026-08-28.md` | PREREG §11 gate-2/3 | corrected N15 numbers, run-identity rule |
| `AUDIT_sources_2026-08-23.md` | our learners vs published RLPD / DP / DreamerV3 | r2d shaping-γ mismatch, terminal guard |
| `AUDIT_silent_defaults_2026-08-17.md`, `AUDIT_rng_*`, `AUDIT_run_identity_*`, `AUDIT_normalization_*` | infra | see files |
| `CRITIQUE_decisions_2026-08-26.md` | the assistant's own judgment | six errors |
| `CRITIQUE_demo_recovery_2026-08-31.md` | demo recovery plan | 61 not 72 solved; parallax; no prereg |
| independent panel 2026-07-08 (`CAN_STARTING_POSITION.md`) | placement recovery | rigged negative control, batch-proxy winners |

## 7. Suggested targets, ranked by what would hurt most if wrong

1. **Matching integrity** (`make_v2_matched.py`, `make_matched_sets.py`): is dDP really the same ICs and N as dH?
   What does `--allow-short` fill with, and is that disclosed? Are manifests' `content_sha256` what the
   launchers gate on? Does the DP teacher's training set overlap the eval ICs (hold is in-dist by design;
   rnd must not be)?
2. **Checkpoint selection**: sel ICs are 15 of the training ICs; confirm hold/rnd are never used for
   selection anywhere (grep `--sets`, `--tag selected`). Per-learner rule differences (ADVISOR_BRIEF §2b).
3. **Eval determinism and load**: official numbers must come from the same eval harness (`eval_core.py`)
   with identical IC file, horizon 1200, and settle rules; July showed load-dependent flips.
4. **RLPD demo insertion**: 50 % demo batch, terminal-reward relabel, action_repeat 4, sparse vs dense
   (A33). Does the machine arm's demo buffer carry any hidden success signal the human arm lacks?
5. **WM shaped reward and clamp**: `r2dreamer` config `genesis_pick_v5d4c_delta_shaped`, return_clamp,
   return-scale; is the WM comparison human-vs-machine on identical reward and clamp settings?
6. **Statistics**: seed as unit, exact permutation on rnd counts, multiple comparisons across 3 learners ×
   2 metrics; Bayesian model priors; "BEST hold ≥ 8/15" ignition threshold origin.
7. **Real-vs-sim claims** (fig7): the 60 Hz / 30 Hz clock question left open by AUDIT_zero_action.
