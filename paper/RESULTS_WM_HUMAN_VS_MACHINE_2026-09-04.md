# Human vs machine demonstrations, three learners, one world — what was run and what it shows

> **Terminology — "mode" vs "deterministic" (corrected 2026-09-07).** `--mode mode` selects the *mode* of the action distribution rather than sampling it. For RLPD this is genuinely deterministic: the policy returns `tanh(mean)` and repeats exactly. For the world models (r2dreamer, dv3) it is **not** run-to-run deterministic, because the agent samples its stochastic latent inside `act` and there is no per-episode reseed; a cell reproduces exactly only when the whole episode sequence is replayed with the same RNG stream (verified: 0/30 differences on the §5.1 sequence check). No comparison is biased by this — both arms are evaluated identically — but "deterministic" overstates it for world-model cells, and the word is used below in that looser sense.

*Results of 2026-09-03/04. Written 2026-09-04 05:30 by the WM-fix session. Every number below comes from a fresh-process evaluation file or a cited document; nothing is from training-loop logs.*

## 1. The question and the design

Does the source of demonstrations — a human teleoperator or a trained machine policy — change how well a learner picks up the can?
Three learners are compared under one design, in the corrected simulator world (`gc_kp4_riser3_shelf6`, the "w3" world of record):

| learner | human arm | machine arm | reward | eval actions |
|---|---|---|---|---|
| **DP** — diffusion policy (lerobot), state input | **dH**, pruned human, 58 tapes | **dDP**, 58 tapes | none (behaviour cloning) | sampled |
| **RLPD** — SAC with demo batches, state input | **dHv2raw**, raw human, 66 tapes | dDP | sparse +1 on the pick | deterministic |
| **r2dreamer** — world model (DreamerV3-style port), state input | dHv2raw | dDP | sparse +1 on the pick | deterministic (also sampled) |

Why the arms differ by learner (James, 2026-09-03): the human arm should be the *authentic* raw recordings wherever a learner can use them; DP cannot (it drops to 0.237 on raw human — pruned beats raw by +0.27, RESULTS §1), so DP's human arm is the pruned set. The machine arm is the *good* machine set for everyone — the frozen dDP matched to the pruned base — because a machine set harvested from the raw base (dDPv2) is not competitive and would make the contrast trivial.

**Statistic of record:** picks on the 30 fixed random initial conditions (`rnd30`), LAST checkpoint of every run, per-seed counts, exact two-sided permutation test on the seed means. The rnd30 file is byte-identical across the two evaluation files in use (sha of the `rnd` list: `c87c17074b5f`), so it is the one cell every learner shares.

## 2. Result

| learner | human | machine (dDP) | Δ (human − machine) | exact perm p | n |
|---|---|---|---|---|---|
| DP (pruned human, sampled) | **0.520** (156/300) | **0.467** (140/300) | +0.053 | **0.123** | 10 v 10 |
| RLPD (raw human, deterministic) | **0.600** (144/240) | **0.517** (124/240) | +0.083 | **0.485** | 8 v 8 |
| r2dreamer (raw human, deterministic) | **0.617** (148/240) | **0.608** (146/240) | +0.008 | **0.875** | 8 v 8 |
| r2dreamer (raw human, sampled) | 0.613 (147/240) | 0.629 (151/240) | −0.017 | 0.641 | 8 v 8 |
| dv3 (DreamerV3-torch), state input | **0.700** (42/60) | **0.633** (38/60) | +0.067 | not testable at n = 2 v 2 | **working configuration found 2026-09-07** (§7 item 9) |

**No learner shows a detectable effect of demonstration source on random-start picks.** Point estimates lean slightly human in all three; none approaches significance. The r2dreamer null is the tight one (per-seed spread ≈ 2 picks in 30, so a true 0.10 gap would have been visible at n = 8); the RLPD null is weak (one dead seed per arm). Within-DP, pruning the human data matters far more than its source (+0.27).

r2dreamer secondary cells, n = 8 v 8, deterministic: hold-15 116/120 vs 119/120 (p 0.733); the 66 training starts of the raw set 515/528 vs 524/528 (p 0.106); all 74 solved demo starts 563/592 vs 573/592 (p 0.238). In-distribution the human arm trails by 1–2 pp, not significant. Per-seed r2dreamer counts (rnd30 deterministic): human 15, 19, 19, 19, 21, 17, 20, 18; machine 20, 18, 18, 18, 16, 19, 19, 18. Neither arm has a dead seed. Full per-run table for every evaluation set and both action modes: `paper/MORNING_TABLE_2026-09-04.md` §2.

## 3. The demonstration sets

All sets live under `$LAB/genesis_pickaplace/baselines/matched_w3/` on the cluster (`$LAB = /cluster/tufts/shortlab/jstale02`); rsync only, never git.

| set | what it is | tapes | rows (decisions, stride 4) | median / max length | manifest sha256 (first 16) |
|---|---|---|---|---|---|
| `dHv2raw` | raw human teleop recordings replayed in the corrected world, unpruned (idle time kept) | 66 | 14,389 raw frames → 14,484 transitions (with the terminal rows) | 147 / 971 decisions | `19b4bee19479e4f2` (set sha `bd77c9c3…`) |
| `dH` | the same recordings, pruned of idle time (frozen block) | 58 | 6,985 → 7,104 | 110 / 247 | `07ce3f36…` |
| `dDP` | rollouts of a DP teacher trained on pruned human demos, matched per initial condition to the pruned base (frozen block) | 58 | 7,343 → 7,476 | 120 / 236 | `109afa719d1c2cfc` (set sha `d3bf95f6…`) |

The world-model arms consume these through `demos_state/<set>/` — one `.npz` per tape plus `repeat.json` — built by
`baselines/rl/to_dreamer_native.py --with-state --repeat 4 --terminal-reward 1` (committed in this repo). `repeat.json` records the world (`gc_kp4_riser3_shelf6`), the action encoding (`delta_joint`, cap 0.025), stride 4, the 17-dim state, the terminal reward (+1 exactly once per tape, 66/66 and 58/58 rewarded), the source path and its sha. The launcher re-checks all of that before training and refuses to start on a mismatch.

## 4. What each learner ran

### 4.1 r2dreamer (new this week; the arm this document is mainly about)
Code of record = the cluster's `$LAB/r2dreamer` at commit **c25eb3b** (2026-08-15) **plus its 11 uncommitted local files** (536-line diff, captured as `paper/wm_fix_r2dreamer_base_uncommitted_2026-09-04.diff` — this is where the `return_clamp`, `act_entropy` and `actor_dist` knobs live) **plus the four files this session changed** (259-line diff, `paper/wm_fix_r2dreamer_vs_base_2026-09-04.diff`: `envs/genesis.py` state observation + sim-variant hook, `envs/__init__.py`, `eval_genesis.py` state passthrough, `demo_prefill.py` state passthrough, new `configs/env/genesis_pick_state.yaml`). The complete patched tree is frozen as `$LAB/wm_fix_2026-09-03/r2dreamer_fix_code_2026-09-04.tgz` (133 KB, sha256 `b7cfee1de8fcb70463be…`). Environment: `$LAB/r2d_venv` (the port's venv); simulator Genesis 0.2.1 through `genesis_pickaplace` (`GENESIS_PICKAPLACE_ROOT=$LAB/genesis_pickaplace`), world selected by `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6`.

The training command, verbatim from the job log (seed 2 of the human arm):
```
$LAB/r2d_venv/bin/python train.py env=genesis_pick_state seed=2 env.steps=1000000 \
  env.demo_dir=$LAB/wm_fix_2026-09-03/demos_state/dHv2raw buffer.max_size=5e5 \
  logdir=$LAB/wm_fix_2026-09-03/runs/s2_r2d_pick_state_dHv2raw_bnormclamp1ent5_s2 \
  env.actor_dist=bounded_normal env.return_clamp=1.0 env.act_entropy=3e-5
```
with `env=genesis_pick_state` setting: 17-dim state observation (MLP encoder/decoder on `state`, no pixels), scope `pick` (+1 and terminate on the hardened pick predicate, nothing else), `time_limit 1200` sim steps, `action_repeat 4` (300 decisions per episode), `delta_joint` actions with cap 0.025 and leash 5, 6 parallel CPU worlds, `train_ratio 512`, imagination `horizon 333`, `reward_scale 1`, demo prefill once (`demo_duplicate 1`, `demo_reinject_every 0`), buffer 5e5 rows on CPU. The three command-line overrides are the trainer fix (§6): stock `bounded_normal` actor instead of the port's clipped one, λ-return targets clamped at the true maximum return 1.0, actor entropy coefficient 3e-5.

Buffer accounting (verified from the runs, not assumed): the buffer holds one row per *decision*; the env-step counter advances 4 per row, so the first logged step equals rows × 4 (dH 7,104 × 4 = 28,416; dDP 7,476 × 4 = 29,904; dHv2raw 14,484 × 4 = 57,936). A 1M-step run therefore adds 250k online rows to a 500k-row buffer, and the demo rows are never evicted. Because the counter includes the prefill, the online budget is 1M − prefill: ≈942k steps for the raw-human arm vs ≈970k for the machine arm (a 3% asymmetry against the human arm, disclosed in §7).

Runs: 8 seeds per arm, seeds 0–7 for both `dHv2raw` (jobs 3244342/43, 3246152–57) and `dDP` (3240975/76, 3246158–63), one GPU each, 2.9–3.3 h per run; 2026-09-03 21:26 → 2026-09-04 03:45 (cluster clock). Every run reached 1M steps with the actor committed (entropy −5.8 … −6.1 at the save); no hung runs (one earlier pruned-human seed hung at 480k and was replaced by a fresh seed; that pair is the pilot, not this result).

### 4.2 RLPD
Frozen-block and raw-pair runs by the main session; conditions and per-seed numbers with file:line citations in `paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` §1–2. In brief: SB3 SAC subclass, UTD 10, 50/50 demo batches from an immutable demo buffer, LayerNorm critics, γ 0.99, sparse +1, `delta_joint` cap 0.025 / leash / repeat 4 (matches the WM arm), 100k decisions (400k sim steps), 17-dim state. Human arm `dHv2raw` seeds 60–67 (wave `g99v2fullw3`), machine arm frozen `dDP` seeds 40–47 (wave `g99w3`); LAST checkpoint = `ckpt_100`. Launch script `cluster/sbatch_rlpd.sh`; evaluation `cluster/eval_sweep.sh` (one fresh process per episode).

### 4.3 DP
Frozen block by the main session (`paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` §2.1): lerobot diffusion policy on the 17-dim state split 8+9, trained on absolute window-end joint targets and mapped into the `delta_joint` hold-4 MDP only at evaluation (`baselines/wandb_eval.py:298-320`), 100k gradient steps, seeds 20–29 per arm (wave `dp_w2final`), LAST = the 100k checkpoint, sampled actions (no deterministic re-score exists). Sets `matched_w3/dH/lerobot` and `matched_w3/dDP/lerobot` (58 tapes each).

## 5. Evaluation protocol (identical for the WM arm and RLPD; DP differs in action selection only)
Fresh process per checkpoint, world `gc_kp4_riser3_shelf6`, 1200 sim steps per episode, success predicate `picked` (the hardened lift predicate in `genesis_can_env.py`). Initial-condition sets:
- `rnd30` — 30 fixed random placements from `baselines/eval_ics.json` (identical list in `eval_ics_v2_w3.json`); the statistic of record.
- `hold15` — 15 held-out demo starts (`eval_ics.json`).
- `holdv2` — the 66 training starts of the raw human set (`eval_ics_v2_w3.json` `hold`); used by the DP-v2 and RLPD-raw rows.
- `alldemo` — every success-labelled demo start once (74 uids after the August recovery); WM arm only.

WM command, verbatim (hold15, sampled):
```
$LAB/r2d_venv/bin/python eval_genesis.py --checkpoint <run>/latest.pt --episodes 15 --mode sample --max-steps 1200 \
  --ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --seed 0 --out <run>/fresh_eval_hold15_sample --device cpu
```
(`--mode mode` for deterministic; `--ic-set rnd --episodes 30`; `--ic-file …/eval_ics_v2_w3.json --ic-set hold` for holdv2; `--ic-set all` for the full demo set). Each evaluation writes `metrics.json` (`picked`, `tipped`, `timeout`, `episodes`, `mode`) and per-episode records. Deterministic and sampled results agree within one episode on every run.

## 6. Why the world-model arm had to be fixed first (one paragraph; full ladder in `paper/WM_FIX_LOG_2026-09-03.md`)
The port as found learned the pick and then collapsed to zero. Two compounding defects: the critic's λ-return targets ran past the maximum attainable return (imagination bootstraps through terminals the model cannot see), flattening the actor's gradient — fixed by clamping targets at the true maximum, 1.0; and the port's `bounded_normal_clipped` actor turned the resulting entropy excursion into a permanent collapse where the stock `bounded_normal` distribution recovers — fixed by using the stock actor; a lower entropy coefficient (3e-5) removes the residual flicker. Each was isolated on a reach task by single-lever ablations (registered before running); the fixed recipe passed a registered stage-2 gate on the pruned pair 4/4 before any raw-human run was launched. The reference tasks (DMC cartpole 996/997, walker 933/937) passed unchanged, so the trainer itself was never broken.

## 7. Asymmetries that remain, disclosed rather than absorbed
1. Budget units are incommensurable: DP 100k gradient steps offline; RLPD 100k decisions; r2dreamer 250k decisions (1M sim steps) plus demo prefill. Each learner is at its own recipe-of-record budget; no common currency exists without changing the learners.
2. Set sizes: 66 raw human tapes / 14.5k rows vs 58 machine tapes / 7.5k rows (1.9×) for RLPD and r2dreamer; DP's pair is 58 vs 58. Same asymmetry inside RLPD's own row.
3. Online budget: the raw-human WM runs get ≈3% fewer online steps (prefill counts toward `env.steps`).
4. Action parametrization at training: DP absolute targets → delta MDP at eval; RLPD and r2dreamer native `delta_joint`.
5. Eval action selection: RLPD deterministic, DP sampled only, r2dreamer both — compare r2dreamer's deterministic cell with RLPD and its sampled cell with DP.
6. In-distribution cells differ by file (hold-15 vs the 66 training starts); the WM arm reports both; rnd30 is the only shared cell.
7. Eval process: DP/RLPD one process per episode; WM one process per IC set with in-process resets. Same predicate, horizon, world, state content.
8. RLPD carries one dead seed per arm (dHv2raw s65 2/30, dDP s45 0/30); with them excluded the arms sit at 0.68–0.70 vs 0.68–0.74. r2dreamer has none.
9. dv3 — **superseded 2026-09-07: a working configuration now exists.** The earlier statement ("every lever tried on the reach proxy is bimodal across seeds — baseline 1 & 0 of 15; fp32 15 & 0; end-effector actions 11 & 3; both 4 & 15 — no pick run exists, so it is excluded as 'no working configuration'") described the PIXEL reach proxy. Given the same state input and the same return clamp that fixed r2dreamer, dv3 learns the pick: gate G2 passed on both human seeds at 0.700 on the out-of-distribution set (registered threshold 0.5), with in-distribution 15/15, 14/15, 15/15, 15/15 across the four runs. Partial comparison at n = 2 v 2: human 0.700 (42/60) vs machine 0.633 (38/60) — directional only, far too small to test; the 4-v-4 completion is running. **Why this matters beyond dv3:** the clamp diagnosis reproduces on an independent port, so it is a property of the reward scale and not of one implementation, and the world-model arm is no longer a single-port result. See `DV3_DEBUG_2026-09-05.md` §8.
10. The r2dreamer recipe was fixed on the pruned human and machine sets; the raw human set played no role in tuning. The human-vs-machine interpretation still awaits the main session's pre-registration (A37).

## 8. Reproducing the r2dreamer arm
1. Cluster: `ssh` to the login node; `LAB=/cluster/tufts/shortlab/jstale02`. Unpack `$LAB/wm_fix_2026-09-03/r2dreamer_fix_code_2026-09-04.tgz` (or apply the two diffs in `paper/` to `$LAB/r2dreamer` at c25eb3b). Use `$LAB/r2d_venv`.
2. Demos: `python baselines/rl/to_dreamer_native.py --with-state --repeat 4 --terminal-reward 1 --src $LAB/genesis_pickaplace/baselines/matched_w3/<dHv2raw|dDP> --dst <out>/demos_state/<set>` (check `repeat.json` against §3).
3. Train: the §4.1 command with `seed=<0..7>` and `env.demo_dir` pointing at the set; exported `GENESIS_PICKAPLACE_ROOT`, `R2D_SIM_VARIANT=gc_kp4_riser3_shelf6`, `MUJOCO_GL=egl`, and a per-job `TORCHINDUCTOR_CACHE_DIR` (two jobs sharing one cache on a node race). The launcher of record is `$LAB/wm_fix_2026-09-03/wmfix_s2.sbatch` (`TAG=bnormclamp1ent5 sbatch wmfix_s2.sbatch <set> <seed> 1000000 env.actor_dist=bounded_normal env.return_clamp=1.0 env.act_entropy=3e-5`), which also runs the demo gate and all six evaluations at the end.
4. Evaluate: the §5 commands on `<run>/latest.pt`; post-hoc `holdv2` via `wmfix_eval.sbatch <run> holdv2 1 <mode> 0`.
5. Statistics: `python3 $LAB/wm_fix_2026-09-03/morning_table.py runs` prints per-run cells, per-arm totals and the exact permutation tests (the test is 40 lines of pure Python in that file; it reproduces the audit's recorded p = 0.983 for the RLPD frozen pair).
Expected wall time: ≈3 h per run on one GPU with 6 CPU worlds; 16 runs fit on the cluster in one night.

## 9. Provenance
- WM runs: `$LAB/wm_fix_2026-09-03/runs/s2_r2d_pick_state_<set>_bnormclamp1ent5_s<seed>/` (metrics.jsonl, latest.pt, fresh_eval_*/metrics.json), slurm logs `slurm/wmfix_s2_<job>.out`, command log `COMMANDS.log`.
- Registrations, gates and the dated log: `paper/WM_FIX_PLAN_2026-09-03.md` §7 (stage 3 registered 2026-09-03 23:05, before submission: statistic, n, and the prediction "|Δ| < 0.10", which was met) and `paper/WM_FIX_LOG_2026-09-03.md`.
- DP/RLPD provenance: `paper/CROSS_LEARNER_CONDITIONS_2026-09-03.md` (registry rows, sidecars, HEADLINE files, seeds, waves).
- Cross-learner table: `paper/MORNING_TABLE_2026-09-04.md`.
- Repository commits: this branch (`4dof-cartesian`), 2026-09-03 07:30 → 2026-09-04 05:00.

## 10. What is not claimed
Not a ranking of learners (budgets and eval modes differ). Not an H4 verdict (pre-registration pending). Not a statement about the old world (where RLPD showed +0.21; that effect did not survive the corrected world and is not revisited here). Not a dv3 result.

*Rendered copy (same content, private link): https://claude.ai/code/artifact/332dca67-a755-477b-b40f-700f18bcba79*

## 11. Addenda (2026-09-04 evening)
- **Broader random-IC retest (300 placements):** dHv2raw 0.602, dDP 0.618 (deterministic, 8 v 8, Δ −0.017, p 0.546; 0.597 / 0.619 sampled) — within 0.02 of the
  30-placement estimates; the null stands. One machine cell counts a deterministic simulator stall on one placement as a failure.
- **All-data human arm `dHv2all`** (66 successes + 40 non-picking/failed recordings): r2dreamer rnd30 0.588 vs 0.617 raw (p 0.58), rnd300
  0.593 vs 0.602, in-distribution cells 0.92–0.94 vs 0.95–0.98; RLPD 0.554 vs 0.600 (one dead seed per arm). The registered "world
  models gain from human failures" prediction was not met. Details: `paper/MORNING_TABLE_2026-09-04.md` §5.
- **Per-phase (place) comparison:** `paper/PHASE_RESULTS_2026-09-05.md`.
