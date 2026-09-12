# {dv3: dreamer losses in the r2 port} — local pick-scope eval vs G2 — 2026-09-11 (lane DV3-5)

Evaluates the run launched and described in `paper/DV3_LOCAL_2026-09-11.md` (lane DV3-1: audit, demo build,
smoke, launch) against the G2 bar defined in `paper/DV3_DEBUG_2026-09-05.md` §6/§8 (fresh-process eval, 15 hold
ICs + 30 rnd ICs, `time_limit 1200` = 300 decisions at `action_repeat=4`, SAMPLED actions the cells of record;
bar hold15 ≥ 14/15, rnd30 ≥ 0.5, measured 0.700 there — a different codebase, `dreamerv3-torch`, not this port).
Every number below carries the command that produced it.

**Verdict: PASS on both sampled cells of record — hold15 15/15 (1.00), rnd30 18/30 (0.60). Step 2 (e2e locally
with `ladder=nested_ramp`) is UNBLOCKED.**

## Hardware (not the class of record)

Local box: **NVIDIA GeForce RTX 3080 Ti** (12288 MiB), **AMD Ryzen 9 5950X** (16 cores / 32 threads), **AVX2**
(no AVX-512: `lscpu | grep -o 'avx[0-9_a-z]*'` -> `avx avx2` only). This is NOT the cluster's pinned 64-core
AVX-512 class of record used for the project's e2e re-scores; no claim here relies on cross-hardware
reproduction.

## 1. Training-process exit confirmation

```
pgrep -f 'env.steps=1000000'                     # -> no match (process 579773 gone)
tail -3 .../metrics.jsonl                        # max step 999978
grep -ci traceback .../console.log               # 0
stat -c '%y' .../latest.pt                       # 2026-09-11 18:42:39 (final save)
```
- Last `step` in `metrics.jsonl`: **999978** (>= 995000 gate). `console.log` has **0** Traceback lines.
  `latest.pt` timestamp 2026-09-11 18:42:39, 3.37 h after the 15:20:28 launch (`LAUNCH.txt`).
- **Final training-data reward stats** (`train/data/reward_sum`, last 20 logged rows, steps 902268-997256 —
  this is a per-training-BATCH sum over many sampled sequences from the replay buffer, not a per-episode return;
  the pick-scope task's own max return is 1.0/episode, confirmed by `ladder_provenance.json`'s
  `max_return: 1.0`): values `[21,23,21,17,19,38,31,31,17,26,28,28,32,14,16,23,42,23,17,34]`, mean **25.05**,
  population stdev **7.59**, range **[14, 42]**. Command:
  ```
  python3 -c "import json; ..." # filters lines with key 'train/data/reward_sum', last 20
  ```
- **Throughput**: wall clock 15:20:28.670 -> 18:42:39.077 = **12130.4 s (3.370 h)** for 1,000,000 configured
  `env.steps`, i.e. **82.4 steps/s** end-to-end (prefill + online + eval-logging overhead included). The
  in-loop `fps/fps` metric averaged **78.7** over its 188 logged rows (`grep -o 'fps/fps [0-9.]*' console.log`),
  consistent with the wall-clock figure. This is far above `paper/DV3_LOCAL_2026-09-11.md` §5's ESTIMATE of
  10-16 raw env-steps/s (extrapolated from a different tree's measurement on the same GPU) — that estimate is
  now superseded by this run's own measured throughput; the run finished in ~3.4 h, not the estimated 16-26 h.

## 2. Evaluation

Ran `~/workspace/r2dreamer/eval_genesis.py` exactly per `paper/DV3_LOCAL_2026-09-11.md` §6, sequentially, in the
foreground of individual tool calls (GPU was free post-training: `nvidia-smi` showed 303 MiB / 8-15 % util
throughout, so `--device cuda` was used for the agent's nets; genesis physics itself is unaffected by this
flag). `env.return_clamp`/`model.return_clamp`/`model.rep_loss`/`env.tip_guard` were NOT re-passed on the eval
command line: `eval_genesis.py` rebuilds the model and env from the run's own `.hydra/config.yaml` (which
already has `return_clamp: 1.0`, `rep_loss: dreamer`, `tip_guard: grip`, `actor_dist: bounded_normal`,
`scope: pick` baked in from training) — confirmed live in every run's `[eval]`/`[ladder]` stamp below.

**Isolation protocol actually run**: `eval_genesis.py` builds **one Genesis world in one fresh process** (never
alongside the training process — Genesis allows one world per process) and rolls out the WHOLE ic-set (15 or 30
episodes) inside that single process — this is NOT the stricter one-fresh-process-per-episode protocol used
elsewhere in this project for full-scope e2e cells (`--ic-index`-driven). It IS the protocol that produced every
G1/G2 number of record in `paper/DV3_DEBUG_2026-09-05.md` (that doc's own "fresh-process" label means "a
separate process from training", i.e. this same one-process-per-cell design — see `eval_genesis.py`'s own
docstring: "loads a checkpoint into a FRESH process ... rolls the policy out for N episodes"). Ran as documented,
no per-episode subprocess driver invoked.

### Commands (shared env, then per-cell flags)
```
cd ~/workspace/r2dreamer
export GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace
export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl PYTHONUNBUFFERED=1
RUN=/home/j/runs_dv3_local/dv3pick_dHfull_pick_local_rlDreamer_s0

.venv/bin/python eval_genesis.py --checkpoint $RUN/latest.pt --episodes 15 --mode sample --max-steps 1200 \
  --ic-file /home/j/workspace/genesis_pickaplace/baselines/eval_ics.json --ic-set hold --seed 0 --device cuda \
  --out $RUN/fresh_eval_hold15_sample
# --mode mode                          -> $RUN/fresh_eval_hold15_mode
# --ic-set rnd --episodes 30 --mode sample -> $RUN/fresh_eval_rnd30_sample
# --ic-set rnd --episodes 30 --mode mode   -> $RUN/fresh_eval_rnd30_mode
```

### Results (each cell's own console stamp + `metrics.json`)

Every cell printed the identical model/env build stamp confirming the checkpoint's own baked-in recipe:
```
[eval] action_mode=delta_joint delta_cap=0.025 state_obs=True ...
[eval] ladder='staged' from the module default (this run config predates the argument)
[eval] tip_guard='grip' from the run config
[ladder] unified-2026-09-10 | ladder=staged | picked=1 placed_v2=1 contact_push=2 slide_success=4 | max_return=8
  | terminal=slide_success+tipped | shaping=off | far_release=off | tip=tilt>60deg&grip@1f
  | full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
  | git=known-good-2026-08-27-902-g9ccabe9
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03
[eval] loaded checkpoint (missing 0, unexpected 0)
```
(`max_return=8` in this line is the `staged` ladder's own nominal full-scope ceiling — a cosmetic artifact
noted already in `paper/DV3_LOCAL_2026-09-11.md` §6; the scope-adjusted `return_clamp` actually enforced during
TRAINING was `1.0`, per `ladder_provenance.json` written at launch. This clamp does not apply during eval — `
eval_genesis.py` never reads `return_clamp`, confirmed by `grep -n return_clamp eval_genesis.py` returning
nothing; it only affects the training-time critic target.)

| cell | n | picked | tipped | timeout | mean steps | metrics.json |
|---|---|---|---|---|---|---|
| hold15 **sample** | 15 | **15/15 = 1.00** | 0/15 | 0/15 | 14 | `fresh_eval_hold15_sample/metrics.json` |
| hold15 mode | 15 | **15/15 = 1.00** | 0/15 | 0/15 | 14 | `fresh_eval_hold15_mode/metrics.json` |
| rnd30 **sample** | 30 | **18/30 = 0.60** | 10/30 (0.333) | 2/30 (0.067) | 41 | `fresh_eval_rnd30_sample/metrics.json` |
| rnd30 mode | 30 | **19/30 = 0.633** | 7/30 (0.233) | 4/30 (0.133) | 54 | `fresh_eval_rnd30_mode/metrics.json` |

All four: `restore_failed 0/N`, `ic_mode: demo` (ICs come from `baselines/eval_ics.json`'s `hold`/`rnd` arrays,
not random uid draws — confirmed per-run by the `[eval] ICs from ... set=hold|rnd -> N episode(s)` line), 100 %
of pinned entries matched on the hold15 cells (`n_enumerated: 15, n_restored_match: 15`; the rnd30 cells don't
use the pinned-entry path, `n_enumerated: 0` — expected, `rnd` ICs are not demo-derived entries). Every cell's
`metrics.json` carries the identical `ladder_provenance` block shown above plus `checkpoint`, `episodes`, `mode`,
`seed=0`, `max_steps=1200`, `scope=pick`.

Per-episode outcome lines (console) for the two rnd30 cells, for the record:
```
sample: ep1 tipped(47) ep2 tipped(48) ep5 tipped(66) ep6 tipped(173) ep8 tipped(8) ep10 tipped(13)
        ep13 timeout(300) ep16 tipped(4) ep19 tipped(2) ep24 timeout(300) ep25 tipped(1) ep26 tipped(8)
        -- the remaining 18/30 picked in 12-17 steps
mode:   ep2 tipped(2) ep5 tipped(2) ep6 timeout(300) ep8 tipped(8) ep12 tipped(136) ep13 timeout(300)
        ep16 tipped(4) ep19 timeout(300) ep24 timeout(300) ep25 tipped(2) ep26 tipped(8)
        -- the remaining 19/30 picked in 12-17 steps
```
Picks that succeed are fast and tight (12-17 steps); every failure is either an early tip (as few as 1-8 steps
— the policy tips the can attempting the approach) or a full 300-step timeout (freezes/stalls rather than a slow
partial attempt) — no failure mode sits in between.

### Provenance / tree-hash cross-check

- `r2dreamer`: `0cf3d9e` (unchanged from training launch — both defects from `DV3_LOCAL_2026-09-11.md` §4 are
  in this commit; eval used the identical tree that trained).
- `genesis_pickaplace`: **`7c994b4` at training launch -> `9ccabe9` at eval time** (26 commits landed on the
  shared working tree during the 3.37 h training run — this is a shared-checkout project, `CLAUDE.md`'s "shared
  working-tree rule"). Checked which of those 26 touched files this run's reward/pick-predicate path reads:
  `git log --oneline 7c994b4..HEAD -- baselines/genesis_can_env.py baselines/rl/full_env.py` returns exactly
  **one** commit, `266998a` ("nested_ramp v2: fold farside's +1 into the slide ramp"). Read the diff
  (`git show 266998a -- baselines/rl/full_env.py`): it edits ONLY the `LADDERS['nested_ramp']` table entry
  (moving `farside`'s flat +1.0 into the slide ramp's `scale`); the `LADDERS['staged']` entry this run actually
  uses (`stage_reward=dict(picked=1.0, placed_v2=1.0, contact_push=2.0, slide_success=4.0)`) is untouched by the
  diff, and `baselines/genesis_can_env.py`'s pick predicate is untouched (its sha stamp is IDENTICAL at both
  times: `40544bf73c8c` in this eval's console stamp == the first 12 hex of
  `40544bf73c8c69bd87db8618df995855106ed3cbe866810cecc38a2de4b278ab` in the run's own launch-time
  `ladder_provenance.json`). `full_env.py`'s own sha DID change (`a8894b00e460...` at launch ->
  `23fe428f222f` at eval), but only because of the `nested_ramp`-only edit just confirmed irrelevant to a
  `scope='pick'`/`ladder='staged'` run. **Net: the code that actually executed during both training and this
  eval, for this run's own reward path, is unchanged.**

## 3. Verdict against G2

| cell (sampled = cell of record) | bar | measured | verdict |
|---|---|---|---|
| hold15 sample | >= 14/15 | 15/15 | **PASS** |
| rnd30 sample | >= 0.5 | 18/30 = 0.60 | **PASS** |
| hold15 mode (secondary) | -- | 15/15 | PASS (also) |
| rnd30 mode (secondary) | -- | 19/30 = 0.633 | PASS (also) |

Both sampled cells of record clear the G2 bar with margin (hold15 exactly at the ceiling; rnd30 at 0.60 vs the
0.5 bar, though below the `dreamerv3-torch` port's own historical 0.700 rnd30 reading on a different codebase —
not a contradiction, since G2's numeric bar (0.5) rather than its historical reading (0.700) is what this task
was asked to clear, and the two ports are not claimed identical, per `DV3_LOCAL_2026-09-11.md` §1's audit).

**Step 2 (e2e locally with `ladder=nested_ramp`) is UNBLOCKED.**

## Commands index (for reproduction)
- Exit confirmation: `pgrep -f 'env.steps=1000000'`; `tail -3 metrics.jsonl`; `grep -ci traceback console.log`;
  `stat -c '%y' latest.pt`.
- Reward/throughput: filter `metrics.jsonl` for `train/data/reward_sum` (last 20 rows); `grep -o 'fps/fps
  [0-9.]*' console.log`; wall-clock delta between `LAUNCH.txt`'s timestamp and `latest.pt`'s mtime.
- Eval: the four `eval_genesis.py` invocations in §2, run sequentially against the shared env block shown there.
- Hardware: `nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu --format=csv`; `lscpu`.
- Provenance: `git -C ~/workspace/r2dreamer rev-parse --short HEAD`; `git -C
  /home/j/workspace/genesis_pickaplace rev-parse --short HEAD`; `git log --oneline 7c994b4..HEAD -- <files>`;
  `git show 266998a -- baselines/rl/full_env.py`.
