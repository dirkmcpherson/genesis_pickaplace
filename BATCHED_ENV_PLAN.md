# Vectorized Genesis envs for DreamerV3 — design notes

**Status:** designed + benchmarked, NOT implemented. Deferred to 2026-07-23 evening
(Fable 5 budget refresh). This is a tricky change; read all of it before starting.

**Why:** we currently parallelize from the OUTSIDE (N processes × 1 world each).
Genesis parallelizes natively INSIDE one process via `scene.build(n_envs=N)`, which
this repo already uses in `can_pos_recovery/batch_harness.py` (B=32, GPU).

---

## 1. Benchmark data (measured 2026-07-22, this box, 3080 Ti)

`baselines/bench_batched_envs.py --envs 1 8 16 32 64 128`

| n_envs | sim GPU | scene.step/s | env-steps/s | vs 1 env |
|---|---|---|---|---|
| 1 | 428 MB | 59.2 | 59 | 1.0× |
| 8 | 428 MB | 48.8 | 390 | 6.6× |
| 16 | 428 MB | 43.1 | 690 | 11.7× |
| 32 | 460 MB | 41.3 | 1323 | 22.4× |
| 64 | 492 MB | 38.7 | 2480 | 42.0× |
| 128 | 524 MB | 34.0 | 4352 | 73.6× |

**Headline: sim GPU memory is ~FLAT in n_envs.** 1→128 worlds costs +96 MB; the
428 MB baseline is the Taichi/Genesis kernel context, not per-env state. Env
parallelism therefore does NOT compete with the training batch for GPU memory.

**Consequence for sizing:** collection stops being the bottleneck somewhere around
n_envs 32-64 (current dv3 single CPU env = 59 env-steps/s; 64 envs = 2480/s). Past
that the GPU TRAINER binds, so the lever becomes batch_size/batch_length and
gradient steps per second. Cluster rule of thumb: one process per GPU, n_envs 64-128
(~0.5 GB), spend the rest of the card on the model. Do NOT replicate processes for
env parallelism — each costs a full 428 MB context + a CPU core.

Caveat: benchmark stepped a fixed zero action with no rendering and no per-env
resets. Camera rendering (our dv3 obs needs 2 cams × N envs) is NOT included and
could dominate — **measure render throughput before trusting these numbers for the
pixel run.**

---

## 2. The integration problem

Dreamer's driver (`tools.simulate`) expects a **list of independent env objects**,
each with `reset()` / `step(action)`, stepped one at a time (`Damy` = in-process
shim, `Parallel` = one process per env). Genesis batching gives **one world holding
N states stepped in lockstep**. These are different shapes:

- Dreamer: `for env in envs: env.step(a_i)` — envs advance independently
- Genesis: `scene.step()` advances ALL N at once; actions are (N, A) arrays

So a drop-in `envs: 64` in the config does NOT use Genesis batching — it would build
64 separate worlds in 64 processes (and 64 × 428 MB + 64 cores, i.e. the wrong thing).

## 3. Design options

**Option A — Vector adapter + patched simulate (recommended).**
Write `envs/genesis_vec.py` exposing a batched env, and teach `tools.simulate` to
accept it. Cleanest fit to how Genesis actually works.
- `reset(indices=None)` → obs dict with leading dim N
- `step(actions (N,7))` → obs/reward/done/info with leading dim N
- Dreamer's `simulate` needs a branch: if env is batched, step once with stacked
  actions instead of looping. Touches `tools.simulate` (~40 lines) — the riskiest
  part, since that function also owns episode bookkeeping/cache/logging per env.

**Option B — N logical env facades over one batched world.**
Each facade holds an index into the batched world; `step()` buffers its action and
blocks until all N have submitted, then one `scene.step()` releases them.
- Zero changes to Dreamer.
- Needs threads or a barrier; with `Damy` (sequential, in-process) it deadlocks
  unless the facades are driven by a coordinating wrapper. Fragile. Not recommended.

**Option C — keep 1 env, use batching only for EVAL.**
Cheap win: `genesis_eval.py` could roll out 16-64 ICs simultaneously instead of
serially (currently ~40 s/episode → an 8-episode eval takes ~5 min). Low risk, no
Dreamer changes, and it validates batched stepping + per-env reset semantics before
betting the training loop on them. **Do this first as a stepping stone.**

## 4. Unknowns to resolve BEFORE writing the training adapter

1. **Per-env reset.** Can we reset env i without disturbing j? `batch_harness.py`
   sets all env states at once (`set_pos` with a (B,3) array). Dreamer needs
   independent episode boundaries — envs finish at different times. Verify:
   `entity.set_pos(pos, envs_idx=[i])` (genesis 0.2.1 API — CHECK IT EXISTS) and
   confirm a single-index reset leaves other envs bit-identical.
   *If per-env reset is unsupported, fall back to synchronized episodes (all envs
   reset together at a fixed horizon) — acceptable for us since our episodes are
   fixed-length 1200 anyway and nested termination is rare.*
2. **Per-env camera rendering.** Our obs is 2 cams × 64×64×3. In a batched scene,
   does `cam.render()` return (N,H,W,3)? The wrist cam is `attach`ed to an eef link —
   verify attachment works per-env in a batched build. **This is the most likely
   blocker for the pixel run** and has no workaround short of rendering serially
   (which would erase the throughput gain).
3. **CPU vs GPU backend.** `batch_harness` uses `gs.gpu`. All our validated
   physics (placements, stage labels, DP/SACfD numbers) came from the **CPU** path.
   CLAUDE.md records a historical **GPU→CPU winner transfer cliff**. Any batched-GPU
   run is a DIFFERENT physical realization — do not compare its stage counts to
   existing numbers without a bridge experiment (re-measure the 25-demo Gate-2
   subset under batched GPU and report both).

## 5. Implementation order (tomorrow)

1. Probe per-env reset + batched camera render (30 min, answers §4.1/§4.2).
2. Option C: batched `genesis_eval.py` — proves the machinery on a low-risk path.
3. Option A: `envs/genesis_vec.py` + `tools.simulate` branch, behind a config flag
   (`genesis_vec: true`) so the current single-env path stays runnable.
4. Bridge experiment: Gate-2 subset under batched GPU vs known CPU numbers.
5. Only then retune `train_ratio` / `batch_size` for the new collection rate.

## 6. Files

- `baselines/bench_batched_envs.py` — the benchmark above (re-runnable)
- `can_pos_recovery/batch_harness.py:28` — `build_batched_world`, existing reference
- `dreamerv3-torch/envs/genesis.py` — current single-env adapter (branch `genesis`)
- `dreamerv3-torch/tools.py::simulate` — the function Option A must modify
- `dreamerv3-torch/genesis_eval.py` — out-of-process evaluator (Option C target)

## 7. Unlocked by this work (do AFTER the adapter lands)

- **Demo-mixing sweep** (see memory `dv3-demo-mixing-experiment`). Demos currently
  dilute to ~18% of the replay buffer late in training (naive length-weighted
  sampling into one shared buffer). Sweeping fixes — Nx duplication, smaller
  dataset_size, or a fixed-fraction demo sampler — needs a fresh run per variant,
  which is only cheap once collection is batched. This is the highest-value
  follow-on given how sparse our reward is (0.11%).
