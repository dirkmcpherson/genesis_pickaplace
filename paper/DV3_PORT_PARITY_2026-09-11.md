# DV3 local-tree port parity — 2026-09-11 (lane DV3-3)

Goal: bring `~/workspace/dreamerv3-torch` (tree of record for the local box) to parity with the cluster tree
`$W/dv3_dbg` that passed G1/G2 (`paper/DV3_DEBUG_2026-09-05.md`), WITHOUT the cluster (VPN down at the time of
this work), then smoke it on the pick scope. Read first: `paper/DV3_TREES_AUDIT_2026-09-11.md` (lane DV3-2's
read-only audit; everything below either closes an item from its re-apply list in sec 3, or explains why it
could not be closed). Work happened entirely on a NEW branch `genesis-dv3dbg-local`; `genesis` (HEAD `d645765`)
was never touched, and neither was `~/workspace/r2dreamer`.

## 0. Rule followed throughout
Every number below carries the command that produced it. An absent value is reported as absent, never as 0.
"Confirmed" means read directly from a file in this repo or the target repo; "reconstructed" means built by
analogy/porting and is flagged as such at the point it is used.

## 1. `return_clamp` port (task 1)

**Commit:** `genesis-dv3dbg-local` @ `5c67ad9` (dreamerv3-torch repo).

Ported from r2dreamer's `self.return_clamp` (`~/workspace/r2dreamer/dreamer.py:41-48` field + rationale comment,
`:502-503` imagination-target application). r2dreamer also clamps a second, REPLAY-anchor target
(`dreamer.py:552-553`, its `repval` loss) — this dreamerv3-torch tree has no equivalent replay-value loss
(`ImagBehavior._train` only computes the imagination-rollout actor/critic loss, `models.py:329-410`), so only
the one application point that has an analog was ported.

- `models.py`: new module-level helper `_clamp_return_target(target, config)` (inserted after the imports, before
  `class RewardEMA`) — `target` here is the TUPLE `tools.lambda_return(...)` returns (`tools.py:952-978`, via
  `torch.unbind` inside `static_scan_for_lambda_return`, `tools.py:931-948`), not a single tensor like
  r2dreamer's `ret`, so the clamp is applied per-element across the tuple, not via one `torch.clamp` call.
- Call site: `ImagBehavior._compute_target` (`models.py:449-483`), right after
  `target = tools.lambda_return(...)` and before the function returns — the exact insertion point the audit
  named (`models.py:468-475`).
- `configs.yaml`: `defaults.return_clamp: 0.0` (new key in the `# Behavior.` block, after `eval_action_mode`).
  dreamer.py's own config→CLI mechanism (`dreamer.py:851-877`: every key surviving the `--configs` merge becomes
  an argparse flag typed from its default) makes this `--return_clamp` automatically — no separate argparse
  wiring needed, unlike r2dreamer's Hydra config tree.

### Unit test (task 1, second half)
`test_return_clamp.py` (repo root, no Genesis, no CUDA required — imports only `torch`, this tree's own `tools`
and `models`). Sets `TORCHDYNAMO_DISABLE=1` so `tools.lambda_return`'s `@torch.compile` decorator runs eager
(irrelevant to what's being tested, avoids a CPU-compile dependency in the test). Builds a synthetic (T=20,B=4)
reward sequence with one reward spike (5.0) and an over-estimating critic (`value`/`bootstrap` pinned at 3.0 —
the exact failure mode `paper/DV3_DIAGNOSIS_2026-08-28.md` §2 point 1 describes: "`value_mean` climbs
monotonically past the maximum attainable return"), feeds it through the real `tools.lambda_return`, then through
the new `_clamp_return_target`.

Run: `cd ~/workspace/dreamerv3-torch && .venv-local/bin/python test_return_clamp.py`

```
PASS: return_clamp=0.0 is a no-op vs raw tools.lambda_return
raw unclamped target max = 7.9051 (> 1.0, expected)
return_clamp=1.0 target max = 1.0000 (<= 1.0, expected)
PASS: return_clamp=1.0 caps the lambda-return target at 1.0
```
Both registered checks pass: `return_clamp=0.0` reproduces the raw (unported) target exactly
(`torch.equal`), and `return_clamp=1.0` caps every element at ≤ 1.0 while the unclamped target the test
constructs genuinely exceeds 1.0 first (so the second check is not vacuous).

## 2. State-only demo-image patch (task 2)

Applied `cluster/dv3dbg/dv3dbg_stateonly_image_patch.py` (checked into `genesis_pickaplace`, not into
`dreamerv3-torch`) verbatim: `python3 cluster/dv3dbg/dv3dbg_stateonly_image_patch.py <tree>/dreamer.py` from the
`genesis_pickaplace` checkout, targeting `~/workspace/dreamerv3-torch/dreamer.py`. Output: `patched dreamer.py`.
Diff (11 lines inserted immediately after the `assert img is not None` anchor the audit located at
`dreamer.py:533-534`):
```
> if (str(config.task).startswith("genesis") and not getattr(config, "genesis_pixels", False)
>         and img.shape[-1] != 3):
>     if not globals().get("_DV3DBG_IMG_NOTE"):
>         print(f"[state-only] demo image placeholder {tuple(img.shape[1:])} -> "
>               f"({config.size[0]},{config.size[1]},3) zeros (encoder cnn_keys={config.encoder['cnn_keys']!r})")
>         globals()["_DV3DBG_IMG_NOTE"] = True
>     ep["image"] = np.zeros((img.shape[0], config.size[0], config.size[1], 3), np.uint8)
>     img = ep["image"]
```
Confirmed idempotent (the script's own guard: re-running it prints `already patched` and exits 0 — not
exercised here since it only needed to run once). `np` was already imported at `dreamer.py:15`, so no additional
import was needed.

## 3. Sim-variant hook (task 3)

`envs/genesis.py`'s `GenesisPickPlace._build()` now:
1. Reads `GENESIS_SIM_VARIANT` first (the var every `cluster/dv3dbg/*.sbatch` launcher exports and asserts
   against via `grep -q GENESIS_SIM_VARIANT envs/genesis.py`), falling back to `R2D_SIM_VARIANT` for parity with
   r2dreamer's own adapter env var name.
2. **Refuses to build with neither set** — an `assert`, not a silent `'base'` default. This is a deliberate
   departure from r2dreamer's own adapter (`~/workspace/r2dreamer/envs/genesis.py:204`:
   `_os.environ.get("R2D_SIM_VARIANT", "base") or "base"`), per the task instruction ("Refuse to build if the
   variable is unset — explicit beats a silent default world") and this project's standing convention against
   silent per-process defaults (`CLAUDE.md`'s "silent-default family" of bugs, `baselines/sim_variant_hook.py`'s
   own docstring rule: "every process that builds a world calls apply_pre(name) BEFORE and apply_post(env,
   name) AFTER").
3. Calls `sim_variant_hook.apply_pre(self._sim_variant)` before constructing either `CartesianFullTaskEnv` or
   `FullTaskEnv` (both branches now go through the same hook, not just the joint-action path r2dreamer's adapter
   covers), then `sim_variant_hook.apply_post(self._env, self._sim_variant)` after — same two-call pattern as
   `~/workspace/r2dreamer/envs/genesis.py` `_build()` (`~199-206, 236`).
4. `apply_post` (`baselines/sim_variant_hook.py:23-31`) prints `[sim-variant] <name>: kp=... gc=... riser=...` —
   exactly the string every `cluster/dv3dbg/*.sbatch` launcher greps for (`dv3dbg_pick.sbatch:60`:
   `grep -c '\[sim-variant\] gc_kp4_riser3_shelf6' $SLURM_OUT`).

**Confirmed live** in the smoke run (§6 below): one `[sim-variant] gc_kp4_riser3_shelf6: ...` line printed once,
at first `reset()` (lazy build, matching the module's own documented contract).

**Known gap, out of scope for this task:** `envs/genesis_vec.py` (the batched-facade adapter used by
`genesis_pixels_vec`/`genesis_vec: true`, e.g. `envs=32`) has its own separate `_build()` (`_SharedWorld`,
`genesis_vec.py:58`) and does NOT get this hook — the pick recipe this branch targets uses `envs: 1`
(non-vec), so it was not touched. Flagged here so a future vec-mode dv3 run does not silently train on the
default world.

## 4. `genesis_dv3std` config reconstruction (task 4)

`configs.yaml`, new block appended after `genesis_pick_msrecipe_shaped`:
```yaml
genesis_dv3std:
  genesis_reward_scale: 1.0     # undoes msrecipe's 100.0 -- plain +1 terminal on the pick
  train_ratio: 512              # undoes msrecipe's 256 -- back to the defaults: value
```

**What is CONFIRMED, not reconstructed**, and the exact source:
- `genesis_dv3std` sets `reward_scale 1` and `train_ratio 512` — this is not a guess: `cluster/dv3dbg/
  dv3dbg_pick.sbatch` lines 5-6 say so in its own header comment ("...the msrecipe overlay sets scope pick,
  delta_joint 0.025 / leash 5, action_repeat 4, batch 16x64, prefill 2500, pretrain 100; **dv3std sets
  reward_scale 1 / train_ratio 512**; --time_limit 1200 = 300 decisions"). That sbatch file is checked into
  `genesis_pickaplace` and travels with git — it did not need the cluster to read.
- `train_ratio: 512` is *already* the `defaults:` block value (`configs.yaml:91`, confirmed by reading the file)
  — `genesis_pick_msrecipe` overrides it down to 256 (the ManiSkill update intensity), so `genesis_dv3std`'s job
  is to undo that override, which this block does.
- `genesis_reward_scale: 1.0` matches `dreamer.py:375`'s own default (`getattr(config, 'genesis_reward_scale',
  1.0)`) and is required by the demo-gate assert at `dreamer.py:516-519` (`terminal_reward == genesis_reward_scale`)
  given the local demo set's `repeat.json` carries `terminal_reward: 1.0` (§5 below) — confirmed by both reading
  the code and by the demo gate passing in the smoke (§6) without raising.
- Every OTHER value DV3_DEBUG §1 point 4 lists for the *reach-goal* stage-1b recipe (discount 0.997,
  discount_lambda 0.95, imag_horizon 15, actor lr 3e-5 / entropy 3e-4, model_lr 1e-4, dyn_deter 512, units 512)
  is **already the `defaults:` block value** in this tree (`configs.yaml:45,55,63,94,101-103` — confirmed by
  reading `configs.yaml`, not reconstructed from the prose table). `delta_joint 0.025 / leash 5`, `action_repeat
  4`, `batch 16x64`, `prefill 2500`, `pretrain 100`, `eval_episode_num 0` are all already set by
  `genesis_pick_msrecipe` (`configs.yaml:423-446`, confirmed present and unmodified). `time_limit 1200` is the
  smoke/launcher's explicit `--time_limit 1200` CLI override (`dv3dbg_pick.sbatch:54`), not a config-file value —
  passed the same way in the local launch command (§6).

**What could NOT be reconstructed (needs the cluster/VPN):** `genesis_touchgoal` and `genesis_reach_goal`
(reach-goal task config blocks). Not written here — the pick recipe this branch targets never references them
(`dv3dbg_pick.sbatch`'s own `CFGS` string omits both), so their absence does not block the G2-recipe smoke this
task asked for. See `DV3_TREES_AUDIT_2026-09-11.md` §3 items 3-4 for what is knowable about their likely content
without the cluster.

## 5. Venv (task 5)

Did NOT install into `~/workspace/r2dreamer/.venv` (per instruction). Also did not do a full `pip install` of
r2dreamer's `pip freeze` output — that would re-download/re-resolve `torch==2.8.0+cu126` (a multi-GB wheel) and
`genesis-world==0.2.1` (an editable install pointing at `~/workspace/Genesis`, not reproducible by a plain `pip
install genesis-world==0.2.1` from PyPI) even though the exact, already-verified-working artifacts sit one
directory over. Cheapest route taken instead, which gives byte-identical versions with no re-download and no
version drift:

1. `python3.11 -m venv ~/workspace/dreamerv3-torch/.venv-local` (system `python3.11`, matching r2dreamer's venv's
   Python 3.11.14 for ABI compatibility with its compiled packages — torch's CUDA extensions, genesis's compiled
   pieces).
2. A single `.pth` file in the new venv's own site-packages
   (`.venv-local/lib/python3.11/site-packages/zzz_r2dreamer_sitepackages.pth`), one line:
   `import site, os; site.addsitedir(os.path.expanduser('/home/j/workspace/r2dreamer/.venv/lib/python3.11/site-packages'))`.
   Using `site.addsitedir()` (not a plain path line) matters: a plain path line only appends the directory to
   `sys.path` and does NOT process `.pth` files living inside it, and r2dreamer's own PEP-660 editable install of
   `genesis-world` (`__editable__.genesis_world-0.2.1.pth`, an `import ...`-form `.pth` file that registers a
   `MetaPathFinder` mapping `genesis` → `~/workspace/Genesis/genesis`) needs exactly that processing to make
   `import genesis` resolve. `site.addsitedir()` is the same mechanism `venv --system-site-packages` uses
   internally, just pointed at a sibling venv's site-packages instead of the OS Python's — so this is NOT the
   `--system-site-packages`-against-system-Python approach the task said was unacceptable ("hides versions"); it
   is a pin to one specific, already-audited venv, at exactly the versions the audit verified.
3. `.venv-local/bin/python -m pip install gym==0.26.2` (the one thing r2dreamer's venv lacks; wheel + its one
   dependency `gym_notices` were both already in the local pip cache — `pip cache list` showed `gym-0.26.2-py3-
   none-any.whl` cached before this install ran, so no network fetch was needed).

Verified:
```
$ .venv-local/bin/python -c "import numpy, torch, wandb, tensorboard, cv2, ruamel.yaml, genesis; ..."
python 3.11.14
numpy 2.4.6
torch 2.8.0+cu126  cuda avail True
wandb 0.28.1
cv2 5.0.0
ruamel.yaml 0.17.4
genesis 0.2.1
```
`import dreamer` (module-level import only, guarded `__main__` block never runs) succeeds — no `ModuleNotFoundError`
for `gym` or anything else (this was the one failure the audit's dependency probe hit on r2dreamer's own venv).

## 6. Smoke (task 6)

### Demo set
`/home/j/data/genesis_pickaplace/demos_state/dHfull_pick_local/` — did NOT need the 20-minute poll; it was
already present when checked (built by lane DV3-1, `generator: scratchpad/build_pick_from_full.py (lane DV3-1,
local, 2026-09-11)`, `created: 2026-09-11T18:43:50Z`). `repeat.json` (74 files present, matching `n_written: 74`):
```
sim_variant: gc_kp4_riser3_shelf6   action_repeat: 4   action_encoding: delta_joint   delta_cap: 0.025
scope: pick   with_state: true   state_dim: 17   state_only: true   terminal_reward: 1.0
n_written: 74   n_pick: 64   n_nopick: 10   total_reward: 64.0
```
This is a PICK-scope cut of a FULL-scope tape set (`src: demos_state_full/dHfull_all`), truncated per-tape at
the first `reward>0` row (its own `cut_rule`) — a different provenance from the cluster G2 demo sets
(`dHv2raw`/`dDP`, which came from `to_dreamer_native.py --phase --state-only` directly), but the same contract
fields the demo gate checks (`action_repeat 4`, `with_state true`, `terminal_reward 1.0`, `scope pick`), so it
passes the same gate. Reward total is 64.0 (64 of 74 tapes picked), not the cluster set's 66/58 — expected, this
is a different, locally-built set, not a copy of either cluster set.

### Command run
```
cd ~/workspace/dreamerv3-torch
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 \
PYTHONUNBUFFERED=1 \
.venv-local/bin/python dreamer.py \
  --configs genesis_pickplace genesis_pick_msrecipe genesis_dv3std usestateonly nowandb \
  --time_limit 1200 --steps 5000 --seed 0 \
  --logdir runs_dv3dbg_local/pick_smoke_dHfull_pick_local_s0 \
  --demodir /home/j/data/genesis_pickaplace/demos_state/dHfull_pick_local \
  --video_pred_log False --return_clamp 1.0 \
  --prefill 800 --pretrain 5 --log_every 200 --compile False
```
`--prefill 800 --pretrain 5 --log_every 200 --compile False` are smoke-only shrinkage, the same four flags
`dv3dbg_pick.sbatch`'s own `SMOKE=1` path uses (`dv3dbg_pick.sbatch:53`) — that launcher's smoke mode also forces
`--device cpu`; this smoke instead ran on the local GPU (available, task 6 allows "≤5k-step smoke" on it) at the
recipe's real `fp16` precision (default `precision: 16`, unlike the cluster's CPU-only smoke which needs fp32).
`--steps 5000` is RAW env/sim steps (confirmed from `dreamer.py:435`: `config.steps //= config.action_repeat`
happens inside `main()`, so the CLI value is pre-division, matching how DV3_DEBUG's own tables read `--steps`) →
1250 decisions after the action_repeat=4 division.

### Observed (`runs_dv3dbg_local/pick_smoke_dHfull_pick_local_s0/20260911T145355/`, untracked — gitignored `run*`)

**`[sim-variant]` line** (printed once, at first `reset()`, confirming task 3's hook actually fires and reads
`GENESIS_SIM_VARIANT`, not a default):
```
[sim-variant] gc_kp4_riser3_shelf6: kp=[800.0, 800.0, 600.0, 400.0, 240.0, 240.0] gc=1.0 riser=0.03
```

**Demo gate** (`repeat.json` action_repeat/terminal_reward asserts at `dreamer.py:503-524`, §6 above): passed
silently (no `AssertionError`), then:
```
Demo keys: ['image', 'action', 'reward', 'discount', 'is_first', 'is_last', 'is_terminal', 'logprob', 'state']
Total reward of demos: 64.0
Train episodes: 75, steps: 15808          # 74 demo episodes + 1 prefill episode
```
(The `[state-only] demo image placeholder ...` print from task 2's patch did NOT fire here — checked why: its
guard is `img.shape[-1] != 3`, and this demo set's `image` array is already `(T,64,64,3)` zeros, not the
`(T,64,64,6)` shape the cluster's `to_dreamer_native.py --with-state` sets that patch was written for. This
locally-built demo set (`scratchpad/build_pick_from_full.py`, lane DV3-1) already writes the online-compatible
3-channel shape, so the patch's branch is inert here but does no harm — it is still needed for demo sets shaped
like the cluster's `dHv2raw`/`dDP`, and the patch is idempotent/harmless when the shapes already match.)

**`return_clamp` echoed in `run_config.json`** (written at `dreamer.py:454`, before training starts):
```
$ python3 -c "import json; d=json.load(open('.../run_config.json')); print(d['return_clamp'])"
1.0
```
Also confirmed in the same file: `genesis_reward_scale 1.0`, `train_ratio 512` (both from `genesis_dv3std`),
`action_repeat 4`, `time_limit 300` (=1200/4), `batch_size 16` / `batch_length 64`, `prefill 800` / `pretrain 5`
(smoke overrides), `genesis_delta_cap 0.025`, `genesis_delta_leash_mult 5.0`, `genesis_scope pick`,
`steps 1250.0` (=5000/4) — every value task 4 claims is confirmed-not-reconstructed is confirmed again here, at
runtime, not just by reading the yaml.

**Training actually ran** (not just plumbing): `model_loss` fell 28.6 → 1.3 over 12 logged rows (raw steps 3200
→ 5000, `update_count` 5 → 230 gradient updates); `model_grad_norm` was `inf` on the very first pretrain row
(step 3200, `update_count 5`, a cold-start artifact of the 5-step `--pretrain 5` smoke override) then finite and
falling every row after (47.7 → 12.2). `target_max`/`value_max` (`tools.tensorstats(target, "target")` /
`(value, "value")`, the exact stats DV3_DEBUG's G1/G2 tables read) stayed near 0 throughout (final row:
`target_max 0.0346`, `value_max 0.0160`) — expected and NOT a clamp check, since at only 230 gradient updates
with zero online picks yet (`log_picked 0.0` every row) the model has not learned to predict returns anywhere
near 1.0 yet; the clamp's own correctness is what `test_return_clamp.py` (§1) checks directly with a
constructed case, not this smoke.

**Throughput:** `fps` (this tree's own field name, raw env-steps/sec, matching how DV3_DEBUG's tables read "fps")
stabilized at **14.8-16.0** across the post-pretrain rows (14.8, 15.0, 15.2, 16.0, 15.2, 15.8 at raw steps
3600-5000) — in the same 13-15 fps band DV3_DEBUG §1 point 4 and §7 measured for the reach/pick recipes on
cluster L40/L40S/H200/A100 GPUs, on a local RTX 3080 Ti. First row (raw step 3200, still inside `--pretrain 5`
warmup, no env stepping yet that iteration) read `fps 0.0`, and the immediately-following row (step 3400) read
`fps 3.9` (still absorbing `torch.compile`'s one-time inductor JIT cost — an inductor warning appears in the log
at that point even though `--compile False` was passed, because `tools.lambda_return`'s `@torch.compile(disable=
STATIC_CONSTANTS.DISABLE_COMPILE)` decorator is gated by a hardcoded dataclass flag in `constants.py`, not by
`config.compile` — a pre-existing property of this tree, unrelated to this task's port).

**GPU memory:** 429 MiB before launch (desktop/other processes only) → 2788 MiB once the model was constructed
→ peaked at **3281 MiB** during training (of 12288 MiB total, so well clear of both this box's ceiling and of
leaving headroom for lane DV3-1) → 351 MiB after the process was stopped (fully released).

**Stopped, not exited on its own — disclosed, not silent.** Checked `dreamer.py:799`:
`while agent._step < config.steps + config.eval_every:` — `agent._step` counts DECISIONS
(`dreamer.py:58,188-189`: `self._step = logger.step // action_repeat`), and `genesis_pick_msrecipe` sets
`eval_every: 2e4` (raw) which was NOT overridden by the smoke flags, so after action_repeat division the loop's
real exit condition was `agent._step < 1250 + 5000 = 6250` decisions = **25000 raw steps**, five times the
task's `≤5k env steps` ceiling (the `+ config.eval_every` padding exists so an in-loop eval always gets one more
chance to fire after `config.steps`; it does not matter here since `eval_episode_num: 0`, but the loop still
keeps training through the padding zone). Rather than let it run 5x over budget, the process was sent `SIGTERM`
the moment the logged raw step reached exactly **5000** (12 metric rows written, PID exited cleanly, GPU memory
released as above). This is a property of `dreamer.py`'s loop condition, not of this task's port — flagged here
so a future reader of `runs_dv3dbg_local/.../metrics.jsonl` (12 rows, ending at step 5000) knows why it stops
there rather than at a config-implied boundary.

### STOP
No 1M-step run was launched. The command that WOULD launch the real G2-parity run (dropping the smoke-only
flags, using the full msrecipe prefill/pretrain/log_every):
```
cd ~/workspace/dreamerv3-torch
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace \
GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 \
.venv-local/bin/python dreamer.py \
  --configs genesis_pickplace genesis_pick_msrecipe genesis_dv3std usestateonly nowandb \
  --time_limit 1200 --steps 1000000 --seed 0 \
  --logdir runs_dv3dbg_local/pick_dHfull_pick_local_s0 \
  --demodir /home/j/data/genesis_pickaplace/demos_state/dHfull_pick_local \
  --video_pred_log False --return_clamp 1.0
```
(This was NOT run — per the task's instruction to stop before any long run.)

## 7. What remains unreconstructable without the cluster
- `genesis_touchgoal` / `genesis_reach_goal` config blocks (§4) — reach-goal task only, not needed for the pick
  smoke this task targeted.
- Whether `$W/dv3_dbg`'s `envs/genesis.py` hook reads `GENESIS_SIM_VARIANT` or `R2D_SIM_VARIANT` FIRST, or in
  some other order, or whether it also refuses a silent default — the cluster launchers only assert the STRING
  `GENESIS_SIM_VARIANT` is present in the file (`dv3dbg_pick.sbatch:45`), which this port satisfies, but the
  exact precedence/refusal behavior on the cluster tree cannot be confirmed without reading it. This port's
  choice (GENESIS_SIM_VARIANT-first, refuse-if-neither-set) is disclosed as a local design decision, not a
  verified copy.
- Whether the cluster's `dv3_dbg` demo sets (`dHv2raw`, `dDP`, 66/58 tapes) are byte-identical to what
  `to_dreamer_native.py --phase --state-only` would produce today from the same source tapes on this box — not
  attempted; this smoke used the locally-built `dHfull_pick_local` set instead (§6), which is provenance-
  disclosed as a different (pick-cut-from-full) construction, not a copy of either cluster set.
