# DV3 local-tree audit — 2026-09-11 (lane DV3-2, READ-ONLY, no training, no GPU)

Scope: four local trees, cross-referenced against `paper/DV3_DEBUG_2026-09-05.md` (the cluster ladder that got
dreamerv3-torch to G1/G2 pass with `return_clamp 1.0`) and `paper/DV3_DIAGNOSIS_2026-08-28.md`. VPN is down, so the
cluster trees (`$W/dv3_fix`, `$W/dv3_dbg`) cannot be read directly — everything below about them is inferred from
the two docs and from `cluster/dv3dbg/*.sbatch`, which are checked into this repo and DO travel with git.

No file was modified. No `dreamer.py` was run. No git remote was left behind (temporary remotes added for
`git diff --stat`/`merge-base` were removed in the same command).

## 1. Tree identities

### `~/workspace/dreamerv3-torch`
- Remote: `git@github.com:dirkmcpherson/dreamerv3-torch.git`
- Branch `genesis`, HEAD `d645765` 2026-08-19 17:26:05 -0400, "Dense lever for dv3: genesis_pick_shaping config
  knob -> FullTaskEnv pick_shaping (gamma 0.997); SHAPED spec var in sbatch + registry knob"
- 22 untracked files/dirs, 0 modified tracked files (`git status --short` — all `??`):
  `MANISKILL_VS_GENESIS.md`, `antigravity_reference/`, `collect_random_episodes.py`, `debug_frame_stack/`,
  `debug_frame_stack_2/`, `debug_frame_stack_3/`, `debug_frame_stack_4/`, `debug_output.txt`,
  `debug_output_fix.txt`, `debug_output_fix2.txt`, `demo_videos.py`, `demo_videos/`, `demonstrations/`, `myjob.sh`,
  `prepare_demos.sh`, `random_episodes/`, `test_depth_range.py`, `test_grid.png`, `test_grid_vis.py`,
  `test_tasks.py`, `tmp.sh`, `wandb/`. None of these are tracked-file edits, so `git diff` is empty; a `git stash`
  would leave nothing behind (nothing to lose from a hard reset either, but none was run).

### `~/workspace/dev-dreamerv3-torch`
- Remote: same fork, `git@github.com:dirkmcpherson/dreamerv3-torch.git`
- Branch `fresh_director`, HEAD `3c4d871` 2025-09-07 21:52:45 -0400, "fix distribution problems"
- 6 modified tracked files (`git status --short`, all ` M`): `configs.yaml`, `director_models.py`,
  `director_ondeck.sh`, `director_utils.py`, `dreamer.py`, `models.py` (diffs not inspected further — read-only
  audit, and none of these files reference `genesis`, see §1a).

### `~/workspace/dreamerv3` (upstream JAX)
- Remote: `git@github.com:danijar/dreamerv3.git` — this **is** Danijar Hafner's original repo, not a fork.
- Branch `main`, HEAD `b65cf81a` 2025-09-23 11:23:47 -0700, "Update paper reference".

### `~/workspace/r2dreamer` (the working port, reference for `return_clamp`)
- Remote: `https://github.com/NM512/r2dreamer.git`
- Branch `main`, HEAD `2c0a686` 2026-09-11 13:35:04 -0400, "Ladder N: emit slide_event beside farside/home".

## 1a. Per-tree feature matrix (a)–(e)

| # | Feature | `dreamerv3-torch` (genesis, d645765) | `dev-dreamerv3-torch` (fresh_director, 3c4d871) |
|---|---|---|---|
| (a) | `return_clamp` flag + λ-return/critic-target use | **NO** | **NO** |
| (b) | Genesis adapter, state-only obs, demo prefill asserting `repeat.json` | **YES** | **NO** |
| (c) | State-only demo-image placeholder fix | **NO** | **NO** |
| (d) | `genesis_pick_shaping` / `pick_shaping` knobs | **YES** | **NO** |
| (e) | Periodic policy eval inside training | **YES (present, but disabled for genesis configs)** | **YES (generic, unused for genesis — no genesis task exists here)** |

Detail, `~/workspace/dreamerv3-torch`:

**(a) `return_clamp` — absent.**
```
$ grep -rn "return_clamp" --include="*.py" --include="*.yaml" .
(no output)
```
`models.py:449` `_compute_target` computes `target = tools.lambda_return(...)` (line 468) and returns it unclamped
(line 479) — no `torch.clamp` anywhere near it. This is the exact function DV3_DIAGNOSIS §"critic target" names as
the unclamped path (`models.py:449-479`), and it is byte-identical here to what that doc describes as the
*pre-fix* state. Consistent with the timeline: this HEAD (2026-08-19) predates the cluster's `return_clamp` addition
to `$W/dv3_fix` (dated 08-28 in DV3_DEBUG §1 point 1 and DV3_DIAGNOSIS line 50: "flag-gated `return_clamp` (default
0 = unchanged behaviour)").

**(b) Genesis adapter + state-only obs + demo/repeat.json gate — present.**
`envs/genesis.py` exists (`envs/genesis.py`, `envs/genesis_vec.py`), and its `observation_space` property emits a
`state` key (17-dim joint / 18-dim cartesian) whenever `self._pixels` is False:
```
envs/genesis.py:138-146:
        if not self._pixels:
            spaces['state'] = gym.spaces.Box(
                -np.inf, np.inf, (18 if self._cartesian else 17,), dtype=np.float32)
```
Demo prefill + the `repeat.json` stride/terminal-reward assert is in `dreamer.py:492-522`:
```
dreamer.py:501-503:
        _stamp = demodir / 'repeat.json'
        if _stamp.exists():
            ... _demo_repeat = int(_meta['action_repeat'])
dreamer.py:504-507:
            assert _demo_repeat == int(config.action_repeat), (
                f'demo stride mismatch: {demodir} was encoded at action_repeat='
                ...
dreamer.py:519-522:
        elif str(config.task).startswith('genesis') and int(config.action_repeat) != 1:
            raise AssertionError(
                f'{demodir} has no repeat.json stride stamp, ...')
```

**(c) State-only demo-image placeholder fix — absent from the tree, present as an unapplied patch script.**
```
$ grep -n 'ep.get("image")\|Demo episode.*missing' dreamer.py
533:            img = ep.get("image")
534:            assert img is not None, f"Demo episode {epkey} missing 'image' key"
```
That is exactly the `old` anchor string matched by `cluster/dv3dbg/dv3dbg_stateonly_image_patch.py` (checked into
*this* repo, `genesis_pickaplace`, not into `dreamerv3-torch`). No `"[state-only] demo image placeholder"` string
exists anywhere in the tree (`grep -rln placeholder` only hits an unrelated comment at `dreamer.py:568` about
logprob placeholders, and a docstring comment in `multitask_dreamer.py:246`). So the patch is available but has
never been run against this checkout.

**(d) `genesis_pick_shaping` / `pick_shaping` — present, and it is what this HEAD's commit message is about.**
```
configs.yaml:478:  genesis_pick_shaping: True
dreamer.py:376:            pick_shaping=getattr(config, 'genesis_pick_shaping', False),
envs/genesis.py:35:        self._pick_shaping = bool(pick_shaping)
envs/genesis.py:106-107:                                    pick_shaping=self._pick_shaping,
                                    pick_shaping_gamma=0.997,  # = dv3 discount (Ng-invariance)
```
`configs.yaml` also carries the `genesis_pick_msrecipe_shaped` overlay block (line 474) documenting how to turn it
on: `--configs genesis_pick_msrecipe genesis_pick_msrecipe_shaped`. This knob is not mentioned anywhere in
DV3_DEBUG (which predates it by a few days) — it is separate, newer work on top of the G1/G2-adjacent recipe.

**(e) Periodic policy eval inside training — present as a mechanism, switched off for every genesis config.**
The generic dv3 eval loop is intact:
```
dreamer.py:788-802:
    while agent._step < config.steps + config.eval_every:
        ...
        if config.eval_episode_num > 0 and not _skip_first_eval():
            print("Start evaluation.")
            eval_policy = functools.partial(agent, training=False)
            tools.simulate(eval_policy, eval_envs, eval_eps, config.evaldir, logger, ...)
```
But every genesis config block sets `eval_episode_num: 0` deliberately:
```
configs.yaml (genesis_pixels block):
  eval_episode_num: 0       # genesis = ONE world per process; a 2nd (eval) env would
                             # call gs.init twice and die. ... Judge progress from
                             # train_return + periodic policy videos instead.
configs.yaml (genesis_pick_msrecipe block):
  eval_episode_num: 0            # one genesis world per process; eval stays out-of-process (genesis_eval.py)
```
So the in-loop eval gate never fires for a genesis run in this tree; out-of-process `genesis_eval.py` (present,
`genesis_eval.py`, 14 KB, dated Aug 14) is the intended progress signal — matching how DV3_DEBUG's launchers score
runs (a separate `genesis_eval.py` invocation after training, never an in-job eval env).

Detail, `~/workspace/dev-dreamerv3-torch`: **no genesis integration exists at all.**
```
$ grep -rln "genesis" --include="*.py" --include="*.yaml" .
(no output)
$ ls envs/
atari.py crafter.py dmc.py dmlab.py memorymaze.py minecraft_base.py minecraft_minerl.py minecraft.py pinpad.py
pusht.py wrappers.py __pycache__ setup_scripts
```
No `envs/genesis.py`, no `genesis` string in `configs.yaml`, no `return_clamp`. The tree's own recent commits
(`fresh_director` branch: "fix distribution problems", "Director almost there. Missing KL Divergence function
required distributions.") are Director-algorithm work, unrelated to the Genesis pick-and-place task and to
`return_clamp`/pick-shaping/demo-prefill. All five answers for this tree are **N/A / absent**.

## 2. Delta between the two local trees, and which is closer to the cluster tree of record

They share history: `git merge-base` (temporary local-path remote, removed after) resolves to
`365220cef2e361d8db44094a087df8dc1b8cef51` ("shaped reward for pinpad. pretraining on the world model and reducing
the LR after pretraining completes.") for **both** `dreamerv3-torch:genesis` and `dev-dreamerv3-torch:fresh_director`
— i.e. they are two branches of the *same* dirkmcpherson fork that diverged at that commit, well before any Genesis
work started.

```
git diff --stat 365220c..dreamerv3-torch/genesis   -> 53 files changed, 8957 insertions(+), 244 deletions(-)
git diff --stat 365220c..dev-dreamerv3-torch/HEAD   ->  7 files changed, 1079 insertions(+),  20 deletions(-)
git diff --stat dev-dreamerv3-torch/HEAD..dreamerv3-torch/genesis -> 57 files changed, 8976 insertions(+), 1322 deletions(-)
```
`dreamerv3-torch:genesis` carries essentially all of the project's Genesis-adapter engineering (53 files,
~9k lines: `envs/genesis.py`, `envs/genesis_vec.py`, `genesis_eval.py`, `genesis_msrecipe_gate.py`,
`convert_genesis_demos_repeat.py`, cluster launch scripts, `MANISKILL_VS_GENESIS.md`, etc.). `dev-dreamerv3-torch`
carries only the Director-algorithm side-branch (`director_models.py`, `director_networks.py`,
`director_utils.py`, `director_ondeck.sh`, plus edits to the shared `dreamer.py`/`models.py`/`networks.py`) and
touches none of the Genesis files. **`dreamerv3-torch` (branch `genesis`) is unambiguously the tree in the lineage
of the cluster's `$W/dv3_fix`/`$W/dv3_dbg`; `dev-dreamerv3-torch` is an unrelated side-experiment and is not a
candidate for reproducing the DV3_DEBUG results.**

Within `dreamerv3-torch:genesis` itself, how far is `d645765` (2026-08-19) from the cluster tree of record? DV3_DEBUG
places the `return_clamp` addition to `$W/dv3_fix` at **08-28** (DV3_DIAGNOSIS line 50) — 9 days after this local
HEAD — and `$W/dv3_dbg` (`dv3_fix` + the demo-image hunk + config blocks `genesis_touchgoal`/`genesis_reach_goal`/
`genesis_dv3std`) is built on top of that, first referenced 2026-09-05 (DV3_DEBUG §3). So the local tree is the
**pre-return_clamp ancestor** of the cluster tree of record: everything present locally (genesis adapter, msrecipe
overlay, repeat.json gate, `pick_shaping`) is inherited unchanged by `dv3_fix`/`dv3_dbg` (nothing in DV3_DEBUG
describes removing or renaming any of it), but `return_clamp`, the demo-image hunk, and three config blocks
(`genesis_touchgoal`, `genesis_reach_goal`, `genesis_dv3std`) were added cluster-side after this checkout was taken
and never synced back to either local tree. This cannot be confirmed by reading the cluster tree (VPN down); it is
inferred from the dated doc and from the local tree's own commit dates being older than the doc's fix dates.

## 3. Re-apply list to reproduce `$W/dv3_dbg` locally, starting from `~/workspace/dreamerv3-torch`

| # | Item | Status locally | Source to re-apply from |
|---|---|---|---|
| 1 | `return_clamp` config field + λ-return target clamp in `models.py._compute_target` | Absent — must be written | Reference implementation in `~/workspace/r2dreamer/dreamer.py`: field + rationale comment at **`dreamer.py:41-48`** (`self.return_clamp = float(config.get("return_clamp", 0.0) or 0.0)`), applied to the **imagination** target at **`dreamer.py:502-503`** (`if self.return_clamp > 0: ret = torch.clamp(ret, max=self.return_clamp)`, right after `ret = self._lambda_return(...)`), and applied again to the **replay-anchor** target at **`dreamer.py:552-553`** (same two lines, after the second `ret = self._lambda_return(...)` in the replay-value block). r2dreamer's config plumbing is a config tree (`configs/model/_base_.yaml:15` `return_clamp: ${oc.select:env.return_clamp,0.0}`, per-env values in `configs/env/*.yaml`, e.g. `genesis_full_state.yaml:52` `return_clamp: 8.0`, asserted equal to the env-side value by `train.py:62-80`). dreamerv3-torch's config system (`configs.yaml` + argparse via `ruamel.yaml`) is different, so this is a **port**, not a copy-paste: the natural insertion point in `dreamerv3-torch/models.py` is inside `_compute_target` (`models.py:449-479`), clamping `target` right after `tools.lambda_return(...)` at **`models.py:468-475`**, plus a `return_clamp: 0.0` default added to `configs.yaml`'s `defaults`/`_base_` block and threaded into `ImagBehavior.__init__` the same way `discount`/`discount_lambda` already are. |
| 2 | State-only demo-image placeholder fix | Not applied, but the exact patch already exists in this repo | `cluster/dv3dbg/dv3dbg_stateonly_image_patch.py` (checked into `genesis_pickaplace`, not into `dreamerv3-torch`). It is a small idempotent text-patcher (`assert s.count(old) == 1`) targeting the identical anchor found in the local tree at `dreamer.py:533-534`. Usage per its own docstring: `dv3dbg_stateonly_image_patch.py <tree>/dreamer.py`. Not run in this audit (no tree modification). |
| 3 | `genesis_touchgoal` / `genesis_reach_goal` config blocks | Absent (no hits anywhere in `dreamerv3-torch` or in `genesis_pickaplace`) | Only referenced by name in `cluster/dv3dbg/dv3dbg_s1b.sbatch:38` (`CFGS="genesis_pickplace genesis_pick_msrecipe genesis_touchgoal genesis_reach_goal genesis_dv3std usestateonly nowandb ..."`). Content unknown — these were evidently added directly to `$W/dv3_fix`'s `configs.yaml` on the cluster and never rsynced back to a local tree or committed to `genesis_pickaplace`. The underlying env-var hook they likely set (`REACH_GOAL_DIST`) already exists in `genesis_pickaplace/baselines/rl/full_env.py:1336,1564` (read via `os.environ.get('REACH_GOAL_DIST')`), so the missing piece is plausibly just a `configs.yaml` block that sets `genesis_scope: 'reach_goal'` and exports `REACH_GOAL_DIST`, but the exact numbers (R=0.25/0.30 used in DV3_DEBUG) are not recoverable from this box — **needs the cluster (VPN)**. |
| 4 | `genesis_dv3std` config block | Absent (confirmed by exhaustive grep of both `dreamerv3-torch` and `genesis_pickaplace`, including the worktree copies) | Referenced by name in all four `cluster/dv3dbg/*.sbatch` launchers (`dv3dbg_s1b.sbatch:38`, `dv3dbg_pick.sbatch:49`, `dv3dbg_eval.sbatch:24`, `dv3dbg_modesweep.sbatch:22`) but its body is nowhere in this repo. From the recipe table in DV3_DEBUG §1 point 4 ("batch 16×64, train_ratio 512, prefill 2500, pretrain 100, discount 0.997, λ 0.95, horizon 15, actor lr 3e-5 entropy 3e-4, model lr 1e-4, deter 512 / units 512") it is plausibly just those hyperparameters as a named overlay — close to but not identical to the already-present `genesis_pick_msrecipe` block (which uses `train_ratio 256`, `batch_size 16`, `batch_length 64` — DV3_DEBUG's `genesis_dv3std` recipe differs at least in `train_ratio` 512 vs 256). **Needs the cluster** to recover exactly; reconstructing it by hand from the prose table would be a guess, not a re-apply. |
| 5 | The `GENESIS_SIM_VARIANT` world-selection hook inside `envs/genesis.py` | **Absent locally — a real gap, not just a missing config.** `grep -n "GENESIS_SIM_VARIANT" envs/genesis.py` returns nothing in the local tree. Every `cluster/dv3dbg/*.sbatch` launcher hard-fails before training if this string is missing (`grep -q 'GENESIS_SIM_VARIANT' envs/genesis.py || { echo "FATAL: tree lacks the sim-variant hook"; exit 2; }`), so the cluster's `dv3_fix`/`dv3_dbg` `envs/genesis.py` **does** call the hook and the local one does not. | The pattern is `sim_variant_hook.apply_pre(<variant>)` / `apply_post`, used by other Genesis-facing adapters in this repo (e.g. `paper/eef_recovery_2026-09-09/localized_pads/render_contact_review.py:15,33`: `from sim_variant_hook import apply_pre, apply_post; apply_pre(meta['variant'])`), combined with reading the `GENESIS_SIM_VARIANT`/`R2D_SIM_VARIANT` env var the way `baselines/rl/full_env.py:791` does (`_vn = _os.environ.get('R2D_SIM_VARIANT') or _os.environ.get('GENESIS_SIM_VARIANT') or 'base'`) — that line is only an *assert* against the already-built world (comment at `full_env.py:799`: "world is built by sim_variant_hook.apply_pre() -- nothing tied them together"), so `envs/genesis.py::_build()` must call `sim_variant_hook.apply_pre(os.environ['GENESIS_SIM_VARIANT'])` (or equivalent) **before** constructing `FullTaskEnv`/`CartesianFullTaskEnv`. Without this, a local run of `dreamerv3-torch` trains on whatever the *default* world is, not the corrected `gc_kp4_riser3_shelf6` world DV3_DEBUG's G1/G2 numbers are scored on — silently, since nothing in the local tree checks for it. |

Summary of the list: items 1, 2 and 5 are recoverable from evidence already on this box (r2dreamer's clamp
implementation, this repo's own patch script, and the sim-variant-hook pattern used elsewhere in the project) —
each still requires writing/porting code, none is a pure copy. Items 3 and 4 (`genesis_touchgoal`,
`genesis_reach_goal`, `genesis_dv3std`) are config blocks whose exact contents live only in the cluster's
`configs.yaml` and cannot be reconstructed exactly without the VPN back up; DV3_DEBUG's prose gives the
hyperparameter *values* used at the time but not a byte-exact block to paste in.

## 4. Which venv can run `dreamerv3-torch`

`~/workspace/r2dreamer/.venv/bin/python` (3.11.14) has every third-party module `dreamer.py` needs **except legacy
`gym`**:
```
OK   numpy        2.4.6
OK   ruamel.yaml  0.17.4
OK   torch        2.8.0+cu126
OK   wandb        0.28.1
OK   tensorboard  2.21.0
OK   cv2          5.0.0
OK   genesis      0.2.1        (pip: genesis-world 0.2.1, editable -> ~/workspace/Genesis)
FAIL gym          ModuleNotFoundError: No module named 'gym'   (gymnasium 1.2.0 is installed instead)
```
All of `dreamer.py`'s *local* imports (`activation_monitor`, `constants`, `blocks`, `dreamerv3_torch/` — a
sub-package with `core.py`/`networks.py`) are files inside the `dreamerv3-torch` tree itself, not pip deps, and are
present.

Attempting `import dreamer` (module import only — `dreamer.py` guards its training entry point behind
`if __name__ == "__main__":`, so importing does not start training) fails with a full traceback rooted in the
genesis adapter's own import chain, not anything genesis-specific:
```
Traceback (most recent call last):
  File "<string>", line 5, in <module>
  File "/home/j/workspace/dreamerv3-torch/dreamer.py", line 23, in <module>
    import envs.wrappers as wrappers
  File "/home/j/workspace/dreamerv3-torch/envs/wrappers.py", line 2, in <module>
    import gym
ModuleNotFoundError: No module named 'gym'
```
`envs/wrappers.py` is imported unconditionally by `dreamer.py` (used for every task, not just genesis), so this
blocks the whole module, not just the Genesis suite. 12 files in the tree do `import gym` at module scope
(`genesis_activations.py`, `envs/{memorymaze,minecraft_base,pinpad,atari,genesis_vec,dmlab,crafter,genesis,
minecraft,wrappers,dmc}.py`). `gym` (OpenAI's legacy package, separate from the installed `gymnasium`) is not
installed in r2dreamer's venv — this is the one missing piece; everything else the tree needs is present and at
compatible versions (same `torch`/`genesis-world` as the project's canonical eval venvs per `CLAUDE.md`).

## 5. Is `~/workspace/dreamerv3` the original JAX implementation?

Yes. `git remote -v` → `origin git@github.com:danijar/dreamerv3.git`, branch `main`, HEAD `b65cf81a` (2025-09-23,
"Update paper reference") — this is Danijar Hafner's own repository, not a fork or a local rewrite.

A JAX venv **does exist on this box**: `~/workspace/dreamerv3/venv/bin/python` (3.10) has a working
`jax 0.4.33` (`import jax; import jax.numpy as jnp; jnp.zeros(3)` succeeds, `jax[cuda12]==0.4.33` per its
`requirements.txt`). None of the other venvs checked (`r2dreamer/.venv`, `dev-dreamerv3-torch/venv`,
`genesis_sim2real/venv`) have `jax` installed.

**Cost of going back to the JAX implementation:** none of the project's Genesis integration work transfers. The
JAX repo's env suite is `embodied`'s own (Atari/DMLab/Minecraft/Crafter/Loconav/Procgen-style `embodied.Env`
objects, driven through `embodied.run.train`/`portal`'s distributed actor model) — a fundamentally different
interface from the `gym`/`gymnasium`-style `step()/reset()` adapter this project has built around `FullTaskEnv`
(`baselines/rl/full_env.py`), which every torch-side learner (RLPD, DP, r2dreamer, and this dreamerv3-torch port)
already shares. Reusing it would mean writing a new `embodied.Env` wrapper around `FullTaskEnv` from scratch — the
genesis diff-stat in §2 (53 files, ~9k lines) is a rough proxy for the size of that undertaking, since it is
mostly the adapter, the demo/repeat.json machinery, the ManiSkill-recipe knobs, and the reward-scope plumbing that
would all need re-doing in `embodied`'s conventions. None of `return_clamp`, `pick_shaping`, or the state-only
demo-image handling exist in the JAX tree either (`return_clamp` was invented specifically for the torch port's
critic-overestimate bug, per DV3_DEBUG/DV3_DIAGNOSIS) — they are torch-tree fixes with no JAX-side analog to
inherit, so this would also need re-diagnosing whether the JAX critic has the same failure mode at all. Given the
torch port already passed G1/G2 on the cluster with `return_clamp`, the JAX path is a full rebuild for uncertain
gain, not a shortcut.
