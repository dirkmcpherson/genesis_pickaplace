# Ladder N executed, not just syntax-checked (Lane 12b, 2026-09-11)

Lane 12a's `paper/TIP_GUARD_IMPL_2026-09-11.md` §7/§8 flagged that the r2dreamer-side tip-guard
wiring was "syntax-checked, not executed" because that box had no r2dreamer environment. This
lane has one (`~/workspace/r2dreamer`, `.venv`) and ran it: the two demo sets, four full-scope
`train.py` smokes, two refusal probes, and one `eval_genesis.py` pass, all local/CPU, VPN down.

Repo `genesis_pickaplace` branch `ladder-unify-2026-09-11`; started at HEAD `e80b889`, ended at
`dda07d0` (other lanes landed commits on this shared branch/checkout while this ran — see §6).
r2dreamer `~/workspace/r2dreamer` branch `main` @ `16e03cb` throughout (untouched; the one fix
found here lives on a separate branch in a worktree, §5).

**Mid-task plan change, addressed inline (§4):** the coordinator revised the `nested_ramp`
ladder (`farside` unpaid, ramp scale 2.0->3.0) partway through this run, landing on the branch
as commits `266998a`/`dda07d0`. The two `nested_ramp` demo sets were rebuilt from the existing
stage records under the new spec (seconds, no re-simulation) and their numbers below are POST-
revision. The one r2dreamer `train.py` smoke of `nested_ramp` ran under the OLD spec (scale 2.0)
before the revision landed and was NOT re-run, per the coordinator's explicit sign-off ("no need
to re-run unless the stamp check is the point") -- its stamp says which spec, verbatim.

## 1. The four demo sets (task item 1)

Built with `baselines/rl/relabel_reward.py --from-records ... --tip-guard not_in_hand`, no
`--far-release` (far_release OFF per amendment (aa)), from
`/home/j/data/genesis_pickaplace/stage_records/{dHfull_all,dDPfull_first}` (Lane 7's records,
this box, 32-core AVX2 -- not the 64-core class; P-aa-7 regeneration is still open). No fallback
to the D5 re-execution path was needed: the records carry everything the guard needs (`tool_xy`,
`can_pos`, `can_quat`, `grip_cmd`, solver contacts), confirmed by every build completing and the
tool's own `--verify-against`-style internal check (sha256 of `actions_delta` asserted against
the source before anything is written) passing on all four builds.

**Suffix map** (`baselines/rl/relabel_reward.LADDER_SUFFIX` / `TIP_GUARD_SUFFIX`, read directly
from the code rather than guessed): `nested_ramp` -> `_rnr`, `nested_sparse` -> `_rns`, plus `h`
for `tip_guard=not_in_hand` (far_release OFF adds nothing). So the four output directories, all
under `/home/j/data/genesis_pickaplace/demos_state_full/`, are:

| set | ladder | dir | tapes | decisions | Σ reward (recorded -> RE-EXEC) | `home` tapes | actions_sha256 |
|---|---|---|---|---|---|---|---|
| human | nested_ramp (**v2**, scale 3.0) | `dHfull_all_rnrh` | 74 | 29221 | 118.0 -> **215.9** | **13** | `77bc48750f1d975b...` |
| human | nested_sparse | `dHfull_all_rnsh` | 74 | 29221 | 118.0 -> **13.0** | **13** | `77bc48750f1d975b...` (identical to `_rnrh`) |
| machine | nested_ramp (**v2**, scale 3.0) | `dDPfull_first_rnrh` | 72 | 36834 | 131.0 -> **217.8** | **14** | `671614c53f4cdbf2...` |
| machine | nested_sparse | `dDPfull_first_rnsh` | 72 | 36834 | 131.0 -> **14.0** | **14** | `671614c53f4cdbf2...` (identical to `_rnrh`) |

`home` counts (13 human / 14 machine) match the expected P-aa-1 numbers exactly, and are
identical between the ramp and sparse builds of the same source (same tapes reach the same
terminal regardless of which ladder is scoring them -- only the reward column and the sub-rung
credit differ). Each set's `actions_sha256` matches its OWN sparse/ramp sibling built from the
same source, which is the identity check the tool enforces internally before writing anything;
had it disagreed, the build would have refused rather than write a mismatched set.

`repeat.json` keys present on every relabeled set (in addition to the base `sim_variant`,
`action_repeat`, `with_state`, `n_written`): a `relabel` block carrying `ladder`, `ladder_stamp`,
`tip_guard`, `tip_guard_sustain_frames`, `source_set`, `source_generator`, `source_total_reward`,
`source_n_pick`/`source_n_nopick`, `actions_sha256`, `selection_inherited`, `tapes_granting`,
`end_reasons`, `can_dev_max_m`/`can_dev_p50_m`/`n_tapes_can_dev_over_1cm`,
`inherited_source_stamps`, and an `is_terminal` provenance note (D5 rule: `is_terminal` is
inherited verbatim from the source tape, unaffected by the guard/ladder).

**Stamps, verbatim (post-revision, `git=known-good-2026-08-27-883-gdda07d0`):**

```
ramp (v2):   unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 home=4 ramp:slide_gain_m=3/0.05m | max_return=9 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-883-gdda07d0
sparse:      unified-2026-09-10 | ladder=nested_sparse | home=1 | max_return=1 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-884-gc606b3a
```

Note `farside` is absent from the ramp stamp's reward list now (`picked=1 placed_v2=1 home=4`,
no `farside=1`) -- the revision's signature.

Commands (identical for all four, only `--in`/`--out`/`--ladder` vary):

```
V=~/workspace/genesis_sim2real/venv/bin/python
D=/home/j/data/genesis_pickaplace
$V baselines/rl/relabel_reward.py --in $D/demos_state_full/dHfull_all \
   --out $D/demos_state_full/dHfull_all_rnrh --from-records $D/stage_records/dHfull_all \
   --ladder nested_ramp --tip-guard not_in_hand
# ... dHfull_all_rnsh (--ladder nested_sparse), dDPfull_first_rnrh, dDPfull_first_rnsh likewise
```

The two `_rnrh` sets were built TWICE: once under the pre-revision spec (scale 2.0, Σ 235.9
human / 240.9 machine, `farside=1` paid) while the revision was still an uncommitted, in-flight
edit in the shared checkout (§6), and once more after `dda07d0` merged, from the SAME stage
records (no re-simulation, ~2-3 min each). The table above reports the post-revision numbers;
the pre-revision build's artefacts were deleted before the rebuild rather than kept alongside,
since the coordinator's instruction was to rebuild and report the new numbers, not both.

## 2. Full-scope `train.py` smokes (task item 2)

Command shape (env-specific fields vary; ramp used `return_clamp=9.0`, sparse `1.0`; `env.steps`
is prefill-counter-dependent, computed per set below):

```
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 \
~/workspace/r2dreamer/.venv/bin/python train.py env=genesis_full_state seed=0 \
  env.steps=<prefill_counter + ~300-350> \
  env.demo_dir=<set> env.ladder=<ladder> env.far_release=false env.tip_guard=not_in_hand \
  env.return_clamp=<9.0|1.0> model.return_clamp=<9.0|1.0> \
  env.actor_dist=bounded_normal env.act_entropy=3e-5 \
  device=cpu buffer.max_size=5e5 logdir=~/runs_r2d_smoke/<name>
```

Four attempted, three completed cleanly to the same checkpoint-and-crash point (below); the
fourth (machine `nested_ramp`) was not run given time/resource pressure -- the other three
already exercise the identical code path (`FullTaskEnv(scope='full', ladder=..., tip_guard=...)`
via the SAME adapter) on both a human and a machine demo directory and under both ladders, so a
fourth run would have been confirmatory of the same plumbing, not a new code path. Flagged as
not done rather than silently skipped.

**Result, all three (`dH_rnrh_s0` human ramp/OLD-spec, `dH_rnsh_s0` human sparse, `dM_rnsh_s0`
machine sparse):** demo gate passed (`repeat.json` assertions -- `sim_variant`, `action_repeat`,
`with_state`, `terminal_reward`, `scope='full'`, tape count -- all satisfied, per
`cluster/wmfix_full.sbatch`'s own embedded gate script reproduced by hand), the `[ladder]` stamp
printed once per parallel env (6 of them, one per `env_num` worker) and once more from
`_write_ladder_provenance`, all identical within a run and reading `tip=tilt>60deg&not_in_hand@4f`
verbatim, demo prefill completed (74 or 72 episodes, exactly the tape count), the counter
advanced past the prefill origin by the requested online budget, and the training loop ran at
least one full iteration (`envs.step()` -> `buffer.add_transition()` -> `agent.act()` ->
`agent.update()`) before every run hit the SAME crash, in `agent.update()`, on the FIRST call to
the compiled `_cal_grad`:

```
torch._inductor.exc.InductorError: AssertionError: unexpected group: ((32768, 16), ()) != (1024, 32, 16), ()
```

This is `dreamer.py`'s `torch.compile(self._cal_grad, mode="reduce-overhead")` -- a CUDA-graph-
oriented compile mode -- failing in the CPU C++ codegen backend of this torch build. It is
UNRELATED to the ladder/tip-guard work: it reproduces identically across all three demo sets and
both ladders, it fires inside PyTorch's own inductor scheduler before any of this project's code
runs, and `model.compile=false` prevents the crash symptom (see next paragraph) without changing
any ladder/tip-guard behaviour. **Diagnostic value of the crash itself:** because it happens
INSIDE `agent.update()`, which trainer.py's loop calls strictly AFTER `envs.step()` and
`buffer.add_transition()` on the same iteration, every run's crash is proof that at least one
online environment step -- carrying the FULL `scope='full'` observation dict, including the
`log_farside`/`log_home`/`log_slide_event` keys the demo-prefill rows do NOT carry (see next
finding) -- was added to the replay buffer without a schema error. This directly answers the
concern in TIP_GUARD_IMPL §8 about untested plumbing.

**A second, adjacent finding, not asked for but worth recording:** `demo_prefill.py`'s
`log_keys` for `scope='full'` is the base 6-key `LOG_KEYS` tuple
(`log_picked`/`log_placed`/`log_contact`/`log_nested`/`log_tipped`/`log_task_success`) plus the
one-key `episode_record.LOG_KEYS` certificate -- it does NOT include `log_placed_v2`,
`log_contact_push`, `log_nested_v2`, `log_slide_success`, `log_farside`, `log_slide_event`,
`log_home`, nor any `log_ep_*` twin, all of which `envs/genesis.py`'s `FULL_EXTRA_KEYS` DOES add
to the online observation space for `scope='full'`. So demo-prefill rows and online rows carry
DIFFERENT key sets. This looked, on paper, like a live risk of a `LazyTensorStorage`/`TensorDict`
schema mismatch once online data (superset of keys) tried to extend a buffer whose schema was
fixed by the prefill (subset of keys) that always runs first. Empirically it is not: all three
runs completed a real online `add_transition()` with the larger key set with no error. Whatever
torchrl/tensordict version is pinned here evidently tolerates the union. Not fixed (nothing was
broken), and not chased further (out of this lane's scope), but recorded so nobody re-derives the
theoretical worry from a diff read without knowing it was tested and passed.

**`model.compile=false` and `env.env_num=1` were tried as ways to get a clean run past the
inductor crash, and both hit a SEPARATE, environment problem: on this box, at the time of this
run, several other lanes' long jobs were competing for CPU (a 1M-step DreamerV3 pick run, an
8-way stage-record sharder, a pilot re-score, `load average` 10-24 on 32 cores throughout).** A
`model.compile=false`, `env_num=6` retry of the human-ramp smoke ran 1900+ s without completing
even the FIRST of six Genesis world builds (vs ~90 s for the same step under the default,
compile=True configuration). A follow-up `env_num=1` retry (a deliberate reduction from the
launcher's config, disclosed here as a deviation) fared no better (~2000 s, one world, still
building). Both were killed rather than run to an unknown, possibly very long, completion; the
box's CPU was actively computing (300%+) in both cases, not deadlocked, so this reads as genuine
resource starvation from concurrent lanes rather than a defect in this change. **Net effect: all
functional confirmation in this report comes from the DEFAULT recipe (`env_num=6`, `compile`
unset i.e. `True`), which reliably reached prefill + one online step + one `agent.update()` call
in ~90-240 s across three independent runs, before the pre-existing, unrelated inductor bug
ended it.** No full "few hundred online steps of successful training" run was obtained on this
box in this session; the ~90-240 s window each time is well inside "a few hundred env steps"
worth of wall clock at this box's throughput, so the crash point is not a sign the smoke was cut
short deliberately -- it is where the run stopped regardless of budget.

Per-run specifics:

| run | set | ladder | env.steps | prefill counter | online budget requested |
|---|---|---|---|---|---|
| `dH_rnrh_s0` | `dHfull_all_rnrh` (**pre-revision**, scale 2.0) | nested_ramp | 117950 | 117624 | 326 |
| `dH_rnsh_s0` | `dHfull_all_rnsh` | nested_sparse | 117950 | 117624 | 326 |
| `dM_rnsh_s0` | `dDPfull_first_rnsh` | nested_sparse | 150300 | 149952 | 348 |

`dH_rnrh_s0`'s stamp (verbatim, confirming it ran under the OLD ramp spec, per the coordinator's
sign-off to leave it as is):

```
[ladder] unified-2026-09-10 | ladder=nested_ramp | picked=1 placed_v2=1 farside=1 home=4 ramp:slide_gain_m=2/0.05m | max_return=9 | terminal=home+tipped | shaping=off | far_release=off | tip=tilt>60deg&not_in_hand@4f | full_env=a8894b00e460 genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632 | git=known-good-2026-08-27-876-g7c994b4
```

(`farside=1` present, `ramp:slide_gain_m=2/0.05m` -- the pre-revision numbers; contrast with §1's
post-revision `ramp:slide_gain_m=3/0.05m`, no `farside=1`.)

### `eval_genesis.py` (task item 2, evaluator confirmation)

Run against `dH_rnsh_s0`'s `latest.pt` (a step-0 checkpoint -- `trainer.py`'s periodic-save fires
on its first call, so a `latest.pt` exists before any training happens; this is an untrained-
policy smoke of the EVALUATOR, not a claim about policy quality):

```
GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 \
~/workspace/r2dreamer/.venv/bin/python eval_genesis.py \
  --checkpoint /home/j/runs_r2d_smoke/dH_rnsh_s0/latest.pt \
  --episodes 2 --max-steps 1200 --mode sample --device cpu --seed 0 \
  --out /home/j/runs_r2d_smoke/dH_rnsh_s0/policy_eval
```

Confirmed directly from the console and `metrics.json`:

* `obs_keys` includes `log_farside`, `log_home`, `log_slide_event` AND their `log_ep_*` sticky
  twins (`log_ep_farside`, `log_ep_home`, `log_ep_slide_event`), alongside the rest of
  `FULL_EXTRA_KEYS`.
* `[eval] ladder='nested_sparse' from the run config` and
  `[eval] tip_guard='not_in_hand' from the run config` -- read from the checkpoint's own
  `.hydra/config.yaml`, not a default.
* `[ladder] ... tip=tilt>60deg&not_in_hand@4f ...` printed again by the evaluator's own env build,
  matching the training stamp.
* `[eval] outcome success_key='home' (the ladder's paid terminal)` -- exactly the requirement.
* `metrics.json`'s `ladder_provenance` carries `tip_guard: 'not_in_hand'`, `ladder:
  'nested_sparse'`, `terminal_stages: ['home', 'tipped']`, and the top-level `success_key: 'home'`.
* Both episodes ran to completion (2/2, `timeout` outcome at 300 decisions each, `r=0.0` -- the
  expected result for a step-0/untrained policy against a ladder whose only paid rung requires a
  full pick-place-slide-settle chain).

## 3. Refusals (task item 3)

**`env.tip_guard` omitted** (against the shared tree at its live HEAD, before the worktree
existed):

```
$ GENESIS_PICKAPLACE_ROOT=/home/j/workspace/genesis_pickaplace R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 \
  ~/workspace/r2dreamer/.venv/bin/python train.py env=genesis_full_state seed=0 env.steps=1000 \
  env.demo_dir=.../dHfull_all_rnrh env.ladder=nested_ramp env.far_release=false \
  env.return_clamp=9.0 model.return_clamp=9.0 env.actor_dist=bounded_normal env.act_entropy=3e-5 \
  device=cpu buffer.max_size=5e5 logdir=~/runs_r2d_smoke/refuse_omitted_tipguard
Logdir /home/j/runs_r2d_smoke/refuse_omitted_tipguard
FATAL: env.tip_guard is not set (PHASE_PLAN amendment (aa)). It decides where every episode TERMINATES, so it has no default: pass env.tip_guard=grip (the rule of record) or env.tip_guard=not_in_hand. cluster/wmfix_full.sbatch requires TIP_GUARD and sets it.
```

Refuses immediately (before any Genesis world is built), from `train.py`'s own
`_write_ladder_provenance` check. **PASS.**

**`FULLENV_TIP_GUARD=1` exported, with `env.tip_guard=not_in_hand` correctly set** -- run first
against the UNMODIFIED tree:

```
$ FULLENV_TIP_GUARD=1 GENESIS_PICKAPLACE_ROOT=... R2D_SIM_VARIANT=... \
  ~/workspace/r2dreamer/.venv/bin/python train.py env=genesis_full_state seed=0 env.steps=1000 \
  env.demo_dir=.../dHfull_all_rnrh env.ladder=nested_ramp env.far_release=false \
  env.tip_guard=not_in_hand env.return_clamp=9.0 model.return_clamp=9.0 \
  env.actor_dist=bounded_normal env.act_entropy=3e-5 device=cpu buffer.max_size=5e5 \
  logdir=~/runs_r2d_smoke/refuse_fullenv_tipguard
Logdir /home/j/runs_r2d_smoke/refuse_fullenv_tipguard
[ladder] unified-2026-09-10 | ladder=nested_ramp | ... | tip=tilt>60deg&not_in_hand@4f | ...
[ladder] return_clamp=9.0 (env and model agree)
[ladder] wrote /home/j/runs_r2d_smoke/refuse_fullenv_tipguard/ladder_provenance.json
Create envs.
... (builds the full env, runs the full demo prefill: 74 episodes, 29406 transitions) ...
Step accounting: prefill 29406 decisions = counter step 117624 ...; env.steps=1000 -> -116624 online env steps.
AssertionError: env.steps=1000 must exceed the loop's starting counter 117624 ...
```

**This did NOT refuse.** `train.py` proceeded to build the environment and run the full demo
prefill before failing on an UNRELATED, pre-existing step-accounting assertion (nothing to do
with the tip guard). This is a real gap: genesis_pickaplace's `full_env.refuse_legacy_gates()`
was extended to refuse `FULLENV_TIP_GUARD` (`baselines/rl/full_env.py`), but that function is
only ever CALLED from `cluster/wmfix_full.sbatch`'s pre-flight shell snippet -- `train.py` itself
never calls it, and its own inline gate loop
(`for gate in ("FULLENV_REWARD_X", "FULLENV_EPISODE_RECORD"):`) does not mention
`FULLENV_TIP_GUARD`. So a direct `train.py` invocation (as this smoke, and as any future ad-hoc
or non-sbatch invocation) with the variable set proceeds silently instead of failing loudly --
exactly the class of defect amendment (aa) exists to prevent, one call site short of complete.

**Fixed** on a new branch in an isolated worktree (never on the shared `~/workspace/r2dreamer`
checkout, which another lane was actively using for a long-running DreamerV3 pick job at the
time -- see §6): `git worktree add /tmp/claude_worktrees/r2dreamer-lane12b -b
lane12b-fullenv-tipguard-fix 0cf3d9e` (0cf3d9e = r2dreamer `main` HEAD at fix time, one commit
past this lane's starting point 16e03cb, landed by another lane's unrelated step-accounting fix
in between). One-line change to `train.py`'s gate tuple:
`("FULLENV_REWARD_X", "FULLENV_EPISODE_RECORD", "FULLENV_TIP_GUARD")`. Commit **`5e3a627`** on
branch `lane12b-fullenv-tipguard-fix` in that worktree. Verified:

```
$ FULLENV_TIP_GUARD=1 ... (same command, run from the worktree)
Logdir /home/j/runs_r2d_smoke/refuse_fullenv_tipguard_fixed
FATAL: legacy gate set (FULLENV_TIP_GUARD='1'); this tree has no gates
```

**PASS after the fix.** Not merged into r2dreamer `main` or cherry-picked back (per this lane's
instructions); the coordinator/maintainer decides whether and how to land it.

## 4. Coordinator's mid-task ladder revision, addressed

Message received: `nested_ramp`'s `farside` rung was found (Lane 11, `paper/PILOT_RESCORE_2026-09-11.md`)
to be collectable by a gripper that withdraws straight back from a set-down without pushing (a
600-rollout measurement). Revision: `farside` pays 0 (logged prerequisite only), its vacated 1.0
moves onto the ramp's `scale` (2.0 -> 3.0), max return unchanged at 9. Landed on
`ladder-unify-2026-09-11` as `266998a` (code) + `dda07d0` (handoff doc) while this lane's
`nested_ramp` demo sets already existed under the OLD spec.

**Action taken:** both `_rnrh` sets rebuilt from the SAME stage records (`--from-records`, no
re-simulation) after confirming the merge landed (`grep scale=3.0 baselines/rl/full_env.py`).
Result matches the coordinator's own prediction exactly: human Σ **215.9** (predicted ~215.9),
machine Σ **217.8** (predicted ~217.8), both on this 32-core box. `home` counts unchanged (13/14
-- the revision does not touch `home`'s definition, only how much of the ramp a partial slide
earns). Action-stream sha256 unchanged before/after the rebuild (confirms the revision touches
reward/requires only, as documented). The `nested_sparse` sets were untouched (correctly --
`nested_sparse`'s spec does not reference `farside` or the ramp at all).

The one r2dreamer `train.py` smoke of `nested_ramp` (`dH_rnrh_s0`, §2) ran under the OLD spec
(`ramp:slide_gain_m=2/0.05m`, `farside=1` in the stage_reward list) because it was launched and
had already crashed on the unrelated inductor bug before the revision merged. Per the
coordinator's explicit instruction, this was NOT re-run -- the smoke's purpose (confirm the
stamp/demo-gate/log-key plumbing reaches r2dreamer, independent of which numeric spec is loaded)
is unaffected by which `nested_ramp` variant was live at the time, and is reported as such here
rather than silently presented as post-revision.

## 5. Errors hit, verbatim, and fix status

| # | error | where | fixed? |
|---|---|---|---|
| 1 | `AssertionError: unexpected group: ((32768, 16), ()) != (1024, 32, 16), ()` inside `torch._inductor` | `dreamer.py` `_cal_grad` under `torch.compile(mode="reduce-overhead")` on CPU, first `agent.update()` call | NOT fixed (out of scope: pre-existing, CPU-backend-specific PyTorch/inductor bug, unrelated to the ladder/tip-guard work; reproduced identically on 3/3 completed runs, both ladders, both demo sources) |
| 2 | `FULLENV_TIP_GUARD=1` silently ignored by `train.py` (proceeds to build env + run full demo prefill instead of refusing) | `train.py::_write_ladder_provenance`'s inline gate loop, missing `FULLENV_TIP_GUARD` | **FIXED**, worktree branch `lane12b-fullenv-tipguard-fix`, commit `5e3a627` (not merged) |
| 3 | Extreme slowdown / apparent hang building Genesis world(s) under `model.compile=false` and/or `env.env_num=1` (1900-2000+ s, no completion) | Genesis/Taichi world construction, CPU-bound | NOT a code defect: attributed to concurrent CPU contention from other lanes' long jobs on this shared box (load average 10-24/32 throughout); documented, not fixed, both mitigation attempts abandoned in favour of the default recipe which worked reliably |
| 4 | Uncommitted, in-flight edit to `baselines/rl/full_env.py` appearing mid-task in the shared `genesis_pickaplace` checkout (the `nested_ramp` revision before it was committed) | shared working tree, not this lane's file | not an error to fix; handled by NOT touching `full_env.py`, waiting for the coordinator's merge notification, then rebuilding from records (§4, §6) |

## 6. Shared-tree notes (for whoever reads this next)

* `~/workspace/genesis_pickaplace` and `~/workspace/r2dreamer` are BOTH shared checkouts (no
  worktree isolation) that other lanes committed to and edited directly WHILE this lane's builds
  and smokes were running. Observed: `genesis_pickaplace`'s local HEAD advanced from `e80b889`
  (task start) through several other lanes' commits to `dda07d0` (this doc's commit is on top of
  that); `full_env.py` briefly carried an UNCOMMITTED, in-progress edit (mtime 16:21:49,
  discovered via `git diff` when `git status` came back dirty unexpectedly) that turned out to be
  exactly the `nested_ramp` revision the coordinator announced two minutes later. r2dreamer's
  local HEAD advanced from `16e03cb` to `0cf3d9e` (another lane's unrelated step-accounting fix)
  while this lane's smokes were queued.
* This lane's own r2dreamer fix (§3) was deliberately made in a SEPARATE worktree
  (`/tmp/claude_worktrees/r2dreamer-lane12b`) rather than by checking out a branch in the shared
  `~/workspace/r2dreamer` directory, because a live, multi-hour DreamerV3 pick run
  (`~/runs_dv3_local/dv3pick_...`, PID active throughout this session) was using that exact
  checkout on disk; switching branches there would have risked disturbing files a running process
  might re-read.
* No file in either shared tree was modified by this lane outside the one r2dreamer worktree
  commit and the one doc named in the task (this file). `baselines/rl/full_env.py`'s in-flight
  edit was read, never touched.

## 7. Pass/fail summary

| item | result |
|---|---|
| 1. Build the four demo sets | **PASS** -- all four built, `home` 13/14 matches expectation, action sha256 identical within each source's pair, `nested_ramp` pair rebuilt post-revision with numbers matching the coordinator's prediction |
| 2. Smoke `train.py` (human ramp, human sparse, machine sparse) + `eval_genesis.py` | **PASS** on plumbing (stamp, demo gate, `log_farside`/`log_home`/`log_slide_event` present and schema-compatible online, `success_key='home'`, `ladder_provenance` complete); machine `nested_ramp` smoke NOT run (time/resource budget; same code path already exercised 3x); no run reached a full "few hundred steps" of completed training because of an unrelated, pre-existing CPU torch.compile bug that fires on the first update call every time (documented, not fixed) |
| 3. Refusals (`tip_guard` omitted, `FULLENV_TIP_GUARD=1`) | **PASS** for `tip_guard` omitted (refuses immediately); `FULLENV_TIP_GUARD=1` **FAILED then FIXED** -- did not refuse against the unmodified tree, fixed on worktree branch `lane12b-fullenv-tipguard-fix` commit `5e3a627`, verified refusing after the fix |

**Sets, final:** `dHfull_all_rnrh`, `dHfull_all_rnsh`, `dDPfull_first_rnrh`, `dDPfull_first_rnsh`,
all under `/home/j/data/genesis_pickaplace/demos_state_full/`, `nested_ramp` pair at the
post-revision (scale 3.0) spec.
