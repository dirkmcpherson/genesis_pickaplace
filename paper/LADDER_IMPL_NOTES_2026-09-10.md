# Ladder unification — Lane 2 implementation notes (2026-09-10/11)

Implements `paper/LADDER_UNIFY_BRIEF_2026-09-10.md` D1, D2, D5, D6, D7, D8 plus the
Lane-1 interface, the coordinator's two corrections, Lane 3's blockers, and the user's
2026-09-11 sparse-ladder addition.

Branch `worktree-agent-af1123f4c6cbf6bc7` (worktree), commits `f841cb6`, `f8cd6a5`,
`3ad2676`, `401c8d2`. The r2dreamer tree `~/workspace/r2dreamer` is committed separately at
**`4185e11`** (sync + ladder) and **`4410bed`** (sparse ladder wiring).

> **Worktree base was wrong and was corrected.** This worktree was branched from `b2113cd`
> (2026-07), not from `3ad144f`. The working tree was clean, so it was reset to `3ad144f`
> before any edit. Anything cut from this branch before that reset is not this work.

No cluster tree was modified. No Slurm job was submitted. All cluster access was read-only
(`scp` / `ssh ls`). Everything below ran on CPU; the local GPU was not used.

---

## 1. What changed, per file

### `baselines/stage_predicates.py` — NEW, and it is a **STUB**

Lane 1 owns this file and replaces it wholesale. It implements the brief's D3 text
literally and nothing else, and says so loudly in its own docstring and in an `IS_STUB`
flag. **The constants are the brief's starting values, not calibrated ones** — in
particular `HELD_LEVER_M = 0.025` is the brief's "start at 2.5 cm", NOT a measured
separation. Nothing in it has been validated against `nested_honest`, the (l) predicate,
the 74 human census tapes or any policy rollout. **No number produced with this file
belongs in a table.**

It exports exactly the brief's interface (`NESTED_TOUCH_DIST`, `AT_REST_MM`,
`AT_REST_FRAMES`, `PUSH_GAIN_MM`, `HELD_LEVER_M`, `StageTracker(goal_xy, shelf_top_z)`
with `reset()` and the keyword-only `update(...)`), plus `FLAG_KEYS` / `DIAG_KEYS` so a
consumer can assert the contract. It imports nothing that pulls Genesis in, so it is
unit-testable offline; its local `tilt_from_quat` is asserted equal to
`replay_harness.tilt_deg` in the tests.

`grip_cmd` is accepted and deliberately **unused** by every predicate: PHASE_PLAN (p)
withdrew (l)'s `grip < 0.3` clause after it passed 2 of 74 human demonstrations.

### `baselines/rl/full_env.py`

* **(a) One ladder, no gate.** `FULLENV_REWARD_X` is gone — not defaulted, *removed*.
  `refuse_legacy_gates()` raises `SystemExit("legacy gate set …; this tree has no gates")`.
  `CartesianFullTaskEnv` keeps its own `_CARTESIAN_STAGE_REWARD` (the old ladder): it is a
  different arm, not part of this unification, and its behaviour is unchanged.
* **(a′) Two named ladders (user, 09-11).** `ladder='staged'` (default) and
  `ladder='sparse'`, a constructor argument. `LADDERS` / `ladder_spec()` / `max_return()`
  are the single definition; an unknown name raises. `self.stage_reward` and
  `self.terminal_stages` drive the reward loop and the terminal, so a run can never pay one
  ladder and terminate on another.
* **(b) The tracker is fed once per env frame, after the sim step**, from the new
  `_full_scope_predicates()`. This required reordering: `placed_v2` used to be computed at
  the BOTTOM of `_step_once`, after the reward loop, so the ladder could not condition on
  it — and both `contact_push` and `slide_success` require it to have been granted.
* **(c) `contact_push` is granted only from the tracker** in full scope. The env's own (g)
  predicate (no release required) is kept as `contact_push_legacy`, and (l)'s
  `slide_success` as `slide_success_legacy`. Neither pays, neither terminates.
* **(d) Termination.** In `scope='full'`: **the ladder's own terminal stage(s), or the tip
  rule. Nothing else.** `terminated = bool(info.get('nested'))` is gone for full scope.
  **Other scopes are untouched**: `place` still never terminates on the proxy, `pick`
  terminates on the pick, `contact`/`carrycontact` on their grant, and the proxy line is
  kept verbatim for every non-full scope (`self.scope != 'place'`). In practice it was
  already dead there — `nested` needs `contact`, which needs `picked`, which terminates
  `pick` first — but it was not this lane's call to change it.
* **(e) Legacy stages stay.** `nested` (proxy), `placed`, bare `contact` are still
  computed, still enter `_granted`, and pay nothing. A new `LOGGED_STAGES` loop enters
  *every* stage into `_granted` whether or not it pays — previously only stages named in
  `STAGE_REWARD` got grant bookkeeping, which is the real reason `contact_push` read
  0.000. `nested_v2` and the tracker diagnostics (`released`, `pushed`, `in_hand`,
  `at_rest`, `goalward_gain_m`, `lever_m`) are added to `info`.
* **(f) D7 `goalward` shaping.** Constructor argument, default off, never an env var, in
  the stamp. Potential-based, applied once per *decision* (the 08-19 timescale lesson),
  φ(terminal)=0. **Documented caveat, in the code**: the gate makes φ discontinuous at its
  boundary, so entering the gate costs −scale·d and leaving refunds it. Bounded and
  sign-consistent, but it is shaping, and any arm run with it must be disclosed.
* **(g) D8 episode record always on.** Gate removed.
* **(h) D6 `ladder_provenance()` / `ladder_stamp()`**: ladder name, stage_reward,
  terminal_stages, logged_stages, max_return, return_clamp_required, shaping, sha256 of
  `full_env.py` / `genesis_can_env.py` / `stage_predicates.py`, `git describe`. Printed at
  construction as `[ladder] …`.

**Two regressions restored** (independently flagged by Lane 3). Commit `809601d` copied
`full_env.py` in wholesale from another tree and silently reverted `7096fe6` (amendment
(w)) and `20f49f8` (the PHASE_PLAN (p) `contact_grant` selector and the contact-scope grant
dispatch). Since `train_rlpd.py:298` passes `contact_grant=` on **every** scope,
`FullTaskEnv` construction raised `TypeError` for every `train_rlpd` run on this branch.
Both are restored verbatim, with (w) ungated per D8. `test_9` binds the constructor
signature against every real call site so this cannot recur silently.

**One deletion** (Lane 3 item 4): the 2026-09-09 "contact_push exposure fix" was a no-op —
`genesis_can_env.step()` has always written `contact_push` into its info dict in both live
trees. Deleted, with the real cause recorded in its place.

### `baselines/genesis_can_env.py`

Adds per-frame `can_goal_touch` / `gripper_goal_touch` to `info` (instantaneous, unlike the
sticky `contact` / `contact_push`), and hoists the gripper↔goal solver read out of the
`picked and bg_touch` branch that owned it so the value exists on every can↔goal contact
frame. One read, in one place, instead of `full_env` re-reading the same solver state.
Logging only; state reads do not perturb the solver (the #26 trace ablation).

### `baselines/rl/train_rlpd.py`

Refuses the legacy gate at start; writes `<logdir>/ladder_provenance.json`; new `--ladder`
and `--goalward-shaping`; `nested_v2` added to the episode-record flags; the Q watchdog
scales with **this run's** ladder ceiling (2 × 8.0 staged, 2 × 1.0 sparse — a watchdog keyed
on the wrong ceiling either never fires or always does).

### Evaluators — `baselines/eval_e2e.py`, `baselines/eval_e2e_annot.py`, `cluster/make_r2_annotator.py`, r2dreamer `eval_genesis.py`

* Stage lists split into `HEADLINE_STAGES` (picked, placed_v2, contact_push,
  slide_success, nested_v2, nested_honest) and `LEGACY_STAGES` (placed, contact,
  nested_proxy, contact_push_legacy, slide_success_settle). `nested_proxy` is out of every
  headline and carries a `stage_notes` entry saying why (precision 0.114 human / 0.029
  machine; it REVERSES the arm ordering).
* `slide_success` is the env's **in-episode** value. The post-episode settle still runs, for
  `nested_honest` (the reference `nested_v2` is validated against) and the legacy settle
  column. In the r2dreamer adapter and evaluator the settle no longer **overwrites**
  `info['slide_success']` in the full scope — that would have fed the sticky emission a
  different predicate from the one the reward paid.
* `ladder_provenance` + `ladder_stamp` into every `metrics.json`; `--ladder` taken from the
  checkpoint sidecar when it records one and refused if they disagree.
* Outcome taxonomy `slide_success | tipped | timeout`.
* Annotator chips follow the ladder: PICK / PLACE / PUSH / NEST2 / SLIDE. The withdrawn
  proxy is deliberately no longer a chip. `make_r2_annotator.py` takes src/dst as argv
  (its defaults still point at `$W/r2dreamer_fix`, which **writes into an in-flight tree** —
  pass explicit paths once `gp_unified` exists).

### `baselines/e2e_table_all.py`

Refuses to merge rows whose `ladder_stamp` differs, printing which seeds sit on which
stamp. Cells that predate the stamp read `unstamped` and the row is labelled **LADDER
UNSTAMPED** — "unstamped" is not "the same ladder"; those are exactly the cells whose
objective cannot be recovered from the artefact. `nested_v2` joins the default stage list.

### `baselines/rl/relabel_reward.py` — rewritten (D5)

The previous version scored tapes with its own offline predicates. That is the same defect
one level up: the demo buffer then pays a ladder the environment cannot (the `_rx` prefill
pays +4 on 13 of 74 human tapes for an event the (x) env could never pay). It now
re-executes each action stream through `FullTaskEnv(scope='full', ladder=…)` on the
training code path, so `tape reward == env reward` by construction. There is no predicate
in the file.

* Two input layouts: the r2dreamer-native **segment** set both launchers assert on
  (`state`/`action`/`reward`/`is_terminal` + `repeat.json`) — the primary — and contract-v1
  recorder tapes. Set provenance comes from `repeat.json`, asserted, never defaulted.
* IC restored through the env's own reset and **verified** against the tape's first state
  (2 mm tolerance, refused otherwise).
* `actions_delta` / `action` sha256 asserted unchanged per tape and over the set; only the
  reward column is written, plus `rz_*` diagnostics.
* Suffix asserted against `--ladder`: `_rz` staged, `_rs` sparse.
* ≤ 8 worker processes, one Genesis world each, driven by re-launching itself with
  `--shard`.

### Launchers

`cluster/sbatch_rlpd_e2e.sh`, `cluster/sbatch_dp_e2e.sh`, `cluster/wmfix_full.sbatch`
(ported from `$W`, which is **not** modified):

* `GENESIS_PICKAPLACE_ROOT` **required**, no `:=$PWD` / `${GP_ROOT:-…}` default. That
  default *is* defect 1: it silently selected `gp_e2e`, the tree with no gate.
* Legacy gates refused.
* The `[ladder]` stamp is read **from the tree the job imports** and printed before
  training. The 09-09 version echoed the gate *values*, which record what was exported, not
  what was loaded.
* `LADDER=` required; `wmfix_full.sbatch` derives `env.return_clamp` **and**
  `model.return_clamp` from the ladder's max return and prints both.
* The requeue guard stays fixed (defect 4): the existence check yields on a restart.
* Per Lane 3, the two e2e launchers are rebased on the **gp_e2e** copies (ahead of the
  repo): the `dDPfirst` arm and its `one_per_ic_first` manifest assert, which every live
  {RLPD} run depends on. `cluster/table_arms.py` and `cluster/eval_fixes_first_flag.py`
  were missing from the repo entirely and are now in it.
* One extra fix: the RLPD disk guard sat behind `FREE_GB=$(df … )` under
  `set -eo pipefail`, so off-cluster the script died *before* the guard could say why —
  the same "an earlier line makes the handler unreachable" family as defect 4.

### `~/workspace/r2dreamer` (commit `4185e11`)

The local tree was at `6af0ec7` (2026-08-19) and ~146 lines behind the live adapter, with a
large uncommitted working tree. Editing it as found would have been editing a tree that is
not the one that ran. It was synced from the cluster trees of record first, with every
source hashed (§5), then the ladder changes applied on top.

**Neither cluster tree was a superset**, so both halves were merged:

| | `$W/r2dreamer_fix` | `release_v4/r2dreamer` |
|---|---|---|
| stage emission | sticky `log_ep_<k>` every step (correct under an outer-wrapper truncation) | `log_<k>` gated on `done` (logs zeros for a truncated episode that had picked) |
| episode record | absent | `envs/episode_record.py`, wired through `envs/wrappers.py` |
| online-budget contract | absent | `longrun_milestones.py`, `R2_LONG_RUN=1`, milestones at 2M/4M |

Kept: r2fix's sticky emission as the **values**, release_v4's module as the **certificate**
(`log_ep_record_valid`), the `wrappers.py` call site (the one exit both termination and the
outer timeout reach), `longrun_milestones`, and the trainer's end-of-budget flush (without
it the last episode of every run is dropped from the curves). `demo_prefill` zero-fills the
certificate column so the buffer key set matches on both halves — a zero there means "no
record", not "a zero outcome".

`train.py` refuses the legacy gates, requires `GENESIS_PICKAPLACE_ROOT` for the genesis
suites, and writes `<logdir>/ladder_provenance.json` with
`budget_unit = online_sim_steps` under `R2_LONG_RUN`.

---

## 2. What I ran, and what it produced

Commands were run from the worktree with
`/home/j/workspace/genesis_sim2real/venv/bin/python`.

### 2.1 Unit tests

```
$ .../venv/bin/python baselines/tests/test_terminal_guard.py        # pre-existing, must keep passing
1. tipped fail tape … OK   …   8. phi definition + demo sha  OK
ALL OK

$ .../venv/bin/python baselines/tests/test_ladder_unified.py        # new
1. held can pressed into the goal: reward 1.0 (picked only), no terminal  OK
2. release -> push -> nest: reward 8.0 (1+1+2+4), terminates on slide_success  OK
3. nested_v2 / legacy proxy: logged, unpaid, non-terminal; tip still terminates  OK
4. provenance stamp present, stable, ladder- and shaping-sensitive  OK
5. FULLENV_REWARD_X refused; no reward/record gate is read anywhere  OK
6. goalward shaping: off by default, constructor-only, exact potential form  OK
7. StageTracker interface + tilt parity with the env  OK
8. stub predicates: carry != push; release+push+rest -> nested_v2 + slide  OK
9. constructor signature binds every real call site (contact_grant restored)  OK
9a. sparse ladder: same episode pays 8 staged / 1 sparse, same logged stages  OK
9b. nested_v2 alone: 0 and non-terminal under staged, 1 and terminal under sparse  OK
ALL OK
```

**`pytest` is not installed in that venv** (nor in any venv on this box, nor system-wide),
so `python -m pytest baselines/tests -q` cannot run here. The test file is written to be
collectable by pytest (module-level `test_*` functions, no fixtures) *and* runnable as a
script, and was run as a script. Lane 4 should run the pytest form once on the cluster.

The ladder tests drive `FullTaskEnv` with `__new__` + a fake `genv` and a **fake tracker**,
so they test the reward loop and the terminal, not the predicates — they keep passing when
Lane 1 replaces `stage_predicates.py`. Tests 7/8 exercise the stub through its documented
interface only.

### 2.2 D5 dry run — re-execution of real human full-task tapes

Source: 6 tapes + `repeat.json` copied read-only from
`$W/demos_state_full/dHfull_all` (the segment set both launchers assert on), with their
`_rx` siblings as the reference. **No full-scope tapes exist on this box** — `~/wm_fix…`
and `baselines/demos_v2` are not present here — so they were fetched to the scratchpad.
An earlier pass used the 7 contract-v1 tapes under `src_dHfull_all`; both layouts work.

```
$ .../venv/bin/python baselines/rl/relabel_reward.py \
    --in <scratch>/seg_dHfull_all --out <scratch>/seg_dHfull_all_rz --ladder staged --procs 3
```

| tape (rollout uid) | decisions | recorded R | re-exec R | `_rx` R | end | can dev | joint dev |
|---|---|---|---|---|---|---|---|
| 100000 | 255 | 3 | **6** | 8 | `slide_success@202` | 4.5 mm | 0.00040 rad |
| 100001 | 170 | 1 | 2 | 2 | stream_exhausted | 78.3 mm | 0.00086 |
| 100002 | 412 | 1 | 2 | 2 | stream_exhausted | 155.1 mm | 0.00185 |
| 100003 | 228 | 1 | 2 | 2 | `tipped@228` | 2.0 mm | 0.01129 |
| 100004 | 387 | 1 | **6** | 6 | `slide_success@333` | 4.1 mm | 0.00127 |
| 100005 | 240 | 1 | 2 | 2 | stream_exhausted | 8.4 mm | 0.01185 |

Same set under `--ladder sparse` (`_rs`): identical grants, identical episodes,
Σ reward **2.0** instead of 20.0, terminals `nested_v2@202` / `nested_v2@333` — i.e. the
same decisions the staged ladder ends on, as expected since `slide_success` requires
`nested_v2`.

Structural contract, checked against **both** the source and the `_rx` reference:

```
ALL STRUCTURAL CHECKS PASS: actions and states byte-identical to both the source and
the _rx reference; only `reward` differs.
```
(12 added `rz_*`/`reward_*` keys; every other key byte-identical.)

**Three findings worth keeping:**

1. **The +4 rung is payable in-episode.** 2 of 6 human tapes reach `slide_success` and the
   episode terminates there. That is the thing audit-brief defect 5 says was impossible
   under (x), where the unpaid proxy ended the episode on the slide window's first frame.
2. **Re-execution agrees with the (x) offline classifier on 5 of 6 tapes** and differs on
   one: uid 100000 is `_rx` 8 vs re-exec 6 — the classifier granted a `contact_push` the
   env does not (the new rung requires a prior release *and* the far-side geometry).
3. **Trajectory fidelity is a per-tape property and must be checked.** The arm re-executes
   faithfully everywhere (worst joint error 0.012 rad over 170–412 decisions), but the CAN
   diverges after contact on 2 of 6 tapes (78 mm and 155 mm; the other four are 2–8 mm).
   Those tapes were recorded on the cluster and re-executed here, and full-scope outcomes
   are known to track machine size. **A relabel must run on the hardware class the tapes
   were recorded on**, and `can_dev_max_m` per tape is how a reader checks that it did.

**A diagnostic I got wrong first, and corrected.** My first fidelity metric was
`max |state − state_hat|` over all 17 channels, which read 1.8–5.4 and looked like a
catastrophic divergence. Broken out per channel it is `grip_effort` — a summed control
force of order 1–5 N — dominating everything while the arm tracked to 0.0004 rad. The
metric is now reported per channel group. Had I reported the aggregate, the conclusion
("the re-execution does not follow the recording") would have been wrong.

### 2.3 Launcher guards

```
$ GENESIS_PICKAPLACE_ROOT= ARM=dH SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh
cluster/sbatch_rlpd_e2e.sh: line 51: GENESIS_PICKAPLACE_ROOT: set GENESIS_PICKAPLACE_ROOT …

$ GENESIS_PICKAPLACE_ROOT=$PWD FULLENV_REWARD_X=1 ARM=dH SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh
FATAL: legacy gate set (FULLENV_REWARD_X); this tree has no gates

$ GENESIS_PICKAPLACE_ROOT=<worktree> ARM=dH SEED=0 DRYRUN=1 bash cluster/sbatch_rlpd_e2e.sh
FATAL: /cluster/tufts/shortlab free ? GB < 150 GB floor -- refusing to train   # expected off-cluster
```

`bash -n` passes on all three launchers.

---

## 3. What is STUBBED

`baselines/stage_predicates.py`. Everything `contact_push`, `pushed`, `nested_v2` and
`slide_success` mean, on every path — env, both evaluators, both annotators, the relabel —
comes from that one module, so **every stage number in §2.2 moves when Lane 1's calibrated
module lands.** Specifically:

* `HELD_LEVER_M = 0.025` is uncalibrated. It decides `in_hand`, which gates `pushed`,
  `nested_v2` and the D7 shaping. Lane 1 reports the measured separation.
* `AT_REST_MM / AT_REST_FRAMES` decide when a can counts as arrived. In the dry run,
  `nested_v2` fired on 2 of 6 human tapes; whether that is right is a Lane-1 question.
* `PUSH_GAIN_MM = 10` decides `pushed`; it fired on 3 of 6.

---

## 4. What is UNVERIFIED

* **Nothing has been trained.** No learner has run against this ladder. Lane 4's cluster
  smoke is what proves both learners start, stamp identically, and agree on the objective.
* **The r2dreamer tree has not been executed at all** — no GPU budget and no demo set on
  this box. It compiles (`py_compile` on every edited file) and the merge is hunk-for-hunk
  from two trees that each ran, but "it runs" is unproven. In particular:
  `envs/episode_record.LOG_KEYS` is now one column (`log_ep_record_valid`) rather than
  release_v4's seven, because r2fix's sticky twins already declare `log_ep_<stage>` and the
  two schemes would have collided on those names. That reasoning is sound on paper and
  untested in a buffer.
* **`pytest` could not be run** (§2.1).
* **The D5 dry run is 6 tapes on one machine**, and 2 of those 6 have a diverging can
  (§2.2 finding 3). It demonstrates the pipeline; it is not a validated set build.
* **`is_terminal` is left as recorded** in relabelled sets. The brief says the reward column
  only, and the launchers assert a terminal may appear only on the last row — marking the
  re-execution's terminal would violate that without also truncating the tape, which would
  change the action stream. So a relabelled tape can carry a recorded terminal at a
  different decision from the re-execution's. It is reported per tape (`rz_end_reason`,
  `rz_end_decision`) and counted in the manifest. **Open decision for the coordinator.**
* **Whether any existing checkpoint can be re-scored under this ladder.** Stage columns can
  be recomputed, but the ladder change is a different MDP (different terminal), so training
  curves cannot be relabelled without re-simulating. Not attempted.
* The **evaluators have not been run end to end** (they need a checkpoint; none is local).
  Only `py_compile` and the env-construction path via the relabel exercise their imports.

---

## 5. Cluster files used, with sha256 (all read-only)

`$W = /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03`,
`$LAB = /cluster/tufts/shortlab/jstale02`.

| source | file | sha256 |
|---|---|---|
| `$W/r2dreamer_fix` | `envs/genesis.py` | `7f7bd8280200471a1f64d45ad23c1671adf9717b13653aee94b4ea4b78f5d6ab` |
| | `eval_genesis.py` | `8401f3309e3446c0e40a17d3b2d86151ec1be80d96ae6686d3bbcbd74c95c8f1` |
| | `demo_prefill.py` | `75e64bcfe6f7b812a7496c1850286d6cd7b540619bb01de84785caf28a915bcb` |
| | `envs/__init__.py` | `dc0259f10009044b71db55e2981a65a85f63eccafcf1726290501b5c34872ffd` |
| | `configs/env/genesis_full_state.yaml` | `43831e6c0814e07cc7f0992dc0dad1c95da3a28e38249f081d77aa42b5537086` |
| | `configs/env/genesis_pick_state.yaml` | `fb64e24573ebbc4a106f3201a78dc61cd6073ef1fc2cb26862138ade64eab317` |
| | `trainer.py` | `924617eb5bdcaa5e1d7fc97d871013a841d7993a40b82d432711345af546a093` |
| `$W/…/release_v4/r2dreamer` | `envs/episode_record.py` | `51236b062a142631dabb2d2b8dd1f77c21c31fd2ffece038629807d38d6d2128` |
| | `longrun_milestones.py` | `386c8b9be291ae105f0c32c923224b7bbee7cc2d18dbcac3208e4e8b515a202f` |
| | `trainer.py` (reference) | `7f60351efd8198bfda57be091b8c496bbaf72e8757493d254fe08939e50fdd84` |
| | `envs/genesis.py` (reference) | `a11aacd7c001e29505ccead3a6623e38a666b6820d680ef58f548f7ffe4354b7` |
| | `envs/wrappers.py` (reference) | `39924dfa42260c3b66002d5d729162a0307474896f2823818652898c9ffc63be` |
| | `demo_prefill.py` (reference) | `5b2de24577a21270ad100d6b66ce74a76a8cc29147cfb4d6fdb9f0c85213e447` |
| `$LAB/gp_e2e/cluster` | `sbatch_rlpd_e2e.sh` | `76a35dfe79e080485d26a6314d7125f0bff5b578df0919e154e3c761da00a8cb` |
| | `sbatch_dp_e2e.sh` | `78de86186fee779a4052214541d342c3cf6ad9ddc4dc6cc7f8b9b8cb81e4fa06` |
| | `eval_fixes_first_flag.py` | `31836da06f79c5d48749e63913b2277569098f0106f44f9eff2956f648770fc1` |
| | `table_arms.py` | `0330cba9d580645dcffbca532542f7424781d7933dc3d07b60387d85890d475e` |
| `$W` | `wmfix_full.sbatch` (as fetched) | `0c3f1052c4eed6e37bf6c3d7a0d81306b07527e021478fc862847b4b312fcef4` |

Verified byte-identical between `~/workspace/r2dreamer` and `$W/r2dreamer_fix` before
committing, and therefore committed unchanged: `buffer.py`, `dreamer.py`, `tools.py`,
`networks.py`, `distributions.py`, `configs/configs.yaml`, `trainer.py`, `train.py`.

Demonstration tapes copied to the scratchpad for §2.2:
`$W/demos_state_full/dHfull_all` (6 of 74 + `repeat.json`), its `_rx` sibling, and
7 contract-v1 tapes from `$W/demos_state_full/src_dHfull_all`.

---

## 6. Where the brief could not be followed, and why

1. **"A stub is acceptable until merge" vs "do not implement the predicates yourself."**
   A stub that returns nothing makes the ladder untestable end to end. I wrote the brief's
   D3 text literally, with no calibration and no tuning, marked `IS_STUB`, and made the
   ladder tests independent of it (fake tracker) so Lane 1's module drops in without
   touching them. §3 lists exactly what moves when it does.
2. **`pytest` is not installed anywhere on this box** (§2.1). The tests run as a script.
3. **No full-scope tapes exist locally**, so the D5 dry run used tapes fetched read-only
   from the cluster (the coordinator confirmed this in-flight). They are local copies, but
   they were not already here.
4. **`is_terminal` is not updated** in relabelled sets (§4) — the brief says reward only,
   and updating it would break the launchers' "terminal only on the last row" assert.
5. **`cluster/make_r2_annotator.py` still defaults to writing into `$W/r2dreamer_fix`.** It
   generates a *new* file there, which is still a write into an in-flight tree. I made the
   paths arguments and documented it rather than changing the default, which would break
   the existing workflow mid-batch. Lane 4 should pass explicit paths.
6. **`full_demos.py` keeps its own `STAGE_REWARD` copy.** It is numpy-only by design (no
   Genesis import) and uses it only for `_reachable_reward_sums`, the validator that
   refuses a tape whose reward is not a sum of ladder rungs. Checked rather than assumed:
   the staged magnitudes are unchanged (1/1/2/4 → reachable {0…8}), and the sparse ladder's
   only values are 0 and 1, both inside that set — so **no relabelled set is rejected by
   it** today. It is still a second copy of a definition D1 says should be single, and it
   would silently accept a wrong value under a future ladder with new magnitudes. Flagged,
   not changed (it is outside this lane's file list).
