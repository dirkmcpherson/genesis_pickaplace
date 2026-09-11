# Tree reconciliation for D1 ("one code tree"), 2026-09-10

Lane 3 of the ladder-unification effort (`paper/LADDER_UNIFY_BRIEF_2026-09-10.md`). Read-only
everywhere except this file. No cluster tree was modified; no Slurm job was submitted.

Repo under audit: `/home/j/workspace/genesis_pickaplace`, branch `e2e-longrun-2026-09-10`,
HEAD `3ad144f` (`git rev-parse HEAD`). Cluster: `ssh -o BatchMode=yes jstale02@login.pax.tufts.edu`,
`LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`.

## 0. Method, and what this document does NOT establish

Every cluster file named in §1 was copied to a local scratch directory with one `tar`-over-`ssh`
call per tree and diffed locally, so every hunk below is reproducible:

    ssh jstale02@login.pax.tufts.edu \
      "cd $LAB/gp_e2e && tar czf - baselines/rl/full_env.py baselines/genesis_can_env.py ..." \
      > gp_e2e.tgz

Repo-side versions come from `git show HEAD:<path>`, not the working tree, so concurrent Lane-1
and Lane-2 worktree edits cannot contaminate the comparison. The working tree was clean for all
tracked files at session start (`git status`: 7 untracked paths only).

**Coverage limits, stated rather than assumed.**

* Only the 12 + 8 files named in the task were diffed hunk-by-hunk. A whole-tree **file-set and
  content** comparison of `gp_e2e` against repo HEAD was also run (§2) and is complete for
  `baselines/`, `cluster/`, `can_pos_recovery/`; **no whole-tree comparison was run for
  `gp_root`, `release_v4/gp`, `r2dreamer_fix` or `release_v4/r2dreamer`.** Sameness outside the
  named files in those four trees is UNVERIFIED.
* `baselines/replay_harness.py` does not exist in any tree; the file is
  `can_pos_recovery/replay_harness.py`. It was diffed under that path.
* `baselines/rl/relabel_reward.py` exists only in the repo and as a loose copy at
  `$W/relabel_reward.py`; no gp tree carries it (`find $LAB -maxdepth 4 -name relabel_reward.py`).
* `cluster/e2e_posthoc_sweep.sh` exists in the repo and as a loose copy at
  `$W/e2e_posthoc_sweep.sh`; no gp tree carries a `cluster/` copy.
* `cluster/wmfix_full.sbatch` does not exist in any gp tree. The live launcher is the loose file
  `$W/wmfix_full.sbatch` (pre-fix copy kept as `$W/wmfix_full.sbatch.bak_0910`).
* `gp_root` and `release_v4/gp` contain **no `cluster/` directory at all**, and no
  `baselines/eval_e2e.py` or `baselines/rl/full_demos.py`. They are world/env trees for the
  r2dreamer adapter, not launch trees. Confirmed by `ls`.
* Local `~/workspace/r2dreamer` has **no** `configs/env/genesis_full_state.yaml` or
  `genesis_pick_state.yaml`, so those two pairs could not be diffed against local.
* Only `$LAB/gp_e2e` and `$LAB/genesis_pickaplace` are git repositories. `gp_root`,
  `release_v4/gp`, `release_v4/r2dreamer` and `r2dreamer_fix` are **plain rsynced directories**
  with no history — retiring them destroys content that exists nowhere in git unless it is ported
  first.

### 0.1 sha256 matrix (first 8 hex chars), genesis_pickaplace side

    for t in repo gp_e2e gp_root rel_gp; do sha256sum $t/<path>; done

| file | repo HEAD | gp_e2e | gp_root | release_v4/gp |
|---|---|---|---|---|
| `baselines/rl/full_env.py` | `80c0ab72` | `b796f3b3` | **`80c0ab72`** | `b019d643` |
| `baselines/genesis_can_env.py` | `b093bb13` | **`96a668e0`** | **`96a668e0`** | `ad2aa913` |
| `baselines/eval_e2e.py` | **`bbfe59e6`** | **`bbfe59e6`** | absent | absent |
| `baselines/rl/train_rlpd.py` | `cc55ddbe` | `202b7264` | **`7ba5f506`** | **`7ba5f506`** |
| `baselines/rl/full_demos.py` | `abab04dc` | `dcdfafd0` | absent | absent |
| `baselines/rl/relabel_reward.py` | `cc4419f6` | absent | absent | absent |
| `baselines/rl/to_dreamer_native.py` | **`530f9361`** | `956f4357` | **`530f9361`** | `c9bd0e40` |
| `baselines/sim_variants.py` | `85b303cf` | **`9728c26b`** | **`9728c26b`** | **`9728c26b`** |
| `can_pos_recovery/replay_harness.py` | **`1c42f74d`** | **`1c42f74d`** | **`1c42f74d`** | **`1c42f74d`** |
| `baselines/record_demos.py` | **`34a673e9`** | **`34a673e9`** | **`34a673e9`** | **`34a673e9`** |
| `cluster/sbatch_rlpd_e2e.sh` | `c3114dcf` | `76a35dfe` | absent | absent |
| `cluster/e2e_posthoc_sweep.sh` | `fc5e3241` | absent | absent | absent |

### 0.2 sha256 matrix, world-model port

    for t in ~/workspace/r2dreamer r2fix rel_r2; do sha256sum $t/<path>; done

| file | `~/workspace/r2dreamer` | `$W/r2dreamer_fix` | `release_v4/r2dreamer` |
|---|---|---|---|
| `trainer.py` | **`924617eb`** | **`924617eb`** | `7f60351e` |
| `dreamer.py` | **`54be315e`** | **`54be315e`** | **`54be315e`** |
| `envs/genesis.py` | `54a0e337` | `7f7bd828` | `a11aacd7` |
| `eval_genesis.py` | `879a8f5c` | **`8401f330`** | **`8401f330`** |
| `demo_prefill.py` | `80eba6f7` | `75e64bcf` | `5b2de245` |
| `configs/env/genesis_full_state.yaml` | absent | **`43831e6c`** | **`43831e6c`** |
| `configs/env/genesis_pick_state.yaml` | absent | **`fb64e245`** | **`fb64e245`** |
| `train.py` | **`84e9296b`** | **`84e9296b`** | `85e9b543` |
| `envs/episode_record.py` | absent | **absent** | **PRESENT** |
| `longrun_milestones.py` | absent | **absent** | **PRESENT** |

---

## 1. Per-file classification of every hunk

Legend: **B** behavioural (changes what training or evaluation does in `scope='full'` or a phase
scope), **L** logging (info/log keys, prints, stamps), **C** comment/cosmetic.

### 1.1 `baselines/rl/full_env.py`

`repo HEAD == gp_root` **byte-identical**. Two trees to compare against.

**(a) repo/gp_root vs `gp_e2e`** — 31 changed lines, two hunks.
`diff -u repo/baselines/rl/full_env.py gp_e2e/baselines/rl/full_env.py`

1. **B (the ladder confound).** repo/gp_root lines 70–72; gp_e2e line 57.

       # repo HEAD / gp_root, lines 70-72
       _STAGE_REWARD_OLD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)
       _STAGE_REWARD_X   = dict(picked=1.0, placed_v2=1.0, contact_push=2.0, slide_success=4.0)
       STAGE_REWARD = _STAGE_REWARD_X if os.environ.get('FULLENV_REWARD_X', '') == '1' else _STAGE_REWARD_OLD

       # gp_e2e, line 57
       STAGE_REWARD = dict(picked=1.0, placed=1.0, contact=2.0, nested=4.0)

   This is audit-brief §2, confirmed verbatim. `STAGE_REWARD` drives both reward and which stages
   reach `_granted`, so it is behavioural twice over.

2. **L, and its stated rationale is FALSE in both live trees.** repo/gp_root lines 726–734:

       if not info.get('contact_push') and getattr(self.genv, '_contact_push', False):
           info['contact_push'] = True

   The comment claims "full_env only ever read `info['contact_push']`, so the key was never
   present". That is **not true of the `genesis_can_env.py` in either live tree**:
   `grep -n "contact_push=" gp_e2e/baselines/genesis_can_env.py` → line 336
   `contact_push=self._contact_push,` inside the `info = dict(...)` of `step()`, and
   `grep -n "_granted.add" gp_e2e/baselines/rl/full_env.py` → lines 710 (`contact_push`) and
   713 (`slide_success`). `gp_e2e` and `gp_root` share `genesis_can_env.py` byte for byte
   (`96a668e0`). So the guard is a redundant belt-and-braces in the trees as they stand today,
   and **{RLPD}'s `contact_push` column is genuinely populated, not an artefact of the missing
   guard.** Whatever produced the reported `contact_push = 0.000` was a different configuration;
   that cause is NOT established here.

Everything else in `full_env.py` is identical between repo/gp_root and gp_e2e — including
`terminated = bool(info.get('nested'))` (repo/gp_root 847 and 978; gp_e2e 820 and 951), so
**defect 5's "terminate on the proxy" applies to both learners**; it is only *unpaid* under (x).

**(b) repo/gp_root vs `release_v4/gp`** — 180 changed lines (file dated 2026-08-28). Behavioural
hunks, all of them "release_v4 lacks":

| # | class | what release_v4/gp lacks | repo line |
|---|---|---|---|
| 1 | B | the (x) gate (same hunk as above; rel_gp line 57 is the plain old ladder) | 70–72 |
| 2 | B | `phase_sparse` constructor arg — tips terminate **with** the place penalty in phase scopes | 289 |
| 3 | B | `scope in ('contact','carrycontact','reach_goal')` — those scopes do not exist; `assert scope in (...)` rejects them | 385 |
| 4 | B | `self.shelf_top_z = BOX_TOP_Z + shelf_dz` read from `R2D_SIM_VARIANT`/`GENESIS_SIM_VARIANT`; rel_gp uses the stale constant `BOX_TOP_Z` in the `placed_v2` band | 397, 681 |
| 5 | B | amendment (j) `_world_shelf_top()` assertion against the BUILT shelf box — rel_gp cannot fail this way, repo/gp_root aborts env construction when the variant env var is unset | 410 |
| 6 | B | `_reset_contact` / `_restore_contact_entry` (the whole contact-phase reset path) | 526–580 |
| 7 | **B for the (x) ladder, L for the old one** | full-scope `placed_v2` bookkeeping (`self._pv2_run`, `info['placed_v2']`, `_granted.add('placed_v2')`) | 833–846 |
| 8 | **B for the (x) ladder, L for the old one** | `_granted.add('contact_push')` and `_granted.add('slide_success')` | 737, 740 |
| 9 | B | `scope in ('contact','carrycontact')` `+1`-and-terminate branch | 797 |

Consequence of #7/#8 worth stating plainly: under `release_v4/gp`, in `scope='full'`
`placed_v2` **never enters `_granted`**. Every stage column that a full-scope evaluator reads out
of `_granted` for the old-ladder runs therefore had to come from a re-score against a newer tree
(it did — see §4.3), not from the tree the run trained in.

`terminated = bool(info.get('nested')) and self.scope != 'place'` is present in all three
(rel_gp 697/816), so the proxy terminal is common to every batch on record.

### 1.2 `baselines/genesis_can_env.py`

`gp_e2e == gp_root` byte-identical (`96a668e0`). Two comparisons.

**(a) repo HEAD vs gp_e2e/gp_root** — 32 lines, **all L**. Repo is ahead by pure diagnostics:
`_slide_clauses()` returns `(ok, reason)` instead of `bool`; `end_of_episode()` returns
`slide_fail_reason` / `slide_fail_frame`; `step()` adds `info['slide_fail_reason']`,
`info['slide_fail_frame']`, `info['end_of_episode']`. The boolean value of every clause is
unchanged (the repo version tests `not self._picked` first and grip second; gp_e2e tests grip
first and `not self._picked` third — same conjunction), the settle step budget is unchanged, and
no reward or terminal changes. One caveat: `info['end_of_episode']` is **read** by
`r2fix/eval_genesis.py:338` (`if SCOPE in (...) and not info.get("end_of_episode")`), so under
the repo's `genesis_can_env.py` the evaluator will skip its own `end_of_episode()` call where it
previously made one. Same settle either way, but it is a behavioural coupling Lane 2 should not
break silently.

**(b) gp_e2e/gp_root vs release_v4/gp** — 123 lines, **all B**, all "release_v4 lacks":
`SLIDE_SUSTAIN` / `GRIP_OPEN_CMD` / `SETTLE_STEPS` constants; the whole `contact_push` block in
`step()` (tool-point far-side test, gripper↔goal exclusion, the `contact_farside*` diagnostics);
the `slide_success` sticky window; `_slide_clauses()`; `end_of_episode()` (release_v4 still calls
`self._nested()`); the tool-offset calibration on every reset
(`if self.workspace_limit or self._tool_offset is None` vs `if self.workspace_limit`).

The last one is behavioural beyond logging: in release_v4 `tool_pos()` is uncalibrated for a
non-workspace-limited env, which is exactly why `contact_push` could not be computed there.

### 1.3 `baselines/eval_e2e.py`

**repo HEAD == gp_e2e, byte-identical** (`bbfe59e6`). No hunks. Absent from `gp_root` and
`release_v4/gp` (those trees have no `eval_e2e.py`).

### 1.4 `baselines/rl/train_rlpd.py`

All four trees differ. `gp_root == release_v4/gp` (`7ba5f506`, dated 2026-09-03).

**(a) repo HEAD vs gp_e2e** — 14 lines. repo is ahead; every hunk is the PHASE_PLAN (p)
contact-grant work:
`--contact-grant` argument (repo 126–130), the `scope=contact` assertion (repo 243), the
`contact_grant=args.contact_grant` env kwarg (repo 298), the sidecar key (repo 492), plus one
blank line. **B for `scope='contact'`; inert for `full`/`pick`/`place` — except that it makes the
repo un-runnable, see §2.1.** gp_e2e is ahead of repo in *nothing* for this file: 539608c already
ported gp_e2e's `EpisodeRolloutLogCallback` into the repo.

**(b) gp_e2e vs gp_root/release_v4** — 169 lines. gp_root's copy predates the whole phase and
e2e programme and is **unusable for either e2e arm**:

* `--scope` is `choices=['full','pick']` only; `--demo-format` is `['legacy','native']` — **no
  `segment`**, which is what `sbatch_rlpd_e2e.sh` passes (`--scope full --demo-format segment`).
* no `--entry-bank`, no `--ckpt-every`, no `PHASE_SCOPES`, no `e2e_segment` precondition block,
  no `segment_transitions_full` demo path, no `EpisodeRolloutLogCallback`,
  no `os.environ['GENESIS_SIM_VARIANT'] = args.sim_variant`, no `q_watch = 2.0*sum(STAGE_REWARD)`
  for full scope, `SidecarCheckpointCallback(save_freq=50_000)` hardcoded.

All **B**. `gp_root` is therefore *not* a candidate base for the unified tree on the RLPD side.

### 1.5 `baselines/rl/full_demos.py`

Only repo and gp_e2e have it. 4 lines; repo is ahead by exactly one key in the written manifest:

    # repo HEAD, baselines/rl/full_demos.py:218
    idle_eps=IDLE_EPS, one_per_ic_best=..., one_per_ic_first=bool(m.get('one_per_ic_first')),

**L** (manifest content), but it feeds a launcher assertion, so treat it as a gate input. Note the
*consumed* manifest for the RLPD e2e gate is `repeat.json` written by `to_dreamer_native.py`, not
this one.

### 1.6 `baselines/rl/relabel_reward.py`

Repo only (`cc4419f6`). The cluster copy is the loose file `$W/relabel_reward.py`, **not diffed**
(it is outside every tree and was not in scope). The `_rx` sets were built with it; their
manifests name `to_dreamer_native.py` as generator and
`.../matched_w3/dHfull_all_rx` as source, so the relabel ran upstream of the converter.

### 1.7 `baselines/rl/to_dreamer_native.py`

`repo HEAD == gp_root` byte-identical (`530f9361`).

**(a) repo/gp_root vs gp_e2e** — 21 lines, one hunk, **B**. The 2026-09-09 provenance-inheritance
fix (audit brief §9, "cost 8 failed dDPfirst RLPD jobs"):

    # repo HEAD / gp_root
    sel_best  = bool(args.one_per_ic_best)  or bool(src_meta.get('one_per_ic_best'))
    sel_first = bool(args.one_per_ic_first) or bool(src_meta.get('one_per_ic_first'))
    sel_inherited = sorted(k for k in ('one_per_ic_best','one_per_ic_first')
                           if src_meta.get(k) and not getattr(args, k))
    # gp_e2e
    one_per_ic_best=bool(args.one_per_ic_best), one_per_ic_first=bool(args.one_per_ic_first)

   It changes what the manifest attests, and the launchers assert on that, so it is behavioural at
   the gate. gp_e2e's converter would stamp `one_per_ic_first: False` on a relabelled copy of an
   already-first-attempt set and the launcher would refuse the run.

**(b) repo/gp_root vs release_v4/gp** — 218 lines, **all B**. release_v4's copy predates
`--with-state`, `--state-only`, `--phase`, `--max-tapes`, `--reward-from-tape`,
`--one-per-ic-best/first`, `--stride1-cap`, the mixed-`sim_variant` refusal, and the
`genesis-<uid>-<k>-<T>.npz` filename scheme. It cannot build any set currently in use.

### 1.8 `baselines/sim_variants.py`

`gp_e2e == gp_root == release_v4/gp` (`9728c26b`). **Repo HEAD is the only one that differs**,
37 lines, and it is ahead: the two arm-yaw variants `gc_kp4_riser3_shelf6_yaw16` / `_yaw17`, the
`yaw` key in `add_entity`, and `yaw=float(v.get('yaw', 0.0))` in the two manifest dicts.

Classification: **C/L for the world of record and B only off it.** For
`gc_kp4_riser3_shelf6` the guard is `if yaw != 0.0:` and the value defaults to `0.0`, so the built
world is bit-identical. The one thing that is *not* inert is that `install()` and the manifest
helper now emit an extra `yaw` key — anything that compares a whole variant dict across trees
will see a difference. No gate found that does so, but this was not exhaustively checked.

### 1.9 `can_pos_recovery/replay_harness.py` and `baselines/record_demos.py`

**Identical in all four trees** (`1c42f74d`, `34a673e9`). No hunks. This is a positive result:
the shelf/footprint constants and the recorder are not a source of drift.

### 1.10 `cluster/sbatch_rlpd_e2e.sh`

Only repo and gp_e2e have it. 7 lines, and **gp_e2e is AHEAD of the repo** — the only file in the
e2e scope where that is true:

    # gp_e2e:50 (repo lacks this line)
    dDPfirst) DEMO=${DEMO:-$W/demos_state_full/dDPfull_first}; N_EXP=72 ;;
    # gp_e2e:51
    *) echo "FATAL: ARM must be dH | dDP | dDPfirst (got $ARM)"; exit 1 ;;
    # gp_e2e:78-79 (repo lacks)
    if arm == 'dDPfirst':
        assert m.get('one_per_ic_first') is True, m

**B.** The repo's launcher would exit 1 on `ARM=dDPfirst`, i.e. it **cannot launch the
de-selected machine arm that every live {RLPD} run uses.**

Neither version stamps the ladder. Verified in the logs:
`grep -c "gates\|STAGE_REWARD" $LAB/gp_e2e/e2e_rlpd_3488583.out` → `0`, while
`grep "\[gates\]" $W/slurm/*_3491299.out` →

    [gates] FULLENV_REWARD_X=1 FULLENV_EPISODE_RECORD=<unset>
    [gates] STAGE_REWARD in force: {'picked': 1.0, 'placed_v2': 1.0, 'contact_push': 2.0, 'slide_success': 4.0}

### 1.11 `cluster/e2e_posthoc_sweep.sh`

Repo only; the cluster copy is `$W/e2e_posthoc_sweep.sh`, which is the same script by inspection
(both scope to `runs/full_r2d_state_d{H,DP}full_*_rx_s9*`, both submit
`$W/full_posthoc_evals.sbatch`). Not byte-compared — the repo file and the loose cluster file were
not hashed against each other.

### 1.12 World-model port: `trainer.py`, `train.py`, `demo_prefill.py`

`~/workspace/r2dreamer == $W/r2dreamer_fix` for `trainer.py`, `dreamer.py`, `train.py`.
`release_v4/r2dreamer` is **not** older — it is a **divergent branch**. Three behavioural hunks,
all present only in release_v4:

1. **B — the online-budget contract.** `release_v4/trainer.py:141-142` + `longrun_milestones.py`:

       from longrun_milestones import Milestones
       milestones = Milestones(self, online_step0)
       ...
       trainer.steps = self.origin + self.budget      # longrun_milestones.py, when R2_LONG_RUN=1

   `r2dreamer_fix` has neither the import nor the module. Effect, measured:

   | batch | tree | prefill origin | counter target | **online steps** |
   |---|---|---|---|---|
   | old ladder, human | `release_v4` | 117 624 | 4 117 624 | **4 000 000** |
   | old ladder, machine | `release_v4` | 149 952 | 4 149 952 | **4 000 000** |
   | (x), human | `r2dreamer_fix` | 29 406 | 4 000 000 | **3 970 594** |
   | (x), machine | `r2dreamer_fix` | 37 488 | 4 000 000 | **3 962 512** |

   Commands: `cat $W/long_run_candidate_2026-09-08/runs/r2long_h_202609080/step_contract.json`
   and `grep "Step accounting" $W/slurm/e2eL_r2_dH_s10_3491299.out`
   (`trainer starts at step 29406; env.steps=4000000 -> 3970594 online env steps`).
   The old batch also wrote milestone checkpoints at 2M and 4M
   (`$W/.../runs/r2long_*/milestones/online_{2000000,4000000}.pt`); **the (x) batch has none**,
   so a 2M-vs-4M budget read is available for the old runs and impossible for the new ones.

2. **B/L — amendment (w) episode record.** `release_v4/r2dreamer/envs/episode_record.py` exists
   and is wired in `envs/genesis.py:31`, `demo_prefill.py:284-289` and
   `trainer.py:259-276` (the end-of-budget flush of `log_ep_record_valid`). **Absent from
   `r2dreamer_fix` and from local.** D8 says this record is always on; the implementation to port
   is release_v4's, not the repo's (see §2.1 — the repo's `full_env` half was reverted).

3. **L — stage emission, and the two trees fix the same bug differently.**
   `release_v4/envs/genesis.py:353`: `hit = done and (k in self._env._granted or info.get(k))`.
   `r2dreamer_fix/envs/genesis.py`: first-grant one-shot `log_<k>` plus a sticky twin
   `log_ep_<k>`, with `self._emitted_stages` reset in `reset()`, and matching
   `log_ep_task_success`. The r2fix form is the correct one for truncated episodes (the outer
   wrapper truncates without setting `done`). **Neither tree is a superset**; a merge must take
   r2fix's emission *and* release_v4's `episode_record` module.

   This affects training curves only. It does **not** affect any eval cell: `eval_genesis.py`
   reads `env._env._granted` directly (`r2fix/eval_genesis.py:369`,
   `_gr = set(getattr(env._env, "_granted", set())) | {k for k in STAGES if bool(info.get(k))}`),
   and `eval_genesis.py` is **byte-identical between `r2dreamer_fix` and `release_v4`**
   (`8401f330`). The evaluator really is matched across the two batches.

`demo_prefill.py`: r2fix vs release_v4 is 8 lines, the `episode_record.LOG_KEYS` schema extension
only (**L**, but it changes the buffer key set, so a mixed pair would fail the key-set assert).
Local vs r2fix is 44 lines — local is behind (no `state_obs` path in the form r2fix has).

### 1.13 World-model port: `envs/genesis.py`, `eval_genesis.py`, configs

* `configs/env/genesis_full_state.yaml` and `genesis_pick_state.yaml`: **identical between
  `r2dreamer_fix` and `release_v4`** (`43831e6c`, `fb64e245`); **absent from local**. The audit
  brief's "byte-identical configs" claim holds.
* `eval_genesis.py`: r2fix == release_v4; local is 240 lines behind.
* `envs/genesis.py`: local is 146 lines behind r2fix — local lacks `FULL_EXTRA_KEYS`
  (`placed_v2`, `nested_honest`, `slide_success`), `SLIDE_EXTRA_KEYS`, the `R2D_EOE`
  end-of-episode settle for `full`/`contact` scopes, `info["nested_proxy"]`, the phase-scope
  `phase_sparse=True` construction, and the first-grant/sticky emission. **All B.** Lane 2's
  local edits to `~/workspace/r2dreamer/envs/genesis.py` are being made on a tree that is missing
  most of the live adapter.

---

## 2. Is repo HEAD a superset of `gp_e2e`? (audit-brief claim about 539608c)

**No.** The claim as stated in the commit message is accurate — 539608c says explicitly "the two
trees diverge in BOTH directions" — but the brief's shorthand ("ported cluster-only e2e work into
the repo") should not be read as "the repo now has everything".

Method: hash every `.py`/`.sh`/`.sbatch` under `baselines/`, `cluster/`, `can_pos_recovery/` in
both, then `comm`/`join` the manifests.

    ssh ... 'cd $LAB/gp_e2e && find baselines cluster can_pos_recovery -type f \
      \( -name "*.py" -o -name "*.sh" -o -name "*.sbatch" \) -not -path "*__pycache__*" \
      | sort | while read f; do echo "$(sha256sum "$f" | cut -c1-16)  $f"; done'
    git ls-tree -r --name-only HEAD | grep -E '^(baselines|cluster|can_pos_recovery)/.*\.(py|sh|sbatch)$' \
      | while read f; do echo "$(git show HEAD:$f | sha256sum | cut -c1-16)  $f"; done

284 files in gp_e2e, 376 in repo HEAD.

**Present in `gp_e2e`, absent from repo HEAD (2 files):**

* `cluster/eval_fixes_first_flag.py`
* `cluster/table_arms.py`

**Present in repo HEAD, absent from `gp_e2e` (≈100 files):** `baselines/rl/relabel_reward.py`,
`cluster/e2e_posthoc_sweep.sh`, `cluster/e2e_health.py`, `cluster/annotate_episodes.sbatch`,
`baselines/eval_e2e_annot.py`, `baselines/eef_*.py`, `can_pos_recovery/slide_predicate.py`, the
whole `can_pos_recovery/*g14*` / pad-candidate family, `baselines/robomimic/*` extras. (The
`can_pos_recovery` bulk is sim-box work irrelevant to e2e.)

**Shared but different (22 files, excluding `robomimic/`, `dv3dbg/`, `a31_chain/` and
`RUN_REGISTRY.jsonl`):**

    baselines/eval_place.py                baselines/rl/train_rlpd.py
    baselines/genesis_can_env.py           baselines/sim_variants.py
    baselines/place_table_all.py           baselines/wandb_eval.py
    baselines/rl/full_demos.py             cluster/eval_fixes/{eval_fixes_table.py,rs2_*}
    baselines/rl/full_env.py               cluster/eval_sweep.sh
    baselines/rl/place_demos.py            cluster/place_{eval_cells,readout}.sh
    baselines/rl/to_dreamer_native.py      cluster/rlpd_eval_sampled.sh
                                           cluster/sbatch_{dp,rlpd}_{contact,e2e}.sh

Direction per file (`diff -u <repo> <gp_e2e>`, counting `+` = gp_e2e-only, `-` = repo-only):

| file | gp_e2e-only lines | repo-only lines |
|---|---|---|
| `cluster/sbatch_dp_e2e.sh` | **9** | 1 |
| `cluster/sbatch_rlpd_e2e.sh` | **2** | 0 (plus 5 context) |
| `baselines/eval_place.py` | 6 | 68 |
| `baselines/place_table_all.py` | 9 | 110 |
| `baselines/rl/place_demos.py` | 3 | 72 |
| `cluster/place_readout.sh` | 7 | 30 |
| `cluster/place_eval_cells.sh` | 4 | 20 |
| `cluster/rlpd_eval_sampled.sh` | 4 | 18 |
| `baselines/wandb_eval.py` | 1 | 24 |
| `cluster/eval_fixes/eval_fixes_table.py` | 1 | 17 |
| `cluster/sbatch_rlpd_contact.sh` | 1 | 11 |
| `cluster/eval_sweep.sh` | 1 | 3 |
| `cluster/sbatch_dp_contact.sh` | 0 | 10 |

Most gp_e2e-only lines in the place-phase files are line-wrapping differences of the same code.
The substantive gp_e2e-only content is in the two e2e launchers:

* `sbatch_rlpd_e2e.sh`: the `dDPfirst` arm case + the `one_per_ic_first` assert (§1.10).
* `sbatch_dp_e2e.sh`: the same `dDPfirst` case, `assert (m.get('one_per_ic_first') is True) ==
  (arm == 'dDPfirst')`, and the `KEEP_CKPTS=1` branch that retains intermediate DP checkpoints
  ("DP is offline — a checkpoint-vs-performance sweep is the ONLY convergence evidence available
  for this learner").

### 2.1 THE BLOCKER: repo HEAD is internally inconsistent and its RLPD trainer cannot start

Commit `809601d` ("full_env: gated amendment-(x) reward ladder + expose the computed
contact_push") is a **whole-file copy of `gp_root`'s `full_env.py` over the repo's**. It therefore
silently reverted two earlier repo commits:

    git log --oneline -3 -- baselines/rl/full_env.py
    809601d full_env: gated amendment-(x) reward ladder + expose the computed contact_push
    7096fe6 amendment (w): episode record on the shared exit path, gated OFF by default
    20f49f8 (o) stopped per PHASE_PLAN (p): the contact grant is now explicit ...

    git diff 7096fe6 HEAD -- baselines/rl/full_env.py       # shows both reverts as deletions

1. **Amendment (w) is gone from `full_env.py`.** `git merge-base --is-ancestor 7096fe6 HEAD` →
   YES, yet `git grep -l FULLENV_EPISODE_RECORD HEAD` returns only `CLAUDE.md` and three `paper/`
   docs — **no source file**. The `EPISODE_RECORD` constant and the sticky `ep_*` emission that
   7096fe6 added were removed by 809601d. (Consistent with `grep FULLENV_EPISODE_RECORD` finding
   nothing in any of the three cluster gp trees either: the emitter was never deployed anywhere.)

2. **`contact_grant` is gone from `full_env.py` but is still passed by two callers.** Static check:

       FullTaskEnv.__init__ params: [... 'entry_bank', 'phase_sparse', 'action_mode', ...]
       accepts contact_grant: False
       train_rlpd passes contact_grant: True

   `git show HEAD:baselines/rl/train_rlpd.py | grep -n contact_grant` → lines 243, **298**, 492;
   line 298 is `contact_grant=args.contact_grant,` inside the `FullTaskEnv(...)` call, which runs
   for **every** scope. `git grep -n contact_grant HEAD -- baselines/` also finds
   `baselines/eval_place.py:162`.

   **Therefore `python baselines/rl/train_rlpd.py` from repo HEAD raises
   `TypeError: __init__() got an unexpected keyword argument 'contact_grant'` at env
   construction, for pick, place, contact and full alike; `baselines/eval_place.py` is broken the
   same way.** `gp_e2e` and `gp_root` are each internally self-consistent (both lack
   `contact_grant` on both sides); only the repo is not.

   A `gp_unified` cloned from this branch would fail on its first RLPD smoke. **Lane 2 must fix
   this before Lane 4 clones anything.** The lost code is recoverable:
   `git show 7096fe6:baselines/rl/full_env.py` (amendment (w)) and
   `git show 20f49f8:baselines/rl/full_env.py` (the `contact_grant` parameter and its PHASE_PLAN
   (p) assertions).

---

## 3. Building `$LAB/gp_unified`: inventory and procedure

### 3.1 What each learner needs at runtime

| need | {RLPD} (`sbatch_rlpd_e2e.sh`) | {r2dreamer} (`wmfix_full.sbatch`) |
|---|---|---|
| interpreter | `module load anaconda/2025.06.0` + `conda activate $LAB/condaenv/genesis` (line 114–115) | `$LAB/r2d_venv/bin/python` (line 22) — python 3.11.15, torch 2.8.0+cu126, genesis 0.2.1 |
| code root | `cd "${GENESIS_PICKAPLACE_ROOT:=$PWD}"` (line 41) — assigns `$PWD` only if unset, so **exporting it points the job anywhere** | `export GENESIS_PICKAPLACE_ROOT=${GP_ROOT:-$W/gp_root}` (line 31); `cd $W/r2dreamer_fix` (line 33) |
| how the root is consumed | the trainer is run as `python baselines/rl/train_rlpd.py` from that cwd | `r2fix/envs/genesis.py:35` `REPO = os.environ.get("GENESIS_PICKAPLACE_ROOT", ...)`, then `sys.path.insert` of `$REPO/baselines`, `$REPO/baselines/rl`, `$REPO/can_pos_recovery`; `eval_genesis.py:28` `os.environ.setdefault(...)` |
| PYTHONPATH | not set; relies on cwd + `full_env.py`'s own `sys.path.insert(REPO/'can_pos_recovery')` | not set; the adapter inserts the three paths itself |
| sim variant | `export GENESIS_SIM_VARIANT=$SIM_VARIANT` (line 57), default `gc_kp4_riser3_shelf6`; `--sim-variant` also passed to the trainer, which re-exports it | `export R2D_SIM_VARIANT=gc_kp4_riser3_shelf6` (line 32) |
| | `full_env.py:397` reads `R2D_SIM_VARIANT or GENESIS_SIM_VARIANT or 'base'` and `:410` asserts it against the BUILT shelf. **Either variable satisfies it; neither set ⇒ the env aborts.** | same file, same assert |
| demo set | `DEMO=$W/demos_state_full/<set>`, gated on `$DEMO/repeat.json` (lines 65–87) | `DEMO=${DEMO_ROOT:-$W/demos_state_full}/$ARM`, gated on `$DEMO/repeat.json` (lines 37–46) |
| demo format | r2dreamer-native `.npz` read through `full_demos.segment_transitions_full` (`--demo-format segment`) | the same directory read by `demo_prefill.py` |
| | **Both learners read the identical directory.** `dHfull_all_rx` (74 tapes, `total_reward` 238.0) / `dDPfull_first_rx` (72, 237.0, `one_per_ic_first: True`) | |
| eval IC file | `cluster/e2e_eval_cells.sh` → `baselines/eval_ics.json` in the code root | `--ic-file $LAB/genesis_pickaplace/baselines/eval_ics.json` — **a FOURTH tree** (`$LAB/genesis_pickaplace`, git `e8d41ff`, branch `4dof-cartesian`). Content is identical to gp_e2e's copy (`sha256 4d2587b9aa7b0a5a9792…` both), so this is a path coupling, not a content risk. |
| run registry | `python3 cluster/run_registry.py check/register --registry cluster/RUN_REGISTRY.jsonl` (lines 109–110). Refuses (exit 2) on a FULL-key match of `(script, arm, seed, semantic knobs, demo fingerprint, git hash)` unless `DUPLICATE_OK=<reason>`; warns when only the git hash differs. **This is what refused 13 of RLPD gen 3.** The registry file lives IN the tree, so a new clone starts with the file it inherits. | none |
| disk guard | lines 59–62: `df -BG /cluster/tufts/shortlab`, refuse below **150 GB**. Currently 306 GB free. | **none** |
| partition / QOS | `#SBATCH -p preempt --qos=preempt --nice=9000 --gres=gpu:1 --constraint="l40s\|a100\|l40\|h200" --exclude=pax077 -N 1 -n 8 --mem=48g -t 1-06:00:00` | `#SBATCH -p gpu,preempt --gres=gpu:1 --constraint=l40s\|a100\|l40\|h200 --exclude=pax077 -N 1 -n 8 --mem=48g -t 2-00:00:00`, `-o $W/slurm/%x_%j.out` |
| requeue guard | `#SBATCH --requeue`; on `SLURM_RESTART_COUNT > 0` it skips the registry and `rm -rf "$OUT"` (lines 108–113). No existence guard ⇒ **defect 4 never applied to {RLPD}** | `#SBATCH --requeue`; line 30 `[ -e "$LOGDIR" ] && [ "${SLURM_RESTART_COUNT:-0}" -eq 0 ] && exit 2` — **this is the fixed form**; line 47 clears the partial logdir on restart. The pre-fix copy is `$W/wmfix_full.sbatch.bak_0910` |
| torch.compile cache | — | `TORCHINDUCTOR_CACHE_DIR=$W/inductor_cache/$SLURM_JOB_ID` (line 17) — required: two jobs sharing one inductor cache crash |
| ladder stamp | **none** — must be added (D6) | lines 58–59, already prints `[gates] FULLENV_REWARD_X=…` and `[gates] STAGE_REWARD in force: {…}` |
| eval stage | end of job, lines 132–134 → `cluster/e2e_eval_cells.sh` → `baselines/eval_e2e.py` | end of job, lines 77–90 → `eval_genesis.py` (4 cells: hold/rnd × sample/mode) |

Other environment variables in play: `MUJOCO_GL=egl`, `PYOPENGL_PLATFORM=egl`,
`PYTHONUNBUFFERED=1` (both); `FULLENV_REWARD_X` (to be REMOVED per D1);
`FULLENV_EPISODE_RECORD` (never implemented in any deployed `full_env.py`);
`R2D_EOE` (adapter, default `"1"`); `REACH_GOAL_DIST` (asserted only for `scope='reach_goal'`);
`CONTACT_GRANT_ALLOW_WITHDRAWN` (repo only, and currently unreachable — §2.1).

### 3.2 Procedure to create `$LAB/gp_unified` (commands; NOT run)

Preconditions, in order: (a) Lane 2 has fixed §2.1 on a branch — `full_env.py` accepts
`contact_grant` again (or `train_rlpd.py`/`eval_place.py` stop passing it), amendment (w) is
restored unconditionally per D8, `FULLENV_REWARD_X` is removed, and `ladder_provenance()` exists;
(b) the two gp_e2e-only launcher hunks (§1.10) are merged into the repo; (c) the branch is pushed.

    # --- 1. clone from the repo branch, on the cluster -------------------------------
    ssh jstale02@login.pax.tufts.edu
    LAB=/cluster/tufts/shortlab/jstale02; W=$LAB/wm_fix_2026-09-03
    df -BG --output=avail /cluster/tufts/shortlab | tail -1      # must be >= 150
    cd $LAB
    git clone -b <unified-branch> $LAB/genesis_pickaplace gp_unified   # or the GitHub remote
    cd gp_unified && git log --oneline -1 && git status --short

    # --- 2. carry the two gp_e2e-only cluster helpers, if still wanted ---------------
    cp $LAB/gp_e2e/cluster/eval_fixes_first_flag.py cluster/
    cp $LAB/gp_e2e/cluster/table_arms.py            cluster/

    # --- 3. the run registry: start from the live one so duplicates stay detectable --
    cp $LAB/gp_e2e/cluster/RUN_REGISTRY.jsonl cluster/RUN_REGISTRY.jsonl
    wc -l cluster/RUN_REGISTRY.jsonl

    # --- 4. verify the tree is internally consistent BEFORE any GPU is requested -----
    $LAB/r2d_venv/bin/python - <<'PY'
    import ast, inspect
    src = open('baselines/rl/full_env.py').read()
    t = ast.parse(src)
    for n in ast.walk(t):
        if isinstance(n, ast.ClassDef) and n.name == 'FullTaskEnv':
            for m in n.body:
                if isinstance(m, ast.FunctionDef) and m.name == '__init__':
                    p = [a.arg for a in m.args.args]
                    print('FullTaskEnv.__init__:', p)
    tr = open('baselines/rl/train_rlpd.py').read()
    for k in ('contact_grant', 'phase_sparse'):
        print(k, 'passed by train_rlpd:', f'{k}=' in tr, '| accepted:', k in p)
    PY
    grep -n "FULLENV_REWARD_X" -r baselines cluster || echo "OK: no legacy gate in the tree"

    # --- 5. sanity: the world builds and the ladder stamps (CPU, ~2 min, no GPU) -----
    GENESIS_SIM_VARIANT=gc_kp4_riser3_shelf6 \
    $LAB/r2d_venv/bin/python -c "
    import sys; sys.path[:0]=['baselines','baselines/rl','can_pos_recovery']
    import full_env; print('[ladder]', full_env.ladder_provenance())"

    # --- 6. RLPD smoke: 2000 decisions, CPU, its own OUT_ROOT, registry bypassed -----
    cd $LAB/gp_unified
    GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified \
    ARM=dDPfirst SEED=9990 STEPS=2000 DEVICE=cpu \
    DEMO=$W/demos_state_full/dDPfull_first_rx \
    OUT_ROOT=baselines/rl/checkpoints/smoke_unified \
    SETS="hold15" MODES="mode" VIDEO_SETS="" CKPT_FRACS=1.0 \
    DUPLICATE_OK="gp_unified smoke 2026-09-xx" \
      sbatch -J smokeU_rl -p preempt --qos=preempt --gres=gpu:0 -t 0-04:00:00 \
        cluster/sbatch_rlpd_e2e.sh
    # dry form first (no conda, no training):
    #   DRYRUN=1 ARM=dDPfirst SEED=9990 GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified \
    #     bash cluster/sbatch_rlpd_e2e.sh

    # --- 7. r2dreamer smoke: 180k steps against the SAME tree -----------------------
    cd $W
    GP_ROOT=$LAB/gp_unified TAG=smokeU \
      sbatch -J smokeU_r2 $W/wmfix_full.sbatch dDPfull_first_rx 9990 180000
    # GP_ROOT is honoured at wmfix_full.sbatch:31; nothing else in that script
    # references gp_root, so no edit to the launcher is needed to retarget it.
    # NOTE: the r2dreamer TREE stays $W/r2dreamer_fix (line 21) -- D1 unifies the
    # genesis_pickaplace side. Unifying the WM port itself is a separate merge (§1.12).

    # --- 8. confirm the [ladder] stamp is in BOTH logs ------------------------------
    grep -h "\[ladder\]\|\[gates\]" $W/slurm/smokeU_r2_*.out
    grep -h "\[ladder\]\|\[gates\]" $LAB/gp_unified/e2e_rlpd_*.out
    # and that both wrote the provenance file:
    cat $W/runs/full_r2d_state_dDPfull_first_rx_smokeU_s9990/ladder_provenance.json
    cat $LAB/gp_unified/baselines/rl/checkpoints/smoke_unified/e2e_rlpd_dDPfirst_s9990/ladder_provenance.json
    # they must be EQUAL except for git-dirty state:
    diff <(jq -S . $W/runs/.../ladder_provenance.json) \
         <(jq -S . $LAB/gp_unified/.../ladder_provenance.json) && echo "LADDER MATCH"

Two things Lane 2/4 must add for step 8 to work at all, because they do not exist today:
`full_env.ladder_provenance()` (D6) and the `[ladder]` print + `ladder_provenance.json` write in
both launchers. The `[gates]` lines already exist on the r2dreamer side and can be kept beside
the new stamp during the transition.

### 3.3 Retarget-vs-clone note

Both launchers can be pointed at a new tree **without editing them**:
`GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified` for {RLPD} (line 41 uses `:=`, which only assigns when
unset) and `GP_ROOT=$LAB/gp_unified` for {r2dreamer} (line 31). That is the low-risk path: it
needs no change to `$W/wmfix_full.sbatch`, which in-flight jobs do not re-read but future ones do.

---

## 4. In-flight hazards: when each tree becomes safe to retire

Snapshot `2026-09-10T23:52:37-04:00`, `squeue -u jstale02`: **21 RUNNING, 7 PENDING**.

### 4.1 Which job imports which tree, and when

| job family | count | tree(s) imported | import moments |
|---|---|---|---|
| `e2eL_r2_*` (16 jobs running) | 16 | `$W/r2dreamer_fix` (cwd), `$W/gp_root` (via `GENESIS_PICKAPLACE_ROOT`), `$W/demos_state_full/*`, `$LAB/genesis_pickaplace/baselines/eval_ics.json` | (i) **job start** — the `[gates]` `full_env` import at line 59 and the demo gate at 37–46; (ii) **training start** — the adapter's `sys.path.insert` + `import full_env` inside `train.py`; (iii) **END of job** — four `eval_genesis.py` processes (lines 77–90), each a FRESH import of `gp_root` |
| `e2eL_rl_*` (5 running, 6 pending) | 11 | `$LAB/gp_e2e` (cwd = `GENESIS_PICKAPLACE_ROOT`), `$W/demos_state_full/*` | (i) **job start** — disk guard, demo gate, `run_registry.py`; (ii) **training** — `python baselines/rl/train_rlpd.py` imports `full_env`/`genesis_can_env`/`full_demos`; (iii) **END of job** — `cluster/e2e_eval_cells.sh` spawns `baselines/eval_e2e.py` per cell, a FRESH import of `gp_e2e` |
| `full_posthoc` (submitted on demand by `e2e_posthoc_sweep.sh`) | 0 now | `$W/r2dreamer_fix` + `$W/gp_root` (`full_posthoc_evals.sbatch:15`) | whole job |

**The eval stage is the binding constraint.** For both learners the evaluator is a *separate
process launched at the end of the job*, so a source edit made hours after a job starts is still
picked up — by the eval, not by the training. That is exactly the failure mode D1 is meant to end,
and it is the reason neither `gp_e2e` nor `gp_root` may be touched while any job in the table is
alive.

### 4.2 Earliest safe retirement dates (walltime ceilings)

`squeue -o "%.20e"` end times; these are **upper bounds** — a job that finishes early frees the
tree early, and a preempted-and-requeued job pushes it out.

| tree | last job that imports it | walltime ceiling | notes |
|---|---|---|---|
| `$W/gp_root` + `$W/r2dreamer_fix` | `e2eL_r2_dM_s12` (3515826) | **2026-09-12 20:23** | 16 r2 jobs; the 6 pending `e2eL_rl_*` do **not** touch these |
| `$LAB/gp_e2e` | `e2eL_rl_dM_s15` (3488586, PENDING, 1-06:00:00 limit) | **unbounded until it starts**; if it started now, 2026-09-12 ~05:50 | 6 jobs are PENDING on `QOSMaxGRESPerUser`; their clocks have not begun |

**Practical reading for the coordinator.** `gp_root`/`r2dreamer_fix` free themselves by
**2026-09-12 evening**. `gp_e2e` cannot be dated at all while 6 RLPD jobs sit PENDING with a
fresh 30-hour limit each — worst case is *start time + 30 h*, and start time is set by the GPU
QOS queue. If a firm date is needed, the pending 6 must be cancelled and resubmitted against
`gp_unified`, or `gp_e2e` must be left in place as a frozen read-only copy. **Leaving a frozen
copy is the cheaper option and costs only disk**: `gp_e2e` is a git repo (`6e98ce3`, branch
`e2e-work`, 2 modified + untracked files), so a `git bundle` plus a tar of the untracked
`baselines/eval_e2e_annot.py` and `cluster/sbatch_dp_e2e.sh.bak_*` preserves everything.

`gp_root`, `release_v4/gp`, `release_v4/r2dreamer` and `r2dreamer_fix` are **NOT git repositories**
(`ls -d <tree>/.git` → absent for all four). Anything in them that is not ported into
`gp_unified` is destroyed by retirement with no recovery path. The specific unported content is
`envs/episode_record.py` and `longrun_milestones.py` (§1.12), which exist in exactly one place on
Earth: `$W/long_run_candidate_2026-09-08/release_v4/r2dreamer/`.

### 4.3 Which runs still need an eval against an old tree

`$W/runs/full_r2d_state_*_rx_s9*`: **16 of 33** directories have
`fresh_eval_rnd30_mode/metrics.json` (15 an hour earlier — this number moves as the monitor's
recovery sweep lands). The remainder includes runs already at the 4M budget with no cell
(`dDPfull_first_rx_s928` at step 3 999 972 was one), which `e2e_posthoc_sweep.sh` will pick up by
submitting `$W/full_posthoc_evals.sbatch` — i.e. **more `gp_root` imports after the last training
job exits**. Budget for that before retiring `gp_root`.

    W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03; n=0; e=0
    for d in $W/runs/full_r2d_state_d*_rx_s9*; do n=$((n+1))
      [ -f "$d/fresh_eval_rnd30_mode/metrics.json" ] && e=$((e+1)); done
    echo "r2 run dirs=$n with_rnd30_mode_cell=$e"   # -> 33 / 16 at 2026-09-11 00:0x

    for d in $W/runs/full_r2d_state_d*_rx_s9*; do
      s=$(tail -c 4000 $d/metrics.jsonl | grep -oE '"step": [0-9]+' | tail -1 | grep -oE '[0-9]+')
      e=$([ -f "$d/fresh_eval_rnd30_mode/metrics.json" ] && echo EVALDONE || echo no-eval)
      echo "$(basename $d) step=$s $e"; done

`$LAB/gp_e2e/baselines/rl/checkpoints/e2e/`: **22 of 31** `e2e_rlpd_*_s9*` run dirs have
`rlpd_final.zip` (and an `e2e_eval.log`); 9 do not.

    cd $LAB/gp_e2e/baselines/rl/checkpoints/e2e && n=0; f=0
    for d in e2e_rlpd_*_s9*; do n=$((n+1)); [ -f "$d/rlpd_final.zip" ] && f=$((f+1)); done
    echo "run dirs=$n with_final=$f"      # -> run dirs=31 with_final=22

---

## 5. Where this contradicts or qualifies the audit brief

1. **`paper/E2E_TRAINING_PROBLEMS_2026-09-10.md` §0, "the learner, the budget and the evaluator"
   are the same across the old-ladder and (x) batches — the BUDGET is not.** `release_v4`
   contains `longrun_milestones.py` and the `R2_LONG_RUN=1` contract that makes the requested
   budget explicitly ONLINE; `r2dreamer_fix` contains neither. Old runs: 4 000 000 online steps
   on top of a 117 624 / 149 952 counter origin. (x) runs: 3 970 594 (human) / 3 962 512
   (machine) online, because the same `4000000` argument is a TOTAL there. The gap is 0.74 %
   (human) and 0.94 % (machine) — far too small to explain nested_honest 0.167 → 0.017, so §0's
   conclusion survives, but the sentence needs amending, and the (x) batch has **no 2M milestone
   checkpoints**, so the 2M-vs-4M read the old batch supports cannot be reproduced for it.
   The *evaluator* claim does hold: `eval_genesis.py` is byte-identical across the two trees and
   reads `_granted` directly, not the `log_*` keys that differ.

2. **`release_v4` is not simply "older" — it is a divergent branch, and it holds the only copy of
   amendment (w) for the world model.** The brief describes release_v4 as the isolated release
   the old runs used. It also contains `envs/episode_record.py` and the trainer-side flush that
   `r2dreamer_fix` lacks, while `r2dreamer_fix` has the first-grant/sticky `log_ep_*` emission
   that `release_v4` lacks. Neither is a superset. D8 ("episode record always on") should be
   implemented by porting release_v4's module, not by re-deriving it.

3. **The rationale attached to the repo/gp_root `contact_push` guard is wrong.** The comment at
   `full_env.py:726` says the env "never puts it into `info`". `genesis_can_env.py:336` in both
   live trees does exactly that, and `full_env.py:710` already adds it to `_granted`. The guard
   is a no-op today. It does not change any number, but the stated cause of the `contact_push =
   0.000` reading is not established, and the audit brief should not carry it as explanation.

4. **`gp_e2e` is ahead of the repo where it matters most for relaunching.** The brief's summary of
   539608c reads as a one-way port. `gp_e2e/cluster/sbatch_rlpd_e2e.sh` and `sbatch_dp_e2e.sh`
   carry the `dDPfirst` arm and the `one_per_ic_first` gate; the repo's copies do not and would
   refuse the arm every live {RLPD} run uses. Two cluster helpers
   (`cluster/eval_fixes_first_flag.py`, `cluster/table_arms.py`) exist only in `gp_e2e`.

5. **Repo HEAD cannot run `train_rlpd.py` or `eval_place.py` at all.** Commit `809601d` copied
   `gp_root`'s `full_env.py` wholesale and reverted both `7096fe6` (amendment (w)) and `20f49f8`
   (the PHASE_PLAN (p) `contact_grant` parameter) while their callers stayed. This is not
   mentioned anywhere in the brief and it blocks Lane 4 outright.

6. **Defect 5's terminal is common to both learners, not specific to (x).**
   `terminated = bool(info.get('nested'))` is present in all three gp trees (repo/gp_root 847/978,
   gp_e2e 820/951, release_v4 697/816). Under the old ladder that terminal pays +4, so it is only
   the *unpaid* terminal that is an (x) property. Worth stating precisely, because "full scope
   terminates on the nested proxy" is a property of the task as built since at least August.

7. **`release_v4/gp` never computed `placed_v2` in full scope.** `_granted.add('placed_v2')` in
   the full-scope branch does not exist there. Any `placed_v2` column reported for the
   old-ladder 4.1M runs therefore came from a re-score against `gp_root` (the eval cells' mtimes
   are 2026-09-09, after the release), not from the tree those runs trained in. That is fine for
   a post-hoc statistic; it must not be described as something the training environment measured.

8. **A fourth tree is in the loop.** `wmfix_full.sbatch:85` and `full_posthoc_evals.sbatch:23`
   read `$LAB/genesis_pickaplace/baselines/eval_ics.json` — a separate git checkout at `e8d41ff`
   on branch `4dof-cartesian`. The file content matches `gp_e2e`'s copy byte for byte today
   (`4d2587b9aa7b0a5a9792…`), so nothing is currently wrong, but `gp_unified` inherits a
   dependency on a checkout nobody is tracking.

---

## 6. Merge plan for D1, in dependency order

1. **Unbreak repo HEAD** (Lane 2, blocking everything): restore `contact_grant` to
   `FullTaskEnv.__init__` from `git show 20f49f8:baselines/rl/full_env.py`, or drop it from
   `train_rlpd.py:298` / `eval_place.py:162`. Restore amendment (w) from
   `git show 7096fe6:baselines/rl/full_env.py`, ungated per D8. Add a CI check that parses both
   files and compares the kwarg set — this class of break is silent until a GPU is allocated.
2. **Merge the two gp_e2e-only launcher hunks** (`dDPfirst` case + `one_per_ic_first` assert) into
   `cluster/sbatch_rlpd_e2e.sh` and `cluster/sbatch_dp_e2e.sh`, plus `KEEP_CKPTS` in the DP one.
   Copy `cluster/eval_fixes_first_flag.py` and `cluster/table_arms.py`.
3. **Remove `FULLENV_REWARD_X`** and add the refusal (D1), `ladder_provenance()` (D6) and the
   `[ladder]` stamp in both launchers. Keep the existing `[gates]` lines in `wmfix_full.sbatch`
   during the transition so old and new logs are comparable.
4. **World-model port**: take `r2dreamer_fix` as the base (it is what the live batch runs and it
   has the correct truncation-safe emission), then port in from `release_v4`
   `envs/episode_record.py`, `longrun_milestones.py`, the `trainer.py` end-of-budget flush, the
   `train.py` step contract, and the `demo_prefill.py` key-set extension. Then bring
   `~/workspace/r2dreamer` forward to that merged state — it is currently 146 lines behind on the
   adapter and has no `genesis_*_state.yaml` at all, so Lane 2's local edits are being made
   against a tree that cannot run the e2e scope.
5. **Only then** clone `gp_unified` and run the two smokes of §3.2. Do not touch `gp_e2e` or
   `gp_root` until §4.2's dates pass; retarget with `GENESIS_PICKAPLACE_ROOT` / `GP_ROOT` instead.
6. **Before deleting anything**, `git bundle` `gp_e2e` and tar `release_v4/r2dreamer` —
   `episode_record.py` and `longrun_milestones.py` exist in no repository.
