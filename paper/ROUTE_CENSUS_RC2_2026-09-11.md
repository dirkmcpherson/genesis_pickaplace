# The s945 train-vs-eval discrepancy — investigated (Lane RC2, 2026-09-11/12)

Follow-up to `paper/ROUTE_CENSUS_2026-09-11.md` / `paper/ROUTE_CENSUS_LOG_2026-09-11.md` (Lane RC), which
flagged: {r2dreamer} sparse-pilot seed `dHfull_all_rs_s945` reports `episode/train_ep_nested_v2 = 1` in a large
fraction of recent TRAINING episodes, yet the route census's cold evaluation of a copy of its `latest.pt` scored
`nested_v2` 0/45 (hold15 + rnd30). Sibling seed `s946` scored 11/45 in the same evaluation. This note asks why,
and answers: **the discrepancy is real, reproduces on a second, later checkpoint, and does not depend on which
evaluator (old training-tree or new route-census tree) or which action mode (sample or mode) is used. The
mechanism inside the training loop that produces it was not found** — eight specific candidate causes were
checked and ruled out (below). The 0/45 figure is the one to trust for "what this checkpoint does on reload";
the training-time counter should not be read as a description of reload behaviour for this run, pending further
work.

Read-only on every existing tree and live run; all new evaluation output lives under
`$W/route_census_2026-09-11/rc2/` on the cluster (`$LAB=/cluster/tufts/shortlab/jstale02`,
`$W=$LAB/wm_fix_2026-09-03`). Local checkout `/home/j/workspace/genesis_pickaplace`, branch
`ladder-unify-2026-09-11`.

---

## Check 1 — copy provenance: both copies are complete, neither is corrupt

`$W/runs/full_r2d_state_dHfull_all_rs_s945` and `..._s946` were STILL TRAINING (Slurm jobs `lz_r2_sparse_dH_s945`
3539261 and `lz_r2_sparse_dH_s946` 3539262, both `RUNNING`, no requeue for s945; s946 was preempted/requeued
once earlier per the pilot log but is running clean now) for the whole duration of this check. `trainer.py`'s
periodic checkpoint save is atomic (`torch.save` to `latest.pt.tmp`, then `os.replace` — confirmed by reading
`trainer.py:40-52`), so a `cp` mid-write cannot produce a torn file; the only question is which of the atomically
-written versions a given `cp` happened to land on.

| copy | source `latest.pt` mtime | metrics.jsonl step near copy time | checkpoint's own `step` (torch.load) | lag | file size |
|---|---|---:|---:|---:|---:|
| s945, original (Lane RC) | 2026-09-11 22:01:50 -0400 | 2,902,295 | **2,817,628** | 84,667 | 118,684,619 B |
| s946, original (Lane RC) | 2026-09-11 22:25:16 -0400 | 1,819,013 | **1,817,624** | 1,389 | 118,684,619 B |
| s945, FRESH (this lane) | 2026-09-11 23:23:23 -0400 (source's own last write) | ~3,154,905 (read 23:33) | **3,117,624** | ~37,281 | 118,684,619 B |

`save_every: 100000.0` (`.hydra/config.yaml`), so a lag under 100,000 between "the last training step visible in
`metrics.jsonl` when the copy was taken" and "the step baked into the checkpoint itself" is exactly the periodic
-save cadence, not evidence of a stall — s945's two lags (84,667 and 37,281) and s946's (1,389) are all within
one cycle. `torch.load` succeeds cleanly on every copy (`keys: ['agent_state_dict', 'optims_state_dict', 'step']`
only); all four copied `latest.pt` files are byte-size-identical to each other and to the live source file at the
time checked (a fixed architecture always serializes to the same size, so size equality is a necessary but not
sufficient integrity check — `torch.load` succeeding and the `step` field being a sane, monotonically-plausible
number is the stronger one, and both hold). **Conclusion: nothing about s945's copies is anomalous relative to
s946's; both are genuine, uncorrupted periodic checkpoints.**

Command used throughout:
```
ssh jstale02@login.pax.tufts.edu
W=/cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03
cp $W/runs/full_r2d_state_dHfull_all_rs_s945/latest.pt      $W/route_census_2026-09-11/rc2/s945_fresh/latest.pt
cp -r $W/runs/full_r2d_state_dHfull_all_rs_s945/.hydra      $W/route_census_2026-09-11/rc2/s945_fresh/.hydra
/cluster/tufts/shortlab/jstale02/r2d_venv/bin/python -c \
  "import torch; print(torch.load('.../latest.pt', map_location='cpu')['step'])"
```

## Check 2 — the training-time counter: s945 and s946 both look equally healthy, right now

Literal last 300 training episodes in each run's CURRENT `metrics.jsonl` (both runs still advancing; read
2026-09-11 ~23:50-11:57 -0400):

| run | step range (last 300 ep) | picked | placed_v2 | nested_v2 | nested_honest | tipped | task_success | mean length |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| s945 | 3,133,684 - 3,226,257 | 0.940 | 0.847 | **0.787** (236/300) | 0.783 | 0.057 | 0.463 | 78.4 |
| s946 | 2,131,543 - 2,242,712 | 0.863 | 0.783 | **0.713** (214/300) | 0.710 | 0.100 | 0.257 | 93.0 |

**s945's own recent counter is not weaker than s946's — it is numerically stronger on every column.** Both series
were also checked in a 150-episode window centred on each seed's evaluated checkpoint step (not just the current
tail): s945 around step 2,817,628 (the original copy) reads picked 0.760 / placed_v2 0.587 / nested_v2 0.527 /
tipped 0.167; around step 3,117,624 (the fresh copy) reads picked 0.840 / placed_v2 0.673 / nested_v2 0.627 /
tipped 0.133. Binned every 200k steps from step 0, nested_v2 for s945 has sat in a 0.59-0.77 band continuously
since step ~1.4M (12 consecutive 150-600-episode bins, no window below 0.59). **s945's training counter is not an
outlier, a transient spike, or a stale read — it is a long, stable, high plateau, exactly like s946's.** This
rules out "the checkpoint just landed on a bad instant inside an otherwise-fluctuating series": the surrounding
half-million steps in both directions tell the same story.

Command: `rc2_metrics_analysis.py` / `rc2_metrics_binned.py` (both under `$W/route_census_2026-09-11/rc2/`) over
each run's `metrics.jsonl`, filtering lines carrying `episode/length`.

## Check 3 — IC distribution: `hold15` is the fair comparison set, but does not by itself explain s945

`baselines/rl/full_env.py` (the tree that trains these runs, `$LAB/gp_unified`), `FullTaskEnv.reset()`,
line 771 (`def reset`) through line 786:
```python
uid = (options or {}).get('uid') or self.fixed_uid
...
if uid is None:
    uid = int(self.np_random.choice(self.success_uids))   # line 785
obs = self.genv.reset(uid=int(uid))
```
For `scope='full'` training (no `entry_bank`, no fixed uid), every episode's start is drawn uniformly from
`self.success_uids` — the 74 demonstration-success UIDs of `dHfull_all` — never a free-form random continuous
position. `hold15`'s `metrics.json` confirms `"ic_mode": "demo"`. **`hold15` is therefore the IC set comparable
to what training itself draws from; `rnd30` (continuous novel starts) is a genuinely harder, out-of-distribution
set for this scope**, consistent with the CLAUDE.md correction from 2026-09-07 that "hold15 is NOT held-out
(14/15 training starts)".

This explains ALL of s946's train-vs-eval gap: new-tree hold15 (8/15 = 0.533) sits close to its
training-window rate (0.62-0.71); new-tree rnd30 (3/30 = 0.10) is far below both, exactly as expected for an
out-of-distribution set. **It explains NONE of s945's gap**: s945's hold15 is ALSO 0/15, on the very IC
population its own training draws from and reportedly succeeds on 53-79% of the time. Check 3, on its own,
narrows the question but does not answer it.

## Check 4 — the decisive test: old tree, new tree, sample, mode — all four agree at 0, then 0 again on rnd30

A FRESH copy of s945's `latest.pt` (checkpoint step **3,117,624**, ~300k steps later than the original census
copy) was evaluated six ways: {old tree, new tree} x {hold15 sample, hold15 mode} plus {old tree, new tree} x
{rnd30 sample}. "Old tree" = `$W/r2dreamer_unified` + `$LAB/gp_unified`, the exact code that is training this
run (`R2_TREE`/`GENESIS_PICKAPLACE_ROOT` read directly off the live Slurm job via `scontrol show job 3539261`
-> `Command=.../gp_unified/cluster/wmfix_full.sbatch`, cross-checked against `paper/PHASE_PLAN_2026-09-04.md:1098`
and `paper/LADDER_PILOT_LOG_2026-09-11.md` §(z).9, which register `R2_TREE=$W/r2dreamer_unified
GENESIS_PICKAPLACE_ROOT=$LAB/gp_unified` for this exact run family). "New tree" = `$W/r2dreamer_ladderN` +
`$LAB/gp_ladderN`, the route census's own tree. `eval_ics.json` is byte-identical (sha256
`4d2587b9...`) across `genesis_pickaplace`, `gp_unified` and `gp_ladderN`, so the IC sets are not a variable here.

| tree | set | mode | n | nested_v2 | tipped | timeout | mean steps | job |
|---|---|---|---:|---:|---:|---:|---:|---:|
| old (`r2dreamer_unified`/`gp_unified`) | hold15 | sample | 15 | 0/15 (0.00) | 7/15 (0.47) | 8/15 (0.53) | 175 | 3594141 |
| old | hold15 | mode | 15 | 0/15 (0.00) | 7/15 (0.47) | 8/15 (0.53) | 179 | 3594142 |
| new (`r2dreamer_ladderN`/`gp_ladderN`) | hold15 | sample | 15 | 0/15 (0.00) | 7/15 (0.47) | 8/15 (0.53) | 175 | 3594143 |
| new | hold15 | mode | 15 | 0/15 (0.00) | 7/15 (0.47) | 8/15 (0.53) | 179 | 3594144 |
| old | rnd30 | sample | 30 | 0/30 (0.00) | 16/30 (0.53) | 14/30 (0.47) | 152 | 3594631 |
| new | rnd30 | sample | 30 | 0/30 (0.00) | 16/30 (0.53) | 14/30 (0.47) | 152 | 3594632 |

**Old and new tree agree to the episode**: not just the same aggregate fractions but the same per-episode
failure pattern (e.g. `rnd` episode 25 tips at step 2, episode 26 tips at step 8, identically, in both the old-
and new-tree rnd30 logs). This is not consistent with the new evaluator changing anything for this checkpoint —
tip_guard, far_release, action denormalisation and the terminal computation all behave identically whichever
tree runs them. Total: **0/90 `nested_v2` across all six cells**, on a checkpoint whose own training telemetry
reports 53-79% in every 150-600-episode window from step 1.4M onward (Check 2). Under a binomial with the
training-implied success probability (even a conservative p=0.5), `P(0/90) ≈ 8×10⁻²⁸`  — this is not sampling
variance around a real 50-79% rate.

Per the task's own branch logic: **because both evaluators agree at ~0, the training-time counter is what needs
explaining, not the route-census evaluator.**

### Eight specific mechanisms checked and ruled out

1. **Checkpoint truncation/corruption** — Check 1: clean `torch.load`, correct file size, sane `step`.
2. **`tip_guard` fallback difference** — the new tree's `tip_guard='grip'` fallback (absent from the sidecar,
   resolved to the rule of record) has `TIP_GUARD_SUSTAIN['grip'] = 1` (`gp_ladderN/baselines/rl/full_env.py:197`)
   — a single-frame tilt+grip-open check, byte-identical in effect to the old tree's hardcoded rule
   (`gp_unified/baselines/rl/full_env.py:1260-1261`, no `tip_guard` parameter exists in that tree at all).
3. **`far_release`** — off/`False` in both trees for this run (the flag postdates it in both).
4. **Action-generation formula** — TRAINING calls `agent.act(trans, agent_state, eval=False)`
   (`r2dreamer_unified/trainer.py:217`), which is `action_dist.rsample()` (`dreamer.py:288`).
   `eval_genesis.py --mode sample` calls `agent.act(trans, state, eval=(args.mode=="mode"))`
   (`eval_genesis.py:376`), i.e. `eval=False` for `--mode sample` too — the SAME `rsample()` call, confirmed by
   the eval script's own comment ("matches trainer's eval=False path", line 13). Not the discriminator.
5. **IC population** — Check 3: `hold15` draws from the same `success_uids` population training draws from.
6. **Episode horizon** — training's `env.time_limit: 1200` equals eval's `--max-steps 1200` exactly (both 300
   decisions at `action_repeat=4`); a longer training horizon completing a slow success that eval's horizon would
   cut off was considered and is not the case.
7. **Live-vs-frozen actor divergence inside the checkpoint** — `dreamer.py`'s `clone_and_freeze()` aliases
   `_frozen_actor`'s parameters to the live, trainable `actor`'s parameters (`param_new.data = param_orig.data`,
   same underlying tensor storage) and is called once at `__init__` and once after `.to(device)`
   (`dreamer.py:184,266`); `.act()` runs entirely on the `_frozen_*` clones. A direct tensor comparison of the
   saved checkpoint found **`actor.*` and `_frozen_actor.*` byte-identical (max |diff| = 0.0 over all 11 matched
   parameter tensors) in both s945's and s946's checkpoints** — no live/frozen skew is present in what was saved.
8. **Config/recipe divergence** — `diff` of `.hydra/{overrides,config}.yaml` between s945 and s946 shows only
   `seed`/`logdir` differ; every other setting (ladder, return_clamp, `actor_dist=bounded_normal`,
   `act_entropy=3e-5`, demo dir, buffer size, `env_num: 6`) is identical.

### What the adapter's sticky-flag mechanism actually does (traced, not assumed)

The task asked specifically whether `train_ep_nested_v2` is a sticky adapter artefact or a genuine terminal
proxy. Traced in the exact tree that trains s945 (`r2dreamer_unified/envs/genesis.py`):
- `GenesisPick.reset()` clears `self._emitted_stages = set()` at line 273, on every episode.
- `FullTaskEnv.reset()` (`gp_unified/baselines/rl/full_env.py:771`) clears `self._granted = set()` at line 782,
  on every episode (and again in `reset_to()`, lines 898-902, and the pick-scope reset, lines 1320-1329).
- `GenesisPick.step()` computes, per stage key, `reached = (k in self._env._granted) or bool(info.get(k))`
  (`envs/genesis.py:396`), sets `obs[f"log_ep_{k}"] = float(reached)` and marks `self._emitted_stages.add(k)`
  the first time it fires — i.e. the sticky "ep" twin only latches WITHIN one episode, sourced from a set that is
  provably cleared at the top of every episode.
- `trainer.py`'s training loop reads this at the episode boundary: `float(trans[key][i, 0])` for each env slot
  `i` where `done[i]` is true (`trainer.py:254-263`), i.e. the transition returned by that slot's own most recent
  `envs.step()` call — not a cross-slot or cross-episode read.

By static reading, this mechanism has no leak: nothing here would let episode N's `nested_v2` survive into
episode N+1's report. **This does not prove the mechanism is bug-free in the live, 6-way-vectorized process** —
only that no defect is visible from the adapter/env source alone. Confirming or refuting it fully would need
in-situ instrumentation of the live training process (e.g. dumping the exact UID + per-decision trace of one
training episode that reports `nested_v2=1`, then replaying that exact sequence in a fresh process) — outside
this lane's time budget; recommended as the next step if the mechanism must be pinned down rather than merely
worked around.

### Related, but not the same failure: lane DV3-6's critic/continuation finding

`paper/DV3_LOCAL_E2E_COLLAPSE_2026-09-11.md` (a different, local run, `model.rep_loss=dreamer`,
`ladder=nested_ramp`, `return_clamp=9.0`) documents a REAL policy collapse where a badly-miscalibrated critic
(`train/val` pinned at 2.6-8.3 against an empirical ceiling of 1.0, `train/con` pinned near 1.0 regardless of the
real tip rate) coincides with training telemetry ALSO showing near-zero success in the same window — i.e. in
that case training and eval AGREE (both ~0). That is a different phenomenon from this one: s945's training
telemetry does not show collapse anywhere after step ~1.4M (Check 2), and s945/s946 both use `return_clamp=1.0`,
which that same document identifies as the well-calibrated setting (matching the sparse ladder's own scope, not
the mismatched 9.0 case). The critic/continuation defect category is real in this codebase family, but does not,
on the evidence gathered, explain s945's specific train-vs-eval disagreement.

---

## Verdict: which figure to trust

**Trust the 0/45 (now 0/90, after this lane's six additional cells) cold-evaluation figure for any claim about
what this checkpoint will do if reloaded** — it reproduces across two independently-maintained code trees (one
of them the literal training tree), two action-selection modes, two checkpoint snapshots ~300k steps apart, and
two IC populations (one matched to training's own draw, one genuinely out-of-distribution). It is the only
figure that has been checked against alternative explanations and survived all eight.

**Do not treat s945's training-time `episode/train_ep_nested_v2` (or the sibling `placed_v2`/`picked` columns) as
a description of "this checkpoint, reloaded, will succeed at this rate."** The mechanism producing the gap
between the training counter (53-94% recent) and the reload behaviour (0/90) was not found in this lane's budget
despite ruling out the eight candidates above. Any phase-learning-curve or headline number for {r2dreamer}
sparse-ladder full-scope runs that is built FROM the training-time `episode/train_ep_*` stream, rather than from
a fresh checkpoint reload, should carry this caveat until the mechanism is pinned down — it may or may not
generalize to other seeds/runs in the same family (s946's own training counter and its cold eval are much closer
to each other, modulo the hold-vs-rnd IC gap Check 3 explains, so this is not necessarily a universal defect of
the pilot).

## What was and was not done

Done: Checks 1-4 in full, including all cells the task specified (fresh copy; hold15 sample+mode and rnd30
sample under both trees). Eight candidate mechanisms named in the task or discovered along the way were checked
against direct evidence (code reads, tensor comparisons, or matched job outputs), not assumed.

Not done, and flagged rather than guessed at: live instrumentation of the training process to catch the sticky-
flag mechanism (if any) in the act; a matched old-tree re-evaluation of s946 (not run — s946's own new-tree eval
already tracks its training counter reasonably, so there is no discrepancy on that seed to isolate the same way,
and this lane's job budget went to the seed that actually needed it, s945). Both are reasonable next steps if the
mechanism needs to be nailed down rather than worked around by trusting the cold-eval numbers.

## Job log (append to `paper/ROUTE_CENSUS_LOG_2026-09-11.md`; also listed there)

r2dreamer, fresh copy of s945 `latest.pt` (step 3,117,624), `$W/route_census_2026-09-11/rc2/s945_fresh/`:
`3594141` (old tree, hold15, sample), `3594142` (old tree, hold15, mode), `3594143` (new tree, hold15, sample),
`3594144` (new tree, hold15, mode), `3594631` (old tree, rnd30, sample), `3594632` (new tree, rnd30, sample).
All `rc=0`, all on `preempt` QOS, at most 4 concurrent (this lane's ceiling), `--require-cores` not applicable
(GPU jobs; the eval itself runs `--device cpu` for the policy forward pass, matching the training launcher's own
convention). Scripts: `$W/route_census_2026-09-11/rc2/eval_r2d_route_rc2.sbatch`,
`rc2_metrics_analysis.py`, `rc2_metrics_binned.py`, `rc2_length_check.py` (unused in the final write-up; kept for
provenance).
