# Route census — job log (Lane RC, 2026-09-11)

Operational log for PHASE_PLAN **(z) P8** / **(aa) P-aa-4** (the route census): whether the sparse
pilot's `nested_v2` arrivals are SLIDES (`home` = `slide_event` ∧ `nested_v2`) or DROPS
(`nested_v2` without `home`). Read-only on every existing tree and checkpoint; writes only under
`$W/route_census_2026-09-11/` (cluster) and this doc + `paper/ROUTE_CENSUS_2026-09-11.md` (repo).

`LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`, local checkout
`/home/j/workspace/genesis_pickaplace` on branch `ladder-unify-2026-09-11`.

Convention, per CLAUDE.md: results carry an explicit `{learner}` header.

---

## Step 0 — why this is answerable at all

The checkpoints under census trained under PHASE_PLAN (z)'s **`sparse`** ladder (`nested_v2` the
only paid, terminal rung; `return_clamp=1.0`; `tip_guard` unset in the sidecar, so both
evaluators fall back to the rule of record, `grip`). That ladder predates amendment (aa)'s
`farside`/`slide_event`/`home` columns, but `baselines/stage_predicates.py` computes those three
under **every** ladder (they are logged, not paid, unless the ladder is `nested_sparse` /
`nested_ramp`) — confirmed directly from the running eval's own stdout (below) rather than
assumed from the source. Because `home := slide_event ∧ nested_v2` and this ladder's terminal
IS `nested_v2`, `home` is never right-censored here: every episode that reaches `nested_v2` also
resolves `home` at that same decision (contrast the STAGED ladder's `slide_success` terminal,
which Lane 11 found DOES right-censor `home` on 28/600 rollouts because `slide_success` can fire
before `home` would — `paper/PILOT_RESCORE_2026-09-11.md` §"`--never-terminate`"). So the route
split asked for here is well-defined from a plain (non-`--never-terminate`) rollout.

## Step 1 — trees, read-only

* `$LAB/gp_ladderN` (genesis_pickaplace) — used AS-IS, at whatever commit it carried when each job
  ran (recorded per-job below via `git describe`); this lane made no commits to it. Not pinned by
  this lane (no training job of this lane's own depends on a fixed commit), but nothing in
  `baselines/rl/full_env.py` / `genesis_can_env.py` / `stage_predicates.py` was touched.
* `$W/r2dreamer_ladderN` @ `0cf3d9e`, untouched.
* `$LAB/gp_unified/baselines/rl/checkpoints/e2e/` — READ ONLY. Two `rlpd_final.zip` files opened
  by path; nothing written under `gp_unified` (the pilot rescore array `3581786` mentioned in
  `LADDER_N_PILOT_LOG_2026-09-11.md` §6 is a DIFFERENT lane's work on the STAGED checkpoints —
  unrelated to this census, and confirmed not disturbed).
* `$W/runs/full_r2d_state_{dHfull_all_rs_s945,dHfull_all_rs_s946,dDPfull_first_rs_s965,dDPfull_first_rs_s966}/`
  — STILL TRAINING for the whole duration of this census (confirmed still `RUNNING` in `squeue`
  under job names `lz_r2_sparse_dH_s945`, `lz_r2_sparse_dH_s946`, `lz_r2_sparse_dM_s965`,
  `lz_r2_sparse_dM_s966` throughout). Only `latest.pt` + `.hydra/{config,overrides,hydra}.yaml`
  were **copied** (never moved, never opened for writing) into
  `$W/route_census_2026-09-11/<run>/`, and the evaluator was pointed at the COPY.

**Copy provenance** (`$W/route_census_2026-09-11/COPY_PROVENANCE.txt`, source mtime + the
training run's own `step` from `metrics.jsonl` at the instant of copy — not the step the eval
runs against a moment later, since training kept advancing underneath):

| run | source `latest.pt` mtime | training step at copy |
|---|---|---:|
| `full_r2d_state_dHfull_all_rs_s945` | 2026-09-11 22:01:50 -0400 | 2,902,295 |
| `full_r2d_state_dHfull_all_rs_s946` | 2026-09-11 22:25:16 -0400 | 1,819,013 |
| `full_r2d_state_dDPfull_first_rs_s965` | 2026-09-11 22:18:28 -0400 | 3,984,960 |
| `full_r2d_state_dDPfull_first_rs_s966` | 2026-09-11 22:16:42 -0400 | 3,993,332 |

These are checkpoints of runs still en route to their 4M-step budget, not final checkpoints — the
census answers "does this policy, as it exists right now, arrive by slide or by drop", not a
claim about the trained-out policy.

## Step 2 — the two checkpoints' training config, confirmed field-for-field

`.hydra/overrides.yaml` (copied alongside `latest.pt`) on all four r2dreamer runs: `env.ladder=sparse`,
`env.return_clamp=1.0`, `model.return_clamp=1.0`, no `env.tip_guard` (predates the argument) and no
`env.far_release` (predates the argument, so `False` by construction). `config.yaml` confirms
`ladder: sparse`, `return_clamp: 1.0` (both env and model blocks), `scope: full`,
`action_repeat: 4`. The two {RLPD} sidecars (`ladder_provenance.json` in
`$LAB/gp_unified/baselines/rl/checkpoints/e2e/{e2e_rlpd_dH_s945,e2e_rlpd_dDPfirst_s965}`) read
`{'ladder': 'sparse', 'tip_guard': None, 'max_return': 1.0, 'terminal_stages': ['nested_v2',
'tipped']}` — same ladder, same fallback tip guard, on the other learner's checkpoint of the same
pilot arm/seed pairing. So "run the evaluator with `env.ladder=sparse env.tip_guard=grip
env.return_clamp=1.0 model.return_clamp=1.0`" is not an override either evaluator needed to be
told: it is what these runs' own saved configuration/sidecar already says, and both evaluators
print it back before scoring a single episode (verbatim, §3).

## Step 3 — the two evaluators, confirmed to carry the calibrated columns

`{r2dreamer}` `eval_genesis.py` (`$W/r2dreamer_ladderN`) reads `env.ladder` / `env.tip_guard` from
the run's own `.hydra/config.yaml` (no CLI override exists or is needed); its `STAGES` tuple ends
`(..., "farside", "slide_event", "home", "nested_honest")` and every episode's `per_episode[i]`
carries a `stages` dict with those keys as booleans.

`{RLPD}` `eval_e2e.py` (`$LAB/gp_ladderN`) DOES take `--ladder`/`--tip-guard` as CLI flags, but
only as an ASSERTION against the checkpoint's own sidecar (a disagreement is fatal); passed here
as `--ladder sparse --tip-guard grip` for exactly that assertion, and it agreed
(no `--ladder`/`--tip-guard` mismatch fired). `HEADLINE_STAGES` includes `farside`, `slide_event`,
`home` (`baselines/eval_e2e.py:371-372`), and `per_episode[i]['stages']` carries the same
per-episode booleans, so the route split can be read directly off a plain `--out` run; `--records-out`
was still passed (per the brief) so every episode's per-frame trajectory is re-scorable offline if
needed later, at ~50-90 KB/episode (Lane 11's measured cost for the same recorder).

## Step 4 — r2dreamer: test job, then the remaining 7

Test: `sbatch -J rc_r2_dHs945_hold $RC/eval_r2d_route.sbatch full_r2d_state_dHfull_all_rs_s945 hold 15`
→ **3592038**. Confirmed on first episode's stdout:

    [ladder] unified-2026-09-10 | ladder=sparse | nested_v2=1 | max_return=1 | terminal=nested_v2+tipped
             | shaping=off | far_release=off | tip=tilt>60deg&grip@1f
             | full_env=23fe428f222f genesis_can_env=40544bf73c8c stage_predicates=a589b4f05632
             | git=known-good-2026-08-27-895-ga40c8aa1-dirty
    [eval] ladder='sparse' from the run config
    [eval] tip_guard='grip' from the rule of record (this run config predates the argument)
    [eval] outcome success_key='nested_v2' (the ladder's paid terminal)
    [eval] ICs from .../gp_ladderN/baselines/eval_ics.json set=hold -> 15 episode(s)

Confirmed working; the remaining 7 submitted as a batch (`rc_r2_dHs945_rnd` 3592091,
`rc_r2_dHs946_hold` 3592092, `rc_r2_dHs946_rnd` 3592093, `rc_r2_dMs965_hold` 3592094,
`rc_r2_dMs965_rnd` 3592095, `rc_r2_dMs966_hold` 3592096, `rc_r2_dMs966_rnd` 3592097) — **8 GPU jobs
total, the registered ceiling for this lane, at once**. Resource shape: `-p gpu,preempt
--qos=preempt --gres=gpu:1 --constraint=l40s|a100|l40|h200 --exclude=pax077 -t 0-02:00:00`
(mirrors `wmfix_full.sbatch`'s GPU request; `--device cpu` inside `eval_genesis.py` itself, same
as the training launcher's own in-job eval loop — the GPU is for Genesis' world build, not the
policy forward pass). Per-episode cost measured on the test job: ~25-50 s/episode depending on
`tipped` (short) vs `timeout` (full 300 decisions), plus a ~23 s one-time world build.

## Step 5 — RLPD: --require-cores fired as designed on 3 of 4 first attempts

First submission (no node pin), `-p batch --qos=normal -N 1 -n 8 --mem=48g`:
`rc_rl_dHs945_hold` **3592128**, `rc_rl_dHs945_rnd` **3592129**, `rc_rl_dMs965_hold` **3592130**,
`rc_rl_dMs965_rnd` **3592131**. Three of four landed on the wrong machine size and refused,
verbatim (`eval_e2e.py`'s own `--require-cores` guard, not a bug in this lane's script):

    FATAL: --require-cores 64 but this machine has 40 physical cores (2 socket(s), Intel(R) Xeon(R)
    Gold 6248 CPU @ 2.50GHz on pax069). Full-scope outcomes track MACHINE SIZE ...
    FATAL: --require-cores 64 but this machine has 36 physical cores (... pax108).
    FATAL: --require-cores 64 but this machine has 40 physical cores (... pax165).

`rc_rl_dMs965_hold` (3592130) landed on pax027 (a 64-physical-core node not in the 09-08 census,
so it slipped in unfiltered) and ran clean. The three failures were resubmitted pinned to the
09-08 census's confirmed 64-physical-core node list (`$LAB/gp_e2e/hw_map.json`, READ ONLY, 55
nodes: `pax004,005,015,019,030,031,032,033,034,036,037,038,039,040,041,043,045,046,048,053,054,
055,056,058,059,060,061,062,064,065,066,067,068,078,079,080,104,110,111,112,113,114,115,116,117,
146,148,149,150,151,152,153,179,180,181`) via `--nodelist`: `rc_rl_dHs945_hold` **3592167**,
`rc_rl_dHs945_rnd` **3592168**, `rc_rl_dMs965_rnd` **3592170** — all three landed on `pax015` and
ran. Cost: 3 x ~5-8 s of wasted CPU allocation, caught immediately by the guard's own assertion,
no wrong-hardware cell produced.

## Step 6 — outcomes

(filled in as jobs complete; see `paper/ROUTE_CENSUS_2026-09-11.md` for the scored results)
