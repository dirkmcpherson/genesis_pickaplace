# {r2dreamer} Ladder-N milestone cells (Lane 14, 2026-09-12)

**The first `home` cells for the world model in the Ladder-N batch.** Until this sweep ran, the only
Ladder-N `home` numbers in existence for {r2dreamer} were training-record counters, and
`paper/ROUTE_CENSUS_RC2_2026-09-11.md` showed those can read 0.79 on a checkpoint that reloads to
0/90. `cluster/wmfix_full.sbatch:153–165` evaluates only `latest.pt` and only at the end of the job,
so every `milestones/online_*.pt` was unevaluated and the registered "matched milestone" rule was
unimplementable for this learner (`paper/AUDIT_STATISTICAL_SHOT_2026-09-12.md`, VERDICT item 2).
**PHASE_PLAN (aa) REVISION 3 schedules the sweep; this document is its readout.**

Convention, per `CLAUDE.md`: every number here is **{r2dreamer}**, scope **e2e (full task)**.

Environment: `LAB=/cluster/tufts/shortlab/jstale02`, `W=$LAB/wm_fix_2026-09-03`, branch
`ladder-unify-2026-09-11`. Trees: `$LAB/gp_ladderN` @ `a40c8aa1` (**pinned; read only here**) and
`$W/r2dreamer_ladderN` @ `0cf3d9e`.

---

## 1. What produced these cells

    bash cluster/ln_r2_milestone_sweep.sh          # deployed as $W/ln14_milestone_sweep.sh
    # -> sbatch cluster/ln_r2_milestone_eval.sbatch <run> <milestone>   (as $W/ln14_milestone_eval.sbatch)
    python3 cluster/ln_r2_milestone_table.py --md  # the table below

**Scope of the sweep:** `$W/runs/full_r2d_state_*_r{nrh,nsh,zh}_s9*` — the (aa) batch, the rev-2
extension, and rev 3 as its runs appear. The pilot's `_rz_` / `_rs_` runs and the `*_lnsmoke_*`
smokes are out by construction (different guard, different ladder, 15k steps).

**Three cells per milestone**, all `--max-steps 1200`, `--seed 0`, `--device cpu`, ICs from
`$GP/baselines/eval_ics.json`:

| cell | set | episodes | action mode | why |
|---|---|---:|---|---|
| `rnd30_mode` | `rnd` | 30 | `mode` | (aa) readout cell |
| `hold15_mode` | `hold` | 15 | `mode` | (aa) readout cell |
| `rnd30_sample` | `rnd` | 30 | `sample` | the route census's statistic, so the two are comparable |

**Provenance.** The checkpoint is copied on the login node BEFORE the job is submitted — `latest.pt`
is rewritten periodically while a run is alive, so a copy taken inside the job would not pin what was
scored — and the copy's sha256 is verified against the milestone sidecar's own `sha256`. The run's
`.hydra/{config,overrides,hydra}.yaml`, `ladder_provenance.json` and `step_contract.json` are copied
beside it and a `provenance.json` records source path, source mtime, both hashes, the sidecar, and the
run's training step at the moment of the copy. The evaluator reads the run's own ladder, tip guard and
return clamp out of that config; nothing is overridden on the command line. No run directory is
written to.

**Hardware, disclosed.** `SWEEP_MODE=cpu64`: `-p batch --qos=normal`, `-N 1 -n 8 --mem=48g` (the same
CPU shape the launcher gives its own in-job evals), on nodes of **64 physical cores**, with the sbatch
asserting the count itself before the first episode. This is a deliberate change from Lane RC's GPU
submission: the r2dreamer adapter builds Genesis with `backend="cpu"`
(`$W/r2dreamer_ladderN/envs/genesis.py:214`) and the policy runs `--device cpu`, so no part of this
evaluation uses a GPU; both GPU allocations were at their cap with this project's own training runs at
the time; and pinning the core count makes every cell below one hardware class, which the unpinned GPU
nodes are not. Every cell stamps `node` / `cpu_model` / `ncpus_machine`, so the class can be checked
rather than assumed. `SWEEP_MODE=gpu` reproduces Lane RC's shape; **do not mix the two inside one
table.**

**Re-run the sweep after each milestone lands.** There is no cron on this cluster. The sweep is
idempotent: a complete cell-triple is skipped, a partial one is resubmitted and the sbatch skips the
cells it already has, a queued one is left alone. Ceiling `MAXJOBS` (default 6).

---

## 2. How to read the table

Counts (`k/N`), never rates. At `N` = 15 or 30 a `home` rate invites exactly the comparison
`AUDIT_STATISTICAL_SHOT` §2 shows the design cannot support (exact-permutation floor p = 0.333 at
n = 2 v 2; MDE 0.27–0.74 on the rate scale). **(aa) REVISION 3 makes the decision statistic an
IGNITION read — "seeds, per arm, with ≥ 1 `home` in a pinned cell" — not a rate**, and these counts
are the input to that read. An absent cell is written `absent`; it is never a zero.

`home` = `slide_event ∧ nested_v2` (the can was pushed goalward after release AND settled in the
goal). `farside` and `slide_event` are logged under every ladder, so the columns are populated for
the `nested_ramp`, `nested_sparse` and `staged` runs alike.

---

## 3. Cells

<!-- TABLE -->

---

## 4. What this does and does not establish

*(filled with the table)*
