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

**Snapshot: 2026-09-12 03:55 EDT — 26 cells, 645 episodes.** The sweep is still running; re-run it
and regenerate this table (`python3 cluster/ln_r2_milestone_table.py --md`). The full first pass is
about 78 cells over 26 (run, milestone) pairs and grows as milestones land.

| run | milestone | cell | n | ladder | picked | placed_v2 | farside | slide_event | home | nested_v2 | nested_honest | tipped | node | cores |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| dDPfull_first_rnrh_s970 | online_1000000 | rnd30_mode | 30 | nested_ramp | 15/30 | 16/30 | 13/30 | 0/30 | 0/30 | 0/30 | 0/30 | 18/30 | pax078 | 64 |
| dDPfull_first_rnrh_s970 | online_1000000 | hold15_mode | 15 | nested_ramp | 13/15 | 12/15 | 12/15 | 0/15 | 0/15 | 0/15 | 0/15 | 8/15 | pax078 | 64 |
| dDPfull_first_rnrh_s970 | online_1000000 | rnd30_sample | 30 | nested_ramp | 16/30 | 14/30 | 11/30 | 0/30 | 0/30 | 0/30 | 0/30 | 21/30 | pax078 | 64 |
| dDPfull_first_rnrh_s970 | online_500000 | rnd30_mode | 30 | nested_ramp | 4/30 | 2/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 17/30 | pax078 | 64 |
| dDPfull_first_rnrh_s970 | online_500000 | hold15_mode | 15 | nested_ramp | 4/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 6/15 | pax078 | 64 |
| dDPfull_first_rnrh_s970 | online_500000 | rnd30_sample | 30 | nested_ramp | 9/30 | 4/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 20/30 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_1000000 | rnd30_mode | 30 | nested_ramp | 18/30 | 10/30 | 6/30 | 0/30 | 0/30 | 1/30 | 1/30 | 21/30 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_1000000 | hold15_mode | 15 | nested_ramp | 15/15 | 6/15 | 4/15 | 0/15 | 0/15 | 1/15 | 1/15 | 8/15 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_1000000 | rnd30_sample | 30 | nested_ramp | 18/30 | 12/30 | 9/30 | 0/30 | 0/30 | 1/30 | 1/30 | 17/30 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_500000 | rnd30_mode | 30 | nested_ramp | 7/30 | 4/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 12/30 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_500000 | hold15_mode | 15 | nested_ramp | 10/15 | 1/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 10/15 | pax078 | 64 |
| dDPfull_first_rnrh_s971 | online_500000 | rnd30_sample | 30 | nested_ramp | 8/30 | 4/30 | 1/30 | 0/30 | 0/30 | 0/30 | 0/30 | 16/30 | pax078 | 64 |
| dDPfull_first_rnrh_s973 | online_500000 | rnd30_mode | 30 | nested_ramp | 14/30 | 4/30 | 1/30 | 0/30 | 0/30 | 0/30 | 0/30 | 20/30 | pax078 | 64 |
| dDPfull_first_rnrh_s973 | online_500000 | hold15_mode | 15 | nested_ramp | 10/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 10/15 | pax078 | 64 |
| dDPfull_first_rnrh_s973 | online_500000 | rnd30_sample | 30 | nested_ramp | 8/30 | 4/30 | 1/30 | 0/30 | 0/30 | 0/30 | 0/30 | 19/30 | pax078 | 64 |
| dDPfull_first_rnsh_s975 | online_1000000 | rnd30_mode | 30 | nested_sparse | 0/30 | 3/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 6/30 | pax078 | 64 |
| dDPfull_first_rnsh_s975 | online_1000000 | hold15_mode | 15 | nested_sparse | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | pax078 | 64 |
| dDPfull_first_rnsh_s975 | online_1000000 | rnd30_sample | 30 | nested_sparse | 0/30 | 2/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 8/30 | pax078 | 64 |
| dHfull_all_rnrh_s950 | final | rnd30_mode | 30 | nested_ramp | 16/30 | 12/30 | 11/30 | 9/30 | 0/30 | 0/30 | 0/30 | 15/30 | pax027 | 64 |
| dHfull_all_rnrh_s950 | online_500000 | rnd30_mode | 30 | nested_ramp | 8/30 | 3/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 19/30 | pax019 | 64 |
| dHfull_all_rnrh_s950 | online_500000 | hold15_mode | 15 | nested_ramp | 6/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 10/15 | pax019 | 64 |
| dHfull_all_rnrh_s950 | online_500000 | rnd30_sample | 30 | nested_ramp | 10/30 | 1/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 22/30 | pax019 | 64 |
| dHfull_all_rnrh_s951 | online_1000000 | rnd30_mode | 30 | nested_ramp | 7/30 | 3/30 | 1/30 | 0/30 | 0/30 | 0/30 | 0/30 | 17/30 | pax004 | 64 |
| dHfull_all_rnrh_s951 | online_1000000 | hold15_mode | 15 | nested_ramp | 7/15 | 4/15 | 2/15 | 0/15 | 0/15 | 1/15 | 1/15 | 10/15 | pax004 | 64 |
| dHfull_all_rnrh_s951 | online_500000 | rnd30_mode | 30 | nested_ramp | 4/30 | 3/30 | 0/30 | 0/30 | 0/30 | 0/30 | 0/30 | 22/30 | pax027 | 64 |
| dHfull_all_rnrh_s951 | online_500000 | hold15_mode | 15 | nested_ramp | 3/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 0/15 | 9/15 | pax027 | 64 |

**Every cell above is 64 physical / 64 logical cores, Intel Xeon Gold 6438M** (pax004, pax019,
pax027, pax078, pax146) — the same CPU model as pax146, which built the six demonstration sets.

`home`, pooled per run over the cells that exist (**an ignition read, not a rate**):

| run | arm | ladder | `home` | episodes |
|---|---|---|---:|---:|
| `dDPfull_first_rnrh_s970` | machine | nested_ramp | **0** | 150 |
| `dDPfull_first_rnrh_s971` | machine | nested_ramp | **0** | 150 |
| `dDPfull_first_rnrh_s973` | machine | nested_ramp | **0** | 75 |
| `dDPfull_first_rnsh_s975` | machine | nested_sparse | **0** | 75 |
| `dHfull_all_rnrh_s950` | human | nested_ramp | **0** | 105 |
| `dHfull_all_rnrh_s951` | human | nested_ramp | **0** | 90 |
| **total** | | | **0** | **645** |

---

## 4. What this does and does not establish

**1. `home` is 0 in every cell — 645 episodes, 6 seeds, both arms, both nested ladders, 0.5M / 1M /
2M-final.** This is the first evaluation-cell evidence about `home` for the world model under Ladder
N, and it is negative at every milestone scored so far. It does **not** say the recipe has failed:
four of the six runs scored here are at 0.5M–1M of a 2M or 4M budget, and the pilot's own staged
arm produced ~0 honest nests at 2M and 0.167 at 4.1M (PHASE_RESULTS §5.6).

**2. The first `slide_event` any Ladder-N {r2dreamer} policy has produced in a CELL:
`dHfull_all_rnrh_s950` at its FINAL 2M checkpoint — `slide_event` 9/30 on `rnd30` mode**, with
`picked` 16/30, `placed_v2` 12/30, `farside` 11/30, and `nested_v2` **0**/30. The human-arm ramp
seed, trained out, releases the can and pushes it goalward in nearly a third of random starts and
**arrives in none of them**. Its outcome taxonomy is `{tipped 15, timeout 15, nested_honest 0}` and
`slide_routes` is `{sustained 0, settle 0}`; mean episode reward 1.691 of a possible 9. The ramp
rung is being earned; the terminal is not. Checkpoint sha256 `e45b637b…`, copied at training step
2,117,636 (= the 2M online budget on a 117,624-step prefill origin).

That is the cell version of the {RLPD} picture in `paper/RL100_READOUT_2026-09-11.md` and of the
whole project's slide problem: **the push happens, the arrival does not.** It is one seed, one cell,
and it must not be reported as an arm difference — the machine arm has no `final` cell yet.

**3. `nested_v2` without `home` appears three times** (`dDPfull_first_rnrh_s971` at 1M, all three
cells, and `dHfull_all_rnrh_s951` 1M `hold15_mode`) — i.e. the can ends in the goal with
`slide_event` false. That is the DROP route P8 / P-aa-4 predicts, now visible under the nested
ladders. At these counts (1 episode per cell) it is an existence proof, not a share.

**4. `dDPfull_first_rnsh_s975` (nested_sparse, machine, 1M) has `picked` 0/45 across its three
cells.** It is not picking at all at 1M, which matches the training-record reading in the audit
(§1.1 note: s975/s976 last-300 `picked` 0.023 / 0.007). `nested_sparse` pays nothing before the
terminal, so a seed that has not found the pick has no gradient to follow; whether 4M is enough is
exactly what (aa) rev 3's four extra seeds per learner exist to answer.

**5. Nothing here is a source comparison.** n is 2 human v 3 machine runs with unequal milestone
coverage, and the registered statistic is an ignition read at 4 v 4 per arm
(`AUDIT_STATISTICAL_SHOT` §2: exact-permutation floor p = 0.333 at n = 2 v 2).

### Open / carried

* **Two cells were scored on a 64-physical / 128-logical node** (pax006, pax012 — same Xeon Gold
  6438M, SMT on) before the sweep asserted both counts. They are quarantined as
  `<cell>_smt128` under `full_r2d_state_dHfull_all_rnrh_s950/online_{1000000,2000000}/` and are NOT
  in the table; the sweep re-scores them on a 64/64 node. **They are worth looking at when the
  re-score lands:** both read `picked` ≈ 0 (`rnd30_mode` 0/30, `hold15_mode` 0/15) for a seed that
  reads 8/30 at 0.5M and 16/30 at 2M on 64/64 nodes. Either the policy dipped at 1M or the machine
  class moved the cell; the re-score separates the two, and it is the cheapest available test of the
  project's own hardware-sensitivity finding.
* `full_r2d_state_dHfull_all_rnrh_s950` has reached its 2M budget, so `final` cells exist for it.
  Every other run in scope is still training; their `final` cells appear as they finish.
* The {r2dreamer} rev-3 runs (`_rnsh` s957/s958/s977/s978, `_rzh` s962/s963/s982/s983) were still
  PENDING when this snapshot was taken. The sweep picks them up automatically once they write a
  milestone.
