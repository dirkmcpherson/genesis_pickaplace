# Route census: do the sparse-pilot policies arrive by SLIDE or by DROP? (Lane RC, 2026-09-11)

PHASE_PLAN **(z) P8** / **(aa) P-aa-4**. The (z) pilot's SPARSE {r2dreamer}/{RLPD} arms pay only
`nested_v2` (settled contact by any route) and, on training rollouts, reach it in a large fraction
of recent episodes. The old `sparse`-ladder records carry no route information. This census
re-evaluates the checkpoints under the calibrated stage tracker (`farside`/`slide_event`/`home`,
PHASE_PLAN amendment (aa)), which computes those columns under EVERY ladder, to split `nested_v2`
arrivals into `home` (= `slide_event` ∧ `nested_v2`, the slide route) and `nested_v2` without
`home` (the drop route). Full setup, defects and job log: `paper/ROUTE_CENSUS_LOG_2026-09-11.md`.

**Prediction of record (P8, PHASE_PLAN (z).8): under both ladders, the majority of `nested_v2`
events are drops (`pushed`/`home` = False). Disconfirm = slide route ≥ 50 % in any arm.** This is
a POLICY question — the demonstrations themselves already answered the analogous question the
other way (§(z).6 item 2: humans 12/14 slide vs 2 drop, machine 13/15 slide vs 2 drop, on the
cluster build of record) — so a policy result here is not required to match the demonstration
split, and P8 was registered anticipating that it would not.

Read-only on every existing tree and checkpoint; all evaluation artefacts live under
`$W/route_census_2026-09-11/` on the cluster (`$W = /cluster/tufts/shortlab/jstale02/wm_fix_2026-09-03`).

---

## 1. What was scored

Four **{r2dreamer}** checkpoints, all still mid-training toward a 4M-step budget at the moment
they were copied (never touched in place — see the LOG doc §1 for exact mtimes/steps):

| run | arm | seed | ladder | tip_guard | return_clamp |
|---|---|---:|---|---|---:|
| `full_r2d_state_dHfull_all_rs_s945` | human | 945 | `sparse` | `grip` (fallback; predates the arg) | 1.0 |
| `full_r2d_state_dHfull_all_rs_s946` | human | 946 | `sparse` | `grip` | 1.0 |
| `full_r2d_state_dDPfull_first_rs_s965` | machine-first | 965 | `sparse` | `grip` | 1.0 |
| `full_r2d_state_dDPfull_first_rs_s966` | machine-first | 966 | `sparse` | `grip` | 1.0 |

Two **{RLPD}** sparse-pilot checkpoints (`rlpd_final.zip`, 250k decisions, FINAL — training
already complete, unlike the r2dreamer arms), read directly from `$LAB/gp_unified` (read-only):

| run | arm | seed | ladder | tip_guard |
|---|---|---:|---|---|
| `e2e_rlpd_dH_s945` | human | 945 | `sparse` | `grip` (fallback; sidecar `tip_guard: None`) |
| `e2e_rlpd_dDPfirst_s965` | machine-first | 965 | `sparse` | `grip` |

Both evaluators confirmed the config field-for-field before scoring a single episode (LOG doc
§2-3); neither needed a CLI override to read `ladder=sparse`/`tip_guard=grip`/`return_clamp=1.0` —
that IS what these runs' own saved configuration says, and both evaluators printed it back.

Cells: `hold15` (15 episodes, the `hold` IC set) and `rnd30` (30 episodes, the `rnd` IC set),
SAMPLED actions, seed 0, `--max-steps 1200` (300 decisions at repeat 4), shared-process protocol
(one process per cell, not per episode — the `preview`-role convention used throughout this pilot;
not the isolated/`record`-role protocol PHASE_RESULTS §5.1 uses for cells of the paper). Twelve
cells total (4 checkpoints × 2 sets for {r2dreamer}, 2 checkpoints × 2 sets for {RLPD}).

Commands, per cell (identical shape across all twelve; the checkpoint/set/N vary):

```
# {r2dreamer}
GENESIS_PICKAPLACE_ROOT=$LAB/gp_ladderN R2D_SIM_VARIANT=gc_kp4_riser3_shelf6 \
  $LAB/r2d_venv/bin/python eval_genesis.py \
  --checkpoint $W/route_census_2026-09-11/<run>/latest.pt \
  --episodes <15|30> --mode sample --max-steps 1200 \
  --ic-file $LAB/gp_ladderN/baselines/eval_ics.json --ic-set <hold|rnd> --seed 0 \
  --out $W/route_census_2026-09-11/<run>/<set>_sample --device cpu

# {RLPD}
python baselines/eval_e2e.py --kind sac --checkpoint $LAB/gp_unified/.../<run>/rlpd_final.zip \
  --ic-file baselines/eval_ics.json --ic-set <hold|rnd> --mode sample --seed 0 \
  --ladder sparse --tip-guard grip \
  --out $W/route_census_2026-09-11/<run>/<set>_sample \
  --records-out $W/route_census_2026-09-11/<run>/records_<set>_sample \
  --role record --require-cores 64 --device cpu   # run from $LAB/gp_ladderN
```

---

## 2. Results

All 12 cells completed cleanly (`rc=0` on every job; `sacct` shows 0 `FAILED` among the cells that
counted — the 3 `--require-cores` refusals were caught before any episode ran, §Step 5 of the LOG
doc, and are not cells). Full per-episode data: `$W/route_census_2026-09-11/<run>/<set>_sample/metrics.json`;
pooled summary: `$W/route_census_2026-09-11/route_census_summary.json`
(`python3 route_census_report.py`, both under the same directory).

### {r2dreamer}

| run (seed) | set | n | picked | placed_v2 | nested_v2 | nested_honest | farside | slide_event | home | tipped |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `dHfull_all_rs_s945` (human) | hold15 | 15 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.133 |
| `dHfull_all_rs_s945` (human) | rnd30  | 30 | 0.000 | 0.100 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.367 |
| `dHfull_all_rs_s946` (human) | hold15 | 15 | 1.000 | 1.000 | 0.533 | 0.533 | 1.000 | 0.333 | 0.267 | 0.000 |
| `dHfull_all_rs_s946` (human) | rnd30  | 30 | 0.733 | 0.767 | 0.100 | 0.100 | 0.667 | 0.100 | 0.033 | 0.233 |
| `dDPfull_first_rs_s965` (machine) | hold15 | 15 | 1.000 | 0.867 | 0.733 | 0.733 | 0.867 | 0.200 | 0.133 | 0.067 |
| `dDPfull_first_rs_s965` (machine) | rnd30  | 30 | 0.733 | 0.767 | 0.567 | 0.567 | 0.700 | 0.067 | 0.067 | 0.233 |
| `dDPfull_first_rs_s966` (machine) | hold15 | 15 | 0.867 | 0.733 | 0.733 | 0.733 | 0.733 | 0.200 | 0.200 | 0.067 |
| `dDPfull_first_rs_s966` (machine) | rnd30  | 30 | 0.500 | 0.433 | 0.367 | 0.367 | 0.367 | 0.167 | 0.167 | 0.233 |

`nested_v2` and `nested_honest` read IDENTICAL on every one of the 8 cells above (not a scripting
artefact — they are two independently-computed predicates, one from the live tracker's own
"at rest" clause, one from a literal 100-step settle at episode end; they simply agree everywhere
in this batch, consistent with §(z).6 item 3's observation that `nested_v2` is the STRICTER of the
two, so an episode it grants should nearly always also pass the softer settle check).

**Seed s945 (human) never once reaches `nested_v2`** in 45 episodes (hold15 + rnd30) — every
route-split number for the {r2dreamer} human arm below therefore comes entirely from **s946**.
This is a real difference between the two human seeds' policies AT THE STEP EACH WAS COPIED
(s945 at step 2,902,295; s946 at step 1,819,013 — s946 had trained for FEWER steps and already
performs better on this measure, so this is not simply "further along the same curve"), not an
artefact of this census.

### {RLPD}

| run (seed) | set | n | picked | placed_v2 | nested_v2 | nested_honest | farside | slide_event | home | tipped |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `e2e_rlpd_dH_s945` (human) | hold15 | 15 | 0.267 | 0.067 | 0.000 | 0.000 | 0.067 | 0.000 | 0.000 | 0.533 |
| `e2e_rlpd_dH_s945` (human) | rnd30  | 30 | 0.267 | 0.233 | 0.000 | 0.000 | 0.200 | 0.000 | 0.000 | 0.333 |
| `e2e_rlpd_dDPfirst_s965` (machine) | hold15 | 15 | 0.733 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.733 |
| `e2e_rlpd_dDPfirst_s965` (machine) | rnd30  | 30 | 0.267 | 0.067 | 0.000 | 0.000 | 0.033 | 0.000 | 0.000 | 0.500 |

**`nested_v2` is 0/90 across every {RLPD} sparse-pilot cell scored** (both arms, both sets). These
FINAL (250k-decision, fully trained) checkpoints pick and even reach `placed_v2` on some starts,
but never arrive at settled contact in this evaluation. The task brief's characterization of "a
large fraction of recent [training] episodes" reaching `nested_v2` was stated only for the four
{r2dreamer} runs (verified against the brief's own numbers, §0); it was never claimed for {RLPD},
and this census does not manufacture that claim — the {RLPD} sparse pilot's route question is
**not answerable from this data: zero observations, not a zero rate**.

### Route split among `nested_v2` episodes

| learner | arm | seed(s) | n(nested_v2) | n(home, slide) | n(drop) | slide share |
|---|---|---|---:|---:|---:|---:|
| {r2dreamer} | human | 945+946 (all from 946) | 11 | 5 | 6 | 45.5 % |
| {r2dreamer} | machine | 965+966 | 50 | 12 | 38 | 24.0 % |
| {r2dreamer} | **pooled** | all 4 | **61** | **17** | **44** | **27.9 %** |
| {RLPD} | human | 945 | 0 | — | — | undefined (no `nested_v2` observed) |
| {RLPD} | machine | 965 | 0 | — | — | undefined (no `nested_v2` observed) |

Per-seed machine detail (the two seeds diverge): `s965` 4/28 home (14.3 %), `s966` 8/22 home
(36.4 %) — both individually majority-drop, so the pooled 24.0 % is not an artefact of averaging
a majority-slide seed against a majority-drop one.

---

## 3. Reading against P8

**P8 (PHASE_PLAN (z).8): under both ladders, the majority of `nested_v2` events are drops;
disconfirm = slide route ≥ 50 % in any arm.**

* **{r2dreamer} machine arm: CONFIRMED, clearly.** 76.0 % drop (38/50) pooled, and both
  individual seeds independently majority-drop (85.7 % and 63.6 % respectively). Well clear of
  the 50 % disconfirm line.
* **{r2dreamer} human arm: CONFIRMED, but at the edge of what n=11 can support.** 54.5 % drop
  (6/11) — technically on the "majority drop" side of P8, but the counts are 6 vs 5 on a SINGLE
  seed's data (s946; s945 contributed zero observations), so one flipped episode would cross the
  50 % line. This arm should be read as "not distinguishable from 50/50 at this sample size,"
  not as a confident confirmation, and is the one result in this census that a larger n could
  plausibly overturn.
* **{RLPD}: NOT EVALUABLE.** Zero `nested_v2` episodes in 90 scored episodes across both arms —
  P8 makes a claim conditional on `nested_v2` occurring, and it did not occur here, so this
  census neither confirms nor disconfirms P8 for {RLPD}'s sparse pilot. (Whether {RLPD}'s
  TRAINING rollouts reach `nested_v2` at all is a separate, unanswered question outside this
  census's scope — see §2's caveat that the "large fraction of recent episodes" claim in the
  task brief was stated only for {r2dreamer}.)
* **Pooled across every episode where a route could be determined (all from {r2dreamer}): 72.1 %
  drop (44/61), comfortably confirming P8's headline claim** that paying the outcome alone
  (rather than the behaviour) does not, on the whole, induce the slide.

**Bottom line for the next decision this feeds:** the (z)/(aa) motivation — that a sparse
`nested_v2`-only reward is answered mostly by dropping the can at the goal rather than sliding it
home — reproduces on POLICY rollouts, not just on the demonstrations, for the {r2dreamer} machine
arm and (more weakly, single-seed) the human arm. It does not reproduce for {RLPD} because {RLPD}
under this reward and budget does not reach `nested_v2` at all, which is itself informative for
anyone deciding whether the sparse ladder is viable for that learner, independent of route.

---

## 4. Caveats

* The four {r2dreamer} checkpoints are **mid-training snapshots**, not the runs' final policies —
  this census describes what each run's policy does RIGHT NOW, at the step recorded in
  `paper/ROUTE_CENSUS_LOG_2026-09-11.md` §1, not a claim about the 4M-step outcome.
* `home` is never right-censored for the `sparse` ladder's own terminal (`nested_v2`) — see LOG §0
  — so every scored episode's route is a determinate slide/drop, not "unknown". This does NOT
  extend to any STAGED-ladder cell (Lane 11 found 28/600 such rollouts right-censored); none of
  those are reused here.
* n is small (15/30 per cell); a cell with zero or one `nested_v2` episode cannot support a
  percentage claim, and is reported as a raw count, not a rate, in the interpretation.
* These are `preview`-role, shared-process cells (fast, in line with this lane's "no more than 8
  GPU jobs at once" budget), not the isolated/pinned-hardware `record`-role protocol PHASE_RESULTS
  §5.1 uses; adequate for "does the route exist at all", not a number for the paper's tables.
* {RLPD}'s `--require-cores 64` guard refused 3 of 4 first-attempt cells outright (LOG §5) rather
  than silently scoring on the wrong machine size; all reported {RLPD} cells ran on a confirmed
  64-physical-core node (`pax015` or `pax027`).
