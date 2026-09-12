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

<!-- FILLED IN FROM route_census_summary.json / the per-cell metrics.json once all 12 cells land -->

### {r2dreamer}

| run | set | n | picked | placed_v2 | nested_v2 | nested_honest | farside | slide_event | home | tipped |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| TBD | | | | | | | | | | |

### {RLPD}

| run | set | n | picked | placed_v2 | nested_v2 | nested_honest | farside | slide_event | home | tipped |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| TBD | | | | | | | | | | |

### Route split among `nested_v2` episodes

| learner | arm | n(nested_v2) | n(home, slide) | n(nested_v2 ∧ ¬home, drop) | slide share |
|---|---|---:|---:|---:|---:|
| TBD | | | | | |

---

## 3. Reading against P8

<!-- state plainly: majority drop confirmed / disconfirmed, per arm, with the counts -->

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
