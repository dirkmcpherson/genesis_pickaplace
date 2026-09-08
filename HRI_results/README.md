# HRI_results

Regenerated tables, a forest plot and a methodology note for the human-vs-machine-demonstration
comparison. **Nothing here is hand-written.** Every number is produced from evaluation artefacts
on the cluster by `make_tables.py`; if a number looks wrong, fix the source or the registry and
rerun, do not edit the output.

## Regenerating

```bash
V=~/workspace/genesis_sim2real/venv/bin/python

$V make_tables.py               # rebuild from the committed seed_counts_raw.csv
$V make_tables.py --refresh     # re-pull per-seed counts from the cluster first (ssh pax)
$V make_tables.py --prior-sweep # print the full prior-sensitivity table
$V hri_stats.py                 # self-test the statistics against known published cells
```

`--refresh` ships `harvest_cluster.py` to the login node, runs it there and pulls the tidy CSV
back. The link drops intermittently; a failed refresh leaves the existing CSV in place, the build
continues from it, and the staleness is printed into `results.md` rather than hidden.

## Files

| file | what it is |
|---|---|
| `make_tables.py` | the generator: selection, statistics, rendering, figure, methodology |
| `harvest_cluster.py` | runs **on the cluster**; walks every evaluation tree and emits one tidy CSV |
| `cells.py` | the registry of comparisons - which cell is of record, and why it is provisional |
| `hri_stats.py` | permutation test, MDE/CI, hierarchical Bayesian equivalence posterior |
| `doc_checks.py` | published numbers, so every regenerated cell is checked against the prose |
| `methodology.py` | the machine-readable recipe spec that `METHODOLOGY.md` is rendered from |
| `seed_counts_raw.csv` | every per-seed count the harvester found (the raw data layer) |
| `seed_counts.csv` | just the per-seed counts used by the table, with provenance paths |
| `results.csv` / `results.md` | the results table |
| `fig_effects.png` / `.pdf` | forest plot |
| `METHODOLOGY.md` | how each learner was made to work |

**Adding a result:** add an entry to `cells.py`. Do not edit tables.

## What each column means

| column | meaning |
|---|---|
| `n` | seeds per arm x episodes per seed. **The per-seed count is the unit of analysis**, never the pooled episode. |
| `human` / `machine` | success rate, pooled over seeds, for display only |
| `Delta` | human - machine, on the success-rate scale |
| `95% CI` | Welch-style t interval on Delta using per-seed rates |
| `p` | **exact two-sided permutation test on per-seed counts** - the project's registered statistic, unchanged |
| `MDE` | minimum detectable effect at 80 % power, from the observed per-seed spread |
| `P(ROPE)` | posterior probability that \|Delta\| < 0.10 - the registered equivalence margin |
| `BF01` | interval Bayes factor for \|Delta\| < 0.10 against \|Delta\| >= 0.10 |
| `prior` | `stable` if the equivalence verdict survives all three priors and a separate-sigma refit |
| `verdict` | `equivalent at +/-0.10` needs P(ROPE) >= 0.90; `INCONCLUSIVE (underpowered)` when MDE > 0.20 |

## Why there is a Bayesian column at all

Almost every comparison here is a null, and the registered predictions are **equivalence**
claims: `|Delta| < 0.10`. A p-value cannot support one. `p = 0.875` says only that we failed to
detect a difference; it is equally consistent with the arms being identical and with the study
being too small to notice a large gap. The two are distinguished by the **MDE** (frequentist) and
by **P(Delta in ROPE)** (Bayesian), which is a direct statement about the quantity the
registration actually names. Reporting these nulls with a p-value alone would overstate all of
them.

`BF01` answers the companion question - how much the data moved belief toward the interval null
relative to the prior - and the `prior` column exists because an equivalence claim that flips
under a reasonable prior change is not a finding. Any such flip is printed, not smoothed over.

## The model, and why not a pooled beta-binomial

Seed-to-seed spread dominates the variance here: inside a single arm the picked count ranges
8-19 of 30, and several arms carry a dead seed. Pooling seeds into one beta-binomial would treat
240 episodes as 240 independent draws and understate the standard error roughly 2.5-fold on the
RLPD pick cell alone - manufacturing exactly the false confidence an equivalence claim must
avoid. So the model puts a random effect on the seed:

```
k_aj ~ Binomial(n_aj, p_aj)          seed j of arm a
logit(p_aj) = mu_a + sigma * eps_aj,  eps_aj ~ N(0,1)
```

with `sigma` shared across the two arms (primary) and a separate-sigma refit as a sensitivity
check. Priors: `mu ~ N(0, 1.5^2)` and `sigma ~ HalfNormal(1)` (primary), plus `wide` (3.0, 2.0)
and `tight` (1.0, 0.5). The estimand is the difference of **population-averaged** rates, with the
seed effect integrated out, because the +/-0.10 margin is a claim about rates. The posterior is
computed by deterministic grid quadrature, so it is reproducible bit-for-bit and has no
convergence to diagnose. Assumptions are stated plainly at the top of `hri_stats.py`.

## Rules this directory enforces

- **A missing number stays missing.** A cell with no data renders as `EMPTY` with the reason.
  Nothing is estimated, interpolated or copied from prose.
- **Cross-learner comparability is checked, not assumed.** A row may only be read as one table if
  every cell in it shares an entry-bank version and evaluation protocol. **An absent bank stamp
  counts as unknown, therefore not comparable** - several cells predate stamping. When the rule
  fires, the renderer refuses to put the learners side by side and prints the stamps instead.
- **Floors get no p-value.** Where both arms sit near zero (`slide_success`; r2dreamer on
  robomimic Can) a null is an artefact of the floor, so no test and no ROPE are computed.
- **Doc disagreements are findings.** Regenerated numbers are checked against the documents of
  record and every mismatch is printed in `results.md`.
- **Re-scores are checked for reproducibility.** Any statistic whose predicate did not change
  between a cell and its `_cp` re-score, yet whose per-seed count moved, is reported.

## Known caveats carried into the table

- The **robomimic source claim is withdrawn** by its own registered quantity control: RLPD reads
  MG200s 0.147, MG718s 0.475 (indistinguishable from human) and MGall 0.610 (above human). The
  DP and BC-RNN separations rest on the same 200-tape draw and are labelled accordingly.
- **"Deterministic" is wrong for the world models.** It is accurate only for RLPD. See the last
  section of `METHODOLOGY.md`.
- **Three contact predicates, three meanings.** `slide_success` is the task outcome (no arm
  learns it), `contact_push` is the discriminating statistic, bare `contact` is legacy and
  overstates capability 1.5-3x. All three are shown where they exist.
- **Hardware.** Instruction set and core count are perfectly collinear on this cluster, so the
  mechanism is not identifiable. Affected rows are marked provisional and the pinned-seed and
  as-published versions are shown side by side.
