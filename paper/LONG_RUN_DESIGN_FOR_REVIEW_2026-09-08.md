# Long-run pilot: concrete proposal for final review

Status: **review only; no long jobs submitted and no accepted PHASE_PLAN amendment
added**. Prepared under the user's authorization to execute the audited deployment
plan. The plan reserves submission for final agreement on the concrete experiment.

## Proposed matrix

| Arm | Dataset | Training seeds | n |
|---|---|---|---:|
| Human all-attempts | frozen copy of `dHfull_all` | 202609080–202609083 | 4 |
| Machine first-attempt | frozen copy of `dDPfull_first` | 202609090–202609093 | 4 |

Fresh initialization, no restart, same frozen w3 world and staged reward. Keep
the existing 500,000-row FIFO, no reinjection, no duplication beyond the existing
stream packing. Use existing full-state R2 model, six training environments,
action repeat four, bounded-normal actor, entropy 3e-5, return clamp eight,
reward scale one, and actor BC zero. In-job evaluation stays disabled.

Request **4M online simulator steps**, saving checkpoints at the first completed
vector step at or above **2M and 4M online steps**. Record actual step and overshoot.
This is explicitly different from the old budget label: old counters included
prefill. Existing runs and their labels are not retroactively changed.

Human prefill packs 29,295 raw rows into 29,406 rows across six streams; machine-first
packs 36,906 into 37,488. The legacy-counter origins are therefore 117,624 and
149,952 simulator steps. Runtime records remain authoritative. Reset rows also
consume FIFO space without advancing the simulator-step counter: demo eviction
cannot be assigned an exact online step from capacity alone. With no reinjection,
all initial demos will be evicted by at most roughly 2M online steps; replacement
starts earlier. This is the preserved replay recipe, not a demo-retention intervention.

## Proposed endpoints and analysis registration

Primary duration endpoint: within-seed change in rnd30 MODE `nested_honest` from
2M to 4M, separately for each arm. Use paired checkpoint comparisons, not independent
episode tests; adjust the two primary arm tests with Holm. A paired sign-flip test
at n=4 has minimum two-sided p=0.125, so this is a feasibility/estimation pilot,
not a design capable of conventional significance on that test. Specify BF10 for
a paired standardized-effect model with a symmetric Cauchy(0, 0.707) alternative
and a point null at zero; report prior sensitivity and the small-n limitation.
No statistical values have been computed from the diagnostic runs.

Report Pick, `placed_v2`, contact and nested proxy as secondary endpoints, with
the secondary family and multiplicity treatment fixed before submission. Machine-
versus-human contrasts use the independent seed groups above. This comparison
removes best-of-three selection but does not isolate every aspect of data source.
Do not pool it with the historical machine-best comparison.

Proposed early-learning definition: a 40-bin fixed grid over 4M online steps;
rate at least 0.2 for three consecutive observed bins, onset at the first bin.
Missing bins do not satisfy persistence. Non-crossers are right-censored, not
assigned a successful rise time at the budget. Report interval resolution and
actual n. **The curve reader's existing seed-summary first-crossing statistic
remains exploratory; it is not this proposed persistence analysis.**

No automatic expansion. Review feasibility after all pilot attempts finish;
report failures and one-arm non-learning. Any expansion must be registered without
using effect direction or significance as its trigger. Accepted Slide remains
unvalidated and is not an endpoint of this pilot; the logged grant is diagnostic.

## Evaluation and resources

Use frozen rnd30 starts, 1200 simulator steps, MODE primary, one fresh process per
initial condition. The prepared evaluator requires an exact CPU model and physical
core count and records affinity, logical CPUs and four Torch threads. **Select the
actual hardware target and training allocation before launch.** A100-only training
narrows GPU variation but does not by itself match training CPU geometry. Prefer
balanced/interleaved arm allocation and check the actual assigned hardware.

Proposed per-job request: one A100, eight CPUs, 48 GiB RAM, 48 hours, no requeue.
Recent existing R2 logs show roughly 67–95 simulator steps/s; extrapolation gives
about 12–17 hours for 4M online steps before startup/variability (not a runtime
guarantee). Existing completed training peaked near 27 GiB RAM. The gradient
diagnostic measured 34,992 replay bytes/row, about 16.3 GiB at 500k rows.

Existing 2M output directories were about 343 MiB each. Milestone files in the
diagnostic were about 113 MiB each. Reserve at least 50 GiB free for the aggregate
pilot and evaluation margin, monitor growth, and never delete other runs to make
space. More than 380 GiB was free at the initial inventory. Long jobs may affect
queue waiting time; they must not alter existing jobs or their code/input paths.

Before final approval: accept the dataset/replay/online-budget contract, complete
endpoint and hardware registration in `paper/PHASE_PLAN_2026-09-04.md`, coordinate
other submitters, and approve the exact matrix. Create approved submission copies
alongside the review artifacts; do not modify the frozen release.
