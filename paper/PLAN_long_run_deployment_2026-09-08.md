# Plan: auditable, isolated long-run deployment

Status: planning and independent audit only. This document authorizes no upload,
code patch, job submission, cancellation, or modification of existing results.
User requested a written plan followed by a minimal-context subagent audit.

## Objective and boundaries

Prepare a reproducible R2Dreamer full-task long-run experiment without changing
the code or inputs used by existing running or queued jobs. Deployment isolation
must not create an undocumented experimental fork. Preserve the frozen w3 world,
existing staged reward, datasets and cells of record. Never update a shared
checkout, overwrite an existing bundle, rsync with deletion, or redirect a shared
symlink. Shared cluster resources can still affect queue waiting time.

The previously tested `episode_record_2026-09-08/bundle_v2` is logging
infrastructure, not a complete long-run deployment. Its real CPU smoke used a
scripted controller without optimization. It established timeout logging and
trajectory parity for that fixture, not GPU learning stability.

## 1. Inspect and reuse existing provenance

Read the OG launchers, `cluster/run_registry.py`, registry examples, cluster R2
launcher and actual run sidecars. Inspect current queue commands and dependency
paths read-only. Document which provenance fields already exist and reuse that
format; add only missing fields, with a schema version if needed. Do not create
a competing registry or rewrite historical entries. Do not infer a live run's
code version from the current checkout alone.

Deliverable: field mapping and inventory of original paths that must remain
unchanged. Any remote inspection in the later execution phase is read-only.

## 2. Make the experimental contract explicit before submission

Proposed primary design from the revised longer-runs proposal: fresh R2 full-task
runs, n = 4 training seeds per arm, existing staged reward, 2M and 4M simulator-step
checkpoints. Recommend human `dHfull_all` versus machine `dDPfull_first` to avoid
best-of-three selection; this recommendation is not yet an accepted arm choice.
Pick reruns, RLPD extensions and sparse-reward training are outside this plan.

Resolve and record:

- Exact dataset versions, fresh seed IDs and whether seeds are paired for analysis.
- Budget semantics: inspect whether the existing trainer includes demonstration
  prefill in its step counter. Store both original counter and actual online
  simulator steps; define checkpoint milestones consistently with the comparison.
- Replay policy: retaining the current 500k-row FIFO permits demo eviction during
  a 4M run. Increasing capacity or reinjection changes the intervention. Quantify
  eviction timing and memory cost, then explicitly choose and disclose the policy;
  do not silently enlarge replay to call the experiment duration-only.
- Fixed evaluation banks, isolated evaluation process, CPU class, action-selection
  protocol and checkpoint evaluation schedule. Evaluation must not advance the
  training environment or consume its RNG. Preserve both 2M and 4M artifacts.
- Pilot feasibility/expansion criteria independent of effect direction or p-value;
  outcome definitions and early-learning threshold/persistence/censoring rules.
  Report actual n, seed-level frequentist tests and BF10 with named null and prior.
- `FULLENV_EPISODE_RECORD=1`, `R2D_EOE=0` on both arms. Label nested as proxy and
  slide_success as diagnostic. Accepted Slide remains a separate validation issue.

Deliverable: concrete run matrix, resource estimate and unresolved design choices
for the user's final review. Existing DP/RLPD completion is not inherently a gate.

## 3. Prepare the complete change locally

After plan/design authorization, add only required checkpoint preservation and
provenance wiring to a new candidate. Preserve milestone checkpoints atomically,
including true step metadata; account for prefill offsets, vector-step increments,
last-budget completion, periodic-save drift and restart behavior. Do not rely on
polling and copying an overwritten `latest.pt` at approximately 2M.

Capture base commit IDs plus exact tracked diffs and relevant untracked source
files for both R2 and GP. Include a source archive where commit/diff alone cannot
reconstruct the tree. Record the deployment patch separately from baseline dirty
state. Preserve other agents' work without commit, stash, revert or cleanup.

Manifest covers launcher, Python, configuration, patch, dependencies/interpreter,
dataset content hashes, banks and relevant assets. Source hashes alone do not
freeze linked inputs or a shared virtualenv. Resolve symlinks and dynamic import
paths; copy small runtime configuration and code rather than link to mutable
shared trees. Verify shared large inputs by content and record their locations.

Keep caches, bytecode and runtime outputs outside the immutable code bundle.
Do not claim complete dependency isolation if shared dependencies remain mutable;
record and verify their versions/content before execution.

## 4. Validate the final candidate

Run focused tests for episode records, prefill/replay schema, milestone saving,
counter semantics and provenance. Validate resolved configuration and actual
module origins under the intended interpreter and environment, including inherited
PYTHONPATH and environment overrides. Fail closed on unexpected code/config paths.

Use a separately authorized, bounded diagnostic with the actual learner and a
small number of optimization steps to check replay sampling, parameter updates,
checkpoint loadability and absence of log fields in model inputs/targets. Keep
diagnostic outputs separate from scientific results. Reuse prior smoke evidence
only when affected code hashes match; repeat relevant parity checks if changed.

Estimate aggregate disk and RAM needs for all jobs/checkpoints/logs and margin;
the existing 10 GiB single-launch guard is not an aggregate capacity plan. Check
wall-time feasibility and specify failure/restart policy before long submission.

## 5. Stage a new immutable release, then review exact submissions

Following authorization to execute this plan, upload only to a fresh staging
directory, verify the manifest remotely, and publish to a never-before-used
release path. No transfer targets an existing code tree or bundle. Verify the
original code/config path inventory remains unchanged and check it for queued-job
dependencies; report concurrent external drift instead of restoring shared files.

New launchers use absolute release paths and unique output roots. Reserve run IDs
before submission and reject duplicates by experiment/code/dataset/seed identity,
not job name alone. Prepare exact sbatch commands, run matrix, dependency graph,
estimated resources and code/config diffs for review. No long jobs are submitted
until the user has authorized the concrete design and submission scope.

## 6. Submit and track without silent retries

Submit only the approved matrix. Record Slurm IDs and submission timestamps
immediately in the existing registry using its supported concurrency mechanism.
Handle an ambiguous submission response by reconciling queue/accounting before
retrying. New jobs verify manifest/input hashes before initialization and write
resolved config, code version, actual host/hardware and checkpoint lineage.

No existing jobs are cancelled, reprioritized or edited. Stop further submissions
on a failed preflight; report already submitted jobs rather than silently cancelling.
Do not replace failed seeds or alter settings after reading outcomes.

## 7. Monitor, evaluate and report with lineage

During active monitoring, reconcile queue/accounting with logs and checkpoint
metadata; distinguish completion, timeout, OOM, invalid metrics and scientific
non-learning. Persistent monitoring requires a separately specified process;
conversation availability is not an unattended service guarantee.

Each evaluation records training run ID, exact checkpoint hash/step, evaluator
version, bank hash and hardware/protocol. Write new cells only. Tables and plots
inherit these IDs; incompatible code/metric versions are not silently pooled.
Failures and missing seeds remain visible. Report n as training seeds, not episodes;
do not compute scientific significance from instrumentation smoke tests.

## Audit request

A fresh subagent should identify operational risks, scientific confounds,
provenance gaps, missing authorization boundaries and unnecessary complexity.
Classify findings as launch blockers or improvements, cite concrete plan/code
evidence, and state what must change. Audit is read-only except for its own report.

## Independent audit disposition

The minimal-context reviewer returned
[AUDIT_long_run_deployment_2026-09-08.md](AUDIT_long_run_deployment_2026-09-08.md).
Verdict: safe for authorized local preparation, not ready for submission. Accept
all four launch blockers. The following clarifies and supersedes assumptions above:

1. The existing registry has **no atomic reservation mechanism**. Reuse its record
   conventions with a small serialized submitter for this deployment, an exclusive
   reservation keyed by candidate/dataset/seed/configuration, and explicit storage
   of the returned Slurm ID. Coordinate other submitters before claiming global
   duplicate protection; historical writers do not honor a newly added lock.
   Do not modify shared registry code used by queued jobs. Reconcile ambiguous
   submissions against queue/accounting rather than retrying automatically.
2. Integrity gates must use explicit fatal checks, not disableable Python asserts.
   Include launchers/configuration and linked input contents; reject unexpected
   executable files as well as missing/modified ones. Verify source hashes before
   and after capture to detect concurrent edits. Externalize runtime caches and
   document residual shared-dependency mutability. Hash checks do not prevent
   later mutation of shared dependencies.
3. Default interruption policy: retain and report an incomplete run, with no
   automatic requeue or restart. Weights/optimizer loading without replay/RNG/
   environment state is not a continuous training trajectory. Any later authorized
   attempt must be separately identified. Full recovery infrastructure is not a
   prerequisite under this policy.
4. Register the accepted design in `paper/PHASE_PLAN_2026-09-04.md` before launch.
   State the primary endpoint and within-run paired 2M-to-4M contrast, secondary
   comparisons/multiplicity policy and n=4 pilot limitations. Machine-first versus
   human all-attempts is a dataset-construction comparison with residual differences,
   not proof of an isolated source effect. Balance training hardware allocation
   across arms and record it, as well as pinning the evaluation protocol.

Implementation should remain one release manifest, the existing registry format
with explicit submission bookkeeping, and checkpoint/evaluation sidecars. No new
registry service, mandatory container system or recovery framework is proposed.
This revision remains a plan only; no upload or job action occurred in this review.
