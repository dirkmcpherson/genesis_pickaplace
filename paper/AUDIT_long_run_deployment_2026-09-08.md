# Independent audit: long-run deployment plan

Date: 2026-09-08. Scope: the deployment plan, the episode-record launcher,
preparer and README, the existing registry implementation, and the revised
longer-runs proposal. No network access, code changes or job actions performed.

**Verdict: safe to proceed to local preparation after the user's authorization.**
The plan correctly separates preparation from submission and protects existing
running/queued jobs by using fresh releases and output paths. It does not claim
the episode-record smoke proves learning stability. The unresolved scientific
choices are appropriately explicit. This is not a launch-ready verdict: close
the gates below before submitting long runs. No finding requires delaying
read-only inventory or isolated candidate construction.

## Concrete launch blockers to resolve during preparation

1. **The registry has no supported atomic reservation/concurrency mechanism.**
   Plan lines 111–120 require reservation before submission and refer to an
   existing supported concurrency mechanism. In `cluster/run_registry.py`,
   `cmd_check` (line 158) reads independently, while `cmd_register` (line 200)
   appends without a lock or duplicate recheck (line 213). Registration reads
   `SLURM_JOB_ID` from the environment (line 207), so a submitting process cannot
   assume that field contains the returned job ID. The dataset fingerprint
   (line 57) hashes names and sizes, so same-size content changes are invisible;
   dirty source is also absent from the git-only identity. Replace the plan's
   assumption with a concrete, backward-compatible reservation and submission
   record protocol: serialize check/reserve for this deployment, persist the
   returned Slurm ID explicitly, and include candidate/content identity. Explain
   coordination with any other submitters before claiming global duplicate
   prevention. Reuse historical records without rewriting them. A small
   serialized submitter is sufficient; a new registry service is unnecessary.

2. **The existing bundle checks do not meet the planned isolation contract.**
   `prepare.py:113` links GP inputs to the live source tree; its manifest at
   line 147 covers R2 files and selected GP Python, but omits launch scripts and
   linked input contents. `launch.sbatch:15` uses a shared interpreter and line
   21 writes compilation caches inside the bundle. Lines 28–36 implement
   integrity and metadata gates with Python `assert`, which optimization can
   disable. These are known candidate changes, not reasons to reject the plan:
   implement its full manifest/input verification, external caches and module
   origin checks; use explicit fatal checks that remain enabled under the
   intended environment. At launch, reject unexpected executable files and
   modified linked inputs as well as missing/mismatched manifest entries.
   Record the residual mutability of shared dependencies honestly; a one-time
   hash check cannot guarantee they remain fixed throughout a queued/active job.

3. **Checkpoint preservation must not imply scientifically valid resumption.**
   The plan addresses milestone boundaries and asks for a restart policy
   (lines 66–68, 101), but should state the default concretely. The proposal
   reports checkpoints lack replay state (`PROPOSAL_longer_runs_2026-09-08.md:93`),
   and the launcher disables requeue (`launch.sbatch:12`). Keep interruption as
   a visible incomplete run by default. Any authorized restart must specify
   whether it is a new attempt from scratch or a demonstrated complete-state
   continuation; loading weights/optimizers into empty replay is not the latter.
   Do not add full replay/RNG/environment recovery merely to satisfy a generic
   restart requirement if the registered policy is to report interruption.

4. **Register the final design and scope of inference before launch.**
   Plan section 2 correctly leaves arm choice unresolved. Choosing
   `dDPfull_first` changes the dataset comparison relative to the historical
   best-of-three arm; it removes that selection rule but does not by itself
   establish a pure human-versus-machine source effect. Record the exact
   construction and label the comparison accordingly. The duration claim is
   the within-run 2M-to-4M contrast under the chosen recipe, not an unqualified
   comparison to old cells. Specify the primary endpoint/contrast and paired
   checkpoint analysis, with multiplicity treatment for secondary stages or
   contrasts; n=4 remains a pilot, including for BF10 interpretation. Name the
   registration destination (`paper/PHASE_PLAN_2026-09-04.md`, identified in the
   proposal at line 45) or explicitly reconcile any replacement. Metric
   definitions must be accepted for the endpoints actually claimed; unresolved
   accepted Slide need not block a pilot limited to already-defined outcomes.

## Useful improvements, not preparation blockers

- **Snapshot consistency:** plan lines 71–76 capture dirty provenance well, but
  record that the source was stable while copying. The current preparer copies
  R2 and then hashes original patched files (`prepare.py:140–141`), so another
  agent's simultaneous edit could produce misleading baseline provenance.
  Verify relevant source contents before/after capture and rebuild only the
  new candidate on drift; never restore the shared tree.
- **Hardware and resource balance:** evaluation CPU class is specified (plan
  line 51), while the launcher allows several GPU types. Record training
  hardware and allocate arms/seeds without systematic arm-to-hardware or
  scheduling confounding. Include evaluator overlap and shared filesystem
  load in capacity planning. Absolute isolation of resource contention is not
  required for safe code preparation.
- **Keep implementation small:** one release manifest plus the existing
  registry format and checkpoint/evaluation sidecars can satisfy this plan.
  Avoid a second provenance database, containerization requirement, full
  checkpoint recovery subsystem or unattended monitoring service unless an
  identified requirement needs it. The plan already appropriately separates
  long-lived monitoring from conversational follow-up.

No upload, deployment or submission is authorized by this audit. Approval of
the concrete design and exact submission scope remains the plan's final gate.
