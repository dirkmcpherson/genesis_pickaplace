# Approved R2 long-run pilot — 2026-09-08

Authorization: user “do it”, followed by approval (“sounds good”) of four-node
parallel allocation, one human run and one machine run per node on separate GPUs.
This replaces the proposed single-A100-node restriction; no long runs had been
submitted under that earlier proposal.

## Fixed training contract

Human dHfull_all versus machine dDPfull_first, **n=4 training seeds per arm**.
Four-million ONLINE simulator steps; immutable checkpoints at the first completed
vector step at/above 2M and 4M, with actual step/overshoot and prefill origin recorded.
Same frozen w3 world and staged reward, 500k-row FIFO, no demo reinjection, default
stream packing, no warm restart or automatic requeue. Do not relabel historical
prefill-inclusive counters as online steps.

Release: long_run_candidate_2026-09-08/release_v4. SHA256:
8a91f0bd5f9fdcf9fbba8b226c497220160f1ba5c043d65d35139fcb5e977efa.
Python, configs, copied data and runtime GP inputs stay read-only and unchanged.
Only approved specifications and submission records are new. Scheduler environment
options select the hardware without editing the frozen launch script.

| Node block | Human seed | Machine seed | GPU per job |
|---|---:|---:|---|
| pax021 | 202609080 | 202609090 | L40S |
| pax022 | 202609081 | 202609091 | L40S |
| pax023 | 202609082 | 202609092 | L40S |
| pax024 | 202609083 | 202609093 | L40S |

All four nodes were idle at allocation inspection, advertise 64 CPUs and L40S 48GB
GPUs. Those scheduler labels are not a proof of CPU equivalence: actual hardware
and affinity are recorded by each job. One H/M pair per node controls host at the
comparison level. Each job requests one GPU, eight CPUs, 48 GiB RAM, 48 hours.
The successful short gradient diagnostic used A100; L40S is chosen for available
parallel capacity, not from a demonstrated speed benchmark. Verify actual startup.

No running/queued existing job, path, priority or shared source is changed.
There is no automatic replacement of failed seeds or expansion based on outcomes.

## Analysis fixed before submission

Primary: paired within-seed 2M-to-4M rnd30 MODE nested_honest change, separately
by arm; exact two-sided paired sign-flip tests, Holm over the two primary tests.
BF10: point null zero versus paired standardized effect Cauchy(0,0.707), with
prior sensitivity. n=4 is a feasibility/estimation pilot; minimum paired two-sided
sign-flip p is 0.125. No equivalence or significance promise.

Secondary within-arm checkpoint outcomes: picked, placed_v2, contact, nested_proxy;
Holm across the eight arm-by-outcome contrasts. Exploratory H/M final-checkpoint
contrasts are BLOCKED BY NODE (one H/M pair per node), with within-block label
swaps, and Holm over the five accepted outcomes. This replaces the review draft's
unblocked independent-seed permutation proposal to match the approved allocation.
Report every attempt; incomplete runs and missing evaluations are not silently
replaced or imputed as completed negative outcomes.

Early learning: fixed 40-bin grid over 4M ONLINE steps, rate >=0.2 for three
consecutive observed bins, onset in first bin. Missing bins fail persistence;
non-crossers are right-censored. The CSV helper's exploratory first-crossing
summary is not this persistence analysis and must not be substituted for it.

Evaluation: new rnd30 MODE cells, one fresh process per IC, 1200 simulator-step
horizon and four Torch threads. Use one fixed evaluation host (pax021) for both
arms/checkpoints; freeze its actual CPU model and physical-core count from the
first runtime hardware manifest, not Slurm feature labels. Record affinity.
Evaluation is deferred to separate jobs and cannot advance a training environment.
No evaluation jobs are submitted by this training launch operation.

Do not pool with historical shared-process or machine-best cells. Machine-first
removes best-of-three selection but leaves other dataset-construction differences.
Accepted Slide remains unvalidated; slide_success stays diagnostic, and nested
proxy remains distinct from settled nested_honest.


## Post-submission deviation — 2026-09-08 22:38 EDT

Preserving the registration above: actual Slurm records show that the submitted
`SBATCH_NODELIST` environment setting did not constrain nodes. All eight jobs do
have `Features=l40s`, but `ReqNodeList=(null)`. The first two H/M pairs landed on
pax020 and pax021; the remaining four jobs are pending for QOSMaxGRESPerUser.
The intended four-node allocation and node-block interpretation are therefore
not established. Do not silently relabel intended blocks as observed hosts or
apply the blocked between-arm analysis without auditing all actual allocations.
Primary within-seed checkpoint comparisons remain defined. No run or queued job
was altered after discovering this deviation. See the OG launch update and
`long_run_submission_2026-09-08/verification.json` for job-level evidence.

## 2026-09-09 — machine-arm RLPD relaunch + a manifest edit, disclosed

The 8 `e2eL_rl_dM_*` runs (RLPD, machine arm, `dDPfull_first_rx`) failed at submission
(exit 1:0, jobs 3484620–27). Cause: `sbatch_rlpd_e2e.sh` asserts
`one_per_ic_first is True` for `ARM=dDPfirst`, and `to_dreamer_native.py` had stamped
`False`, because it derives that key from its own CLI flag and the selection had been
applied UPSTREAM (the relabelled set is a copy of the already-selected `dDPfull_first`).
The gate behaved correctly — it refused a set that could not attest its own provenance.

**Before** editing anything, the claim was checked rather than assumed: the 72 tape keys
match `dDPfull_first` one-for-one, and all 72 **action streams are byte-identical by
sha256**; only per-frame reward differs (70 tapes; totals 237.0 vs 131.0, the amendment (x)
ladder). On that basis `one_per_ic_first: True` was written into
`demos_state_full/dDPfull_first_rx/repeat.json`, together with `provenance_src_set` and a
`provenance_check` string recording the above. Prior file kept as
`repeat.json.bak_before_provenance`. Runs resubmitted as **3484659–67**, all alive.

Root cause fixed in `to_dreamer_native.py` (commit b756259): the converter now inherits the
source manifest's selection attestation when its own flag is unset and records
`selection_inherited_{from,keys}`, instead of stamping a false `False`.

Note this is the same failure mode the project has hit repeatedly — a field named for the
flag that sets it rather than the fact it asserts. The key reads as "was `--one-per-ic-first`
passed to *this* invocation", but every consumer treats it as "these tapes are the
first-attempt set".
