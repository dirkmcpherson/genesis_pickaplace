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

## 2026-09-09 — {RLPD} episode record was hollow; all 16 runs relaunched

Caught ~20 min into the batch by inspecting the records directly rather than
trusting the monitor's summary (the monitor read `ok`, because its check was
shaped around r2dreamer's output and RLPD's different shape passed vacuously).

**Defect.** `EpisodeRolloutLogCallback` point-read `info` at the done step.
`full_env` reports a stage in `info` at the step it is GRANTED, and SB3 auto-resets
on done — so the record captured *only the stage that ended the episode*. Measured on
`e2e_rlpd_dDPfirst_s920`: `tipped` (which terminates) fired in 4 of 17 episodes while
`picked` read 0.0 in all 17. The sticky `ep_*` twins were absent entirely, as were
`placed_v2` / `nested_honest` / `task_success`.

**Why it could not be repaired after the fact.** Reading the env's own `_granted` set
from a callback fails for the same reason — SB3 has already auto-reset by then. And no
per-episode return is logged for RLPD (no `Monitor`, no `monitor.csv`), so the
score-derived decoding that works elsewhere (ladder 1/1/2/4 ⇒ ≥1 picked, ≥2 placed,
≥4 contact, =8 slide) had nothing to decode. RLPD phase curves were unrecoverable.

**Fix** (`gp_e2e` commit `6e98ce3`): accumulate flags across every step of the episode
and emit sticky `episode/train_ep_<stage>` at done, keeping the legacy terminal read
beside it. Correct whether or not `info` is sticky. Logging-only, and confined to
`train_rlpd.py`, which r2dreamer does not import — its 16 running jobs were untouched.

**Verified so far:** all 8 `ep_*` keys now present in every record (they did not exist
before). **NOT yet verified:** that the values populate — an untrained policy has not
picked yet, so `ep_picked` and the terminal read are both 0 and do not yet discriminate.
The discriminating check is `ep_picked > picked` once picking begins.

Two guards fired correctly during the relaunch and are worth keeping: the demo-set
provenance gate (above), and a run registry that refuses a duplicate
`(script, arm, seed, git)` key — it refused 13 resubmissions because the fix was still
uncommitted, which is exactly the "these seeds are not independent" error it exists to
prevent. 16 relaunched under `6e98ce3`: 13 RUNNING, 3 PENDING on the 30-GPU cap, 0 refused.

### CORRECTION (2026-09-09, later) — the claimed {RLPD} defect is DISPROVEN for `picked`

The section above justified relaunching 16 runs with: "the record captured only the stage
that ended the episode; `tipped` fired in 4 of 17 episodes while `picked` read 0 in all 17."
**That inference was wrong.** The counts were real, but the explanation was not: `picked`
read 0 because an untrained policy had not picked yet — not because of a plumbing fault.

Measured on the relaunched runs once picking began (660 episodes, both arms), comparing the
new sticky flag against the legacy terminal read **per episode**:

    both=6   ep_only=0   term_only=0   neither=649

Zero disagreement in either direction. `info['picked']` is sticky at the done step, so the
original point-read was recording `picked` correctly. Aggregate equality alone would not have
settled this (offsetting errors); the per-episode split is what does.

**What the change did legitimately gain, and what it did not.** It did not fix a `picked`
defect — there was none. It did add the `ep_*` keys, which did not exist; it made `placed_v2`
and `nested` explicit 0/1 instead of silently absent (`inf.get` returned `None`, so they were
omitted from rows entirely); and it moved the sticky guarantee into the logger instead of
relying on env behaviour that happens to be sticky for `picked` and may not be for transient
predicates such as `contact_push` / `slide_success` — for which no evidence either way exists
yet. Those are real improvements, but they are completeness and robustness, not a bug fix.

**Cost of the error:** ~20 minutes of compute on 16 runs, relaunched at a few minutes old.
**Standing lesson:** an absent signal from a policy that cannot yet produce it is not evidence
of a broken recorder. The discriminating test (`ep` vs `term` per episode) was available before
the relaunch and would have taken one training-warm run to apply; I acted on the pattern instead
of the test. The earlier `contact_push=0.000` misdiagnosis this week was the same shape — reading
a zero as an absence rather than a measurement.

## 2026-09-10 (overnight) — one seed lost to a bus error; the health monitor was blind to it

**{r2dreamer} `e2eL_r2_dH_s6` (job 3484598, seed 906, HUMAN arm) died after 4h25m at ~1.1M of
4M steps: `Bus error (core dumped)` on pax141, MaxRSS 21 GB.** Infrastructure fault, not code or
data — the demo gate, reward gate and stage emission were all correct in its log. Left the arms
at 7 v 8. Logdir preserved as `full_r2d_state_dHfull_all_rx_s906_FAILED_buserror_3484598`;
resubmitted fresh as **3486259** (not resumed: r2dreamer's replay buffer is not checkpointed,
so a resumed run is not the same experiment as the seven that ran straight through).

**The monitor did not catch it, and reported `failed=0` while it was dead.** The checker built
hours earlier used `%%`-escaped format strings written through a quoted heredoc, so `sacct`
received `+%%Y-%%m-%%dT%%H:%%M` literally and answered `Invalid time specification (pos=0)`,
returning zero lines. Every terminal-state counter — done, failed, preempted — therefore read 0
structurally, regardless of the truth. Three consecutive "0 failures" status reports to the user
rested on it. Fixed: single `%`, scoped to the current batch by job id, and an explicit
`BAD-SACCT-RETURNED-NOTHING` verdict so an empty query can never again present as health.

This is the third instance today of the same defect class, twice in my own tooling: a value that
is *absent* being rendered as a confident zero. The first (`contact_push=0.000`) I misread as
unreachable code; the second (RLPD `picked=0`) I misread as a broken recorder and relaunched 16
jobs over; this one inverted — a real death displayed as health. The guard that generalises is
the one now added: **distinguish "the query returned zero" from "the query returned nothing".**

**Also measured, and it settles an open question from the RLPD correction above.** The failed
run's log shows `episode/train_picked 0.0` beside `episode/train_ep_picked 1.0` on the same
episode, repeatedly. So for **{r2dreamer}** the sticky twin carries information the first-grant
read does not — the opposite of **{RLPD}**, where the two agree on every one of 7000+ episodes.
Both learners now emit both, so no comparison depends on which is which; but the r2dreamer
`log_*` first-grant values must NOT be read as per-episode stage outcomes.

## 2026-09-10 — extension to 16 v 16 per learner (registered BEFORE submission)

User authorised adding 8 further seeds per arm for both learners, on seeing the first
runs complete. This takes each learner from 8 v 8 to **16 v 16**, the sample size left
open as a decision in `OVERNIGHT_STATE_2026-09-07.md` (e2e MDE at 8 v 8 ≈ 0.21).

- **{RLPD}** human seeds 908-915, machine seeds 928-935 (16 runs, 250k decisions)
- **{r2dreamer}** human seeds 908-915, machine seeds 928-935 (16 runs, 4M steps)

**Code is unchanged from the first 16 of each**: RLPD at `gp_e2e` `6e98ce3` (sticky
episode record), r2dreamer via `wmfix_full.sbatch` with `FULLENV_REWARD_X=1`, same
demo sets `dHfull_all_rx` / `dDPfull_first_rx` with the same manifest gates. The new
seeds are therefore poolable with the existing ones; had any code changed, they would
not be.

**State at the decision, so the basis is auditable:** {RLPD} 9 of 16 COMPLETED at full
budget with eval cells and `disagree=0` over 16.5k episodes; {r2dreamer} 16 of 16 alive
at 2.8-3.5M of 4M after 10h, sticky keys present in all. Disk 324 GB free, r2 runs 0.5 GB
each, so the extension costs ~10-20 GB.

**Disclosed risk:** no {r2dreamer} run had completed end-to-end when these were submitted,
so its train->eval handoff was unproven in this batch (the shared evaluator was proven by
13 RLPD cells, and the world-model e2e path ran in earlier work). If that handoff proves
broken, 32 r2dreamer runs train correctly and produce no cells; the training records
survive either way and the cells can be regenerated post hoc from checkpoints.
