# Planner pixel cohort queued — 2026-09-15

**Actual submissions: 32 training jobs, n=8 per learner, seeds 0–7.** All were submitted, their held dependency graph verified, then released to Slurm. At **20:31:13 UTC**, all32 were PENDING / Dependency, with no user holds on planner jobs. No resident planner dispatcher exists. [Verified scheduler snapshot and exact submission records](planner_queue_2026-09-15/verified_submission.json).

| Seed | RLPD | DV3 loss in R2 chassis | R2Dreamer | Pixel diffusion policy |
|---|---|---|---|---|
| 0 | 3729323 | 3729324 | 3729325 | 3729326 |
| 1 | 3729327 | 3729328 | 3729329 | 3729330 |
| 2 | 3729331 | 3729332 | 3729333 | 3729334 |
| 3 | 3729335 | 3729336 | 3729337 | 3729338 |
| 4 | 3729339 | 3729340 | 3729341 | 3729342 |
| 5 | 3729343 | 3729344 | 3729345 | 3729346 |
| 6 | 3729347 | 3729348 | 3729349 | 3729350 |
| 7 | 3729351 | 3729352 | 3729353 | 3729354 |

## Queue order and operations

Every planner job depends on the **start** of reference DP job **3706858** (`ah_dp_px_dM_s3`), which was pending at submission. Within each learner, seed k>0 also depends on the start of seed k−1. These four chains admit at most four not-yet-started planner jobs at a time without waiting for prior training to finish. Preempted/requeued jobs can add waiting jobs after their original start; the shared QOS still enforces the20-GPU cap. All32 submissions use partition `gpu,preempt`, QOS `preempt`.

The15 remaining reference RLPD seeds were held before submission and remain held. The already-installed, backed-up `wm_fix_2026-09-03/px_release.sh` hook reserves planner precedence whenever invoked. Once all planner jobs have started, invoke that same one-shot helper to admit the held reference seeds. No persistent process performs that future invocation. Existing training jobs, their commands, and their run paths were not changed by this submission.

**Do not run dispatch.py or blindly rerun queue_once.py/submit.py.** The cohort is already queued. Cluster `DISPATCH_STATUS.json` and `PREPARATION.json` now state `QUEUED_SLURM_DEPENDENCIES_NO_DAEMON`; `NO_CLUSTER_DISPATCHER.txt` records the user instruction. STOP_SUBMISSIONS was removed to enable the existing priority hook; it does not imply a controller is running. No automatic stop-on-training-failure service exists. Record failed seeds, inspect them, and do not replace them based on performance. Intermediate world-model milestone scoring still needs the private one-shot sweep; this submission is not evidence that any policy evaluation completed.

## Provenance and checks

Registration: **P-MP-20260915 revision4**, appended before submission. [One-shot supplement and registration](../cluster/planner_queue_2026-09-15/REGISTRATION.md). The supplement imports the original sbatch command builder, runs all existing preparation and data hash gates, takes the submission/controller locks, and records all sbatch commands and dependency IDs. It adds only scheduler holds/dependencies/comment; training arguments and dataset paths are unchanged. Dependency-graph checks covered all6561 combinations of four chain start prefixes.

- Frozen preparation FILES.json SHA256: `82e55cd0c92154f0937e0044968947f86f5856f602d5eb70a75ea55a85db46b9`.
- One-shot queue script SHA256: `071ccb5a0cdd9764d24ff0e5398878544b9fd2761984318216df4ae1b28b7b2f`.
- Training archive: `planner_matched72_training_w3_reference_terminals.zip`, SHA256 `2ca2b16e1ca816fec999842b62a375eb9d8cb4a3da89b521aadd20758c78ffd9`.
- All587 dataset manifest members verified at submission; pinned learner trees and launcher syntax passed integrity checks. No learner source or hashed preparation file was edited for queueing.
- Training budgets remain RLPD250k decisions, DV3/R2 2M online counters, DP100k gradient updates. Seeds share the same planner dataset; this remains the exploratory arm with the documented distribution confounds.

Cluster root: `/cluster/tufts/shortlab/jstale02/planner_px_2026-09-15`. Exact ledger: `SUBMISSIONS.jsonl`; complete one-shot record: `ONESHOT_SUBMISSION.json`; pre-submission queue snapshot: `QUEUE_BEFORE_ONESHOT.json`; supplement: `queue_once_2026-09-15/`. This document supersedes earlier stopped/unsubmitted or n=16 dispatch snapshots. Scheduling and integrity counts are not experimental outcomes; no significance test applies.
