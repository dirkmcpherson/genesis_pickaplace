# Planner pixel campaign — prepared for original-agent review, 2026-09-15

**PREPARED, NOT SUBMITTED.** The user authorized rsync and run preparation, then added Diffusion Policy and requested original-agent review before proceeding. No training, evaluation, sweep submission, scheduler hold/release, or existing-run change was performed.

Cluster root: `/cluster/tufts/shortlab/jstale02/planner_px_2026-09-15`.

## Review entry points

- `RUN_PLAN.json`: every seed, environment variable, executable, budget, output and queue.
- `SUBMIT_COMMANDS.txt`: 64 exact future sbatch commands, **not executed**.
- `diffs/`: complete differences from the fetched reference launchers.
- `LOCAL_VALIDATION.json`: unchanged learner command arrays, distinct outputs, seeds and shell syntax.
- Cluster `preparation/REMOTE_VALIDATION.json` and `preparation/dry_runs/`: actual launcher dry-run results.
- Cluster `DATA_TRANSFER.json`: archive and every extracted member verified.
- `FILES.json`: hash manifest of the preparation overlay; checked before submission and inside each dataset gate.

## Conditions

One planner dataset arm, **training n=16 per learner**, seeds 0–15: **64 GPU jobs**. Collection n=72 is a separate quantity; none of these training jobs has run.

| Learner | Training recipe | Budget | Checkpoints |
|---|---|---|---|
| DV3 losses in R2 chassis | `genesis_full_pixel`, `model.rep_loss=dreamer`, shift4, proprio slice8, bounded_normal, entropy3e-5, return clamp10, replay500k | 2M online counter steps | 0.5M / 1M / 1.5M / 2M |
| R2Dreamer | Same chassis/config; `model.rep_loss=r2dreamer` | 2M online counter steps | Same |
| RLPD pixels | Original TRAIN_ARGS unchanged: gamma.99, UTD10, E10/Z2, 128 demo +128 online batch, shared critic encoder, shift4, replay300k | 250k decisions | Every25k, original 0.4/1.0 fraction checkpoints and final |
| DP pixels | LeRobot diffusion, batch64, proprio8 + top/wrist, no object/goal state, random56×56 crop, default backbone from scratch | 100k gradient updates | Save50k/100k; retain final weights after successful training |

All use frozen experimental w3 `gc_kp4_riser3_shelf6`, `nested_sparse10`, `tip_guard=not_in_hand`, far_release off. DP ignores reward during fitting; its sidecar carries these evaluation settings. Preserve the original full-task training/evaluation horizons and action-repeat4. The demonstration collection's 600-decision budget does not override any learner's environment horizon.

World-model/RLPD jobs request one GPU,8CPUs,48GB, gpu/preempt + preempt QOS; DP preserves gpu + normal QOS. Seeds can run on different nodes. No GPU packing. GPU types remain l40s/a100/l40/h200, excluding pax077. The existing eligible-job limit/hold helper is untouched; these planner-prefixed job names are outside its filters. Submit in reviewed bounded waves rather than inserting 64 eligible jobs ahead of the evaluation backlog.

## Dataset and provenance

Transferred archive: `artifacts/planner_matched72_training_w3.zip` (235,038,799 bytes).

SHA256: `33684f34d6623665bc9c6bc90a9421d1dae43ea2925da8df5e01f0279661f49f`.

Verified extraction: `data/training_v1/`.

- RLPD/DV3/R2: `native_rns10h_img/`.
- DP: `lerobot_pixels/`, with `dp_raw/` retained for action provenance.
- 72 attempts,70 nonempty tapes,17,550 executed action transitions. Two pre-action aborts remain in `zero_transition_attempts/` and all-attempt HDF5; no invented actions.
- The **same fixed collection** supplies every training seed. Collection seeds40000–40071 are not learner training seeds0–15.
- Images are verified single-scene replays, top RGB then wrist RGB. Rejected direct-collector frames are not in this archive.
- Native training rewards are already +10: keep R2 reward_scale1, clamp10. The legacy `terminal_reward=1` stamp refers to source units; it is not another scaling instruction.
- Planner DP labels are absolute requested integrator targets, verified to decode back to executed deltas. Metadata fps is **8.333333333333334**, reflecting0.12s decisions. Existing human/DP LeRobot sets advertise7.5fps. We preserve the new data's verified timing rather than falsify it to satisfy a legacy builder gate; the policy still consumes the same integer frame offsets. Original agent should retain this disclosure.

Independent code clones (no shared worktree or hard-linked git objects):

- `gp/` pinned to `e8287af018d06552384bed4b9fc34f4a838fee5c`, cloned from current gp_px.
- `r2dreamer/` pinned to `0b1b9d8fee0b084998a4077761d452ad5712a8f7`, cloned from r2dreamer_px.
- Preparation overlay resides **outside** both trees under `preparation/`; neither algorithm implementation is patched.
- Five world/action/predicate files match the collection's frozen source hashes; verified on cluster by gate.py.
- RLPD's existing registry writes only the private gp clone's registry. Per-job planner provenance additionally records the archive hash, dataset/overlay/run-plan hashes, source commits, seed, job ID and restart number under this campaign's `provenance/`.
- Shared runtime paths remain the reference environments: world models `$LAB/r2d_venv`; RLPD/DP `$LAB/condaenv/genesis`. No installation or runtime environment modification. The private RLPD launcher refuses missing/wrong SB3 rather than pip-installing into a shared environment.

## Differences requiring explicit review

1. **New source arm:** planner instead of human/DP-teacher. Gates assert the actual planner builder,70 usable tapes,72 attempts, hashed camera/state/action exports and timing; no relabeling as dH/dM and no fabricated manifest fields.
2. **Budgets and replication:** all new world-model seeds use the (ag)2M extension recipe. Existing DV3 and R2 16-seed cohorts mix1M pilots and2M extensions; the matched primary comparison remains mean rnd30 MODE home at0.5M and1M. All-seed2M planner results must not be compared to a fictitious16-seed2M reference. DP's existing pixel recipe has n=4 per source; planner n=16 extends replication, not its100k-update budget.
3. **DP evaluation placement:** the old launcher evaluates inline on its GPU node, while the registration names64-core evaluation. The planner launcher submits its final evaluation to an explicit64-physical/64-logical CPU job, preserving evaluator, bank, horizon and MODE inference. This is orchestration/hardware enforcement, not a model change.
4. **Paths and gates:** new output/log/registry roots, archive/overlay checks, and the no-install RLPD dependency gate. Learner command arrays compare byte-for-byte equal to the reference launcher arrays (seed/dataset/output resolve differently as intended).
5. **Unremoved source confounds:** this collection has matched72 DP starts and a corresponding72-human subset, but current reference human training uses74 tapes. All nonempty planner tapes cover65 initial geometries; two aborts provide no supervision. The dataset also differs in length, success distribution, privileged planner access and action labeling provenance. Do not claim a quantity-matched causal demonstration-source ablation.

## Evaluation and commands for later use

Nothing below has been submitted. Read the exact commands first:

```bash
cd /cluster/tufts/shortlab/jstale02/planner_px_2026-09-15/preparation
python3 verify_preparation.py --root /cluster/tufts/shortlab/jstale02/planner_px_2026-09-15
python3 submit.py                       # prints all64; submits nothing
python3 submit.py --learner dreamer --seed 0   # prints one
```

After original-agent/user review, one explicitly selected training job can be submitted with `python3 submit.py --learner <rlpd|dreamer|r2dreamer|dp> --seed <0..15> --submit`. The submitter checks hashes, output collisions and existing named jobs, then records the returned ID. The text manifest exposes raw sbatch commands for review; prefer the checked Python entry point for actual submission.

World-model milestone evaluation:

```bash
bash evaluate_world_models.sh           # dry-run; does not submit
# Later: bash evaluate_world_models.sh --submit
```

This uses the unchanged reference milestone sweep confined to this campaign's runs/evaluation roots. It copies/hash-checks milestones, reads the run's pinned trees, and evaluates rnd30 MODE, hold15 MODE and rnd30 SAMPLED on64/64 CPUs. Final aliases must not be counted as independent checkpoints.

RLPD retains its automatic final CPU evaluation (hold15/rnd30, MODE/SAMPLED, original isolated diagnostics). DP automatically calls `eval_final.py` after its final checkpoint, submitting hold15/rnd30 MODE on64/64 CPUs. Missing or failed evaluation cells remain missing; shared full-task home is the comparison endpoint, not an isolated merged headline.

No GPU smoke, training update or policy evaluation has been run for this cluster campaign. Prior local dataset loader/update/reload checks passed; current preparation adds file integrity, recipe comparisons and launcher dry runs. Cluster GPU/runtime validation is still a distinct next action after review.
