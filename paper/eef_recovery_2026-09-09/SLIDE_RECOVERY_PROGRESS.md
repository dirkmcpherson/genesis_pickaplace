# Final-slide recovery and engine recheck, September 9

**Fidelity correction (September 9):** The default development manifest contains **eight
unmodified-command reconstructions**. Trial 242's outcome-fitted initial pose is separate in
`manifest_fitted_initial_pose.json`. Three added-motion or grip/timing records (242, 275, 308)
are separate in `manifest_modified_commands.json`. The historical 11-source total is **not a
faithful-recovery count**. Og4 accelerates recorded opening; it is a diagnostic command modification,
not a fidelity fix or a passive-finger model. Unmodified commands alone do not validate timing,
estimated initial poses or contact physics. Files remain in place; select through a manifest,
not a directory glob. `manifest_mixed_historical.json` preserves the old packaging for provenance.


User priorities: more complete pick–place–slide demonstrations, especially the final slide;
some recordings may remain unrecoverable. The engine pin may be reconsidered. Existing frozen
datasets, learner controllers, and official stage definitions remain untouched.

## Result so far

**Later follow-up:** [275 and 308 now have verified repaired episodes](REPAIR_FOLLOWUP.md).
The bank has 11 source UIDs. The 8/74 unmodified-grip census below is a raw EEF control, not
the previously adopted og4 recorder's recovery rate.

**Census extension complete:** all 74 December-18 source trials were collected through the
precision EEF controller, preserving archived initial poses and recorded motion. Eight complete
the physical sequence and pass independent saved-action replay and phase-frame visual review:
232, 233, 255, 273, 294, 299, 300, 305. Their trajectories, actions and observations reproduce
exactly. These are verified EEF conversions, not eight newly rescued joint-replay failures.
The bank contains nine independent source trials including the earlier 242 rescue. The common
10 mm slide-repair sweep attempts eight failures and completes only 242; the earlier-contact
three-trial probe completes none. [Full readout](FULL_POOL_READOUT.md) records all outcomes.

The original one-trial rescue result below remains distinct from those conversions.

**One additional source trial (242) now has an independently verified complete EEF episode.**
The preferred route keeps the recorded motion and corrects initial can x by +10 mm; precise
Cartesian IK reproduces the successful joint control exactly. The saved EEF action sequence,
replayed in a fresh process without source joint targets, reproduces that trajectory bit-for-bit.
Full video review confirms pickup, shelf placement, release, a later push into contact and withdrawal.

Pilot bank: [`../../baselines/demos_eef_recovery_2026-09-09/manifest.json`](../../baselines/demos_eef_recovery_2026-09-09/manifest.json).
Video: [242, unchanged recorded motion](slide_repair_v2/242_precision_ik_full.mp4).
An alternative uses the original initial position and a bounded 10 mm push extension plus 0.06 s
contact hold. It also reproduces exactly from saved EEF actions, but is explicitly motion-repaired
and is not another independent source trial. [Alternative video](slide_repair_v2/242_contact_hold_full.mp4).

This is a development result, not evidence that the full corpus is recovered or that finger
compliance explains the failures. The larger goal remains active.

## Tests and failures retained

| Test | Result |
|---|---|
| 118: full-hook 3×3 initial XY grid, ±10 mm | 0/9 picks; parked rather than expanding the search |
| 242: same 3×3 grid, joint replay | 2/9 physically complete sequences, at offsets (+10,-10) and (+10,0) mm |
| Those two 242 positions with original Genesis IK | 0/2 complete EEF sequences despite passing the old p99 target-error gate |
| 242 (+10,0) with double-precision local IK refinement | Complete; measured trajectory exactly equals successful joint control |
| Fresh replay of those saved EEF actions | Complete; measured trajectory and actions bit-identical |
| 242 original position, bounded 10 mm extra push | Reached goal for one frame only; rejected |
| Same plus holding tool target at contact | Complete with two extra hold frames; fresh saved-action replay identical |
| Fresh joint controls 302, 275, 304 | All pick and release onto shelf but fail the full slide rule |
| Same three, precision IK + bounded 10 mm push + up to 0.3 s hold | 0/3 complete; final surface gaps 5.29, 0.166, 6.89 mm respectively; do not relabel tiny gap as contact |

All candidate traces, including failures, are retained in the corresponding `position_grid_*`,
`slide_candidates`, and `slide_repair_v*` directories with plans and source/code hashes.
The precision solver never reads the source joint waypoint: Genesis supplies the branch seed,
then a local analytic URDF Jacobian refines the desired Cartesian pose in float64. It passed
100 perturbed-pose convergence checks before replay. Its tiny geometric residual is not a claim
of comparable physical tracking accuracy or real robot calibration accuracy.

## Physical acceptance rule

`can_pos_recovery/eef_task_sequence.py` is a separate development filter. It requires:

1. A prior pick, upright shelf solver contact within 4 mm of the correct resting centre height,
   then at least three consecutive supported frames with no robot–can contact.
2. A later robot contact followed by at least 10 mm can motion toward the goal, with shelf support
   on at least 80% of that interval, no height excursion beyond 8 mm, and no tilt of 20° or more.
3. At least three consecutive supported upright can–goal contact frames, then final upright shelf
   support with surface gap within ±2 mm.

It uses actual contact and pose, not gripper feedback <30. Eight adversarial tests reject carried
contact, table shoves without pickup, second airborne carries, passive drift, missed contact,
subsequent separation and deep final interpenetration. Thresholds were declared for development,
with the symmetric gap amendment recorded before reading the 242 grid outcomes. They do not
retroactively replace the frozen paper evaluator. Visual inspection is additional evidence.

Initial v1/v2 generation NPZs stored observations as object-wrapped `{state: ...}` dictionaries.
Their numerical traces/actions are unaffected. Verification outputs and the pilot bank store
plain float32 `(N+1,17)` states, loadable without pickle. Early verification sidecars record
`hold_contact=false`/`insert_after=null` because those generation options are inactive during action-only
playback; consult the linked generation sidecar and action-kind/source-index arrays for the repair
history. The pilot manifest explicitly carries that history.

## Engine pin rechecked

The installed current engine reports **0.2.1**, from upstream `31951c3f` plus headless-render commit
`f41427d`. This agrees with `cluster/patches/GENESIS_PIN.md`; it is not a stock 0.28 release.
The old rejection of 1.2.1 is in `CAN_STARTING_POSITION.md`, July 10 upgrade-gate section.
It predates the corrected shelf world and contains an assumed 0.0025 s effective time constant,
questioned later by METHODS §engine caveat. That motivated a new limited check.

The retained `.venv-g12` has no interpreter launcher, but its Python 3.10 packages import and run
using system Python plus an explicit package path. Actual imports report Genesis 1.2.1 and torch
2.13.0+cu130. Both variant hooks, the same URDF, same initial can positions, raw joint/grip streams,
fixed tape clock and one effective CPU thread were used. The 30-frame FK smoke check passed.

**Native-default 1.2.1 full replays of 233, 242 and 304 all fail pickup (0/3), while all three current
engine controls pick (3/3).** Results and plan are in `engine_1_2_1/`. This rejects an immediate
drop-in switch for this recovery lane. It does not establish that every calibrated newer-engine
model must fail: solver defaults/dependencies differ, and force/compliance matching remains a
separate task. Current engine retained for now; no installed engine was replaced.

## Correction to the preceding world-provenance report

Gravity compensation is applied in the **pre-build** hook and was present in the early recovery
runs. The omitted post-hook changes arm gains and shifted goal spawn height for these variants.
The prior wording claiming absent gravity compensation was incorrect and has been corrected in
CONFOUNDS 56, the early-trials brief, and the pilot report. The independently observed 118 pickup
mismatch between pre-only and full-hook runs remains valid.

## Remaining work

Expand verified recovery beyond this one source trial, prioritize shelf/slide failures, and test
grasp mechanics against real evidence. The 10 mm repair is not a general solution: it failed all
three additional near-contact candidates. Improved EEF numerical fidelity is useful, but does not
by itself validate the rigid finger model or fix the corpus's carry and release losses.
