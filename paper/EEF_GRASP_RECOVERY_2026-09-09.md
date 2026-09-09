# EEF differential replay and grasp recovery

Objective: recover real pick, shelf placement, release and slide-to-contact demonstrations using EEF differential position/orientation actions; reduce carry/release losses without confusing motion tracking with grasp fidelity.

## Diagnostic registration (before new replay readout)

1. Isolate the action representation first. Compare raw measured-joint waypoint replay against EEF differential replay of exactly the same waypoint path, world, initial can position, grip stream, and frame schedule. This tests representation/controller fidelity, not the accuracy of the raw tape clock. Neither arm is the timestamp-faithful final dataset yet.
2. Use the actual URDF tool frame and built mount transform (including early-day yaw). Compose rotation increments on SO(3). Convert target tool pose to target wrist pose with the **target** rotation. Do not pin roll/yaw or infer a tool offset from a single reference position.
3. Pilot uids 233, 242 and 118: two previously reviewed complete real demonstrations and the user-identified early-day drop. These are development examples, not held-out confirmation. Each trial/mode uses a fresh process and the same corrected world settings. Historical renderer counts are not controls.
4. Predicted controller gate: FK matches built tool pose within 0.1 mm / 0.01 degrees at reset; EEF IK target errors p99 below 0.1 mm / 0.1 degrees; no large arm branch switch; realized path tracking close to joint replay. Stage outcomes and can paths may be contact-sensitive and must be reported even when the gate passes. Failure means repair the controller before judging passive-gripper changes.
5. Save per-frame actions, actual joint poses, actual tool poses and can poses, IK errors, stage diagnostics and endpoint results. Existing stage flags are descriptive; full completion ultimately requires supported shelf placement, release, subsequent slide and contact verified from the trajectory and video. Carried contact is not completion.

The current gripper is kept fixed for this comparison. Passive distal adaptation is the next controlled mechanism test, motivated by CONFOUNDS 55 and Kinova user-guide Figure 9. No passive model or improvement is established yet. Preserve frozen datasets and existing learner controllers; any new demonstrations must be separately identified and validated.

## First readout

Eight full diagnostic runs and their interpretation are saved in
[`eef_recovery_2026-09-09/README.md`](eef_recovery_2026-09-09/README.md).
233 and 242 pass the initial EEF gates; 118 passes after matching joint replay's unclipped
targets (the recorded joint-4 target slightly exceeds the URDF limit; physical limits remain).
233 has a supported release followed by a push into goal contact in both modes. 242's nested
label represents near-contact rather than sustained supported goal contact. Existing feedback-based
slide scoring rejects physical releases, corroborating CONFOUNDS 47.

The early-world provenance issue must be addressed before a gripper explanation is tested:
the recovery/render scripts omit the variant post-build hook. At the same recorded can position,
118 picks in the archived pre-only configuration and fails to pick with the full hook, in both
joint and EEF modes. This is logged as CONFOUNDS 56. Existing early winner positions cannot be
assumed to transfer to the fully configured world, and the 88-winner count remains unverified there.
