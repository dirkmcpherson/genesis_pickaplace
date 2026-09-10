# Frozen soft-pad validation

No general end-to-end improvement is established. Soft pads improve this hand relative to its rigid-contact controls, but the complete hand does not improve the original recovery model across the selected demos.

| Demo | Day | Cohort | Original fixed: distance / strict | Matching rigid: distance / strict | Soft candidate: distance / strict |
|---|---|---|---|---|---|
| 113 | Dec16 | calibration | 697.16 mm / no_supported_release | 312.51 mm / not_picked | 293.91 mm / not_picked |
| 184 | Dec17 | calibration | 92.51 mm / no_supported_push_to_contact | 206.38 mm / not_picked | 218.52 mm / not_picked |
| 233 | Dec18 | calibration | 65.81 mm / complete | 179.33 mm / no_supported_push_to_contact | 70.19 mm / no_supported_push_to_contact |
| 176 | Dec16 | reserved | 65.84 mm / complete | 68.91 mm / no_supported_push_to_contact | 65.75 mm / complete |
| 185 | Dec17 | reserved | 108.27 mm / no_supported_push_to_contact | 318.95 mm / no_supported_release | 154.66 mm / no_supported_push_to_contact |
| 237 | Dec18 | reserved | 66.55 mm / complete | 73.68 mm / final_not_retained | 72.47 mm / final_not_retained |

Distances are final center-to-center distances. The unchanged supplied metric passes 3/6 for the original fixed model, 2/6 for the matching rigid hand and 3/6 for the soft candidate. Strict completions are 3/6, 0/6 and 1/6 respectively. These denominators contain four early and two late demos, selected for calibration/validation, not a population census.

The reserved soft 176 reaches and retains goal contact; rigid 176 stops 2.91 mm short of surface contact. Both pass the supplied 81 mm metric. Soft 185 restores supported release relative to its rigid control but finishes 154.66 mm from the goal; the original fixed trace finished 108.27 mm away. Both 237 candidates reach goal contact and then lose final retention (soft surface gap 6.47 mm); the original fixed 237 retained contact.

The first strict release in 176 is a short separation followed by regrasp, not the final release: contacts resume, including a lifted interval near 25 s. A sustained separation begins at 27.12 s, with subsequent shelf recontacts. The camera review shows discrepancies in intermediate seating and can position. Keep the strict score and review evidence separate; the supplied metric and strict thresholds are unchanged.

Annotated reviews: [late 233](233_soft_pad_real_sim.mp4), [early 176](176_soft_pad_real_sim.mp4). Both include the actual final frame, clock-matched recorded views and saved simulation poses; no added settling. Every video frame decoded successfully. The view angles differ.

Independent saved-action reproduction passed for both 233 and 176: every saved array, including finger joints, contacts, can trajectory and EEF actions, is bit-identical in fresh processes (`saved_action_verification.json`). No training bank or existing recovery set has been changed.

Next unresolved issue: calibrating the passive transmission over the different closure/loading regimes. The lower-closure early grasps fail under this hand, while strongly closed late 233 and early 176 can be carried. Further arbitrary softness tuning on the reserved trials would not resolve that model uncertainty.
