# Hardware and task constraints rechecked

The manufacturer-authored [Gen3 lite user guide](https://static.generation-robots.com/media/Kinova-lite-fiche-technique.pdf), printed page21, describes a flexible structural plastic and a softer rubber-like gripping material, driven by one linear actuator in the wrist. The checked gripping and specification sections do not supply a numerical pad hardness, friction coefficient or damping law. Searches for Shore and damping returned no matches in that document. This is not a claim that no such data exists elsewhere. No numeric material parameter is inferred from these qualitative descriptions.

The local `paper/METHODS_draft_2026-08-28.md` section1.4 explicitly records user-validated233/242 as ending in goal contact. Section2.3 also discloses outcome-fitted placements and goal recovery. The normal-damped soft233 supplied-metric pass is retained unchanged, but its0.47mm ending gap and absence of goal contacts in saved decision states do not establish the strict contact sequence. No threshold or placement is changed to turn the near miss into completion.

The methods record and video annotations do not give calibrated millimeter-level real can/goal poses. Lower simulated goal movement alone is therefore a diagnostic, not proof of improved real-world fidelity.

There is an additional interpretation limit: the existing goal estimate itself
used233/242 end-contact constraints. Therefore a submillimeter strict-contact
difference between hand models is not an independently calibrated real-world
error measurement. Keep the supplied0.081m proximity metric and strict physical
sequence diagnostic side by side. Do not infer that the new model is physically
worse solely from its0.47mm simulated gap, or that the objective is achieved
solely from the proximity pass. Broader paired outcomes and real-video evidence
are needed; the goal position is not changed in this experiment.
