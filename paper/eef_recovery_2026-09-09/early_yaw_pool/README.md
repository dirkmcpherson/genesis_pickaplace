# Early-day EEF recovery: December 16 and 17

This is a separate 89-candidate pool, not the 74 December-18 census.
The December 16 portion is now complete: three of 49 pass the supplied slide
metric and are independently verified, reviewed and packaged. See
`DEC16_READOUT.md` for the full failure funnel and initial-geometry qualifications.
The December 17 portion is still running.

The pool includes
49 December-16 and 40 December-17 candidates. Full-world EEF replay uses mount yaw
-9.7 and -19.2 degrees respectively. The shelf, table and goal remain in the common
world frame. All archived seed rotations were checked mathematically; winner
positions are already world coordinates and are not rotated again.

Eighty-eight initial positions were selected by the archived pre-only search, so
these remain fitted reconstructions, not new ground-truth measurements. Trial 183
has no winner and uses its rotated seed. Old picked/contact/nested labels are not
accepted as full-sequence evidence. No frozen tape, placement or bank is modified.

All source measurements are freshly extracted using ROS1 timestamp arithmetic and
interpolated at 30 ms for this first census. Trial 207's old Python-port windowing
error is corrected only in these isolated source artifacts. Both world hooks run;
worker.py checks built yaw, arm gains, mount height and shelf position before
stepping, retaining a world_audit.json per trial. No og4 or added slide is used.

The runner first executes 118 and 183, one per day, before dispatching the remaining
candidates with two workers. Each process retains source, logs, measured trajectory,
EEF actions and sequence outcome. Passing the numerical scorer alone does not
admit an episode; independent saved-action replay and visual review remain required.

The original candidate selection excludes December 15 and other recordings without
the specified carry/release signals; this is not a claim to cover all 120 early bags.
