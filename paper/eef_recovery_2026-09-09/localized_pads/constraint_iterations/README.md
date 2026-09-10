# Contact solver iteration-cap refinement

The observation-only CG100 control exactly reproduced all 14 archived trial233
arrays, but retained a request for further iteration on 559 source substeps,
including 10.17% of the 20–23 second placement interval. This single declared
follow-up raises only the cap to 1,000. Float32, tolerance, line search, eight
substeps, material, hand mechanics, geometry and recorded source motion stay fixed.

`run.py` freezes the source and diagnostic wrapper before launch. It verifies
source commands and timing fields after the complete replay. The wrapper's raw
runtime qualification describes the observation hook; this experiment also
changes the declared iteration option at build time, recorded in runtime options.
Compare both termination diagnostics and full task outcomes; no candidate is
adopted because this particular recording happens to pass.

Both control and cap1000 replay complete the strict sequence and supplied
metric. Raising the cap eliminates all559 final-flag cap exits in the observed
source substeps. Final center distance is66.090mm (control65.992mm), with5.042mm
goal movement (control9.863mm). Can trajectories differ by up to14.776mm, first
exceeding1mm at6.03s. The limit affects dynamics; this does not yet show that
it explains GPU/timestep differences. `summary.json` checks all physical/input
invariants and retains both outcomes. The frozen plan's inherited phrase
“No material or numerical change” is incorrect for its numerical cap: the
declared purpose and command explicitly change100 to1000; material stays fixed.
