# Observation-only contact-solver diagnostic

Full-task outcomes change with CPU/GPU execution and timestep refinement, while
the unloaded hand trajectory becomes closer with refinement. This diagnostic
records the existing contact solver's gradient norm, half grad dot Mgrad,
cost improvement, scaled tolerance, constraint count and `improved` flag after
every constraint-force call. It does not change solver, material or source motion.

The frozen control is soft trial 233 with CG, 100 iterations, float32 and eight
substeps. All 14 archived full-replay arrays must match exactly before interpreting
the diagnostic. An above-tolerance gradient alone does not establish failure:
the engine also permits termination on a small positive cost improvement.
No solver or precision setting is selected by recovery yield.

The full observation control completed and all 14 arrays match the archived
replay exactly. Of 23,088 source substeps, 559 (2.42%) retain `improved=True` on
return. In the installed monolithic CG loop this indicates iteration-cap
exhaustion; actual iteration numbers were not instrumented. The fraction is
244/2,400 (10.17%) during 20–23 seconds. Most other substeps satisfy the permitted
small positive cost-improvement exit; their larger gradient norms alone do not
prove failure. `summary.json` preserves interval counts and the inspected engine
source hash. A matched 1,000-iteration-cap follow-up is in `../constraint_iterations`.
