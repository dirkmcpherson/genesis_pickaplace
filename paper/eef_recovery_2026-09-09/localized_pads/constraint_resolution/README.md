# Timestep check with the higher iteration cap fixed

Both8- and16-substep CG1000 replays complete execution with no observed iteration
cap exits. All declared physical/source invariants pass. The8-substep case
completes the full task;16 establishes release but fails the final supported push,
ending102.470mm from the goal. The maximum can-trajectory difference is49.866mm,
and the final difference45.573mm. Goal movement at16 is2.387mm.

Thus removing cap exhaustion does not resolve timestep sensitivity. The difference
is smaller than with the old cap, but remains consequential. No physical or
numerical setting is selected by a passing task result. `summary.json` preserves
both unchanged predicates and the independent full-source/physical checks.
