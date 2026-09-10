# Higher-accuracy contact-solver references

At CG1000 the tested CPU/GPU source now completes on both backends, but8 versus16
substeps still changes the outcome despite zero cap exits. This bounded diagnostic
compares two solver implementations at matching higher precision and tolerance.

The float32 CG1000 control must reproduce all14 archived arrays exactly before
the float64 cases run. Both references use1000 iterations, eight substeps and
tolerance1e-8, one CG and one Newton. Intended physical parameters and recorded
source path stay fixed. Runtime precision/solver type/cap are asserted. Compare
solver agreement, residual diagnostics and task trajectories; a passing score
alone is not a reason to choose one. These are one-source numerical controls,
not added demonstrations or a new soft-pad result.
