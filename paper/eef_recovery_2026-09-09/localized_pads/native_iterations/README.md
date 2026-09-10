# Native GPU test after raising the contact iteration limit

The native CPU control reproduces physical states, contacts, picked flags and
tool poses from the separate CG1000 replay exactly. The16-lane GPU case then
completes all962 decisions. Both CPU and GPU now complete the strict sequence
and supplied slide metric on233; the earlier GPU CG100 run tipped and failed.
All copied GPU lanes match in physical and diagnostic arrays.

CPU/GPU can trajectories still differ by up to14.024mm, with10.955mm endpoint
difference. Final center distances are66.090mm CPU and66.159mm GPU; goal movement
is5.042mm CPU and0.889mm GPU. Agreement on task completion does not establish
equivalent physics or fidelity to the real recording. The GPU CG1000 case has
not been repeated and is not admitted as a training tape or material result.

Warm GPU throughput including contact observation is112.56 aggregate decisions/s.
Concurrent host load differs from previous benchmarks; this is not evidence that
raising the iteration cap improves speed. Sixteen copied lanes are one source.
`readout.py` verifies source, options, hashes and lane equality in `summary.json`.
