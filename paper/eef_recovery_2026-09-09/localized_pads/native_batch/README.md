# Native Genesis GPU batching: implementation verification

The user clarified that the desired parallelism is one Genesis scene with many
environments on the GPU, not separate CPU processes. Genesis provides this through
`scene.build(n_envs=...)` ([official parallel simulation guide](https://genesis-world.readthedocs.io/en/latest/user_guide/getting_started/parallel_simulation.html)).
The host has an RTX3060 with12GiB VRAM. The existing CPU material panel remains
unchanged and continues separately.

`batched_surface_pad_candidate_g14.py` adds the environment axis to every pad
contact state and counter. `bench_native_batch_g14.py` reproduces the adaptive
finger law in device-side Torch arithmetic with no explicit CPU readback in its
substep hook, preserving double-precision control arithmetic and refreshed pose
feedback. It executes frozen archived joint/grip commands. This is not yet a
batched EEF decoder or complete demonstration exporter; normal damping2 is also
not ported yet.

The standalone CPU port reproduces every saved arm/finger joint, can pose and goal
pose exactly over all962 decisions of233 (`cpu_full_equivalence.json`). That is
one full-source implementation control, not a new recording. The first400-decision
CPU prefix was also exact. Both preserve reset and substep counts.

A16-environment CUDA prototype completed400 decisions per environment. All16
copies of233 are bit-identical to each other. Warm throughput was88.59 aggregate
control decisions/s versus14.04 for the CPU standalone prefix, excluding initial
build/reset and the first50 decisions. Builds took128.8s GPU and14.1s CPU. CPU
material-panel workers and desktop applications were active during measurement;
this is not an uncontended hardware benchmark.

The CUDA prototype used Torch2.7+cu126, which Genesis1.4 warns is unsupported.
Its CPU/GPU trajectories are not identical: maximum finger-joint difference.1404rad,
maximum can-position difference3.420mm over the12s prefix, final difference.297mm.
This establishes that native batching executes, not that it is interchangeable
with the CPU reconstruction. No GPU result enters the material comparison.

The initial Torch2.8+cpu GPU attempt fell back to CPU and failed after reset; it
is not a GPU timing result. A subsequent supported CUDA installation terminated
with exit143 before package installation; its queued stack probe failed safely
and launched no simulations. The reason for termination is unknown. Completed
wheels are retained. The supported stack is being assembled in isolated `/tmp`
directories, reusing matching local CUDA libraries. Existing environments remain
unchanged. See installation logs and `attempt1.json`.

`bench_native_batch_g14_v2.py` supports heterogeneous source assignments within
a common mount/geometry group. `supported_plan.json` freezes standalone233 and262
controls followed by16/64-environment mixed batches,400 decisions each. Compare
per-UID trajectories and repeated copies, not different UIDs against one another.
Those supported-stack cases have now completed; see the results below. Full individual endpoints, both
material conditions, damping2, batched contact/EEF export and independent action
verification remain required before using the new path for reconstruction.

## Supported stack results

Torch2.8.0+cu126, Genesis1.4.0 and the RTX3060 passed the isolated-stack probe.
All four frozen cases completed. Warm aggregate decisions/s: standalone2336.70,
standalone2627.70, mixed16 88.52, mixed64 326.03. The CPU control was14.04;
64 lanes provide23.2x one CPU worker, not23.2x the existing eight-worker panel.
Measurements exclude reset/build and first50 decisions and retain concurrent
CPU panel/desktop load. These are400-decision prefixes of only two distinct UIDs.

All duplicate copies within either batch are exact. Commands, geometry, collision
policy, URDF and material hashes match standalone controls. State trajectories
do not: maximum can-position discrepancies versus standalone are2.789/6.197mm
for233/262 at16 lanes and4.021/7.096mm at64 lanes.16 versus64 also differs by
3.002/1.467mm. First233 difference occurs at decision1 in quaternion components
at approximately1e-12, then grows. This is consistent with numerical sensitivity,
but the cause is not established. No tolerance was relaxed to declare equivalence.

`compare_supported.py` writes `supported_comparison.json` with exact flags,
per-component errors, invariants and hashes. `lane1_plan.json` declares the next
two adaptive controls: same code and commands, n_envs=1, to isolate the shape
change from n_envs=0. Full-task export and GPU material comparisons remain pending.

## Repeatability diagnostic

The n_envs=1 controls complete;233 differs from the earlier n_envs=0 run by up
to2.964mm. More decisively, repeating exactly the n_envs=0 configuration changes
233 can position by up to3.028mm and finger angle0.1233rad over the400-decision
prefix (first difference at decision12). Therefore the earlier across-run batch
comparisons do not isolate batch size as the cause. Mixed16 repeat is running.
No exact GPU repeatability claim; the CPU port identity remains separately proven.
`repeat_plan.json` and `repeat_single233_comparison.json` retain this control.

Mixed16 repeat also differs: max can2.548mm for233 and5.303mm for262;
max finger differences0.1277/0.1484rad. Duplicates within each run still agree.
Inspection of installed rigid_solver.py identifies timing-based dispatch between
numerically different solver implementations. `use_deterministic_algorithms=True`
pins prefer_decomposed_solver=1 for this scene. `deterministic_plan.json` freezes
standalone233, mixed16, exact mixed16 repeat, and mixed64 with this supported flag.
The wrapper asserts and records actual dispatch before replay. Results pending;
this source explanation is a hypothesis until repeat controls verify it.

## Deterministic prefix checks and full-task export

All four deterministic cases finished. Mixed16 repeat is exact for every saved
array, and mixed16 versus64 states are exact for both233 and262. Warm throughput
single2335.26, mixed16 65.80, repeat16 74.54, mixed64 299.39 decisions/s under
concurrent host load. Standalone versus batched233 still differs up to2.090mm;
repeatability within the tested batch sizes is distinct from CPU/standalone
numerical equivalence. Actual prefer_decomposed_solver=1 is asserted and saved.

`bench_native_full_task_g14.py` adds observed wrist/tool poses, padded contact
counts, the unchanged picked guard, strict sequence and supplied120ms-cadence
metric at full source endpoints. Equal source lengths are required; no extra
scored holds. Metric effort is explicitly unavailable (NaN); this is diagnostic
export, not a training tape or independent EEF decoder verification.

The fullCPU233 control reproduces all962 physical states, contacts, picked flags,
tool poses and strict sequence exactly. It also passes the user metric.
`full_task_cpu_equivalence.json` records this. The first launcher exited143 after
the successful CPU result while waiting for the prefix batch; reason unknown.
Its successful results are retained. `run_full_task_resume.py` resumes only four
unstarted GPU cases after the exact prefix gate. Session95131 is live at this
snapshot. FullGPU outcome and repeatability results remain pending.

## Full episodes expose a consequential CPU/GPU mismatch

Both fullGPU233 runs are exact repeats, including physical states, wrist/tool,
contacts and picked flags, but fail the strict sequence and supplied arrival
metric. CPU completes. GPU tips during the post-release motion around23.7s and
ends214.73mm from the goal (CPU65.99mm); final CPU/GPU can-position difference
197.98mm. Goal movement51.60mm GPU versus9.86mm CPU. First can differences above
1/5/10/50mm occur6.06/19.89/22.71/23.70s. The divergence figure was inspected;
this locates the failure, not its numerical cause. FullGPU262 first run completes;
its repeated full run is pending. GPU results cannot replace the CPU panel.

`full233_repeat_equivalence.json`, `full_task_summary.json`, and
`full233_cpu_gpu_divergence.json` retain the exact-repeat and outcome evidence.
The full-native export costs are included in timing: first233 GPU16 throughput
75.68 aggregate decisions/s. No training tape or material adoption follows.

All five full-task cases are now terminal. Both full262 GPU runs also match
exactly, including diagnostic contact/tool/picked arrays, and complete with
65.994mm final center distance. Thus full GPU repeatability is demonstrated for
both tested sources, while task equivalence to CPU fails on233. This confirms
that throughput/repeatability cannot substitute for physical/task validation.
No further GPU material screening is launched from these results.
