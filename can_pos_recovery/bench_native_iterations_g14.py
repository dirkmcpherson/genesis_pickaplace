"""Native full-task benchmark with an explicit numerical iteration-cap override."""
from pathlib import Path
import json
import runpy
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'baselines'), str(ROOT / 'can_pos_recovery')]
import replay_harness as h
from genesis.utils.misc import qd_to_numpy

index = sys.argv.index('--iterations')
cap = int(sys.argv[index + 1])
assert cap == 1000
del sys.argv[index:index + 2]
out = Path(sys.argv[sys.argv.index('--out') + 1])
original_build = h.build_world
runtime = {}


def build(*args, **kwargs):
    kwargs['rigid_extra'] = dict(kwargs['rigid_extra'], iterations=cap)
    world = original_build(*args, **kwargs)
    solver = world['scene'].sim.rigid_solver
    assert int(qd_to_numpy(solver.rigid_info.iterations)) == cap
    runtime.update(options={k: str(v) for k, v in kwargs['rigid_extra'].items()},
                   actual_iterations=int(qd_to_numpy(solver.rigid_info.iterations)),
                   substep_dt_s=solver._substep_dt,
                   qualification='Only numerical iteration cap changed from100 to1000. '
                   'The inner benchmark engine_options snapshot predates this override; '
                   'these actual build options are authoritative. No physical/source change.')
    return world


h.build_world = build
try:
    runpy.run_path(str(ROOT / 'can_pos_recovery/bench_native_full_task_g14.py'), run_name='__main__')
finally:
    if out.exists():
        (out / 'iteration_runtime.json').write_text(json.dumps(runtime, indent=2))
