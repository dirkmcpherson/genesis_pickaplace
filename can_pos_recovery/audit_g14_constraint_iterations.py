"""Observe a declared solver iteration-cap refinement with physical/source settings fixed."""
from pathlib import Path
import argparse
import json
import runpy
import sys

import numpy as np
import genesis as gs
from genesis.utils.misc import qd_to_numpy

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'baselines'), str(ROOT / 'can_pos_recovery')]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path)
parser.add_argument('--out', type=Path, required=True)
parser.add_argument('--preset', type=Path, required=True)
parser.add_argument('--iterations', type=int, choices=[100, 1000], required=True)
args = parser.parse_args()

import genesis_can_env

build = genesis_can_env.build_world
rows = []
runtime = {}


def observed_world(*positional, **kwargs):
    kwargs['rigid_extra'] = dict(kwargs['rigid_extra'], iterations=args.iterations)
    world = build(*positional, **kwargs)
    solver = world['scene'].sim.rigid_solver
    cs = solver.constraint_solver.constraint_state
    ri = solver.rigid_info
    runtime.update(
        engine_version=gs.__version__,
        options={k: str(v) for k, v in kwargs['rigid_extra'].items()},
        substep_dt_s=solver._substep_dt,
        n_dofs=solver.n_dofs,
        prefer_decomposed_solver=solver.rigid_config.prefer_decomposed_solver,
        gradient_dtype=str(qd_to_numpy(cs.grad).dtype),
        qualification='Observation only. Read after every constraint-force call. '
        'An above-tolerance gradient alone is not proof of a failed solve: '
        'the engine also permits positive cost-improvement termination. '
        'The improved flag is retained directly; no engine field is modified.',
    )
    original = solver._func_constraint_force

    def observe(*a, **kw):
        result = original(*a, **kw)
        grad = qd_to_numpy(cs.grad)[:, 0].astype(np.float64)
        mgrad = qd_to_numpy(cs.Mgrad)[:, 0].astype(np.float64)
        threshold = (float(qd_to_numpy(ri.meaninertia)[0]) * max(1, solver.n_dofs)
                     * float(qd_to_numpy(ri.tolerance)))
        rows.append([
            len(rows), int(qd_to_numpy(cs.n_constraints)[0]),
            float(np.linalg.norm(grad)), float(.5 * (grad @ mgrad)),
            float(qd_to_numpy(cs.ls_improvement)[0]), threshold,
            bool(qd_to_numpy(cs.improved)[0]),
        ])
        return result

    solver._func_constraint_force = observe
    return world


genesis_can_env.build_world = observed_world
sys.argv = ['run_g14_hand.py', str(args.source), '--out', str(args.out),
            '--preset', str(args.preset), '--cone', 'elliptic', '--impratio', '10']
try:
    runpy.run_path(str(ROOT / 'can_pos_recovery/run_g14_hand.py'), run_name='__main__')
finally:
    if args.out.exists():
        (args.out / 'constraint_runtime.json').write_text(json.dumps(runtime, indent=2))
        np.savez_compressed(
            args.out / 'constraint_convergence.npz', values=np.asarray(rows).reshape(-1, 7),
            columns=np.array(['callback_index', 'n_constraints', 'gradient_norm',
                              'half_grad_dot_Mgrad', 'cost_improvement',
                              'scaled_tolerance', 'improved']))
