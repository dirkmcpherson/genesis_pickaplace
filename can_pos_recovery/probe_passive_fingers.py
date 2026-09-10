"""Isolated, uncalibrated distal-compliance experiment; never a bank collector.

Keep the existing base-joint actuation, contact model and joint limits. Remove
direct distal PD actuation and soften only the two distal mimic constraints.
This tests whether contact can produce independent motion; solver time constants
are NOT measured Kinova spring parameters.
"""
import argparse
import hashlib
import json
from pathlib import Path
import runpy
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(REPO/'baselines'), str(REPO/'can_pos_recovery')]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('source', type=Path)
    p.add_argument('--out', type=Path, required=True)
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--timeconst', type=float)
    mode.add_argument('--free-tip', action='store_true',
                      help='Diagnostic only: disable distal equalities and tip PD, with no return spring')
    a = p.parse_args()
    if not a.free_tip and a.timeconst not in (.1, .3):
        raise ValueError('Exploratory registered constants: 0.1 or 0.3 seconds')
    if a.out.exists():
        raise FileExistsError(a.out)
    a.out.mkdir(parents=True)
    import numpy as np
    import sim_variant_hook
    original = sim_variant_hook.apply_post
    treatment = dict(status='uncalibrated exploratory surrogate; not bank accepted',
                     distal_mimic_timeconst_s=a.timeconst, distal_kp=0., distal_kv=0.,
                     free_tip_without_return_spring=a.free_tip,
                     source=str(a.source.resolve()),
                     wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    def post(env, variant):
        result = original(env, variant)
        w = env.w
        robot, solver = w['kinova'], w['scene'].sim.rigid_solver
        names = {f'mimic_{side}_finger_tip_joint_to_right_finger_bottom_joint'
                 for side in ('left', 'right')}
        eqs = [e for e in solver.equalities if e.entity is robot and e.name in names]
        assert {e.name for e in eqs} == names
        params = solver.equality_info.sol_params.to_numpy()
        before = params.copy()
        for eq in eqs:
            assert np.allclose(eq.eq_data[:2], [.149, -.676])
            if not a.free_tip:
                params[eq.idx, :, 0] = a.timeconst
        solver.equality_info.sol_params.from_numpy(params)
        actual = solver.equality_info.sol_params.to_numpy()
        assert np.array_equal(actual, params)
        unchanged = [i for i in range(len(params)) if i not in {e.idx for e in eqs}]
        assert np.array_equal(actual[unchanged], before[unchanged])
        if a.free_tip:
            # This pinned solver dispatches CONNECT/WELD/JOINT explicitly and
            # skips other types. Keep all indices and the proximal equality.
            types = solver.equality_info.eq_type.to_numpy()
            original_types = types.copy()
            for eq in eqs:
                types[eq.idx, :] = -1
            solver.equality_info.eq_type.from_numpy(types)
            readback = solver.equality_info.eq_type.to_numpy()
            assert np.array_equal(readback, types)
            assert np.array_equal(readback[unchanged], original_types[unchanged])
            treatment.update(equality_types_before=original_types.tolist(),
                             equality_types_after=readback.tolist())
        tips = [robot.get_joint(f'{side}_finger_tip_joint').dof_idx_local
                for side in ('left', 'right')]
        robot.set_dofs_kp(np.zeros(2), dofs_idx_local=tips)
        robot.set_dofs_kv(np.zeros(2), dofs_idx_local=tips)
        # This pinned engine's non-batched gain getter misroutes index masks;
        # read the same solver fields used by its PD kernel directly.
        global_tips = np.asarray(tips) + robot.dof_start
        kp = solver.dofs_info.kp.to_numpy()[global_tips]
        kv = solver.dofs_info.kv.to_numpy()[global_tips]
        assert np.all(kp == 0) and np.all(kv == 0)
        treatment.update(equality_names=sorted(names), equality_indices=[e.idx for e in eqs],
                         equality_before=before.tolist(), equality_after=actual.tolist(),
                         distal_dof_indices=tips, distal_kp_readback=kp.tolist(),
                         distal_kv_readback=kv.tolist())
        (a.out/'treatment.json').write_text(json.dumps(treatment, indent=2))
        return result

    sim_variant_hook.apply_post = post
    sys.argv = ['repair_eef_slide.py', str(a.source), '--out', str(a.out),
                '--max-extension', '0', '--polish-ik']
    runpy.run_path(str(REPO/'can_pos_recovery/repair_eef_slide.py'), run_name='__main__')
    for path in a.out.glob('*_eef_delta.json'):
        meta = json.loads(path.read_text())
        meta['physics_treatment'] = treatment
        meta['provenance'] = 'experimental distal-compliance replay; not a validated real hand model'
        path.write_text(json.dumps(meta, indent=2))


if __name__ == '__main__':
    main()
