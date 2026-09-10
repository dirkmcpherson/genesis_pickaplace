"""Run an isolated hand bench or full replay with audited finger inertia signs."""
from pathlib import Path
import argparse, hashlib, json, runpy, sys
import numpy as np
import adaptive_gripper_candidate as adaptive
from align_finger_inertia import align, AUDIT

REPO = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__, add_help=False)
p.add_argument('--bench', action='store_true')
p.add_argument('--original-inertia', action='store_true')
args, rest = p.parse_known_args()
out = Path(rest[rest.index('--out') + 1])
reference = json.loads(AUDIT.read_text())
original_make, original_install = adaptive.make_urdf, adaptive.install

def make(path, **kwargs):
    result = original_make(path, **kwargs)
    if not args.original_inertia:
        correction = align(path)
        (out / 'inertia_alignment.json').write_text(json.dumps(correction, indent=2))
        result['finger_inertia_alignment'] = correction
        result['candidate_sha256'] = correction['after_sha256']
    return result

def install(world, *a, **kw):
    robot = world['kinova']
    solver = world['scene'].sim.rigid_solver
    pos = solver.links_info.inertial_pos.to_numpy()
    inertia = solver.links_info.inertial_i.to_numpy()
    mass = solver.links_info.inertial_mass.to_numpy()
    quat = solver.links_info.inertial_quat.to_numpy()
    rows = []
    for row in reference['records']:
        link = robot.get_link(row['link'])
        idx = link.idx
        expected_pos = row['published_com' if args.original_inertia else 'proposed_com']
        expected_i = row['published_inertia' if args.original_inertia else 'proposed_inertia']
        np.testing.assert_allclose(pos[idx], expected_pos, rtol=1e-6, atol=1e-10)
        np.testing.assert_allclose(inertia[idx], expected_i, rtol=1e-6, atol=1e-12)
        np.testing.assert_allclose(mass[idx], row['mass_kg'], rtol=1e-6)
        np.testing.assert_allclose(quat[idx], [1, 0, 0, 0], atol=1e-7)
        rows.append(dict(link=row['link'], com=pos[idx].tolist(),
                         inertia=inertia[idx].tolist(), mass_kg=float(mass[idx]),
                         inertial_quat_wxyz=quat[idx].tolist()))
    readback = dict(original_inertia=args.original_inertia, records=rows,
                    audit_sha256=hashlib.sha256(AUDIT.read_bytes()).hexdigest(),
                    all_runtime_assertions_passed=True)
    (out / 'inertia_runtime_readback.json').write_text(json.dumps(readback, indent=2))
    state, audit = original_install(world, *a, **kw)
    audit['finger_inertia_readback'] = readback
    return state, audit

adaptive.make_urdf, adaptive.install = make, install
entry = 'bench_unloaded_hand.py' if args.bench else 'run_hand_preset.py'
sys.argv = [entry] + rest
runpy.run_path(str(REPO / 'can_pos_recovery' / entry), run_name='__main__')
if args.bench:
    report_path = out / 'report.json'
    report = json.loads(report_path.read_text())
    report['bench'] = ('Hand only, fixed base, collisions disabled, zero gravity; '
                       + ('published' if args.original_inertia else 'mesh-sign-aligned')
                       + ' finger inertias. Not a hardware mass calibration.')
    report_path.write_text(json.dumps(report, indent=2))
