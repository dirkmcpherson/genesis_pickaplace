"""Independent EEF replay with built-world checks for the disclosed IC correction."""
import hashlib
import json
from pathlib import Path
import runpy
import sys

import numpy as np

root = Path(__file__).resolve().parent
repo = root.parents[2]
sys.path[:0] = [str(repo / 'baselines'), str(repo / 'can_pos_recovery')]
import sim_variant_hook
from score_recovered_slides import adapt

original_post = sim_variant_hook.apply_post


def audited_post(env, variant):
    result = original_post(env, variant)
    from replay_harness import BOX_SIZE
    w = env.w
    robot = w['kinova']
    def array(value):
        return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)
    q = array(robot.get_quat()).reshape(-1)
    yaw = float(np.rad2deg(2*np.arctan2(q[3],q[0])))
    shelf = next(e for e in w['scene'].entities if e.morph.__class__.__name__=='Box' and np.allclose(e.morph.size, BOX_SIZE))
    shelf_position = array(shelf.get_pos()).reshape(-1)
    indices = np.asarray(w['kdofs'][:6]) + robot.dof_start
    kp = w['scene'].sim.rigid_solver.dofs_info.kp.to_numpy()[indices].reshape(-1)
    assert abs(yaw + 19.2) < .01
    assert np.allclose(shelf_position, [.75, -.1875, .11], atol=1e-6)
    assert np.allclose(kp, [800,800,600,400,240,240])
    assert abs(float(array(robot.get_pos()).reshape(-1)[2]) - .08) < 1e-6
    assert abs(w['goal_start_z'] - .263) < 1e-6
    (root / 'verification_world_audit.json').write_text(json.dumps(dict(
        built_yaw_deg=yaw, shelf_position=shelf_position.tolist(), arm_kp=kp.tolist(),
        goal_start_z=w['goal_start_z'], full_post_hook=True,
        wrapper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()), indent=2))
    return result


sim_variant_hook.apply_post = audited_post
source = root / 'collection/193_eef_delta.npz'
output = root / 'metric_verification'
sys.argv = ['repair_eef_slide.py', str(source), '--out', str(output), '--max-extension', '0', '--verify-actions']
runpy.run_path(str(repo / 'can_pos_recovery/repair_eef_slide.py'), run_name='__main__')
checked = output / source.name
with np.load(source) as a, np.load(checked) as b:
    comparison = {k: bool(np.array_equal(a[k],b[k])) for k in ('trajectory','actions_eef','observations')}
assert all(comparison.values()), comparison
actual = adapt(checked, output / 'metric_adapter')['metric']
expected = json.loads((root / 'metric_result.json').read_text())['metric']
assert actual == expected and actual['slide_success']
(root / 'metric_verification_execution.json').write_text(json.dumps(dict(
    uid=193, returncode=0, verified=True, array_comparison=comparison, metric=actual), indent=2))
print('VERIFIED corrected 193', flush=True)
