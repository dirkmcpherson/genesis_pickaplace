"""Check whether removing the observed CG cap exhaustion closes the CPU/GPU task gap."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import time
import numpy as np

D = Path(__file__).resolve().parent
P = D.parent
R = D.parents[3]
assert not (D / 'plan.json').exists()
prior = json.loads((P / 'constraint_iterations/summary.json').read_text())
assert prior['records']['1000']['intervals'][0]['improved_still_true'] == 0
assert all(json.loads((P / 'constraint_iterations/source_identity.json').read_text()).values())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
files = ['can_pos_recovery/bench_native_iterations_g14.py', 'can_pos_recovery/bench_native_full_task_g14.py',
         'can_pos_recovery/bench_native_batch_g14_v2.py', 'can_pos_recovery/batched_surface_pad_candidate_g14.py',
         'can_pos_recovery/adaptive_gripper_candidate.py', 'can_pos_recovery/eef_task_sequence.py',
         'can_pos_recovery/slide_predicate.py', 'can_pos_recovery/replay_harness.py']
(D / 'executed_sources').mkdir()
for f in files:
    shutil.copy2(R / f, D / 'executed_sources' / Path(f).name)
trace = P / 'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
reference = P / 'constraint_iterations/233_cg1000/233_eef_delta.npz'
preset = P / 'proximal_compliance/presets/all_soft_control.json'
plan = dict(trace=str(trace), reference=str(reference), preset=str(preset),
            trace_sha256=sha(trace), reference_sha256=sha(reference), preset_sha256=sha(preset),
            source_code_sha256={f: sha(R / f) for f in files},
            cases=[dict(name='cpu233_cg1000', backend='cpu', n_envs=0),
                   dict(name='gpu16_233_cg1000', backend='gpu', n_envs=16)],
            purpose='Matched numerical-cap diagnostic; unchanged soft233 physics/source. '
            'CPU native port must exactly match the independently executed cap1000 trace '
            'before GPU full replay starts. Not a material or training-tape adoption. '
            'One unique recording;16 copied lanes do not broaden the sample.')
(D / 'plan.json').write_text(json.dumps(plan, indent=2))
rows = []
for case in plan['cases']:
    for f, h in plan['source_code_sha256'].items():
        assert sha(R / f) == h
    env = dict(os.environ, OPENBLAS_NUM_THREADS='1', QD_NUM_THREADS='1', OMP_NUM_THREADS='1',
               MPLCONFIGDIR='/tmp/eef-recovery-mpl')
    if case['backend'] == 'gpu':
        env['PYTHONPATH'] = '/tmp/genesis-cuda28'
    folder = D / case['name']
    cmd = ['/tmp/genesis-contact-1.4/bin/python', 'can_pos_recovery/bench_native_iterations_g14.py',
           '--trace', str(trace), '--preset', str(preset), '--out', str(folder),
           '--backend', case['backend'], '--n-envs', str(case['n_envs']), '--steps', '962',
           '--iterations', '1000']
    print('START', case['name'], flush=True)
    start = time.time()
    with (D / (case['name'] + '.log')).open('w') as log:
        result = subprocess.run(cmd, cwd=R, env=env, stdout=log, stderr=subprocess.STDOUT)
    rows.append(dict(**case, cmd=cmd, returncode=result.returncode, elapsed_s=time.time() - start))
    (D / 'execution.json').write_text(json.dumps(rows, indent=2))
    print('DONE', case['name'], result.returncode, flush=True)
    assert result.returncode == 0
    if case['backend'] == 'cpu':
        a = np.load(folder / 'trace.npz')
        b = np.load(reference)
        d = np.load(folder / 'full_task_diagnostic.npz')
        physical = np.c_[b['trajectory'][:, :6], b['finger_joint'], b['trajectory'][:, 13:20], b['goal_pose']]
        checks = dict(physical_states_exact=bool(np.array_equal(a['state'][:, 0], physical)),
                      contacts_exact=bool(np.array_equal(d['contact_counts'][:, 0], b['contact_counts'])),
                      picked_exact=bool(np.array_equal(d['picked'][:, 0], b['stages'][:, 0])),
                      tool_exact=bool(np.array_equal(d['trajectory'][:, 0, 6:13], b['trajectory'][:, 6:13])))
        (D / 'cpu_identity.json').write_text(json.dumps(checks, indent=2))
        assert all(checks.values()), checks
print('NATIVE_ITERATIONS_TERMINAL', flush=True)
