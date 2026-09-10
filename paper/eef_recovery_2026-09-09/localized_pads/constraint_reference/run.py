"""Compare higher-accuracy CG/Newton solves after a float32 wrapper identity gate."""
from concurrent.futures import ThreadPoolExecutor, as_completed
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
assert json.loads((P / 'constraint_resolution/summary.json').read_text())['records']['16']['improved_still_true'] == 0
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
files = ['can_pos_recovery/audit_g14_constraint_reference.py', 'can_pos_recovery/run_g14_hand.py',
         'can_pos_recovery/surface_pad_candidate_g14.py', 'can_pos_recovery/adaptive_gripper_candidate.py',
         'can_pos_recovery/repair_eef_slide.py', 'can_pos_recovery/align_finger_inertia.py']
(D / 'executed_sources').mkdir()
for f in files:
    shutil.copy2(R / f, D / 'executed_sources' / Path(f).name)
source = P.parent / 'timestamp_full_pool/233/source.npz'
preset = P / 'proximal_compliance/presets/all_soft_control.json'
reference = P / 'constraint_iterations/233_cg1000/233_eef_delta.npz'
cases = [dict(name='cg32_control', precision='32', solver='CG'),
         dict(name='cg64_reference', precision='64', solver='CG'),
         dict(name='newton64_reference', precision='64', solver='Newton')]
plan = dict(source=str(source), preset=str(preset), reference=str(reference),
            source_sha256=sha(source), source_metadata_sha256=sha(source.with_suffix('.json')),
            preset_sha256=sha(preset), reference_sha256=sha(reference),
            source_code_sha256={f: sha(R / f) for f in files}, cases=cases,
            numerical_settings='1000 iterations,8 substeps, fixed scene/control intervals. '
            'Float32 control uses tolerance1e-5; both float64 references use1e-8. '
            'Compare CG/Newton agreement at matching high accuracy, not only whether either passes.',
            physical_settings='Same intended source commands, endpoint, poses, hand, materials and geometry. '
            'Floating-point readbacks may differ with precision; report differences and retain strict physical checks.',
            gate='Float32 control must reproduce all14 archived CG1000 arrays exactly before64-bit cases start.',
            qualification='Numerical reference diagnostic on one selected recording, not a material adoption or validation sample.')
(D / 'plan.json').write_text(json.dumps(plan, indent=2))
env = dict(os.environ, OPENBLAS_NUM_THREADS='1', QD_NUM_THREADS='1', OMP_NUM_THREADS='1',
           MPLCONFIGDIR='/tmp/eef-recovery-mpl')


def run(case):
    for f, h in plan['source_code_sha256'].items():
        assert sha(R / f) == h
    out = D / case['name']
    cmd = ['/tmp/genesis-contact-1.4/bin/python', 'can_pos_recovery/audit_g14_constraint_reference.py',
           str(source), '--out', str(out), '--preset', str(preset), '--iterations', '1000',
           '--precision', case['precision'], '--solver', case['solver']]
    print('START', case['name'], flush=True)
    start = time.time()
    with (D / (case['name'] + '.log')).open('w') as log:
        result = subprocess.run(cmd, cwd=R, env=env, stdout=log, stderr=subprocess.STDOUT)
    row = dict(**case, cmd=cmd, returncode=result.returncode, elapsed_s=time.time() - start)
    (D / (case['name'] + '_execution.json')).write_text(json.dumps(row, indent=2))
    print('DONE', case['name'], result.returncode, flush=True)
    return row


rows = [run(cases[0])]
(D / 'execution.json').write_text(json.dumps(rows, indent=2))
assert rows[0]['returncode'] == 0
a = np.load(D / cases[0]['name'] / '233_eef_delta.npz')
b = np.load(reference)
assert a.files == b.files
identity = {k: bool(np.array_equal(a[k], b[k])) for k in a.files}
(D / 'identity.json').write_text(json.dumps(identity, indent=2))
assert all(identity.values()), identity
with ThreadPoolExecutor(max_workers=2) as pool:
    for future in as_completed([pool.submit(run, c) for c in cases[1:]]):
        rows.append(future.result())
        (D / 'execution.json').write_text(json.dumps(rows, indent=2))
print('REFERENCE_TERMINAL', len(rows), flush=True)
