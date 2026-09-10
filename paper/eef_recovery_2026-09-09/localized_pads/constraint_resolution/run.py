"""Fixed CG1000 timestep refinement after eliminating observed cap exhaustion."""
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
assert all(json.loads((P / 'constraint_iterations/source_identity.json').read_text()).values())
assert json.loads((P / 'constraint_iterations/summary.json').read_text())['records']['1000']['intervals'][0]['improved_still_true'] == 0
(D / 'executed_sources').mkdir()
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
files = ['can_pos_recovery/audit_g14_constraint_resolution.py',
         'can_pos_recovery/run_g14_hand_resolution_v2.py', 'can_pos_recovery/run_g14_hand.py', 'can_pos_recovery/surface_pad_candidate_g14.py',
         'can_pos_recovery/adaptive_gripper_candidate.py', 'can_pos_recovery/repair_eef_slide.py']
for f in files:
    shutil.copy2(R / f, D / 'executed_sources' / Path(f).name)
source = P.parent / 'timestamp_full_pool/233/source.npz'
preset = P / 'proximal_compliance/presets/all_soft_control.json'
reference = P / 'constraint_iterations/233_cg1000/233_eef_delta.npz'
plan = dict(source=str(source), preset=str(preset), reference=str(reference),
            source_sha256=sha(source), preset_sha256=sha(preset),
            reference_sha256=sha(reference), source_metadata_sha256=sha(source.with_suffix('.json')),
            source_code_sha256={f: sha(R / f) for f in files},
            purpose='At fixed CG1000, halve the integration interval using16 substeps. '
            'Source/physical settings and control endpoint fixed. Compare with8-substep cap1000 '
            'to test whether removing cap exhaustion also reduces timestep sensitivity. '
            'Not a material candidate; no numerical setting selected by task yield.')
(D / 'plan.json').write_text(json.dumps(plan, indent=2))
out = D / '233_cg1000_ss16'
cmd = ['/tmp/genesis-contact-1.4/bin/python', 'can_pos_recovery/audit_g14_constraint_resolution.py',
       str(source), '--out', str(out), '--preset', str(preset), '--iterations', '1000', '--substeps', '16']
env = dict(os.environ, OPENBLAS_NUM_THREADS='1', QD_NUM_THREADS='1', OMP_NUM_THREADS='1',
           MPLCONFIGDIR='/tmp/eef-recovery-mpl')
start = time.time()
with (D / '233_cg1000_ss16.log').open('w') as log:
    result = subprocess.run(cmd, cwd=R, env=env, stdout=log, stderr=subprocess.STDOUT)
(D / 'execution.json').write_text(json.dumps(dict(cmd=cmd, returncode=result.returncode,
                                                elapsed_s=time.time() - start), indent=2))
assert result.returncode == 0
a = np.load(out / '233_eef_delta.npz')
b = np.load(reference)
assert a.files == b.files
fixed = {k: bool(np.array_equal(a[k], b[k])) for k in ['source_grip', 'mount', 'source_frame_index', 'action_kind']}
np.testing.assert_allclose(a['actions_joint'], b['actions_joint'], rtol=0, atol=1e-8)
np.testing.assert_allclose(a['target_tool'], b['target_tool'], rtol=0, atol=1e-12)
(D / 'source_identity.json').write_text(json.dumps(fixed, indent=2))
assert all(fixed.values()), fixed
print('ITERATION_REFINEMENT_COMPLETE', flush=True)
