"""Frozen observation-only solver audit, gated by exact full-replay identity."""
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
(D / 'executed_sources').mkdir()
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
files = ['can_pos_recovery/audit_g14_constraint_convergence.py',
         'can_pos_recovery/run_g14_hand.py', 'can_pos_recovery/surface_pad_candidate_g14.py',
         'can_pos_recovery/adaptive_gripper_candidate.py', 'can_pos_recovery/repair_eef_slide.py']
for f in files:
    shutil.copy2(R / f, D / 'executed_sources' / Path(f).name)
source = P.parent / 'timestamp_full_pool/233/source.npz'
preset = P / 'proximal_compliance/presets/all_soft_control.json'
reference = P / 'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'
plan = dict(source=str(source), preset=str(preset), reference=str(reference),
            source_sha256=sha(source), preset_sha256=sha(preset),
            reference_sha256=sha(reference), source_metadata_sha256=sha(source.with_suffix('.json')),
            source_code_sha256={f: sha(R / f) for f in files},
            purpose='Observe every constraint-force callback under unchanged CG100, float32, '
            '8 substeps and physical/source settings. No material or numerical change. '
            'All14 archived arrays must reproduce exactly before interpreting the diagnostic.')
(D / 'plan.json').write_text(json.dumps(plan, indent=2))
out = D / '233_observed'
cmd = ['/tmp/genesis-contact-1.4/bin/python', 'can_pos_recovery/audit_g14_constraint_convergence.py',
       str(source), '--out', str(out), '--preset', str(preset)]
env = dict(os.environ, OPENBLAS_NUM_THREADS='1', QD_NUM_THREADS='1', OMP_NUM_THREADS='1',
           MPLCONFIGDIR='/tmp/eef-recovery-mpl')
start = time.time()
with (D / '233_observed.log').open('w') as log:
    result = subprocess.run(cmd, cwd=R, env=env, stdout=log, stderr=subprocess.STDOUT)
(D / 'execution.json').write_text(json.dumps(dict(cmd=cmd, returncode=result.returncode,
                                                elapsed_s=time.time() - start), indent=2))
assert result.returncode == 0
a = np.load(out / '233_eef_delta.npz')
b = np.load(reference)
assert a.files == b.files
identity = {k: bool(np.array_equal(a[k], b[k])) for k in a.files}
(D / 'identity.json').write_text(json.dumps(identity, indent=2))
assert all(identity.values()), identity
print('OBSERVATION_IDENTITY_PASSED', flush=True)
