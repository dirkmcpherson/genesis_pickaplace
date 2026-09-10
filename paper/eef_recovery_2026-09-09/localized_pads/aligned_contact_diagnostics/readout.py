"""Regenerate the exact-replay-verified, every-substep carry readout."""
from pathlib import Path
import json
import numpy as np
ROOT=Path(__file__).resolve().parent
run=ROOT/'113_soft'
plan=json.loads((ROOT/'plan.json').read_text())
z=np.load(run/'113_eef_delta.npz');ref=np.load(plan['reference'])
exact={k:bool(np.array_equal(z[k],ref[k])) for k in z.files}
assert all(exact.values()),exact
audit=json.loads((run/'contact_dynamics_audit.json').read_text())
stats=json.loads((run/'transmission_stats.json').read_text())
assert audit['substep_calls']==stats['substep_calls']==8+24*len(z['trajectory'])
assert audit['every_substep'] and audit['contact_window_s']==[7,16]
data=np.load(run/'contact_dynamics.npz');x=data['values'];c={k:i for i,k in enumerate(data['columns'])}
t=(x[:,0]-8)*.00125
assert t.min()>=7 and t.max()<=16 and np.isfinite(x).all()
mask=(((x[:,c['kind_a']]==1)&(x[:,c['kind_b']]==2))|
      ((x[:,c['kind_b']]==1)&(x[:,c['kind_a']]==2)))&(x[:,c['normal_force_N']]>.05)
rows=[]
for lo,hi in [(7,8),(8,9),(9,10),(10,11),(11,12),(12,16)]:
    v=x[mask&(t>=lo)&(t<hi)];row=dict(start=lo,end=hi,n=len(v))
    if len(v):
        row.update(Fn_p50_p95=np.quantile(v[:,c['normal_force_N']],[.5,.95]).tolist(),
                   util_p50_p95=np.quantile(v[:,c['pyramid_utilization']],[.5,.95]).tolist(),
                   saturated_frac=float(np.mean(v[:,c['pyramid_utilization']]>=.95)),
                   vt_mm_s_p50_p95=(1000*np.quantile(v[:,c['relative_tangent_speed_m_s']],[.5,.95])).tolist())
    rows.append(row)
out=dict(all_reference_arrays_exact=exact,contact_rows=len(x),loaded_finger_can_rows=int(mask.sum()),
         intervals=rows,qualification='Every substep in 7–16 s window. Pre-integration speed, not steady-state coefficient measurement. Loaded force cutoff 0.05 N is diagnostic only.')
(ROOT/'summary.json').write_text(json.dumps(out,indent=2))
print('Exact full replay verified;',int(mask.sum()),'loaded finger-can contact samples.')
