"""Terminal critical-return comparisons against the preceding low-inertia hand."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
rows=[]
for f in sorted((ROOT/'logs').glob('*_execution.json')):
 status=json.loads(f.read_text());uid=status['uid'];folder=ROOT/status['name']
 if status['returncode']:
  rows.append(dict(**status,error='Execution failed; not scored'));continue
 path=folder/f'{uid}_eef_delta.npz';z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());preset=meta['physics_treatment']['declared_preset']['configuration'];tc=preset['pad_timeconst_s']
 reference=ROOT.parent/f'low_armature/{uid}_tc{tc:g}/{uid}_eef_delta.npz';base=np.load(reference)
 identity={k:bool(np.array_equal(z[k],base[k])) for k in z.files}
 assert all(identity[k] for k in ['actions_eef','actions_joint','source_grip','mount'])
 if status['preset']=='default_control':assert all(identity.values()),identity
 n=len(z['actions_eef']);stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n
 contact=np.load(folder/'contact_observations.npz')['values'];assert contact.shape==(1+3*n,3)
 counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
 if tc>.02:assert counts[:,1].sum()>0
 tr=z['trajectory'];cc=z['contact_counts'];first=np.flatnonzero(cc[:,1]>0);pre_error=None
 if len(first) and first[0]>0:
  end=first[0];g=z['source_grip'][:end];theta=.96-1.05*np.clip(g/100,0,1);expected=np.c_[-theta,theta,.149-.676*theta,.149-.676*theta]
  pre_error=float(np.rad2deg(abs(z['finger_joint'][:end]-expected)).max())
 row=dict(uid=uid,name=status['name'],preset=preset,reference=str(reference),reference_arrays_exact=identity,armature=meta['physics_treatment']['finger_armature'],sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],max_abs_joint_velocity_rad_s=stats['max_abs_velocity'],max_abs_generalized_torque=stats['max_abs_torque'],before_first_can_contact_max_unloaded_joint_error_deg=pre_error,
 qualification='Pre-can-contact interval may include other-world contacts; the separate isolated bench is the causal unloaded test.')
 (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows),indent=2))
for r in rows:print(r['name'],r.get('error') or (r['sequence']['reason'],r['metric']['slide_success'],round(r['metric']['final_dist']*1000,2),r['max_abs_joint_velocity_rad_s']))
