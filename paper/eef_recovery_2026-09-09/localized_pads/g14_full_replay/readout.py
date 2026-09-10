"""Validate and score completed full-world engine-port controls, retaining failures."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
plan=json.loads((ROOT/'plan_port3.json').read_text())
for filename,sha in plan['source_code_sha256'].items():
 source=ROOT/plan['executed_source_archive']/Path(filename).name
 assert hashlib.sha256(source.read_bytes()).hexdigest()==sha
execution=json.loads((ROOT/'execution_port3.json').read_text())
ref=np.load(ROOT.parent/'finger_inertia_alignment/233_tc0.02/233_eef_delta.npz')
rows=[]
for entry in execution:
 name=entry['name'];folder=ROOT/name
 if entry['returncode']:
  rows.append(dict(name=name,error='Execution failed; not scored'));continue
 path=folder/'233_eef_delta.npz';z=np.load(path);n=len(z['source_grip'])
 meta=json.loads(path.with_suffix('.json').read_text());audit=json.loads((folder/'transmission_audit.json').read_text())
 assert n==962 and meta['extension_m']==0 and not meta['hold_contact']
 diffs={k:float(np.max(abs(z[k]-ref[k]))) for k in ['target_tool','actions_eef','actions_joint','mount','source_grip']}
 np.testing.assert_allclose(z['target_tool'],ref['target_tool'],atol=1e-12)
 np.testing.assert_allclose(z['actions_joint'],ref['actions_joint'],atol=1e-8)
 np.testing.assert_array_equal(z['source_grip'],ref['source_grip'])
 np.testing.assert_array_equal(z['mount'],ref['mount'])
 stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n
 counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
 tc=audit['surface_pad_treatment']['pad_timeconst_s']
 assert counts[:,0].sum()>0,'No classified material-region contacts'
 if tc==.02:assert counts[:,1].sum()==0
 else:assert counts[:,1].sum()>0,'Soft treatment never applied'
 assert hashlib.sha256((folder/'gen3_lite_2f_g14_candidate.urdf').read_bytes()).hexdigest()==audit['urdf']['candidate_sha256']
 rigid_noop=None
 if name=='233_pyramid_rigid_port3':
  old=np.load(ROOT/'233_pyramid_rigid_port2/233_eef_delta.npz')
  rigid_noop={k:bool(np.array_equal(z[k],old[k])) for k in z.files};assert all(rigid_noop.values())
 shift=np.linalg.norm(z['goal_pose'][:,:2]-z['goal_pose'][0,:2],axis=1)*1000
 tool_error=np.linalg.norm(z['trajectory'][:,6:9]-z['target_tool'][:,:3,3],axis=1)*1000
 row=dict(name=name,sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],
  source_command_max_differences=diffs,rigid_material_mapping_noop_exact=rigid_noop,
  material_contact_count_sums=counts.sum(axis=0).tolist(),can_mass_kg=audit['can_mass_kg'],
  final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),
  measured_tool_target_error_mm_p50_p95_max=np.percentile(tool_error,[50,95,100]).tolist(),
  max_abs_joint_velocity_rad_s=stats['max_abs_velocity'])
 (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
 print(name,row['sequence']['reason'],row['metric']['slide_success'],round(row['metric']['final_dist']*1000,2),'counts',row['material_contact_count_sums'])
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows,qualification='One demo, four conditions. Not population validation or hardware calibration. Same-engine contrasts retain processed geometry and analytic mass; cross-engine comparison also changes these.'),indent=2))
