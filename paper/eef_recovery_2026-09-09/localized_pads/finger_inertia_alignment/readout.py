"""Score only terminal inertia-alignment runs and retain the original controls."""
from pathlib import Path
import hashlib, json, sys
import numpy as np
ROOT=Path(__file__).resolve().parent; REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
rows=[]
for f in sorted((ROOT/'logs').glob('*_execution.json')):
    st=json.loads(f.read_text());uid=st['uid'];folder=ROOT/st['name']
    if st['returncode']:
        rows.append(dict(**st,error='Execution failed; not scored'));continue
    path=folder/f'{uid}_eef_delta.npz';z=np.load(path)
    ref=ROOT.parent/f'critical_return/{uid}_tc{st["tc"]:g}/{uid}_eef_delta.npz';b=np.load(ref)
    equality={k:bool(np.array_equal(z[k],b[k])) for k in z.files}
    assert all(equality[k] for k in ['actions_eef','actions_joint','source_grip','mount'])
    if st['original']:assert all(equality.values()),equality
    meta=json.loads(path.with_suffix('.json').read_text())
    runtime=json.loads((folder/'inertia_runtime_readback.json').read_text())
    assert runtime['all_runtime_assertions_passed'] and runtime['original_inertia']==st['original']
    urdf=folder/'gen3_lite_2f_adaptive_candidate.urdf'
    urdf_sha=hashlib.sha256(urdf.read_bytes()).hexdigest()
    if not st['original']:
        alignment=json.loads((folder/'inertia_alignment.json').read_text())
        assert urdf_sha==alignment['after_sha256']
    n=len(z['actions_eef']);stats=json.loads((folder/'transmission_stats.json').read_text())
    assert stats['substep_calls']==8+24*n
    counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
    if st['tc']>.02:assert counts[:,1].sum()>0
    else:assert counts[:,1].sum()==0
    goal=z['goal_pose'];shift=np.linalg.norm(goal[:,:2]-goal[0,:2],axis=1)*1000
    row=dict(name=st['name'],uid=uid,tc=st['tc'],original=st['original'],reference=str(ref),reference_arrays_exact=equality,
             sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],
             final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),
             realized_urdf_sha256=urdf_sha,
             urdf_hash_qualification='Initial wrapper retains the pre-alignment factory hash in the outer URDF metadata; nested inertia_alignment.after_sha256 and this checked realized hash identify the actual loaded file.',
             max_abs_joint_velocity_rad_s=stats['max_abs_velocity'])
    (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows),indent=2))
for r in rows:print(r['name'],r.get('error') or (r['sequence']['reason'],r['metric']['slide_success'],round(r['metric']['final_dist']*1000,2),round(r['final_goal_shift_mm'],2)))
