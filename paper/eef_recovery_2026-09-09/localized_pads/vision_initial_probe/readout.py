"""Retain all conditional outcomes, with unchanged commands and altered-pose provenance."""
from pathlib import Path
import hashlib,json,sys
import numpy as np
R=Path(__file__).resolve().parents[4];D=Path(__file__).resolve().parent;P=D.parent
sys.path.insert(0,str(R/'can_pos_recovery'))
from score_recovered_slides import adapt
plan=json.loads((D/'plan.json').read_text());source=Path(plan['source'])
assert hashlib.sha256(source.read_bytes()).hexdigest()==plan['source_npz_sha256']
assert hashlib.sha256(source.with_suffix('.json').read_bytes()).hexdigest()==plan['source_metadata_sha256']
for f,sha in plan['source_code_sha256'].items():assert hashlib.sha256((D/'executed_sources'/Path(f).name).read_bytes()).hexdigest()==sha
baseline=np.load(R/'paper/eef_recovery_2026-09-09/early_yaw_pool/113/collection/113_eef_delta.npz')
rows=[]
for job in plan['jobs']:
    condition=job['condition'];status=D/'logs'/f'{condition}_execution.json'
    if not status.exists():rows.append(dict(condition=condition,status='pending'));continue
    execution=json.loads(status.read_text())
    if execution['returncode']:
        rows.append(dict(condition=condition,status='execution_failed',execution=execution));continue
    folder=D/condition;path=folder/'113_eef_delta.npz';z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());n=len(z['trajectory'])
    assert n==len(baseline['trajectory']) and meta['can_pos']==plan['conditional_can_pos']
    assert meta['extension_m']==0 and not meta['hold_contact']
    for key in ['source_grip','mount']:np.testing.assert_array_equal(z[key],baseline[key])
    np.testing.assert_allclose(z['target_tool'],baseline['target_tool'],atol=1e-12)
    np.testing.assert_allclose(z['actions_joint'],baseline['actions_joint'],atol=1e-8)
    np.testing.assert_array_equal(z['source_frame_index'],np.arange(n));assert np.all(z['action_kind']=='recorded_path')
    if condition!='original_fixed':
        audit=json.loads((folder/'transmission_audit.json').read_text());stats=json.loads((folder/'transmission_stats.json').read_text())
        assert stats['substep_calls']==8+24*n and stats['max_used_feedback_difference_rad']==0
        assert hashlib.sha256(Path(job['preset']).read_bytes()).hexdigest()==audit['declared_preset']['sha256']
    shift=np.linalg.norm(z['goal_pose'][:,:2]-z['goal_pose'][0,:2],axis=1)*1000
    meta['conditional_initial_pose_probe']=dict(plan=str(D/'plan.json'),original_position=plan['original_can_pos'],conditional_position=plan['conditional_can_pos'],qualification=plan['qualification'])
    meta['provenance']='Conditional vision-derived initial-position probe, not admitted to a recovery bank'
    path.with_suffix('.json').write_text(json.dumps(meta,indent=2))
    row=dict(condition=condition,status='complete',sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],final_goal_shift_mm=float(shift[-1]),max_goal_shift_mm=float(shift.max()),frames=n)
    rows.append(row);(folder/'readout.json').write_text(json.dumps(row,indent=2))
report=dict(records=rows,all_terminal=all(r['status']!='pending' for r in rows),qualification=plan['qualification'])
(D/'summary.json').write_text(json.dumps(report,indent=2))
for r in rows:print(r['condition'],r['status'],r.get('sequence',{}).get('reason'),r.get('metric',{}).get('final_dist'),r.get('final_goal_shift_mm'))
