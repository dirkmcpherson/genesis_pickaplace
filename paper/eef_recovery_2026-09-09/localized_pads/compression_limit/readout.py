"""Read terminal compression-limit trials without changing either task metric."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
rows=[]
for f in sorted((ROOT/'logs').glob('*_execution.json')):
 st=json.loads(f.read_text());folder=ROOT/st['name'];uid=st['uid']
 if st['returncode']:rows.append(dict(**st,error='Execution failed; not scored'));continue
 path=folder/f'{uid}_eef_delta.npz';z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text());base=np.load(ROOT.parent/f'critical_return/{uid}_tc0.02/{uid}_eef_delta.npz')
 equality={k:bool(np.array_equal(z[k],base[k])) for k in z.files};assert all(equality[k] for k in ['actions_eef','actions_joint','source_grip','mount'])
 if st['compression_limit']==0:assert all(equality.values()),equality
 n=len(z['actions_eef']);stats=json.loads((folder/'transmission_stats.json').read_text());assert stats['substep_calls']==8+24*n
 counts=np.load(folder/'surface_pad_observations.npz')['counts'];assert counts.shape==(1+3*n,3)
 assert meta['physics_treatment']['surface_pad_treatment']['soft_compression_limit_m']==st['compression_limit']
 if st['compression_limit']>0 and counts[:,0].sum():assert counts[:,1].sum()>0
 if st['compression_limit']==0:assert counts[:,1].sum()==0
 row=dict(name=st['name'],uid=uid,compression_limit_m=st['compression_limit'],sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],reference_arrays_exact=equality,pad_counts_sum=counts.sum(axis=0).tolist(),max_abs_joint_velocity_rad_s=stats['max_abs_velocity'])
 (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows),indent=2))
for r in rows:print(r['name'],r.get('error') or (r['sequence']['reason'],r['metric']['slide_success'],round(r['metric']['final_dist']*1000,2)))
