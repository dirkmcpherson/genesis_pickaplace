"""Score terminal attribution runs, requiring exact unchanged-material controls."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[3];POOL=ROOT.parents[1]
sys.path.insert(0,str(REPO/'can_pos_recovery'))
from score_recovered_slides import adapt
rows=[]
for file in sorted((ROOT/'logs').glob('*_execution.json')):
 status=json.loads(file.read_text());assert status['returncode']==0,status
 uid=status['uid'];folder=ROOT/status['name'];path=folder/f'{uid}_eef_delta.npz';z=np.load(path);meta=json.loads(path.with_suffix('.json').read_text())
 ref=POOL/f'{"early_yaw_pool" if uid<233 else "timestamp_full_pool"}/{uid}/collection/{uid}_eef_delta.npz';base=np.load(ref)
 assert all(np.array_equal(z[k],base[k]) for k in ['actions_eef','actions_joint','source_grip','mount'])
 equality={k:bool(np.array_equal(z[k],base[k])) for k in z.files}
 if status['pad_timeconst_s']==.02:assert all(equality.values()),equality
 n=len(z['actions_eef']);obs=np.load(folder/'pad_observations.npz');audit=json.loads((folder/'pad_audit.json').read_text())
 assert obs['counts'].shape==(1+3*n,3);assert audit['detection_calls']>=8+24*n
 if status['pad_timeconst_s']>.02:assert obs['counts'][:,1].sum()>0
 row=dict(uid=uid,pad_timeconst_s=status['pad_timeconst_s'],reference=str(ref),reference_arrays_exact=equality,sequence=meta['sequence'],metric=adapt(path,folder/'metric_adapter')['metric'],pad_counts_sum=obs['counts'].sum(axis=0).tolist())
 (folder/'readout.json').write_text(json.dumps(row,indent=2));rows.append(row)
(ROOT/'summary.json').write_text(json.dumps(dict(records=rows),indent=2))
for r in rows:print(r['uid'],r['pad_timeconst_s'],r['sequence']['reason'],r['metric']['slide_success'],r['metric']['final_dist'])
