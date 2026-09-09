from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
R=Path(__file__).resolve().parent
source=R.parent/'early_yaw_pool/113/collection/113_eef_delta.npz'
probe=R/'113/collection/113_eef_delta.npz'
with np.load(source) as a,np.load(probe) as b:
 prefix={k:bool(np.array_equal(a[k][:267],b[k][:267])) for k in ['trajectory','actions_eef','observations','finger_joint','contact_counts']}
 # observations has a pre-action reset; include final prefix observation explicitly.
 prefix['observations']=bool(np.array_equal(a['observations'][:268],b['observations'][:268]))
 assert all(prefix.values()),prefix
 rows=[]
 for name,z in [('recorded_continuation',a),('static_target_hold',b)]:
  tr=z['trajectory'][266:434];q=tr[:,9:13];rot=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix();local=np.einsum('nji,nj->ni',rot,tr[:,13:16]-tr[:,6:9]);contacts=z['contact_counts'][266:434]
  rows.append(dict(condition=name,frames=len(tr),tool_displacement_m=float(np.linalg.norm(tr[-1,6:9]-tr[0,6:9])),can_relative_to_tool_displacement_m=float(np.linalg.norm(local[-1]-local[0])),initial_local_can_m=local[0].tolist(),final_local_can_m=local[-1].tolist(),max_tilt_deg=float(tr[:,20].max()),final_tilt_deg=float(tr[-1,20]),robot_contact_fraction=float((contacts[:,1]>0).mean()),shelf_contact_fraction=float((contacts[:,0]>0).mean())))
result=dict(prefix_bit_identical=prefix,results=rows,scope='One reproduced grasp, modified static-target diagnostic, no demo admission; cannot identify which physical parameter is wrong')
(R/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
