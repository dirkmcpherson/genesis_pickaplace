"""Describe the actual233 post-release slide without changing either scorer."""
from pathlib import Path
import json,sys
import numpy as np
D=Path(__file__).resolve().parent;P=D.parent;R=D.parents[3]
sys.path.insert(0,str(R/'can_pos_recovery'))
from eef_task_sequence import sustained_starts
records=[]
for label,path in [('soft_g1',P/'g14_feedback_full/233_elliptic10_soft/233_eef_delta.npz'),('soft_normal_g2',D/'233_original_soft_g2/233_eef_delta.npz')]:
 z=np.load(path);tr=z['trajectory'];c=z['contact_counts'];g=z['goal_pose'];m=json.loads(path.with_suffix('.json').read_text());release=m['sequence']['release_start']
 support=(c[:,0]>0)&(abs(tr[:,15]-.2205)<=.004)&(tr[:,20]<20)
 hand=np.flatnonzero(support&(c[:,1]>0)&(np.arange(len(tr))>release+2));touch=np.flatnonzero(c[:,2]>0)
 row=dict(label=label,release_s=(release+1)*.03,goal_contact_frames=touch.tolist(),goal_contact_3frame_starts=sustained_starts(c[:,2]>0).tolist(),final_surface_gap_mm=1000*(np.linalg.norm(tr[-1,13:15]-g[-1,:2])-.066),final_support=bool(support[-1]))
 if len(hand):
  i=int(hand[0]);toward=g[i,:2]-tr[i,13:15];toward/=np.linalg.norm(toward)
  row.update(first_supported_hand_s=(i+1)*.03,movement_to_final_toward_goal_mm=float((tr[-1,13:15]-tr[i,13:15])@toward*1000),remaining_support_fraction=float(support[i:].mean()),max_height_error_mm=float(abs(tr[i:,15]-.2205).max()*1000),max_tilt_deg=float(tr[i:,20].max()))
 records.append(row)
(D/'233_final_slide_diagnostic.json').write_text(json.dumps(dict(records=records,qualification='Descriptive post-release supported-hand-to-end measurements. Does not change either success criterion or convert a proximity pass to contact success.'),indent=2))
