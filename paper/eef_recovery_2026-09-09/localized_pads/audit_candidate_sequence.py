"""Additional post-pick audit; neither scorer nor its constants are changed."""
from pathlib import Path
import sys,json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parents[2]/'can_pos_recovery'))
from slide_predicate import classify
from eef_task_sequence import score_sequence
records=[]
for tc in [.02,.03,.04]:
 folder=ROOT/f'233_fastreturn_tc{tc:g}';path=folder/'233_eef_delta.npz';z=np.load(path)
 strict=score_sequence(z);release=strict['release_start'];tr=z['trajectory'];c=z['contact_counts'];g=z['goal_pose'];n=len(tr)
 result=dict(name=folder.name,strict_sequence=strict)
 if release is not None:
  metric=np.load(folder/'metric_adapter/233.npz');keep=metric['source_frame_index']>=release
  suffix=folder/'metric_adapter/233_post_supported_release.npz'
  np.savez_compressed(suffix,states=metric['states'][keep],eef_pos=metric['eef_pos'][keep],tipped=metric['tipped'][keep],uid=np.array('233_post_supported_release'))
  result['unchanged_predicate_on_post_supported_release_suffix']=classify(suffix)
  support=(c[:,0]>0)&(abs(tr[:,15]-.2205)<=.004)&(tr[:,20]<20)
  candidates=np.flatnonzero(support&(c[:,1]>0)&(np.arange(n)>=release+3))
  result['supported_push_to_final']=None
  for start in candidates:
   toward=g[start,:2]-tr[start,13:15];toward/=np.linalg.norm(toward)
   movement=float((tr[-1,13:15]-tr[start,13:15])@toward)
   fraction=float(support[start:].mean());low=bool(np.all(abs(tr[start:,15]-.2205)<=.008));upright=bool(np.all(tr[start:,20]<20))
   if movement>=.01 and fraction>=.8 and low and upright:
    result['supported_push_to_final']=dict(start_frame=int(start),start_time_s=float((start+1)*.03),goalward_movement_m=movement,support_fraction=fraction,stays_low=low,stays_upright=upright,post_release_goal_contact_frames=int((c[release:,2]>0).sum()))
    break
  result['release_time_s']=(release+1)*.03
 result['qualification']='Suffix score and push-to-final audit are additional diagnostics, not replacements for either full-tape metric or strict goal-contact sequence.'
 records.append(result)
(ROOT/'candidate_sequence_audit.json').write_text(json.dumps(dict(records=records,slide_predicate_sha256=hashlib.sha256((ROOT.parents[2]/'can_pos_recovery/slide_predicate.py').read_bytes()).hexdigest()),indent=2))
print(json.dumps(records,indent=2))
