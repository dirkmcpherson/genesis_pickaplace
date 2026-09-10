"""Merge the declared64-substep follow-up with frozen8/16/32 results."""
from pathlib import Path
import json,numpy as np
D=Path(__file__).resolve().parent;P=D.parent
folders={ss:P/'integration_resolution'/f'233_soft_ss{ss}' for ss in [8,16,32]};folders[64]=D/'233_soft_ss64'
rows={};traces={};audits={}
for ss,folder in folders.items():
 if not (folder/'readout.json').exists():continue
 rows[ss]=json.loads((folder/'readout.json').read_text());traces[ss]=np.load(folder/'233_eef_delta.npz');audits[ss]=json.loads((folder/'transmission_audit.json').read_text())
comparisons=[]
for lo,hi in [(8,16),(16,32),(32,64),(8,64)]:
 if lo not in traces or hi not in traces:continue
 for k in ['geometry','collision_policy','armature_readback','normal_parameters','finger_inertial_readback','can_mass_kg','goal_mass_kg']:assert audits[lo][k]==audits[hi][k],k
 a=traces[lo];b=traces[hi];d=1000*np.linalg.norm(a['trajectory'][:,13:16]-b['trajectory'][:,13:16],axis=-1)
 comparisons.append(dict(substeps=[lo,hi],max_can_position_difference_mm=float(d.max()),final_can_position_difference_mm=float(d[-1]),max_finger_difference_rad=float(abs(a['finger_joint']-b['finger_joint']).max()),contact_count_disagreement_frames=int(np.any(a['contact_counts']!=b['contact_counts'],axis=1).sum())))
(D/'combined_summary.json').write_text(json.dumps(dict(records=rows,comparisons=comparisons,qualification='Same intended physical parameters, fixed commands/endpoint. No resolution selected by passing scores; compare successive refinement and physical-image evidence.'),indent=2))
print(json.dumps(comparisons,indent=2))
