"""Paired six-demo assessment; do not count parameter/replay repetitions as demos."""
from pathlib import Path
import sys,json
ROOT=Path(__file__).resolve().parent;sys.path.insert(0,str(ROOT.parents[2]/'can_pos_recovery'))
from score_recovered_slides import adapt
rows=[]
for uid in [113,184,233,176,185,237]:
 pool='early_yaw_pool' if uid<233 else 'timestamp_full_pool'
 baseline=ROOT.parent/f'{pool}/{uid}/collection/{uid}_eef_delta.npz'
 paths={'original_fixed':baseline}
 for label,tc in [('matching_rigid',.02),('soft_candidate',.03)]:
  folder=f'{uid}_fastreturn_tc{tc:g}' if uid in [113,184,233] else f'{uid}_validation_tc{tc:g}'
  paths[label]=ROOT/folder/f'{uid}_eef_delta.npz'
 result=dict(uid=uid,cohort='calibration' if uid in [113,184,233] else 'reserved',day='Dec16' if uid in [113,176] else 'Dec17' if uid in [184,185] else 'Dec18')
 for label,path in paths.items():
  meta=json.loads(path.with_suffix('.json').read_text());metric=adapt(path,ROOT/'comparison_metrics'/str(uid)/label)['metric']
  result[label]=dict(trace=str(path),sequence=meta['sequence'],metric=metric)
 rows.append(result)
totals={key:dict(metric_pass=sum(r[key]['metric']['slide_success'] for r in rows),strict_complete=sum(r[key]['sequence']['complete'] for r in rows),n=len(rows)) for key in paths}
(ROOT/'validation_readout.json').write_text(json.dumps(dict(records=rows,totals=totals,qualification='Selected six-demo comparison; calibration and reserved labels retained. Historical reserved outcomes were seen but excluded from new fitting. Not a population-rate estimate or hardware calibration.'),indent=2))
print(json.dumps(totals,indent=2))
lines=['# Frozen soft-pad validation','', 'No general end-to-end improvement is established. Soft pads improve this hand relative to its rigid-contact controls, but the complete hand does not improve the original recovery model across the selected demos.','', '| Demo | Day | Cohort | Original fixed: distance / strict | Matching rigid: distance / strict | Soft candidate: distance / strict |','|---|---|---|---|---|---|']
for r in rows:
 cells=[str(r['uid']),r['day'],r['cohort']]
 for key in paths:
  v=r[key];cells.append(f"{v['metric']['final_dist']*1000:.2f} mm / {v['sequence']['reason']}")
 lines.append('| '+' | '.join(cells)+' |')
lines+=['','Distances are final center-to-center distances. The unchanged supplied metric passes 3/6 for the original fixed model, 2/6 for the matching rigid hand and 3/6 for the soft candidate. Strict completions are 3/6, 0/6 and 1/6 respectively. These denominators contain four early and two late demos, selected for calibration/validation, not a population census.','', 'The reserved soft 176 reaches and retains goal contact; rigid 176 stops 2.91 mm short of surface contact. Both pass the supplied 81 mm metric. Soft 185 restores supported release relative to its rigid control but finishes 154.66 mm from the goal; the original fixed trace finished 108.27 mm away. Both 237 candidates reach goal contact and then lose final retention (soft surface gap 6.47 mm); the original fixed 237 retained contact.','', 'The first strict release in 176 is a short separation followed by regrasp, not the final release: contacts resume, including a lifted interval near 25 s. A sustained separation begins at 27.12 s, with subsequent shelf recontacts. The camera review shows discrepancies in intermediate seating and can position. Keep the strict score and review evidence separate; the supplied metric and strict thresholds are unchanged.','', 'Annotated reviews: [late 233](233_soft_pad_real_sim.mp4), [early 176](176_soft_pad_real_sim.mp4). Both include the actual final frame, clock-matched recorded views and saved simulation poses; no added settling. Every video frame decoded successfully. The view angles differ.','', 'Independent saved-action reproduction is recorded in `saved_action_verification.json` only after both fresh runs terminate and every saved array matches. No training bank or existing recovery set has been changed.','', 'Next unresolved issue: calibrating the passive transmission over the different closure/loading regimes. The lower-closure early grasps fail under this hand, while strongly closed late 233 and early 176 can be carried. Further arbitrary softness tuning on the reserved trials would not resolve that model uncertainty.']
(ROOT/'VALIDATION_READOUT.md').write_text('\n'.join(lines)+'\n')
