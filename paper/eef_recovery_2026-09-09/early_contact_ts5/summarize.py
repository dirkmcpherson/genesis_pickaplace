from pathlib import Path
import json
ROOT=Path(__file__).resolve().parent
plan=json.loads((ROOT/'plan.json').read_text())
base=json.loads((ROOT.parent/'slide_metric_of_record/early_timestamp/results.json').read_text())['records']
base={r['uid']:r for r in base}
records=[]
for uid in plan['uids']:
 p=ROOT/str(uid)/'result.json'
 if not p.exists():continue
 result=json.loads(p.read_text())['score'];control=base[uid]
 records.append(dict(uid=uid,is_success_control=uid in plan['controls'],baseline_metric=control['metric'],probe_metric=result['metric'],final_distance_change_m=result['metric']['final_dist']-control['metric']['final_dist'],baseline_tilt_deg=control['final_tilt_deg'],probe_tilt_deg=result['final_tilt_deg'],baseline_sequence=control['strict_contact_diagnostic'],probe_sequence=result['strict_contact_diagnostic']))
report=dict(declared=len(plan['uids']),completed=len(records),records=records,new_metric_passes=[r['uid'] for r in records if r['probe_metric']['slide_success'] and not r['baseline_metric']['slide_success']],lost_metric_passes=[r['uid'] for r in records if not r['probe_metric']['slide_success'] and r['baseline_metric']['slide_success']],new_strict_completions=[r['uid'] for r in records if r['probe_sequence']['complete'] and not r['baseline_sequence']['complete']],lost_strict_completions=[r['uid'] for r in records if not r['probe_sequence']['complete'] and r['baseline_sequence']['complete']],qualification='Changed physics, not calibrated real compliance; no result is admitted by this report')
(ROOT/'summary.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k!='records'},indent=2))
