"""Freeze an expanded panel using source gripper signals, not sim outcomes."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent;POOL=ROOT.parents[1]
excluded_fit={113,184,233};old_reserved={176,185,237}
invalid=set(json.loads((POOL/'early_yaw_pool/initial_geometry_audit.json').read_text())['overlap_uids'])
rows=[];missing=[]
for pool in ['early_yaw_pool','timestamp_full_pool']:
 for source in sorted((POOL/pool).glob('*/source.npz'),key=lambda p:int(p.parent.name)):
  uid=int(source.parent.name)
  if uid in excluded_fit|old_reserved|invalid:continue
  meta=json.loads(source.with_suffix('.json').read_text());z=np.load(source);g=z['source_grip'];ix=np.flatnonzero(g>=50)
  if not len(ix) or ix[0]+100>len(g):missing.append(uid);continue
  first=int(ix[0]);closure=float(np.median(g[first+34:first+100]));day='Dec16' if meta['variant'].endswith('yaw16') else 'Dec17' if meta['variant'].endswith('yaw17') else 'Dec18'
  rows.append(dict(uid=uid,day=day,closure_stratum='lower' if closure<80 else 'higher',initial_closure_median_percent=closure,first_50_percent_time_s=first*.03,source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
rng=np.random.default_rng(20260909);selected=[]
for day in ['Dec16','Dec17','Dec18']:
 for stratum in ['lower','higher']:
  group=[r for r in rows if r['day']==day and r['closure_stratum']==stratum];assert group,(day,stratum)
  selected.append(group[int(rng.integers(len(group)))])
plan=dict(new_panel=selected,old_reserved=sorted(old_reserved),excluded_calibration=sorted(excluded_fit),excluded_initial_model_overlap=sorted(invalid),excluded_no_eligible_signal_window=missing,
 selection='One random source per day × initial closure stratum, seed 20260909. Initial closure is median recorded gripper feedback 1.02–2.97 s after first >=50% sample; lower<80%, higher>=80%. No simulated outcome used for selection.',
 qualification='Source-signal proxy, not measured grasp depth. All old full-pool outcomes were historically available; these six have not been used in pad fitting. Excluding known model-overlap ICs defines the panel, not permanent exclusion of recordings.',
 eligible=rows,preset_sha256={f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in (ROOT.parent/'critical_return/presets').glob('*.json')},
 gate='Compare frozen soft and same-hand rigid controls against each original full-world replay. Retain failures and regressions, verify actual supported slide, inspect real-video fidelity before any acceptance. One exact saved-action replay of 233 included. No new fitting on this panel.')
assert not (ROOT/'selection.json').exists();(ROOT/'selection.json').write_text(json.dumps(plan,indent=2))
print(json.dumps(selected,indent=2));print('total validation uids',sorted(old_reserved|{r['uid'] for r in selected}))
