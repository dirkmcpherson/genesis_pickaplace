"""Require exact fresh-process reproduction of saved candidate actions and states."""
from pathlib import Path
import json,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parent
rows=[]
for uid,parent in [(233,'233_fastreturn_tc0.03'),(176,'176_validation_tc0.03')]:
 name=f'{uid}_saved_action_verification'
 status=json.loads((ROOT/'logs'/f'{name}_execution.json').read_text());assert status['returncode']==0
 original=ROOT/parent/f'{uid}_eef_delta.npz';replay=ROOT/name/f'{uid}_eef_delta.npz'
 a=np.load(original);b=np.load(replay);meta=json.loads(replay.with_suffix('.json').read_text())
 assert meta['action_replay_verification'] and meta['repair_note']=='saved_actions_only'
 assert meta['extension_m']==0 and not meta['hold_contact']
 assert a.files==b.files
 equality={key:bool(np.array_equal(a[key],b[key])) for key in a.files};assert all(equality.values()),equality
 assert json.loads(original.with_suffix('.json').read_text())['sequence']==meta['sequence']
 rows.append(dict(uid=uid,source=str(original),replay=str(replay),source_sha256=hashlib.sha256(original.read_bytes()).hexdigest(),replay_sha256=hashlib.sha256(replay.read_bytes()).hexdigest(),arrays_exact=equality,frames=len(a['actions_eef']),sequence=meta['sequence']))
(ROOT/'saved_action_verification.json').write_text(json.dumps(dict(passed=True,records=rows),indent=2))
print('PASS: both fresh-process action replays reproduce every saved array exactly')
