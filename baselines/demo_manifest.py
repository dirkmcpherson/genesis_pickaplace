"""Reference manifest classifying EVERY demo, for the RL-on-everything vs DP-on-success
experiment. Merges: trial_placements.json (label, placement-solve status) + the corrected-goal
gallery metrics (outcome stage reached) + v6 (which reached contact-success -> DP dataset).

Columns:
  uid, label(success/fail), status, source(solved|fk-seed|stub), stage(no-pick..nested), in_v6_dp

  source: solved   = status ok/ok_batch  -> validated can-start pose, in the training env
          fk-seed  = status no_shelf/no_slide -> replayable only via FK-guessed can start
          stub     = 290,322 (gripper never closes)
  stage: highest funnel stage reached under the corrected goal (0.662,-0.057), from the gallery.

Writes baselines/demo_manifest.{json,csv}. This is the PRELIMINARY manifest (gallery = render
path); collect_all_classified.py refines `stage` by replaying every demo in the training env.
"""
import json, pathlib, glob, os, csv
REPO = pathlib.Path('/home/j/workspace/genesis_pickaplace')
t = json.loads((REPO / 'can_pos_recovery/trial_placements.json').read_text())['trials']
gal = {r['uid']: r for r in json.loads((REPO / 'can_pos_recovery/videos_gallery/metrics.json').read_text())}
v6 = {int(os.path.basename(f)[:-4]) for f in glob.glob(str(REPO / 'baselines/episodes_raw_v6/*.npz'))}
STUB = {'290', '322'}
OUTCOME2STAGE = {'S-nested': 'nested', 'S': 'contact', 'placed-only': 'placed',
                 'picked-drop': 'picked', 'no-pick': 'no-pick'}

rows = []
for u, r in t.items():
    uid = int(u)
    label = r.get('label', '?')
    status = r.get('status', '?')
    if u in STUB:
        source = 'stub'
    elif status in ('ok', 'ok_batch'):
        source = 'solved'
    else:
        source = 'fk-seed'
    g = gal.get(uid)
    stage = OUTCOME2STAGE.get(g['outcome'], 'unmeasured') if g else 'unmeasured'
    rows.append(dict(uid=uid, label=label, status=status, source=source, stage=stage,
                     in_v6_dp=(uid in v6)))

rows.sort(key=lambda x: x['uid'])
(REPO / 'baselines/demo_manifest.json').write_text(json.dumps(rows, indent=2))
with open(REPO / 'baselines/demo_manifest.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=['uid', 'label', 'status', 'source', 'stage', 'in_v6_dp'])
    w.writeheader(); w.writerows(rows)

# summary
from collections import Counter
def tally(key, subset=None):
    rs = rows if subset is None else [r for r in rows if subset(r)]
    return dict(Counter(r[key] for r in rs))
print(f"TOTAL demos: {len(rows)}")
print(f"by label:  {tally('label')}")
print(f"by source: {tally('source')}")
print(f"\nstage x source (success-labeled only):")
succ = [r for r in rows if r['label'] == 'success']
STAGES = ['no-pick', 'picked', 'placed', 'contact', 'nested', 'unmeasured']
print(f"  {'source':10s} " + " ".join(f"{s:>9s}" for s in STAGES))
for src in ['solved', 'fk-seed']:
    c = Counter(r['stage'] for r in succ if r['source'] == src)
    print(f"  {src:10s} " + " ".join(f"{c.get(s,0):9d}" for s in STAGES))
print(f"\nRL-'everything' pool (all non-stub, both labels): {sum(1 for r in rows if r['source']!='stub')}")
print(f"DP-'success-only' pool (in v6, contact-success):  {sum(1 for r in rows if r['in_v6_dp'])}")
print(f"\nwrote baselines/demo_manifest.{{json,csv}}")
