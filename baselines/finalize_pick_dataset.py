"""Run AFTER collect_all_classified.py. (1) Merge the per-worker _stage shards into an
authoritative manifest (baselines/demo_manifest_auth.json, measured in the training env at
the corrected goal). (2) Build the DP "successful picks" dataset = demos that reached >= picked,
as symlinks into baselines/episodes_pick/ so convert_to_lerobot can pack it.

By default uses SOLVED demos only (validated can-start poses); --include-fk also pulls the
FK-seeded picked demos (guessed ICs, more data).

Usage: python baselines/finalize_pick_dataset.py [--include-fk]
"""
import argparse, json, pathlib as pl, glob, os
REPO = pl.Path('/home/j/workspace/genesis_pickaplace')
ALL = REPO / 'baselines/episodes_all'
PICK = REPO / 'baselines/episodes_pick'
PICKED_STAGES = {'picked', 'placed', 'contact', 'nested'}

ap = argparse.ArgumentParser()
ap.add_argument('--include-fk', action='store_true', help='also include fk-seed picked demos')
ap.add_argument('--include-fail', action='store_true',
                help='also include FAIL-labeled demos (default: success-only -- fails are the neg control)')
args = ap.parse_args()

# 1. merge shards
manifest = {}
for shard in glob.glob(str(ALL / '_stage_*.json')):
    manifest.update({int(k): v for k, v in json.loads(pl.Path(shard).read_text()).items()})
manifest = dict(sorted(manifest.items()))
(REPO / 'baselines/demo_manifest_auth.json').write_text(json.dumps(manifest, indent=2))
from collections import Counter
print(f"authoritative manifest: {len(manifest)} demos")
print(f"  stage x source:")
for src in ('solved', 'fk-seed'):
    c = Counter(v['stage'] for v in manifest.values() if v['source'] == src)
    print(f"    {src:8s}: {dict(c)}")

# 2. build episodes_pick (symlinks)
if PICK.exists():
    for f in PICK.glob('*.npz'):
        f.unlink()
PICK.mkdir(exist_ok=True, parents=True)
n = 0
for uid, v in manifest.items():
    if v['stage'] not in PICKED_STAGES:
        continue
    if v.get('label') != 'success' and not args.include_fail:
        continue   # fails are the negative control -> keep out of the success-only DP set
    if v['source'] == 'fk-seed' and not args.include_fk:
        continue
    src = ALL / f'{uid}.npz'
    if src.exists():
        (PICK / f'{uid}.npz').symlink_to(src)
        n += 1
print(f"\nepisodes_pick: {n} successful-pick demos "
      f"({'solved+fk' if args.include_fk else 'solved only'})")
print(f"  -> convert with: convert_to_lerobot.py baselines/episodes_pick "
      f"baselines/lerobot_dataset_pick/genesis_pickaplace 8 4")
