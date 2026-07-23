#!/bin/bash
# Full re-collection of episodes_all after the max_steps/_nested collector fix +
# 11 new re-search winners (solved 61 -> 72). 16-way parallel by uid chunks.
# Old dataset archived (post-1200 tails of long demos were desynced -- see
# collect_all_classified.py comment at env.max_steps).
set -e
cd /home/j/workspace/genesis_pickaplace
PY=.venv-eval/bin/python
OUT=baselines/episodes_all
ARCHIVE=baselines/episodes_all_maxstepsbug

if [ -d "$OUT" ] && [ ! -d "$ARCHIVE" ]; then
  mv "$OUT" "$ARCHIVE"
  echo "archived old episodes_all -> $ARCHIVE"
fi
mkdir -p "$OUT"

# 91 non-stub uids split into 16 contiguous chunks
UIDS=$($PY - <<'EOF'
import json
tbl = json.loads(open('can_pos_recovery/trial_placements.json').read())['trials']
uids = sorted(int(u) for u in tbl if u not in ('290', '322'))
n = 16
chunks = [uids[i::n] for i in range(n)]
print(';'.join(' '.join(str(u) for u in c) for c in chunks))
EOF
)
IFS=';' read -ra CHUNKS <<< "$UIDS"
for k in "${!CHUNKS[@]}"; do
  $PY baselines/collect_all_classified.py --outdir $OUT --uids ${CHUNKS[$k]} \
      > baselines/recollect_w$k.log 2>&1 &
done
wait
echo "ALL COLLECTION WORKERS DONE"

# merge worker shards -> authoritative manifest
$PY - <<'EOF'
import json, glob, pathlib as pl
from collections import Counter
recs = {}
for p in sorted(glob.glob('baselines/episodes_all/_stage_*.json')):
    recs.update(json.loads(pl.Path(p).read_text()))
pl.Path('baselines/demo_manifest_auth.json').write_text(json.dumps(
    {str(k): v for k, v in sorted(recs.items(), key=lambda kv: int(kv[0]))}, indent=1))
n = len(recs)
succ = [r for r in recs.values() if r['label'] == 'success']
print(f'manifest: {n} demos')
print('success stages:', Counter(r['stage'] for r in succ))
print('fail stages:', Counter(r['stage'] for r in recs.values() if r['label'] == 'fail'))
EOF
echo "RECOLLECT PIPELINE DONE"
