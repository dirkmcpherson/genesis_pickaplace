#!/usr/bin/env bash
# Build two mpv playlists from the settled slide_success predicate: every success,
# and every failure. Videos are the existing real|sim census renders, so nothing is
# re-rendered. Tapes are keyed by rollout uid; the video is keyed by the trial uid
# (`ic_uid`), which is why the mapping below is read from the tapes rather than assumed.
#
# usage: slide_reels.sh [results.json] [video_dir]
set -euo pipefail
cd "$(dirname "$0")/.."
RES=${1:-can_pos_recovery/slide_predicate_results.json}
VID=${2:-can_pos_recovery/videos_census}
PY=${PY:-$HOME/workspace/genesis_sim2real/venv/bin/python}

"$PY" - "$RES" "$VID" <<'PYEOF'
import json, sys, glob, os
import numpy as np
res, vid = sys.argv[1], sys.argv[2]
rows = json.load(open(res))
# rollout uid -> trial uid, read from the tapes themselves
m = {}
for d in ('/home/james/wm_fix_2026-09-03/fulltapes/dHfull_w3_partial',
          '/home/james/wm_fix_2026-09-03/fulltapes/dHfull_w3'):
    for f in glob.glob(os.path.join(d, '*.npz')):
        try:
            z = np.load(f, allow_pickle=True)
            m[str(z['uid'])] = str(z['ic_uid'])
        except Exception:
            pass
def vpath(u):
    t = m.get(u, u)
    g = glob.glob(os.path.join(vid, f'{t}_*.mp4'))
    return g[0] if g else None
for name, keep in (('successes', True), ('failures', False)):
    sel = [r for r in rows if r['slide_success'] is keep]
    paths, missing = [], []
    for r in sorted(sel, key=lambda r: r['uid']):
        p = vpath(r['uid'])
        (paths.append(p) if p else missing.append(r['uid']))
    out = f'can_pos_recovery/slide_{name}.m3u'
    with open(out, 'w') as fh:
        fh.write('\n'.join(paths) + '\n')
    print(f'{name}: {len(paths)} clips -> {out}' + (f'  ({len(missing)} without video)' if missing else ''))
PYEOF
echo
echo "watch:  mpv --playlist=can_pos_recovery/slide_successes.m3u"
echo "        mpv --playlist=can_pos_recovery/slide_failures.m3u"
