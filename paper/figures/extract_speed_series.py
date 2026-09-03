import numpy as np, glob, pathlib as pl
out = {}
for arm in ['dH', 'dDP']:
    for f in sorted(glob.glob(f'/cluster/tufts/shortlab/jstale02/genesis_pickaplace/baselines/matched_w3/{arm}/*.npz')):
        z = np.load(f, allow_pickle=True)
        eef = z['eef_pos'].astype(np.float32)
        sp = np.linalg.norm(np.diff(eef, axis=0), axis=1)          # per-decision EEF speed
        out[f'{arm}|{int(z["ic_uid"])}|{int(z["uid"])}'] = sp
np.savez_compressed('/cluster/tufts/shortlab/jstale02/tape_stats_out/speed_series_w3.npz', **out)
print('tapes:', len(out))
