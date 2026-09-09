"""Declared development timing probe; does not modify frozen tapes or placements."""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[2]
PYTHON = '/home/james/workspace/genesis_sim2real/venv/bin/python'


def prepare(uid, timed_path=None):
    parent = REPO / 'paper/eef_recovery_2026-09-09'
    folder = parent / ('upright_ic_probe' if uid == 234 else 'full_pool') / str(uid)
    timed_path = timed_path or REPO / f'inthewild_trials/{uid}_timed.npz'
    with np.load(folder/'source.npz') as raw, np.load(timed_path) as timed:
        assert np.array_equal(raw['joint_waypoints'], timed['q_frame'])
        assert np.array_equal(raw['source_grip'], timed['g_frame'])
        t = timed['t_frame'] - timed['t_frame'][0]
        assert np.all(np.diff(t) > 0)
        # Joint tapes are continuous radians; reject a wrap before interpolating.
        q = timed['q_frame']
        assert np.max(np.abs(np.diff(q, axis=0))) < np.pi
        # Targets are applied at the start of each 30 ms action. Include the
        # final sample, clamped at the endpoint; no explicit settling tail.
        sample = np.arange(int(np.ceil(t[-1]/.03))+1)*.03
        joints = np.stack([np.interp(sample,t,q[:,j]) for j in range(6)],axis=1)
        grip = np.interp(sample,t,timed['g_frame'])
        dst = ROOT/str(uid)
        dst.mkdir(exist_ok=False)
        np.savez_compressed(dst/'source.npz',joint_waypoints=joints,source_grip=grip,
                            sample_time_s=sample,original_frame_time_s=t)
        meta = json.loads((folder/'source.json').read_text())
        meta['timing_reconstruction'] = dict(
            method='linear interpolation of matched bag-window means at action-start times',
            source=str(timed_path),sha256=hashlib.sha256(timed_path.read_bytes()).hexdigest(),
            original_frames=len(t),original_span_s=float(t[-1]),
            decision_dt_s=.03, replay_duration_s=float(len(sample)*.03),
            endpoint_overrun_s=float(len(sample)*.03-t[-1]),
            limitation='window-close timestamps represent window means; interpolation is approximate, not original hardware commands')
        meta['source_frames']=len(sample)
        (dst/'source.json').write_text(json.dumps(meta,indent=2))


def run(uid):
    dst=ROOT/str(uid)
    with (dst/'run.log').open('w') as log:
        p=subprocess.run([PYTHON,str(REPO/'can_pos_recovery/repair_eef_slide.py'),
                          str(dst/'source.npz'),'--out',str(dst/'collection'),
                          '--max-extension','0','--polish-ik'],stdout=log,stderr=subprocess.STDOUT)
    result=dict(uid=uid,returncode=p.returncode)
    path=dst/'collection'/f'{uid}_eef_delta.json'
    if path.exists():
        meta=json.loads(path.read_text())
        meta['timing_reconstruction']=json.loads((dst/'source.json').read_text())['timing_reconstruction']
        meta['provenance']='timestamp-resampled real measurements; development timing probe'
        path.write_text(json.dumps(meta,indent=2))
        result['sequence']=meta['sequence']
    (dst/'execution.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)


if __name__=='__main__':
    uids=[233,234,235]
    (ROOT/'plan.json').write_text(json.dumps(dict(uids=uids,physics='unchanged w3',
        correction_234='upright quaternion and z from prior isolated probe; archived xy',
        grip='unfiltered measured grip, jointly resampled with arm using bag times',
        outcome='unchanged full sequence scorer; retain all failures; no bank admission'),indent=2))
    for uid in uids:prepare(uid)
    with ThreadPoolExecutor(max_workers=3) as pool:list(pool.map(run,uids))
